#include "zxh_bms.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#ifdef USE_ESP32

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace esphome::zxh_bms {

namespace espbt = esphome::esp32_ble_tracker;

static const char *const TAG = "zxh_bms";

ZxhBMS *ZxhBMS::mux_owner_ = nullptr;
std::vector<ZxhBMS *> ZxhBMS::mux_waiting_;

static const uint16_t TEMP_OFFSET = 2732;  // raw units are (degC * 10) + 2732

static const char *const PROTECTION_BITS[16] = {
    "cell voltage difference large",  "voltage detect line open",
    "MOS high temperature",           "protect board locked",
    "chip failure",                   "short circuit",
    "discharge overcurrent",          "charge overcurrent",
    "discharge low temperature",      "discharge high temperature",
    "charge low temperature",         "charge high temperature",
    nullptr,                          nullptr,  // reserved
    "cell undervoltage",              "cell overvoltage",
};

/// CRC16/XMODEM (poly 0x1021, init 0), same as the app's crc.js.
static uint16_t crc16_xmodem(const uint8_t *data, size_t len) {
  uint16_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t) data[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static uint16_t be16(const uint8_t *d) { return (uint16_t)((d[0] << 8) | d[1]); }

static float decode_temp(uint16_t raw) {
  float c = (raw - TEMP_OFFSET) / 10.0f;
  if (c < -40.0f)
    return NAN;
  return c;
}

static uint8_t bcd_byte(uint8_t b) { return (uint8_t)((b >> 4) * 10 + (b & 0x0F)); }

/// Scan rx_ for the first complete CRC-valid frame matching (addr, function).
/// On success the frame and any garbage before it are consumed from buf.
static bool extract_response(std::vector<uint8_t> &buf, uint8_t addr, uint8_t function,
                             std::vector<uint8_t> &out) {
  const size_t n = buf.size();
  for (size_t i = 0; i + 6 < n; i++) {
    if (buf[i] != addr || buf[i + 1] != function)
      continue;
    uint16_t length = (uint16_t)((buf[i + 2] << 8) | buf[i + 3]);
    if (length > 240)
      continue;
    const size_t total = (size_t) length + 6;
    if (n - i < total)
      continue;
    const uint8_t *frame = buf.data() + i;
    uint16_t want = (uint16_t)((frame[total - 2] << 8) | frame[total - 1]);
    if (crc16_xmodem(frame, total - 2) != want)
      continue;
    out.assign(frame + 4, frame + total - 2);
    buf.erase(buf.begin(), buf.begin() + i + total);
    return true;
  }
  return false;
}

void ZxhBMS::setup() {
  if (this->configured_address_ != 0) {
    this->address_ = this->configured_address_;
    this->address_discovered_ = true;
  }
}

void ZxhBMS::dump_config() {
  ESP_LOGCONFIG(TAG, "ZXH BMS:");
  if (this->configured_address_ != 0)
    ESP_LOGCONFIG(TAG, "  Modbus address: %u (fixed)", this->configured_address_);
  else
    ESP_LOGCONFIG(TAG, "  Modbus address: auto-discover");
  if (this->sequential_)
    ESP_LOGCONFIG(TAG, "  Mode: sequential (cycle timeout: %" PRIu32 "s)", this->cycle_timeout_ms_ / 1000);
  else
    ESP_LOGCONFIG(TAG, "  Mode: persistent (keeps the BLE link up)");
  LOG_UPDATE_INTERVAL(this);
}

void ZxhBMS::reset_connection_state() {
  this->profile_found_ = false;
  this->notify_registered_ = false;
  this->write_pending_ = false;
  this->rx_.clear();
  this->queue_.clear();
  this->cmd_pos_ = 0;
  this->phase_ = CyclePhase::IDLE;
  this->identify_done_ = false;
  this->address_discovered_ = this->configured_address_ != 0;
  if (this->configured_address_ != 0)
    this->address_ = this->configured_address_;
}

void ZxhBMS::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                 esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_SEARCH_CMPL_EVT:
      if (param->search_cmpl.status == ESP_GATT_OK && !this->profile_found_)
        this->discover_profile();
      break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
      if (param->reg_for_notify.status == ESP_GATT_OK) {
        this->notify_registered_ = true;
        // All handles are snapshotted; let the parent free the GATT cache.
        this->node_state = espbt::ClientState::ESTABLISHED;
        ESP_LOGI(TAG, "Notifications enabled, BMS ready");
      } else {
        ESP_LOGW(TAG, "Register for notify failed, status=%d", param->reg_for_notify.status);
      }
      break;
    case ESP_GATTC_NOTIFY_EVT:
      if (param->notify.handle != this->notify_handle_)
        break;
      this->rx_.insert(this->rx_.end(), param->notify.value, param->notify.value + param->notify.value_len);
      if (this->rx_.size() > RX_BUFFER_LIMIT)
        this->rx_.clear();  // resync after garbage
      break;
    case ESP_GATTC_WRITE_CHAR_EVT:
      this->write_pending_ = false;
      if (param->write.status != ESP_GATT_OK)
        ESP_LOGW(TAG, "Characteristic write failed, status=%d", param->write.status);
      break;
    case ESP_GATTC_DISCONNECT_EVT:
      this->reset_connection_state();
      if (this->sequential_)
        this->release_slot();  // already disconnected; free the slot + block auto-reconnect
      break;
    default:
      break;
  }
}

void ZxhBMS::discover_profile() {
  // Same profiles, in the same order, as the app/CLI's KNOWN_PROFILES.
  static const char *const SERVICE_PREFIXES[3] = {"00002760", "6E400001", "0003CDD0"};
  static const char *const NOTIFY_UUIDS[3] = {
      "00002760-08C2-11E1-9073-0E8AC72E0002",
      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
      "0003CDD1-0000-1000-8000-00805F9B0131",
  };
  static const char *const WRITE_UUIDS[3] = {
      "00002760-08C2-11E1-9073-0E8AC72E0001",
      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
      "0003CDD2-0000-1000-8000-00805F9B0131",
  };

  auto gattc_if = (esp_gatt_if_t) this->parent_->get_gattc_if();
  auto conn_id = this->parent_->get_conn_id();

  for (uint16_t si = 0; si < 64; si++) {
    esp_gattc_service_elem_t svc;
    uint16_t n = 1;
    if (esp_ble_gattc_get_service(gattc_if, conn_id, nullptr, &svc, &n, si) != ESP_GATT_OK || n == 0)
      break;

    char sbuf[37];
    espbt::ESPBTUUID::from_uuid(svc.uuid).as_128bit().to_str(sbuf);
    int prof = -1;
    for (int p = 0; p < 3; p++) {
      if (strncmp(sbuf, SERVICE_PREFIXES[p], 8) == 0) {
        prof = p;
        break;
      }
    }
    if (prof < 0)
      continue;

    uint16_t nh = 0, wh = 0, fb_nh = 0, fb_wh = 0;
    bool nr = false, fb_nr = false;
    for (uint16_t ci = 0; ci < 64; ci++) {
      esp_gattc_char_elem_t chr;
      uint16_t cn = 1;
      if (esp_ble_gattc_get_all_char(gattc_if, conn_id, svc.start_handle, svc.end_handle, &chr, &cn, ci) !=
              ESP_GATT_OK ||
          cn == 0)
        break;
      auto cuuid = espbt::ESPBTUUID::from_uuid(chr.uuid).as_128bit();
      if (cuuid == espbt::ESPBTUUID::from_raw(NOTIFY_UUIDS[prof])) {
        nh = chr.char_handle;
      } else if (cuuid == espbt::ESPBTUUID::from_raw(WRITE_UUIDS[prof])) {
        wh = chr.char_handle;
        nr = (chr.properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR) != 0;
      } else {
        auto props = (uint32_t) chr.properties;
        if (!fb_nh && (props & (ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_INDICATE)))
          fb_nh = chr.char_handle;
        if (!fb_wh && (props & (ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR))) {
          fb_wh = chr.char_handle;
          fb_nr = (props & ESP_GATT_CHAR_PROP_BIT_WRITE_NR) != 0;
        }
      }
      if (nh && wh)
        break;
    }
    // Fallback: first notify-capable + first write-capable characteristic, like the CLI.
    if (!nh)
      nh = fb_nh;
    if (!wh) {
      wh = fb_wh;
      nr = fb_nr;
    }
    if (nh && wh) {
      this->notify_handle_ = nh;
      this->write_handle_ = wh;
      this->write_with_response_ = !nr;
      this->profile_found_ = true;
      ESP_LOGI(TAG, "GATT profile %s: notify handle=0x%02x write handle=0x%02x write_with_response=%d",
               SERVICE_PREFIXES[prof], nh, wh, this->write_with_response_);
      auto status =
          esp_ble_gattc_register_for_notify(gattc_if, this->parent_->get_remote_bda(), nh);
      if (status != ESP_GATT_OK)
        ESP_LOGE(TAG, "esp_ble_gattc_register_for_notify failed, status=%d", status);
      return;
    }
    ESP_LOGW(TAG, "Service %s matched a known profile but has no notify/write pair", sbuf);
  }
  ESP_LOGE(TAG, "No known BMS GATT profile found on this device");
}

void ZxhBMS::update() {
  if (this->sequential_) {
    if (this->mux_queued_ || ZxhBMS::mux_owner_ == this) {
      ESP_LOGD(TAG, "Already queued for or holding the radio slot, skipping this interval");
      return;
    }
    this->mux_queued_ = true;
    ZxhBMS::mux_waiting_.push_back(this);
    return;
  }
  if (!this->notify_registered_)
    return;
  if (this->phase_ != CyclePhase::IDLE) {
    ESP_LOGD(TAG, "Previous poll cycle still running, skipping this interval");
    return;
  }
  if (!this->identify_done_)
    this->start_identify_cycle();
  else
    this->start_status_cycle();
}

void ZxhBMS::try_connect_() {
  uint32_t now = millis();
  if (now - this->next_connect_try_ < CONNECT_RETRY_MS)
    return;
  this->next_connect_try_ = now;
  if (this->parent_->state() == espbt::ClientState::IDLE)
    this->parent_->connect();
}

void ZxhBMS::release_slot() {
  if (ZxhBMS::mux_owner_ != this)
    return;
  ZxhBMS::mux_owner_ = nullptr;
  this->slot_cycle_pending_ = false;
  this->reset_connection_state();
  // Disconnects if still up, and blocks scan-triggered auto-reconnects so
  // this client cannot grab the radio back between slots.
  this->parent_->set_enabled(false);
}

void ZxhBMS::mux_loop() {
  if (ZxhBMS::mux_owner_ != this) {
    if (this->mux_queued_ && ZxhBMS::mux_owner_ == nullptr) {
      this->mux_queued_ = false;
      ZxhBMS::mux_waiting_.erase(
          std::remove(ZxhBMS::mux_waiting_.begin(), ZxhBMS::mux_waiting_.end(), this),
          ZxhBMS::mux_waiting_.end());
      ZxhBMS::mux_owner_ = this;
      this->slot_deadline_ = millis();
      this->next_connect_try_ = 0;
      this->slot_cycle_pending_ = true;
      this->parent_->set_enabled(true);
      ESP_LOGI(TAG, "Radio slot granted to %s", this->parent_->address_str());
      this->try_connect_();
    }
    return;
  }
  // Owning the slot.
  if (millis() - this->slot_deadline_ > this->cycle_timeout_ms_) {
    ESP_LOGW(TAG, "Cycle timeout for %s, handing the radio to the next battery", this->parent_->address_str());
    this->release_slot();
    return;
  }
  if (!this->notify_registered_) {
    this->try_connect_();
    return;
  }
  if (this->slot_cycle_pending_ && this->phase_ == CyclePhase::IDLE) {
    this->slot_cycle_pending_ = false;
    if (!this->identify_done_)
      this->start_identify_cycle();
    else
      this->start_status_cycle();
  }
}

void ZxhBMS::start_identify_cycle() {
  this->queue_.clear();
  if (!this->address_discovered_)
    this->queue_.push_back({CommandKind::DISCOVER_ADDRESS, FUNC_READ_PARAM, 4008, 1, 0});
  this->queue_.push_back({CommandKind::LABEL, FUNC_READ_PARAM, 4000, 5, 0});
  this->queue_.push_back({CommandKind::MANUFACTURER, FUNC_READ_PARAM, 4009, 3, 0});
  this->phase_ = CyclePhase::IDENTIFY;
  this->cmd_pos_ = 0;
  this->attempts_ = 0;
  this->transmit(this->queue_[0]);
}

void ZxhBMS::start_status_cycle() {
  this->queue_.clear();
  this->queue_.push_back({CommandKind::INSTRUMENT, FUNC_READ_STATUS, 3000, 5, 0});
  if (this->cell_count_ > 0) {
    uint16_t done = 0, page = 0;
    while (done < this->cell_count_) {
      uint8_t cnt = (uint8_t) std::min<uint16_t>(7, this->cell_count_ - done);
      this->queue_.push_back({CommandKind::CELLS, FUNC_READ_STATUS, (uint16_t)(3012 + 14 * page), cnt, done});
      done += cnt;
      page++;
    }
  }
  this->queue_.push_back({CommandKind::BASIC, FUNC_READ_STATUS, 3076, 7, 0});
  this->queue_.push_back({CommandKind::TEMPS, FUNC_READ_STATUS, 3087, 4, 0});
  this->phase_ = CyclePhase::STATUS;
  this->cmd_pos_ = 0;
  this->attempts_ = 0;
  this->transmit(this->queue_[0]);
}

void ZxhBMS::transmit(const Command &cmd) {
  if (this->write_handle_ == 0) {
    this->abort_cycle("no write characteristic");
    return;
  }
  uint8_t frame[7];
  frame[0] = cmd.kind == CommandKind::DISCOVER_ADDRESS ? 0 : this->address_;
  frame[1] = cmd.function;
  frame[2] = (uint8_t)(cmd.reg >> 8);
  frame[3] = (uint8_t)(cmd.reg & 0xFF);
  frame[4] = cmd.count;
  uint16_t crc = crc16_xmodem(frame, 5);
  frame[5] = (uint8_t)(crc >> 8);
  frame[6] = (uint8_t)(crc & 0xFF);

  this->rx_.clear();
  auto err = esp_ble_gattc_write_char((esp_gatt_if_t) this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                      this->write_handle_, 7, frame,
                                      this->write_with_response_ ? ESP_GATT_WRITE_TYPE_RSP
                                                                 : ESP_GATT_WRITE_TYPE_NO_RSP,
                                      ESP_GATT_AUTH_REQ_NONE);
  this->last_tx_ = millis();
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "esp_ble_gattc_write_char failed: %d", err);
    this->write_pending_ = false;
  } else {
    this->write_pending_ = true;
  }
}

void ZxhBMS::abort_cycle(const char *reason) {
  const Command &cmd = this->queue_[this->cmd_pos_];
  ESP_LOGW(TAG, "Poll aborted at command kind=%d reg=%u: %s", (int) cmd.kind, cmd.reg, reason);
  this->queue_.clear();
  this->phase_ = CyclePhase::IDLE;
  if (this->sequential_)
    this->release_slot();
}

void ZxhBMS::advance_cycle() {
  this->cmd_pos_++;
  if (this->cmd_pos_ < this->queue_.size()) {
    this->attempts_ = 0;
    this->transmit(this->queue_[this->cmd_pos_]);
    return;
  }
  this->queue_.clear();
  if (this->phase_ == CyclePhase::IDENTIFY) {
    this->identify_done_ = true;
    this->phase_ = CyclePhase::IDLE;
    this->start_status_cycle();
  } else {
    this->phase_ = CyclePhase::IDLE;
    this->publish_summary();
    if (this->sequential_)
      this->release_slot();  // done reading; pass the radio to the next battery
  }
}

void ZxhBMS::loop() {
  if (this->sequential_)
    this->mux_loop();
  if (!this->notify_registered_ || this->phase_ == CyclePhase::IDLE || this->queue_.empty())
    return;
  if (!this->parent_->connected())
    return;  // link lost; disconnect event resets, ble_client reconnects

  if (this->write_pending_) {
    if (millis() - this->last_tx_ < WRITE_PENDING_TIMEOUT_MS)
      return;
    this->write_pending_ = false;  // write-complete event lost
  }

  const Command &cmd = this->queue_[this->cmd_pos_];
  uint8_t want_addr = cmd.kind == CommandKind::DISCOVER_ADDRESS ? 0 : this->address_;
  std::vector<uint8_t> data;
  if (extract_response(this->rx_, want_addr, cmd.function, data)) {
    this->handle_response(cmd, data);
    this->advance_cycle();
    return;
  }
  if (millis() - this->last_tx_ >= ATTEMPT_TIMEOUT_MS) {
    this->attempts_++;
    if (this->attempts_ >= MAX_CMD_ATTEMPTS) {
      this->abort_cycle("no valid response after retries");
      return;
    }
    ESP_LOGD(TAG, "Timeout on reg=%u, retry %u", cmd.reg, this->attempts_);
    this->transmit(cmd);
  }
}

void ZxhBMS::handle_response(const Command &cmd, const std::vector<uint8_t> &data) {
  const uint8_t *d = data.data();
  const size_t len = data.size();
  switch (cmd.kind) {
    case CommandKind::DISCOVER_ADDRESS: {
      if (len < 1) {
        ESP_LOGW(TAG, "Address discovery response too short (%u)", (unsigned) len);
        break;
      }
      this->address_ = d[0];
      this->address_discovered_ = true;
      if (this->address_ == 0)
        ESP_LOGW(TAG, "Address discovery returned 0, commands may not be answered");
      else
        ESP_LOGI(TAG, "Discovered Modbus address %u", this->address_);
      break;
    }
    case CommandKind::LABEL: {
      if (len < 8) {
        ESP_LOGW(TAG, "Label response too short (%u)", (unsigned) len);
        break;
      }
      uint8_t cells = d[0];
      if (cells == 0 || cells > MAX_CELLS) {
        ESP_LOGW(TAG, "Implausible cell count %u, cell voltages disabled", cells);
        cells = 0;
      }
      this->publish_label(cells, d[1], be16(d + 2) / 10.0f, be16(d + 4) / 10.0f, be16(d + 6) / 10.0f);
      break;
    }
    case CommandKind::INSTRUMENT: {
      if (len < 12) {
        ESP_LOGW(TAG, "Instrument response too short (%u)", (unsigned) len);
        break;
      }
      // 3-byte signed big-endian current; sign threshold mirrors the vendor app (data[0] > 124).
      int32_t raw = (int32_t)((d[0] << 16) | (d[1] << 8) | d[2]);
      if (d[0] > 124)
        raw -= 0x1000000;
      this->current_a_ = raw / 1000.0f;
      this->soc_ = d[3];
      this->mos_temp_c_ = decode_temp(be16(d + 4));
      this->equilibrium_ = (uint32_t(d[6]) << 24) | (uint32_t(d[7]) << 16) | (uint32_t(d[8]) << 8) | d[9];
      this->protection_ = be16(d + 10);
      this->publish_instrument(this->current_a_, this->soc_, this->mos_temp_c_, this->equilibrium_,
                               this->protection_);
      break;
    }
    case CommandKind::CELLS: {
      if (len < (size_t)(2 * cmd.count)) {
        ESP_LOGW(TAG, "Cells response too short (%u)", (unsigned) len);
        break;
      }
      size_t need = cmd.cell_offset + cmd.count;
      if (this->cells_.size() < need)
        this->cells_.resize(need, 0);
      for (uint8_t i = 0; i < cmd.count; i++)
        this->cells_[cmd.cell_offset + i] = be16(d + 2 * i);
      break;
    }
    case CommandKind::BASIC: {
      if (len < 11) {
        ESP_LOGW(TAG, "Basic info response too short (%u)", (unsigned) len);
        break;
      }
      this->publish_basic(be16(d) / 10.0f, be16(d + 2), d[4]);
      break;
    }
    case CommandKind::TEMPS: {
      if (len < 8) {
        ESP_LOGW(TAG, "Temperature response too short (%u)", (unsigned) len);
        break;
      }
      float temps[TEMP_PROBE_COUNT];
      for (int i = 0; i < TEMP_PROBE_COUNT; i++)
        temps[i] = decode_temp(be16(d + 2 * i));
      this->publish_temps(temps);
      break;
    }
    case CommandKind::MANUFACTURER: {
      if (len < 4) {
        ESP_LOGW(TAG, "Manufacturer response too short (%u)", (unsigned) len);
        break;
      }
      char dbuf[16];
      snprintf(dbuf, sizeof(dbuf), "20%02u-%02u-%02u", bcd_byte(d[1]), bcd_byte(d[2]), bcd_byte(d[3]));
      // data[4:14] is fixed-width ASCII; strip padding, keep inner spaces.
      std::string fw;
      for (size_t i = 4; i < len && i < 14; i++)
        fw += (char) d[i];
      while (!fw.empty() && (fw.back() == '\0' || fw.back() == ' '))
        fw.pop_back();
      size_t start = fw.find_first_not_of(" \0");
      fw = (start == std::string::npos) ? std::string() : fw.substr(start);
      this->publish_manufacturer(dbuf, fw);
      break;
    }
  }
}

void ZxhBMS::publish_label(uint8_t cell_count, uint8_t temp_probes, float nominal_v, float nominal_ah,
                           float full_ah) {
  if (cell_count != this->cell_count_)
    this->cells_.assign(cell_count, 0);
  this->cell_count_ = cell_count;
  if (this->cell_count_sensor_ != nullptr)
    this->cell_count_sensor_->publish_state(cell_count);
  if (this->nominal_voltage_sensor_ != nullptr)
    this->nominal_voltage_sensor_->publish_state(nominal_v);
  if (this->nominal_capacity_sensor_ != nullptr)
    this->nominal_capacity_sensor_->publish_state(nominal_ah);
  if (this->full_capacity_sensor_ != nullptr)
    this->full_capacity_sensor_->publish_state(full_ah);
  ESP_LOGI(TAG, "Pack: %u cells, %u temp probes, %.1fV %.1fAh nominal", cell_count, temp_probes, nominal_v,
           nominal_ah);
}

void ZxhBMS::publish_instrument(float current_a, uint8_t soc, float mos_temp, uint32_t equilibrium,
                                uint16_t protection) {
  if (this->current_sensor_ != nullptr)
    this->current_sensor_->publish_state(current_a);
  if (this->soc_sensor_ != nullptr)
    this->soc_sensor_->publish_state(soc);
  if (this->mos_temperature_sensor_ != nullptr)
    this->mos_temperature_sensor_->publish_state(mos_temp);
  if (this->balancing_binary_sensor_ != nullptr)
    this->balancing_binary_sensor_->publish_state(equilibrium != 0);
  if (this->protection_binary_sensor_ != nullptr)
    this->protection_binary_sensor_->publish_state(protection != 0);
  if (this->protection_text_sensor_ != nullptr) {
    std::string text;
    for (int bit = 0; bit < 16; bit++) {
      if (PROTECTION_BITS[bit] != nullptr && (protection & (1 << bit))) {
        if (!text.empty())
          text += ", ";
        text += PROTECTION_BITS[bit];
      }
    }
    if (text.empty())
      text = "None";
    this->protection_text_sensor_->publish_state(text);
  }
}

void ZxhBMS::publish_basic(float capacity_ah, uint16_t cycles, uint8_t health) {
  if (this->remaining_capacity_sensor_ != nullptr)
    this->remaining_capacity_sensor_->publish_state(capacity_ah);
  if (this->cycles_sensor_ != nullptr)
    this->cycles_sensor_->publish_state(cycles);
  if (this->health_sensor_ != nullptr)
    this->health_sensor_->publish_state(health);
}

void ZxhBMS::publish_temps(const float *temps) {
  for (int i = 0; i < TEMP_PROBE_COUNT; i++) {
    this->temps_[i] = temps[i];
    if (this->temperature_sensors_[i] != nullptr)
      this->temperature_sensors_[i]->publish_state(temps[i]);
  }
}

void ZxhBMS::publish_manufacturer(const std::string &date, const std::string &firmware) {
  if (this->date_text_sensor_ != nullptr)
    this->date_text_sensor_->publish_state(date);
  if (this->firmware_text_sensor_ != nullptr && !firmware.empty())
    this->firmware_text_sensor_->publish_state(firmware);
}

void ZxhBMS::publish_summary() {
  if (this->cells_.empty() || this->cells_.size() < this->cell_count_)
    return;
  uint32_t sum = 0;
  uint16_t lo = 0xFFFF, hi = 0;
  char json[8 + MAX_CELLS * 6];
  int pos = 0;
  json[pos++] = '[';
  for (size_t i = 0; i < this->cells_.size(); i++) {
    uint16_t mv = this->cells_[i];
    sum += mv;
    lo = std::min(lo, mv);
    hi = std::max(hi, mv);
    pos += snprintf(json + pos, sizeof(json) - pos, i == 0 ? "%u" : ",%u", (unsigned) mv);
  }
  json[pos++] = ']';
  json[pos] = '\0';

  float voltage = sum / 1000.0f;
  if (this->voltage_sensor_ != nullptr)
    this->voltage_sensor_->publish_state(voltage);
  if (this->power_sensor_ != nullptr && !std::isnan(this->current_a_))
    this->power_sensor_->publish_state(voltage * this->current_a_);
  if (this->cell_min_sensor_ != nullptr)
    this->cell_min_sensor_->publish_state(lo);
  if (this->cell_max_sensor_ != nullptr)
    this->cell_max_sensor_->publish_state(hi);
  if (this->cell_delta_sensor_ != nullptr)
    this->cell_delta_sensor_->publish_state(hi - lo);
  if (this->cells_text_sensor_ != nullptr)
    this->cells_text_sensor_->publish_state(json);
}

}  // namespace esphome::zxh_bms

#endif  // USE_ESP32
