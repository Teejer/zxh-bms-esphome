#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/ble_client/ble_client.h"

#ifdef USE_ESP32

#include <cstdint>
#include <string>
#include <vector>

namespace esphome::zxh_bms {

/// Modbus-like function codes used by this BMS family.
static constexpr uint8_t FUNC_READ_PARAM = 0x03;
static constexpr uint8_t FUNC_READ_STATUS = 0x04;

static constexpr uint16_t MAX_CELLS = 32;
static constexpr uint8_t TEMP_PROBE_COUNT = 4;
static constexpr uint8_t MAX_CMD_ATTEMPTS = 3;
static constexpr uint32_t ATTEMPT_TIMEOUT_MS = 1500;
static constexpr uint32_t WRITE_PENDING_TIMEOUT_MS = 500;
static constexpr uint32_t CONNECT_RETRY_MS = 2000;
static constexpr size_t RX_BUFFER_LIMIT = 512;

enum class CommandKind : uint8_t {
  DISCOVER_ADDRESS,
  LABEL,
  INSTRUMENT,
  CELLS,
  BASIC,
  TEMPS,
  MANUFACTURER,
};

struct Command {
  CommandKind kind;
  uint8_t function;
  uint16_t reg;
  uint8_t count;
  uint16_t cell_offset;  // CELLS only
};

/// Read-only BLE client for the zxh/uni-app "smart BMS" boards, speaking the
/// same request/response protocol as the lifepo4-cli Python tool.
class ZxhBMS : public PollingComponent, public ble_client::BLEClientNode {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

  void set_modbus_address(uint8_t address) { this->configured_address_ = address; }
  /// Sequential mode: share the radio with other hubs, one link at a time.
  void set_sequential(bool sequential) { this->sequential_ = sequential; }
  void set_cycle_timeout(uint32_t ms) { this->cycle_timeout_ms_ = ms; }

  void set_voltage_sensor(sensor::Sensor *s) { this->voltage_sensor_ = s; }
  void set_current_sensor(sensor::Sensor *s) { this->current_sensor_ = s; }
  void set_power_sensor(sensor::Sensor *s) { this->power_sensor_ = s; }
  void set_soc_sensor(sensor::Sensor *s) { this->soc_sensor_ = s; }
  void set_cell_min_sensor(sensor::Sensor *s) { this->cell_min_sensor_ = s; }
  void set_cell_max_sensor(sensor::Sensor *s) { this->cell_max_sensor_ = s; }
  void set_cell_delta_sensor(sensor::Sensor *s) { this->cell_delta_sensor_ = s; }
  void set_cell_count_sensor(sensor::Sensor *s) { this->cell_count_sensor_ = s; }
  void set_nominal_voltage_sensor(sensor::Sensor *s) { this->nominal_voltage_sensor_ = s; }
  void set_nominal_capacity_sensor(sensor::Sensor *s) { this->nominal_capacity_sensor_ = s; }
  void set_full_capacity_sensor(sensor::Sensor *s) { this->full_capacity_sensor_ = s; }
  void set_remaining_capacity_sensor(sensor::Sensor *s) { this->remaining_capacity_sensor_ = s; }
  void set_cycles_sensor(sensor::Sensor *s) { this->cycles_sensor_ = s; }
  void set_health_sensor(sensor::Sensor *s) { this->health_sensor_ = s; }
  void set_mos_temperature_sensor(sensor::Sensor *s) { this->mos_temperature_sensor_ = s; }
  void set_temperature_sensor(uint8_t probe, sensor::Sensor *s) {
    if (probe >= 1 && probe <= TEMP_PROBE_COUNT)
      this->temperature_sensors_[probe - 1] = s;
  }

  void set_protection_binary_sensor(binary_sensor::BinarySensor *s) { this->protection_binary_sensor_ = s; }
  void set_balancing_binary_sensor(binary_sensor::BinarySensor *s) { this->balancing_binary_sensor_ = s; }

  void set_cells_text_sensor(text_sensor::TextSensor *s) { this->cells_text_sensor_ = s; }
  void set_protection_text_sensor(text_sensor::TextSensor *s) { this->protection_text_sensor_ = s; }
  void set_firmware_version_text_sensor(text_sensor::TextSensor *s) { this->firmware_text_sensor_ = s; }
  void set_manufacture_date_text_sensor(text_sensor::TextSensor *s) { this->date_text_sensor_ = s; }
  void set_device_name_text_sensor(text_sensor::TextSensor *s) { this->device_name_text_sensor_ = s; }

 protected:
  enum class CyclePhase : uint8_t {
    IDLE,
    IDENTIFY,
    STATUS,
  };

  // GATT / connection state
  bool profile_found_{false};
  bool notify_registered_{false};
  bool write_pending_{false};
  uint16_t notify_handle_{0};
  uint16_t write_handle_{0};
  bool write_with_response_{true};
  uint16_t device_name_handle_{0};
  std::vector<uint8_t> rx_;

  // Protocol state
  uint8_t configured_address_{0};
  uint8_t address_{0};
  bool address_discovered_{false};
  bool identify_done_{false};

  // Sequential (multiplexed) mode state. The statics serialize cycles across
  // all sequential hubs: exactly one owns the radio slot at a time.
  bool sequential_{false};
  uint32_t cycle_timeout_ms_{90000};
  bool mux_queued_{false};
  bool slot_cycle_pending_{false};
  uint32_t slot_deadline_{0};
  uint32_t next_connect_try_{0};
  static ZxhBMS *mux_owner_;
  static std::vector<ZxhBMS *> mux_waiting_;

  CyclePhase phase_{CyclePhase::IDLE};
  std::vector<Command> queue_;
  size_t cmd_pos_{0};
  uint8_t attempts_{0};
  uint32_t last_tx_{0};

  // Decoded values
  uint8_t cell_count_{0};
  std::vector<uint16_t> cells_;
  float current_a_{NAN};
  uint8_t soc_{0};
  uint32_t equilibrium_{0};
  uint16_t protection_{0};
  float temps_[TEMP_PROBE_COUNT]{NAN, NAN, NAN, NAN};
  float mos_temp_c_{NAN};

  // Sensors
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *soc_sensor_{nullptr};
  sensor::Sensor *cell_min_sensor_{nullptr};
  sensor::Sensor *cell_max_sensor_{nullptr};
  sensor::Sensor *cell_delta_sensor_{nullptr};
  sensor::Sensor *cell_count_sensor_{nullptr};
  sensor::Sensor *nominal_voltage_sensor_{nullptr};
  sensor::Sensor *nominal_capacity_sensor_{nullptr};
  sensor::Sensor *full_capacity_sensor_{nullptr};
  sensor::Sensor *remaining_capacity_sensor_{nullptr};
  sensor::Sensor *cycles_sensor_{nullptr};
  sensor::Sensor *health_sensor_{nullptr};
  sensor::Sensor *mos_temperature_sensor_{nullptr};
  sensor::Sensor *temperature_sensors_[TEMP_PROBE_COUNT]{};

  binary_sensor::BinarySensor *protection_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *balancing_binary_sensor_{nullptr};

  text_sensor::TextSensor *cells_text_sensor_{nullptr};
  text_sensor::TextSensor *protection_text_sensor_{nullptr};
  text_sensor::TextSensor *firmware_text_sensor_{nullptr};
  text_sensor::TextSensor *date_text_sensor_{nullptr};
  text_sensor::TextSensor *device_name_text_sensor_{nullptr};

  void reset_connection_state();
  void mux_loop();
  void try_connect_();
  void release_slot();
  void discover_profile();
  void read_device_name();
  void publish_label(uint8_t cell_count, uint8_t temp_probes, float nominal_v, float nominal_ah, float full_ah);
  void publish_instrument(float current_a, uint8_t soc, float mos_temp, uint32_t equilibrium, uint16_t protection);
  void publish_basic(float capacity_ah, uint16_t cycles, uint8_t health);
  void publish_temps(const float *temps);
  void publish_manufacturer(const std::string &date, const std::string &firmware);
  void publish_summary();
  void start_identify_cycle();
  void start_status_cycle();
  void transmit(const Command &cmd);
  void advance_cycle();
  void abort_cycle(const char *reason);
  void handle_response(const Command &cmd, const std::vector<uint8_t> &data);
};

}  // namespace esphome::zxh_bms

#endif  // USE_ESP32
