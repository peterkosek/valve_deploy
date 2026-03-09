#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <atomic>

#include "LoRaWan_APP.h"
#include "Wire.h"
#include "Adafruit_SHT4x.h"
#include "sensor_solenoid.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// -------------------------
// Shared config-key strings
// -------------------------
namespace cfg {
  inline constexpr const char *NS_LORAWAN = "lorawan";
  inline constexpr const char *K_DEVEUI   = "devEui";
  inline constexpr const char *K_APPKEY   = "appKey";
  inline constexpr const char *K_PROV     = "provisioned";

  inline constexpr const char *NS         = "cfg";
  inline constexpr const char *K_LAKE_MM  = "lake_depth_mm";
  inline constexpr const char *K_WAKE_TH  = "wake_th";
  inline constexpr const char *K_NAME     = "screenMsg";
  inline constexpr const char *K_INV_M    = "inv_m_u32";
  inline constexpr const char *K_BX10     = "b_x10";
  inline constexpr const char *K_SCHEMA   = "schema_ver";
  inline constexpr const char *K_ROLE     = "device_role";
  inline constexpr const char *K_FPORT    = "uplink_fport";
  inline constexpr const char *K_LAKE_TYP = "lake_type";

  inline constexpr const char *K_V0_ACT   = "v0_act_ms";
  inline constexpr const char *K_V1_ACT   = "v1_act_ms";
  inline constexpr const char *K_V0_FWD   = "v0_open_fwd";
  inline constexpr const char *K_V1_FWD   = "v1_open_fwd";
}

// -------------------------
// Shared constants / enums
// -------------------------
inline constexpr uint8_t CFG_SCHEMA_VERSION = 1;

inline constexpr uint8_t ROLE_LAKE     = 1;
inline constexpr uint8_t ROLE_VALVE    = 2;
inline constexpr uint8_t ROLE_METER    = 3;
inline constexpr uint8_t ROLE_TEST     = 4;
inline constexpr uint8_t ROLE_SOIL_AIR = 5;
inline constexpr uint8_t ROLE_SOIL     = 6;

inline constexpr uint32_t EVT_WORK_START     = (1u << 0);
inline constexpr uint32_t EVT_SETTLE_EXPIRED = (1u << 1);
inline constexpr uint32_t EVT_BUSY_EXPIRED   = (1u << 2);
inline constexpr uint32_t EVT_UI_REQUEST     = (1u << 3);

inline constexpr uint32_t STATUS_UPLINK_DELAY_MS = 3000;
inline constexpr uint32_t TX_CYCLE_FAST_TIME     = 60000ul;

enum WorkPhase : uint8_t {
  WORK_IDLE = 0,
  WORK_DO_UI,
  WORK_WAIT_UI_GUARD,
  WORK_DO_SEND,
  WORK_WAIT_SEND_GUARD,
};

struct BoardIdentity {
  bool provisioned;
  bool valve_open_fwd[2];
};

// -------------------------
// Global objects / handles
// -------------------------
extern SemaphoreHandle_t g_uiSem;

extern TaskHandle_t g_work_task;
extern TimerHandle_t g_settle_timer;
extern TimerHandle_t g_busy_timer;

extern std::atomic<bool> g_work_pending;
extern portMUX_TYPE g_work_mux;
extern volatile bool g_work_busy;
extern volatile WorkPhase g_work_phase;
extern volatile bool g_deferred_settle_start;

extern Preferences prefs;
extern Adafruit_SHT4x sht4;
extern TwoWire *I2C;
extern Adafruit_MCP3421 adc;

// -------------------------
// Global state
// -------------------------
extern uint32_t g_last_tx_ms;
extern bool i2c_ok;
extern uint8_t i2c_fail_streak;
extern uint32_t i2c_quiet_until_ms;
extern bool i2c_ready;
extern bool join_inflight;
extern uint32_t join_retry_at_ms;

extern bool g_skip_next_decrement;
extern volatile bool g_need_display;
extern volatile bool g_need_vlv_update;
extern volatile uint32_t g_awake_until_ms;

extern char buffer[64];
extern bool valve_open_fwd[2];
extern BoardIdentity boardId;

extern volatile bool g_need_delayed_send;
extern uint32_t g_pending_send_at_ms;
extern uint32_t g_hold_awake_until_ms;
extern volatile bool g_send_after_settle;
extern volatile uint32_t g_send_due_ms;
extern uint16_t pressResult;

// -------------------------
// LoRaWAN globals
// -------------------------
extern RTC_DATA_ATTR uint8_t devEui[8];
extern uint8_t appEui[8];
extern RTC_DATA_ATTR uint8_t appKey[16];

extern uint8_t nwkSKey[16];
extern uint8_t appSKey[16];
extern uint32_t devAddr;

extern uint16_t userChannelsMask[6];
extern LoRaMacRegion_t loraWanRegion;
extern DeviceClass_t loraWanClass;

extern RTC_DATA_ATTR uint32_t appTxDutyCycle;
extern RTC_DATA_ATTR volatile uint32_t TxDutyCycle_hold;
extern RTC_DATA_ATTR volatile int8_t initialCycleFast;
extern RTC_DATA_ATTR volatile uint32_t g_sched_override_ms;

extern bool overTheAirActivation;
extern bool loraWanAdr;
extern bool isTxConfirmed;
extern uint8_t confirmedNbTrials;

// -------------------------
// RTC-backed config/state
// -------------------------
extern RTC_DATA_ATTR volatile uint32_t g_status_uplink_at;

extern RTC_DATA_ATTR volatile ValveCmd_t g_cmd;
extern RTC_DATA_ATTR volatile ValveCmd_t g_pending_cmd;

extern RTC_DATA_ATTR uint32_t inv_m_u32;
extern RTC_DATA_ATTR int32_t b_x10;
extern RTC_DATA_ATTR uint16_t g_lake_depth_mm;
extern RTC_DATA_ATTR uint16_t g_lake_sensor_mm;
extern RTC_DATA_ATTR int16_t g_lake_level_mm;
extern RTC_DATA_ATTR char g_name[12];
extern RTC_DATA_ATTR uint32_t g_coldboot_valve_init_done;
extern RTC_DATA_ATTR uint16_t g_v0_act_ms;
extern RTC_DATA_ATTR uint16_t g_v1_act_ms;
extern RTC_DATA_ATTR bool g_v0_open_fwd;
extern RTC_DATA_ATTR bool g_v1_open_fwd;
extern RTC_DATA_ATTR uint8_t g_device_role;
extern RTC_DATA_ATTR uint8_t g_uplink_fport;
extern RTC_DATA_ATTR uint8_t g_lake_type;
#define LAKE_SENSOR_OFFLINE 0xEEEE
extern RTC_DATA_ATTR uint8_t g_schema_ver;
extern RTC_DATA_ATTR uint8_t appPort;

// -------------------------
// External globals from library
// -------------------------
extern int revrssi;
extern int revsnr;