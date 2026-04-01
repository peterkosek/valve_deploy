#include "globals.h"

// -------------------------
// Global objects / handles
// -------------------------
SemaphoreHandle_t g_uiSem = nullptr;

TaskHandle_t g_work_task = nullptr;
TimerHandle_t g_settle_timer = nullptr;
TimerHandle_t g_busy_timer = nullptr;

std::atomic<bool> g_work_pending{false};
portMUX_TYPE g_work_mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool g_work_busy = false;
volatile WorkPhase g_work_phase = WORK_IDLE;
volatile bool g_deferred_settle_start = false;

Preferences prefs;
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
TwoWire *I2C = &Wire;
Adafruit_MCP3421 adc;

// -------------------------
// Global state
// -------------------------
uint32_t g_last_tx_ms = 0;
bool i2c_ok = true;
uint8_t i2c_fail_streak = 0;
uint32_t i2c_quiet_until_ms = 0;
bool i2c_ready = 0;
bool join_inflight = false;
uint32_t join_retry_at_ms = 0;

bool g_skip_next_decrement = false;
volatile bool g_need_display = false;
volatile bool g_need_vlv_update = false;
volatile uint32_t g_awake_until_ms = 0;

char buffer[64];
bool valve_open_fwd[2] = { true, true };
BoardIdentity boardId{};

volatile bool g_need_delayed_send = false;
uint32_t g_pending_send_at_ms = 0;
uint32_t g_hold_awake_until_ms = 0;
volatile bool g_send_after_settle = false;
volatile uint32_t g_send_due_ms = 0;
uint16_t pressResult = 0;

// -------------------------
// LoRaWAN globals
// -------------------------
RTC_DATA_ATTR uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x53, 0xff };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
RTC_DATA_ATTR uint8_t appKey[] = { 0x74, 0xD6, 0x6E, 0x63, 0x45, 0x82, 0x48, 0x27, 0xFE, 0xC5, 0xB7, 0x70, 0xBA, 0x2B, 0x50, 0x55 };

uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67 };
uint32_t devAddr = (uint32_t)0x007e6ae1;

uint16_t userChannelsMask[6] = { 0xff00, 0x0000, 0x0000, 0x0001, 0x0000, 0x0000 };
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t loraWanClass = CLASS_A;

#if defined(DEBUG_TIMING)
RTC_DATA_ATTR uint32_t appTxDutyCycle = 60 * 60 * 1 * 1000;
#else
RTC_DATA_ATTR uint32_t appTxDutyCycle = 60 * 60 * 3 * 1000;
#endif

RTC_DATA_ATTR volatile uint32_t TxDutyCycle_hold = 3600000;
RTC_DATA_ATTR volatile int8_t initialCycleFast;
RTC_DATA_ATTR volatile uint32_t g_sched_override_ms = 0;

bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = false;
uint8_t confirmedNbTrials = 4;

// -------------------------
// RTC-backed config/state
// -------------------------
RTC_DATA_ATTR volatile uint32_t g_status_uplink_at = 0;

RTC_DATA_ATTR volatile ValveCmd_t g_cmd;
RTC_DATA_ATTR volatile ValveCmd_t g_pending_cmd = { 0, 0, 0 };

RTC_DATA_ATTR uint32_t inv_m_u32 = 346;
RTC_DATA_ATTR int32_t b_x10 = -117;
RTC_DATA_ATTR uint16_t g_lake_depth_mm = 0;
RTC_DATA_ATTR uint16_t g_lake_sensor_mm = 0;
RTC_DATA_ATTR int16_t g_lake_level_mm = 0;
RTC_DATA_ATTR char g_name[12] = { 0 };
RTC_DATA_ATTR uint32_t g_coldboot_valve_init_done = 0;
RTC_DATA_ATTR uint16_t g_v0_act_ms = 200;
RTC_DATA_ATTR uint16_t g_v1_act_ms = 200;
RTC_DATA_ATTR bool g_v0_open_fwd = true;
RTC_DATA_ATTR bool g_v1_open_fwd = true;
RTC_DATA_ATTR uint8_t g_device_role = 0;
RTC_DATA_ATTR uint8_t g_uplink_fport = 0;
RTC_DATA_ATTR uint8_t g_lake_type = 0;
RTC_DATA_ATTR uint8_t g_schema_ver = 0;
RTC_DATA_ATTR uint8_t appPort;

const char* const ulp_var_names[] = {
  "ULP_RSSI",
  "ULP_SNR",
  "ULP_BAT_PCT",
  "ULP_LAST_SENT",
  "ULP_COUNT_LO",
  "ULP_COUNT_HI",
  "ULP_PREV_STATE",
  "ULP_VALVE_A",
  "ULP_VALVE_B",
  "ULP_DEBUG_PIN_STATE",
  "ULP_TICK_POP",
  "ULP_TS_DELTA_LO",
  "ULP_TS_DELTA_HI",
  "ULP_TS_DELTA_TICK_POP",
  "ULP_TIMER_LO",
  "ULP_TIMER_HI",
  "ULP_REED_DELTA",
  "ULP_FLOW_RATE",
  "ULP_VOLUME_DELTA",
  "ULP_WAKE_THRESHOLD",
  "ULP_TXCYCLETIME",
  "ULP_TXCYCLEFAST",
  "ULP_COUNT_PENDING",
  "ULP_COUNT"
};