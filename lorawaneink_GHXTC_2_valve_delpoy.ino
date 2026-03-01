// Heltec Automation LoRaWAN communication example, modified

//        TYPE OF NODE HERE, defines before includes so that they can modify pre-complie directives in the includes

//#define DEBUG_TIMING true  //  rapid cycling for debugging, not deployment
//#define CALIBRATION_MODE true  //  FOR CALIBRATING THE SENSORS, NO LORAWAN, SERIAL OUTPUT, trips valves on and off

//define REED_NODE       true      //  count reed closures of one switch for water flow meter
#define VALVE_NODE true  //  two valve controlller and possible line pressure
//#define SOIL_SENSOR_NODE true     //  two soil temp moist and possible pH
//#define LAKE_NODE  true           //  one 16 bit number reflecting lake depth, calibration constants in device table

#include <Arduino.h>
#include "LoRaWan_APP.h"
//#include "LoRaMacCommands.h"
#include "Wire.h"
#include "GXHTC.h"
#include "HT_DEPG0290BxS800FxX_BW.h"
#include "sensor_solenoid.h"
#include "esp_sleep.h"
#include "esp32s3/ulp.h"  //  this also includes ulp_common.h
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include <Preferences.h>
#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"  // for RTC_CNTL_RTC_UPDATE_REG
#include "soc/soc.h"
#include "soc/sens_struct.h"
#include "soc/sens_reg.h"
#include <Arduino.h>
#include "Adafruit_SHT4x.h"
#include "esp_heap_caps.h"
#include "valve_logic.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"


// 2) A handy macro to get a pointer to any struct at a given word-index:
// makes rtc vars volatile
#define RTC_SLOW_BYTE_MEM ((uint8_t *)SOC_RTC_DATA_LOW)
#define RTC_SLOW_MEMORY ((volatile uint32_t *)SOC_RTC_DATA_LOW)
#define RTC_SLOW_STRUCT_PTR(type, idx) \
  ((volatile type *)(RTC_SLOW_BYTE_MEM + (idx) * sizeof(uint32_t)))

//  for prefs namespace access and UI semaphore
static const char *NS_LORAWAN = "lorawan";
static const char *K_DEVEUI = "devEui";  // 8 bytes
static const char *K_APPKEY = "appKey";  // 16 bytes
static const char *K_PROV = "provisioned";
constexpr const char *NS = "cfg";
constexpr const char *K_LAKE_MM = "lake_depth_mm";
constexpr const char *K_WAKE_TH = "wake_th";
constexpr const char *K_NAME = "screenMsg";
constexpr const char *K_INV_M = "inv_m_u32";
constexpr const char *K_BX10 = "b_x10";
constexpr const char *NVS_NS = "lorawan";  //  for nvs storage of devEui and appKey
constexpr const char *K_SCHEMA = "schema_ver";
constexpr const char *K_ROLE = "device_role";
constexpr const char *K_FPORT = "uplink_fport";
constexpr const char *K_LAKE_TYP = "lake_type";

// Valve keys stored in cfg (per your decision B)
constexpr const char *K_V0_ACT = "v0_act_ms";
constexpr const char *K_V1_ACT = "v1_act_ms";
constexpr const char *K_V0_FWD = "v0_open_fwd";
constexpr const char *K_V1_FWD = "v1_open_fwd";

// ---- schema control ----
constexpr uint8_t CFG_SCHEMA_VERSION = 1;

// ---- device roles (must match provisioning values) ----
constexpr uint8_t ROLE_LAKE = 1;
constexpr uint8_t ROLE_VALVE = 2;
constexpr uint8_t ROLE_METER = 3;
constexpr uint8_t ROLE_TEST = 4;
constexpr uint8_t ROLE_SOIL_AIR = 5;
constexpr uint8_t ROLE_SOIL = 6;

//  semaphjore for the screen
SemaphoreHandle_t g_uiSem;

volatile ValveState_t *valveState = RTC_SLOW_STRUCT_PTR(ValveState_t, ULP_VALVE_A);

RTC_DATA_ATTR volatile uint32_t g_status_uplink_at = 0;
static const uint32_t STATUS_UPLINK_DELAY_MS = 3000;  // ~3 s after RX2


DEPG0290BxS800FxX_BW display(5, 4, 3, 6, 2, 1, -1, 6000000);  // rst,dc,cs,busy,sck,mosi,miso,frequency
//GXHTC gxhtc;
Preferences prefs;  // for NVM
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
TwoWire *I2C = &Wire;
Adafruit_MCP3421 adc;

static uint32_t g_last_tx_ms = 0;
static bool i2c_ok = true;
static uint8_t i2c_fail_streak = 0;
static uint32_t i2c_quiet_until_ms = 0;
static bool i2c_ready = 0;
static bool join_inflight = false;
static uint32_t join_retry_at_ms = 0;

RTC_DATA_ATTR volatile ValveCmd_t g_cmd;
bool g_skip_next_decrement = false;
// for display
volatile bool g_need_display = false;
volatile bool g_need_vlv_update = false;
volatile uint32_t g_awake_until_ms = 0;  // keep CPU on until this time
RTC_DATA_ATTR volatile ValveCmd_t g_pending_cmd = { 0, 0, 0 };

// [GPT] helper: wait for E-Ink BUSY to go idle without blocking the whole system
struct BoardIdentity {
  bool provisioned;
  bool valve_open_fwd[2];
};



char buffer[64];

/* OTAA para*/
RTC_DATA_ATTR uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x53, 0xff };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
RTC_DATA_ATTR uint8_t appKey[] = { 0x74, 0xD6, 0x6E, 0x63, 0x45, 0x82, 0x48, 0x27, 0xFE, 0xC5, 0xB7, 0x70, 0xBA, 0x2B, 0x50, 0x55 };

/* ABP para --  not used for this project*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67 };
uint32_t devAddr = (uint32_t)0x007e6ae1;

/*LoraWan channelsmask, default channels US915 band 2 hybrid*/
uint16_t userChannelsMask[6] = { 0xff00, 0x0000, 0x0000, 0x0001, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
#if defined(DEBUG_TIMING)
RTC_DATA_ATTR uint32_t appTxDutyCycle = 60 * 60 * 1 * 1000;  //  60 second sleeps
#define CYCLE_TIME_VALVE_ON 45000                            // debuf, go fast, should be 600000
#else
RTC_DATA_ATTR uint32_t appTxDutyCycle = 60 * 60 * 3 * 1000;  //min/hr * sec/min * hrs * min/ms
#define CYCLE_TIME_VALVE_ON 600000  // 10 min in ms, cycle time with valve on, should be 600000
#endif
RTC_DATA_ATTR volatile uint32_t TxDutyCycle_hold = 3600000;  //  backup TxDutyCycleHold
RTC_DATA_ATTR volatile int8_t initialCycleFast;              //  number of times to cycle fast on startup, initialized with FAST_BURST_COUNT on cold boot
#define FAST_BURST_COUNT 10                                  // cycles to conclude with rapid time for ADR on cold boot
RTC_DATA_ATTR volatile uint32_t g_sched_override_ms = 0;     //  pending cycle time changes to apply just before sleep

static const uint32_t TX_CYCLE_FAST_TIME = 60000ul;  //  set the fast cycle time that has limited cycles

RTC_DATA_ATTR uint32_t inv_m_u32 = 346;  //  valves: (1/m, b_x10):  1/8 in ch (extra sleve) 224, -103, no sleve (1/4 inch) 100, -130, or 128, -150...
RTC_DATA_ATTR int32_t b_x10 = -117;
RTC_DATA_ATTR uint32_t g_lake_depth_mm = 0;
RTC_DATA_ATTR char g_name[12] = { 0 };
RTC_DATA_ATTR uint32_t g_coldboot_valve_init_done = 0;
RTC_DATA_ATTR uint16_t g_v0_act_ms = 200;
RTC_DATA_ATTR uint16_t g_v1_act_ms = 200;
RTC_DATA_ATTR bool g_v0_open_fwd = true;  // default for legacy nodes
RTC_DATA_ATTR bool g_v1_open_fwd = true;  // default for legacy nodes
RTC_DATA_ATTR uint8_t g_device_role = 0;
RTC_DATA_ATTR uint8_t g_uplink_fport = 0;
RTC_DATA_ATTR uint8_t g_lake_type = 0;
RTC_DATA_ATTR uint8_t g_schema_ver = 0;
RTC_DATA_ATTR uint8_t appPort = 0;  // set on cold boot from cfg/uplink_fport
volatile bool g_need_delayed_send = false;
uint32_t g_pending_send_at_ms = 0;
uint32_t g_hold_awake_until_ms = 0;
// --- delayed re-uplink after valve actuation ---
volatile bool g_send_after_settle = false;
volatile uint32_t g_send_due_ms = 0;
uint16_t pressResult = 0;

//These are in RTC defined in lorawanapp.cpp
extern int revrssi;
extern int revsnr;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = false;

// * Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
// * the datarate, in case the LoRaMAC layer did not receive an acknowledgment
// */
uint8_t confirmedNbTrials = 4;

// Main cycle order (sketch):
// ValveCmd_t g_cmd; snapshot_cmd(&g_cmd);
// apply_downlink_snapshot(&g_cmd, &valveState);
// tick_timers(&valveState);              // your existing decrement/auto-off
static BoardIdentity boardId;

void schedule_delayed_send(uint32_t delay_ms) {
  g_send_after_settle = true;
  g_send_due_ms = millis() + delay_ms;
}

bool valve_open_fwd[2] = { true, true };  // defaults = safe

bool eink_wait_idle(uint32_t timeout_ms) {
  // Make sure pin is actually configured
  pinMode(EPD_BUSY_PIN, INPUT);

  const uint32_t start = millis();

  // Determine BUSY polarity at runtime:
  // After a refresh starts, BUSY should be "busy" for a while.
  // We treat "busy" as the *current* level at entry, then wait for it to change.
  const int busy_level = digitalRead(EPD_BUSY_PIN);

  while (digitalRead(EPD_BUSY_PIN) == busy_level) {
    vTaskDelay(pdMS_TO_TICKS(25));  // small yield; doesn't hog timing

    if ((uint32_t)(millis() - start) >= timeout_ms) {
      return false;  // timed out
    }
  }
  return true;
}


const char *deviceStateToString(eDeviceState_LoraWan s) {
  switch (s) {
    case DEVICE_STATE_INIT: return "DEVICE_STATE_INIT";
    case DEVICE_STATE_JOIN: return "DEVICE_STATE_JOIN";
    case DEVICE_STATE_SEND: return "DEVICE_STATE_SEND";
    case DEVICE_STATE_CYCLE: return "DEVICE_STATE_CYCLE";
    case DEVICE_STATE_SLEEP: return "DEVICE_STATE_SLEEP";
    default: return "DEVICE_STATE_UNKNOWN";
  }
}

static bool load_lorawan_identity_from_nvs() {
  Preferences p;

  if (!p.begin(NVS_NS, true)) {  // read-only
    Serial.println("[NVS] open FAILED");
    return false;
  }

  bool provisioned = p.getBool("provisioned", false);
  if (!provisioned) {
    Serial.println("[NVS] not provisioned");
    p.end();
    return false;
  }

  uint8_t tmpDevEui[8] = { 0 };
  uint8_t tmpAppKey[16] = { 0 };

  size_t n1 = p.getBytes("devEui", tmpDevEui, sizeof(tmpDevEui));
  size_t n2 = p.getBytes("appKey", tmpAppKey, sizeof(tmpAppKey));

  if (n1 != sizeof(tmpDevEui) || n2 != sizeof(tmpAppKey)) {
    Serial.printf("[NVS] bad sizes: devEui=%u appKey=%u\n",
                  (unsigned)n1, (unsigned)n2);
    p.end();
    return false;
  }

  // --- NEW: load valve polarity (board-specific physical truth) ---
  valve_open_fwd[0] = p.getBool("v0_open_fwd", true);
  valve_open_fwd[1] = p.getBool("v1_open_fwd", true);

  p.end();

  // Overwrite your existing globals (must NOT be const)
  memcpy(devEui, tmpDevEui, sizeof(tmpDevEui));
  memcpy(appKey, tmpAppKey, sizeof(tmpAppKey));

  // Sanity prints
  Serial.print("[NVS] devEui loaded: ");
  for (int i = 0; i < 8; i++) {
    Serial.printf("%02X", devEui[i]);
    if (i < 7) Serial.print(":");
  }
  Serial.println();

  Serial.printf("[NVS] valve polarity: v0_open_fwd=%u v1_open_fwd=%u\n",
                valve_open_fwd[0], valve_open_fwd[1]);

  return true;
}


static inline bool still_awake() {
  return (int32_t)(millis() - g_awake_until_ms) < 0;
}

static inline bool rx_windows_clear() {
  return (g_last_tx_ms == 0) ? 0 : ((int32_t)(millis() - (g_last_tx_ms + 2500)) >= 0);
}

inline uint32_t read_count32() {
  uint16_t hi1 = (uint16_t)RTC_SLOW_MEMORY[ULP_COUNT_HI];
  uint16_t lo = (uint16_t)RTC_SLOW_MEMORY[ULP_COUNT_LO];
  uint16_t hi2 = (uint16_t)RTC_SLOW_MEMORY[ULP_COUNT_HI];
  if (hi1 != hi2) {
    lo = (uint16_t)RTC_SLOW_MEMORY[ULP_COUNT_LO];
    hi1 = hi2;
  }
  return ((uint32_t)hi1 << 16) | lo;
}

// Returns pressure * 100 (e.g., 12.34 m -> 1234)
inline uint16_t depth_m_from_raw(int16_t raw) {
  if (inv_m_u32 == 0) return 0;  // guard
  // wide, signed math; avoid overflow on *100
  int64_t num = static_cast<int64_t>(raw);
  int64_t scale = static_cast<int64_t>(inv_m_u32);
  int64_t bx10 = static_cast<int64_t>(b_x10);

  // depth_m = raw / inv_m_u32 + b_x10/10
  // return depth_m * 100 as uint16_t with clamp 0..65535
  int64_t depth100 = ((num / scale) + (bx10 / 10)) * 100;  // (b_x10/10)*100 = b_x10*10
  if (depth100 < 0) depth100 = 0;
  if (depth100 > 65535) depth100 = 65535;
  return static_cast<uint16_t>(depth100);
}

// scheduler with three cases, fast, valve on and normal
// Pick base cycle time:
//  - on cold boot, FAST_BURST_COUNT cycles at TX_CYCLE_FAST_TIME while no valves are timed
//  - otherwise, if any valve timer is non-zero, use CYCLE_TIME_VALVE_ON
//  - otherwise, use appTxDutyCycle (baseline)
static inline uint32_t choose_cycle_base_ms(void) {
  // keep your existing fast-ADR bursts on cold boot, only when both valves idle
  if ((initialCycleFast > 0) && (valveState->timeA == 0) && (valveState->timeB == 0)) {
    Serial.print(initialCycleFast);
    Serial.println(" CycleFast>0, no valves on");
    return TX_CYCLE_FAST_TIME;
  }

  // if either valve has a non-zero timer, use the "valve active" period
  if ((valveState->timeA != 0) || (valveState->timeB != 0)) {
    Serial.println("valves on");
    return CYCLE_TIME_VALVE_ON;
  }

  // otherwise, use the normal duty cycle
  Serial.println(" ApxDutyCycle cycle time");
  return appTxDutyCycle;
}

static void take_downlink_snapshot(void) {
  noInterrupts();

  g_pending_cmd.flags = g_cmd.flags;
  g_pending_cmd.unitsA = g_cmd.unitsA;
  g_pending_cmd.unitsB = g_cmd.unitsB;

  g_cmd.flags = 0;
  g_cmd.unitsA = 0;
  g_cmd.unitsB = 0;

  interrupts();

  Serial.printf("[DL] queued valve cmd: flags=0x%02X uA=%u uB=%u\n",
                (unsigned)g_pending_cmd.flags,
                (unsigned)g_pending_cmd.unitsA,
                (unsigned)g_pending_cmd.unitsB);
}

// Simple scheduler: pick base period, add random dither, program LoRaWAN
static inline void schedule_next_cycle(void) {
  const uint32_t base_ms = choose_cycle_base_ms();

  const int32_t dither_ms = (int32_t)randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
  int32_t next_ms = (int32_t)base_ms + dither_ms;
  if (next_ms < 0) next_ms = 0;

  txDutyCycleTime = (uint32_t)next_ms;

  Serial.print(txDutyCycleTime);
  Serial.println(" cycle time set...");


  LoRaWAN.cycle(txDutyCycleTime);
}

void display_status() {
  Serial.println("in display_status fx ");
  pop_data();
  // [GPT] init-once in setup; removed display.init() here to avoid heap churn

  display.clear();  // wipe framebuffer before drawing
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  //common screen entries:  battery, cycle time, rssi, snr, display name
  snprintf(buffer, sizeof(buffer), "battery: %lu %%", RTC_SLOW_MEMORY[ULP_BAT_PCT]);  // bat_cap8() populates this, and is run in prepareDataFrame
  display.drawString(210, 50, buffer);
  snprintf(buffer, sizeof(buffer), "cycle %lu min", choose_cycle_base_ms() / 60000);  //  SHOULD BE 60000 milliseconds to mins
  display.drawString(210, 70, buffer);
  snprintf(buffer, sizeof(buffer), "rssi (dBm): %d snr: %d", revrssi, revsnr);
  display.drawString(210, 90, buffer);
  snprintf(buffer, sizeof(buffer), "Eui: ...%02x%02x", devEui[6], devEui[7]);  //  last two bytes of devEui

  display.drawString(210, 110, buffer);
  display.setFont(ArialMT_Plain_24);
  display.drawString(60, 100, g_name);  //  screen name


  if (g_device_role == ROLE_VALVE) {
    display.drawLine(0, 25, 120, 25);
    display.drawLine(150, 25, 270, 25);
    display.drawString(60, 0, "valve");
    show_vlv_status(0);
    display.drawString(60, 40, buffer);
    show_vlv_status(1);
    display.drawString(60, 65, buffer);
    snprintf(buffer, sizeof(buffer), "%.1f psi", float(pressResult / 100));
    display.drawString(210, 0, buffer);
  }

  if (g_device_role == ROLE_METER) {
    display.drawLine(0, 25, 80, 25);
    display.drawLine(95, 25, 300, 25);
    display.drawString(80, 0, "interval  total c");
    snprintf(buffer, sizeof(buffer), "%u gal/m", (uint32_t)(RTC_SLOW_MEMORY[ULP_FLOW_RATE]));
    display.drawString(60, 35, buffer);
    snprintf(buffer, sizeof(buffer), "%u gal", (uint32_t)(RTC_SLOW_MEMORY[ULP_VOLUME_DELTA]));
    display.drawString(60, 65, buffer);
    uint32_t count = read_count32();
    snprintf(buffer, sizeof(buffer), "%lu", count);  // counter
    display.drawString(220, 0, buffer);
    display.setFont(ArialMT_Plain_16);
    snprintf(buffer, sizeof(buffer), "reed/wake: %lu", RTC_SLOW_MEMORY[ULP_WAKE_THRESHOLD]);  // reed closures per wake cycle
    display.drawString(210, 30, buffer);
  }

  if (g_device_role == ROLE_SOIL) {
    display.setFont(ArialMT_Plain_16);
    display.drawString(60, 0, "Soil s, d");
    if (soilSensorOut[0] || soilSensorOut[3]) {
      snprintf(buffer, sizeof(buffer), "H20%% %u,  %u", soilSensorOut[0], soilSensorOut[3]);
      display.drawString(60, 30, buffer);
    }
    if (soilSensorOut[1] || soilSensorOut[4]) {
      snprintf(buffer, sizeof(buffer), "degC %u, %u", soilSensorOut[1], soilSensorOut[4]);
      display.drawString(60, 50, buffer);
    }
    if (soilSensorOut[2] || soilSensorOut[5]) {
      snprintf(buffer, sizeof(buffer), "pH %.1f, %.1f", (float)soilSensorOut[2] / 10, (float)soilSensorOut[5] / 10);  //
      display.drawString(60, 70, buffer);
    }
  }

  if (g_device_role == ROLE_LAKE) {
    // --- Lake depth display (calibrated meters), uplink remains uncalibrated ---
    // RTC carries last pressResult

    uint16_t depth_m = depth_m_from_raw(pressResult);

    display.setFont(ArialMT_Plain_24);
    snprintf(buffer, sizeof(buffer), "Depth: %.3f", float(depth_m / 100));
    display.drawString(120, 30, buffer);
  }


  //Serial.print("about to display.display \n");
  //display.clear();

  display.display();  //  true for a full refresh

  // for(int i = 0; i < 25; i++){
  // Serial.printf(" [%2d]: 0x%08X\n", i, RTC_SLOW_MEMORY[i]);
  // }

  // [GPT] wait for E-Ink to finish via BUSY pin (non-blocking yield with timeout)
  bool ok = eink_wait_idle(4000);
  if (!ok) Serial.println("E-ink wait timeout; proceeding cautiously");
  delay(3000);  //  more time to be sure...

}  // of function
/*
POPULATE THE DATA FIELDS OF PRESSURE, COUNT DELTA AND TIMER DELTA FOR THE LAST REED COUNT

wPress[2]
reedCount[2]
ticksDelat[2]
ULP_BAT_PCT (uint8_t)

*/

void pop_data(void) {

  //  BATTERY PERCENTAGE TO rtc

  (void)bat_cap8();

  if (g_device_role == ROLE_VALVE) {
    if (i2c_ready) {
      pressResult = depth_m_from_raw(readMCP3421avg_cont());
      wPres[0] = (pressResult >> 8);
      wPres[1] = (pressResult & 0xFF);
      Serial.printf("pressure result 1, 2 %u, %u\n", wPres[0], wPres[1]);
    } else {
      // optional fallback: zero
      wPres[0] = wPres[1] = 0;
      Serial.printf("pressure result fallback to zero \n");
    }
  }


  if (g_device_role == ROLE_METER) {

    uint32_t count = read_count32();  //  guards against the change of the upper 16 bits
    uint16_t lo = (uint16_t)RTC_SLOW_MEMORY[ULP_COUNT_LO];
    uint16_t last = (uint16_t)RTC_SLOW_MEMORY[ULP_LAST_SENT];
    uint16_t diff = (uint16_t)(lo - last);  // modulo-16, safe across wrap

    //  reed count delta FROM rtc , low two bytes only
    Serial.printf("ULP_COUNT %lu \n", count);
    Serial.printf("ULP_LAST_SENT %lu \n", RTC_SLOW_MEMORY[ULP_LAST_SENT]);

    RTC_SLOW_MEMORY[ULP_REED_DELTA] = diff;  // used in display screen
    RTC_SLOW_MEMORY[ULP_LAST_SENT] = lo;     // advance marker
    Serial.printf("ULP_TS_DELTA_TICK_POP %lu \n", RTC_SLOW_MEMORY[ULP_TS_DELTA_TICK_POP]);
    Serial.printf("ULP_REED_DELTA %lu \n", RTC_SLOW_MEMORY[ULP_REED_DELTA]);
    //  flow calc stored in RTC_SLOW_MEMORY[ULP_FLOW_RATE]
    //  if ULP_TICK_POP <> 0 or no reed delta, then flow is zero
    uint32_t ticks_delta = (((uint32_t)((uint16_t)RTC_SLOW_MEMORY[ULP_TS_DELTA_HI] << 16)) | (uint16_t)(RTC_SLOW_MEMORY[ULP_TS_DELTA_LO]));
    if (diff && ticks_delta) {

      RTC_SLOW_MEMORY[ULP_FLOW_RATE] = (uint32_t)(((uint64_t)VOLUME_PER_TICK * (uint64_t)TICKS_PER_MIN) / (uint64_t)ticks_delta);
    } else {
      RTC_SLOW_MEMORY[ULP_FLOW_RATE] = 0;
    }
    Serial.printf("ticks_delta %lu \n", ticks_delta);
    //  only determine rate if the delta_tick_pop is not set (indicates that the timer has overflowed) and if the reed count has changed
    if ((RTC_SLOW_MEMORY[ULP_TS_DELTA_TICK_POP]) || (RTC_SLOW_MEMORY[ULP_REED_DELTA] == 0)) {

      RTC_SLOW_MEMORY[ULP_VOLUME_DELTA] = 0;
    } else {
      uint32_t vol = (uint32_t)(((uint64_t)diff * (uint64_t)VOLUME_PER_TICK) & 0xFFFFFFFFu);
      RTC_SLOW_MEMORY[ULP_VOLUME_DELTA] = vol;  // or clamp if you prefer
    }
    RTC_SLOW_MEMORY[ULP_TS_DELTA_TICK_POP] = 0x0000;  //  clear the pop flag here, so we can see if the timer pops off again
  }

  if (g_device_role == ROLE_SOIL) {
    uint8_t buf[64];
    if (readModbusFrame(0x01, 0x0001, 2, buf, sizeof(buf), 9600, 120)) {
      uint16_t reg0 = (buf[3] << 8) | buf[4];  // temp?
      uint16_t reg1 = (buf[5] << 8) | buf[6];  // moisture?
    }
    if (readModbusFrame(0x02, 0x0001, 2, buf, sizeof(buf), 9600, 120)) {
      uint16_t reg0 = (buf[3] << 8) | buf[4];  // temp?
      uint16_t reg1 = (buf[5] << 8) | buf[6];  // moisture?
    }
  }

  if (g_device_role == ROLE_LAKE) {
    delay(100);  //  allow sensor to stabilize
    uint16_t lakeResult = readDepthSensor(200, 7);
    pressResult = depth_m_from_raw(lakeResult);
    wPres[0] = (pressResult >> 8);
    wPres[1] = (pressResult & 0xFF);
  }

  //xSemaphoreGive(g_uiSem);
}

/* Prepares the payload of the frame and decrements valve counter or turns off if time is up*/
static void prepareTxFrame(uint8_t port) {
  /*resolve valve status -- ? do we need to turn a valve off?  if so do that and then 
   *reset the valve cycle time if both valves are off
   *two ways to close a valve, based on end of timer and based on a close command.  
   *eiher way resets the cycle time only if BOTH valvees are closed.  
   *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
   *data is pressure (msb, lsb) in raw adc conversion needs to be calibrated and converted to psi
   */

  appDataSize = 0;
  pop_data();

  if (g_device_role == ROLE_METER) {
    appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_REED_DELTA] >> 8));    //  msb
    appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_REED_DELTA] & 0xff));  //  lsb
    appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_FLOW_RATE] >> 8));     //  msb
    appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_FLOW_RATE] & 0xff));   //  lsb
    appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_COUNT_HI] >> 8));      //  msb
    appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_COUNT_HI] & 0xff));    //  lsb
    appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_COUNT_LO] >> 8));      //  msb
    appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_COUNT_LO] & 0xff));    //  lsb
    appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_BAT_PCT] & 0xff));     //
  }

  if (g_device_role == ROLE_VALVE) {
    appData[appDataSize++] = wPres[0];                      //  msb (* 100)
    appData[appDataSize++] = wPres[1];                      //  lsb
    appData[appDataSize++] = (uint8_t)(valveState->timeA);  //  valve A
    appData[appDataSize++] = (uint8_t)(valveState->timeB);  //  valve B
    uint8_t flags =
      ((valveState->onA ? 1 : 0)) |                                             // bit0
      ((valveState->onB ? 1 : 0) << 1) |                                        // bit1
      ((valveState->latchA ? 1 : 0) << 2) |                                     // bit2
      ((valveState->latchB ? 1 : 0) << 3);                                      // bit3
    appData[appDataSize++] = flags;                                             // NEW: latch/ON bits
    appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_BAT_PCT] & 0xff));  //
  }

  if (g_device_role == ROLE_SOIL) {
    appData[appDataSize++] = (uint8_t)(soilSensorOut[0]);                       //  moist shallow
    appData[appDataSize++] = (uint8_t)(soilSensorOut[1]);                       //  temp C shallow
    appData[appDataSize++] = (uint8_t)(soilSensorOut[2]);                       //  pH shallow *10
    appData[appDataSize++] = (uint8_t)(soilSensorOut[3]);                       //  moist deep
    appData[appDataSize++] = (uint8_t)(soilSensorOut[4]);                       //  temp C deep
    appData[appDataSize++] = (uint8_t)(soilSensorOut[5]);                       //  pH deep * 10
    appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_BAT_PCT] & 0xff));  //  battery pct
  }

  if (g_device_role == ROLE_LAKE) {
    appData[appDataSize++] = (uint8_t)(wPres[0]);                               //  msb * 100
    appData[appDataSize++] = (uint8_t)(wPres[1]);                               //  lsb
    appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_BAT_PCT] & 0xff));  //  battery pct
  }

  // for(int i = 0; i < 16; i++){
  //   Serial.printf(" [%2d]: 0x%08X\n", i, RTC_SLOW_MEMORY[i]);
  //   }
}



void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
  if (!mcpsIndication || !mcpsIndication->RxData) {
    Serial.println("[DL] MAC-only (no FRMPayload) — handler exit");
    return;
  }
  g_awake_until_ms = millis() + 8000;                // now stay awake to process this
  static uint32_t last_dl = 0xFFFFFFFF;              // guard: ignore same DL twice
  if (mcpsIndication->DownLinkCounter == last_dl) {  // Semtech stack provides this
    Serial.println("[DL] duplicate counter — ignored");
    return;
  }
  last_dl = mcpsIndication->DownLinkCounter;

  const uint8_t *buf = mcpsIndication->Buffer;
  const uint8_t len = mcpsIndication->BufferSize;
  const uint8_t port = mcpsIndication->Port;
  const char *rxw = (mcpsIndication->RxSlot) ? "RXWIN2" : "RXWIN1";
  Serial.printf("[DL] win=%s  port=%u  len=%u  cnt=%lu  rssi=%d  snr=%d\n",
                rxw, (unsigned)port, (unsigned)len,
                (unsigned long)mcpsIndication->DownLinkCounter,
                (int)mcpsIndication->Rssi, (int)mcpsIndication->Snr);

  Serial.printf("+++++REV DATA:%s,RXSIZE %u,PORT %u\r\n",
                rxw, (unsigned)len, (unsigned)port);

  revrssi = mcpsIndication->Rssi;
  revsnr = mcpsIndication->Snr;

  switch (port) {
    case 5:  //  change the cycle time
      {
        if (len != 1 && len != 2) {
          Serial.println("cycle-time ignored (len!=1/2)");
          break;
        }
        uint32_t minutes = (len == 1) ? (uint32_t)buf[0]
                                      : (((uint32_t)buf[0] << 8) | (uint32_t)buf[1]);
        if (minutes < 10) minutes = 10;
        if (minutes > 1440) minutes = 1440;
        uint32_t ms = minutes * 60000u;

        appTxDutyCycle = ms;
        g_sched_override_ms = ms;
        deviceState = DEVICE_STATE_CYCLE;  // re-run scheduler once

        display_status();                 //  display it
        vTaskDelay(pdMS_TO_TICKS(3000));  //  give SPI bus a chance

        Serial.printf("cycle req: %u min -> %lu ms\n", (unsigned)minutes, (unsigned long)ms);
        break;
      }

    case 6:  //  set the valve status
      {      // ValveCmd_t downlink = [flags, unitsA, unitsB]
        if (len != 3) {
          Serial.printf("[DL6] bad length: %u (want 3)\n", (unsigned)len);
          break;
        }

        ValveCmd_t cmd{};
        cmd.flags = buf[2];   // <— flags last
        cmd.unitsA = buf[0];  // <— A ticks
        cmd.unitsB = buf[1];  // <— B ticks

        post_cmd(&cmd);            // drop into mailbox (see Fix 2)
        take_downlink_snapshot();  // atomically read+clear & act
        g_need_vlv_update = true;
        //xSemaphoreGive(g_uiSem);
        g_awake_until_ms = millis() + 8000;  // stay awake 8s for debug/response
        break;
      }

    case 7:  // name the device
      {
        constexpr size_t MAX_NAME = 12;
        if (len == 0 || len > MAX_NAME) {
          Serial.println("name ignored (len)");
          break;
        }
        char new_name[MAX_NAME + 1];
        size_t wrote = 0;
        bool okAscii = true;
        for (size_t i = 0; i < len; ++i) {
          char c = (char)buf[i];
          if ((unsigned char)c < 0x20 || (unsigned char)c > 0x7E) {
            okAscii = false;
            break;
          }
          new_name[i] = c;
          wrote = i + 1;
        }
        if (!okAscii) {
          Serial.println("name ignored (non-ASCII)");
          break;
        }
        new_name[wrote] = '\0';
        strncpy(g_name, new_name, sizeof(g_name) - 1);
        g_name[sizeof(g_name) - 1] = '\0';
        if (prefs.begin(NS, false)) {
          (void)prefs.putString(K_NAME, new_name);
          prefs.end();
        }
        Serial.println("#####name is updated");

        break;
      }

    case 8:
      {  // ULP wake threshold
        uint16_t th = (len == 1) ? (uint16_t)buf[0]
                                 : (len >= 2 ? (uint16_t)buf[0] << 8 | (uint16_t)buf[1] : 0);
        RTC_SLOW_MEMORY[ULP_WAKE_THRESHOLD] = th;
        if (prefs.begin(NS, false)) {
          (void)prefs.putUShort(K_WAKE_TH, th);
          prefs.end();
        }
        Serial.printf("#####NVS reed threshold write (value=%u)\n", (unsigned)th);

        break;
      }

    case 22:
      {  // calib: inv_m_u32 (u32 BE), b_x10 (s32 BE)
        if (len != 8) {
          Serial.println("calib ignored (len!=8)");
          break;
        }
        uint32_t invm = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
                        | ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
        int32_t bx10 = (int32_t)(((uint32_t)buf[4] << 24) | ((uint32_t)buf[5] << 16)
                                 | ((uint32_t)buf[6] << 8) | (uint32_t)buf[7]);
        if (invm == 0 || invm > 5000000u) {
          Serial.println("calib ignored (inv_m out of range)");
          break;
        }
        inv_m_u32 = invm;
        b_x10 = bx10;
        bool ok = false;
        if (prefs.begin(NS, false)) {
          ok = (prefs.putUInt(K_INV_M, inv_m_u32) == sizeof(uint32_t));
          ok &= (prefs.putInt(K_BX10, b_x10) == sizeof(int32_t));
          prefs.end();
        }
        Serial.printf("#####calib %s: inv_m=%u cnt/m  b=%.1f m\n", ok ? "stored" : "FAILED",
                      inv_m_u32, b_x10 / 10.0f);
        break;
      }

    case 23:
      {  // lake depth setpoint (sensor depth). **BIG-ENDIAN** [MSB, LSB]
        if (len != 2) {
          Serial.println("lake-depth ignored (len!=2)");
          break;
        }
        uint16_t depth_mm = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];  // BE: MSB first
        g_lake_depth_mm = depth_mm;
        bool ok = false;
        if (prefs.begin(NS, false)) {
          ok = (prefs.putUShort(K_LAKE_MM, depth_mm) == sizeof(uint16_t));
          prefs.end();
        }
        Serial.printf("#####lake-depth set: %u (%.3f m) %s\n",
                      depth_mm, depth_mm / 1000.0f, ok ? "stored" : "NVS_FAIL");

        break;
      }

    case 24:
      {  // valve config: v0_ms(2) v1_ms(2) flags(1) [reserved(1)]  **BIG-ENDIAN**
        if (len != 5 && len != 6) {
          Serial.println("valve-cfg ignored (len!=5/6)");
          break;
        }

        uint16_t v0_ms = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
        uint16_t v1_ms = ((uint16_t)buf[2] << 8) | (uint16_t)buf[3];
        uint8_t flags = buf[4];

        bool v0_fwd = (flags & 0x01) != 0;
        bool v1_fwd = (flags & 0x02) != 0;

        // Clamp (safety)
        v0_ms = (v0_ms < 50) ? 50 : (v0_ms > 1000) ? 1000
                                                   : v0_ms;
        v1_ms = (v1_ms < 50) ? 50 : (v1_ms > 1000) ? 1000
                                                   : v1_ms;

        // Apply immediately (RTC vars)
        g_v0_act_ms = v0_ms;
        g_v1_act_ms = v1_ms;
        g_v0_open_fwd = v0_fwd;
        g_v1_open_fwd = v1_fwd;

        // Persist
        bool ok = false;
        if (prefs.begin(NS, false)) {
          bool ok1 = (prefs.putUShort("v0_act_ms", g_v0_act_ms) == sizeof(uint16_t));
          bool ok2 = (prefs.putUShort("v1_act_ms", g_v1_act_ms) == sizeof(uint16_t));
          bool ok3 = prefs.putBool("v0_open_fwd", g_v0_open_fwd);
          bool ok4 = prefs.putBool("v1_open_fwd", g_v1_open_fwd);
          prefs.end();
          ok = ok1 && ok2 && ok3 && ok4;
        }

        Serial.printf("#####valve-cfg set: v0=%u ms (%s)  v1=%u ms (%s) flags=0x%02X %s\n",
                      (unsigned)g_v0_act_ms, g_v0_open_fwd ? "FWD" : "REV",
                      (unsigned)g_v1_act_ms, g_v1_open_fwd ? "FWD" : "REV",
                      flags,
                      ok ? "stored" : "NVS_FAIL");

        break;
      }

  }  // of switch

  xSemaphoreGive(g_uiSem);
  revrssi = mcpsIndication->Rssi;
  revsnr = mcpsIndication->Snr;

  Serial.println("downlink processed");
}  // of function

/* set up the ULP to count pulses, and wake on 100 pulses, after each lorawan data send, the last_pulse_count is advanced*/

static const ulp_insn_t ulp_program[] = {
  M_LABEL(ULP_ENTRY_LABEL),
  I_DELAY(0x8fff),  // needs to be calibrated to time

  // At top of ulp_program, BEFORE merging PENDING into COUNT:
  I_RD_REG(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_MAIN_STATE_IN_IDLE_S, RTC_CNTL_MAIN_STATE_IN_IDLE_S),
  M_BG(ULP_SKIP_MERGE, 0),  // if CPU is awake (>0), skip the merge this cycle

  // ── Merge PENDING (16-bit) into 32-bit COUNT (HI:LO) ──
  I_MOVI(R1, ULP_COUNT_PENDING),
  I_LD(R0, R1, 0),
  I_SUBI(R0, R0, 0),
  M_BXZ(ULP_SKIP_MERGE),

  I_MOVI(R2, ULP_COUNT_LO),
  I_LD(R3, R2, 0),
  I_ADDR(R3, R3, R0),  // add pending → LO
  I_ST(R3, R2, 0),
  M_BXF(ULP_BUMP_HI_MERGE),  // overflow → bump HI
  M_BX(ULP_CLR_PENDING),

  M_LABEL(ULP_BUMP_HI_MERGE),
  I_MOVI(R2, ULP_COUNT_HI),
  I_LD(R3, R2, 0),
  I_ADDI(R3, R3, 1),
  I_ST(R3, R2, 0),

  M_LABEL(ULP_CLR_PENDING),
  I_MOVI(R0, 0),
  I_MOVI(R1, ULP_COUNT_PENDING),
  I_ST(R0, R1, 0),

  M_LABEL(ULP_SKIP_MERGE),

  //── 0) ULP_TIMER (low/hi) ────────────────────────────────
  I_MOVI(R1, ULP_TIMER_LO),
  I_LD(R3, R1, 0),
  I_ADDI(R3, R3, 1),
  I_ST(R3, R1, 0),
  I_MOVR(R0, R3),
  M_BG(ULP_NO_TIMER_WRAP, 0),  // skip if low > 0

  // wrap → bump hi or tick-pop
  I_MOVI(R1, ULP_TIMER_HI),
  I_LD(R2, R1, 0),
  I_MOVI(R0, 0xFFFF),
  I_SUBR(R0, R0, R2),         // 0xFFFF - hi
  M_BL(ULP_SET_TICK_POP, 1),  // if hi == max → pop
  I_ADDI(R2, R2, 1),
  I_ST(R2, R1, 0),
  M_BX(ULP_NO_TIMER_WRAP),

  M_LABEL(ULP_SET_TICK_POP),
  I_MOVI(R0, 1),
  I_MOVI(R1, ULP_TICK_POP),
  I_ST(R0, R1, 0),
  I_MOVI(R2, 0),
  I_MOVI(R1, ULP_TIMER_HI),
  I_ST(R2, R1, 0),
  M_BX(ULP_NO_TIMER_WRAP),

  M_LABEL(ULP_NO_TIMER_WRAP),

  //── 1) Sample GPIO → raw_bit ─────────────────────────────
  I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_INDEX + RTC_GPIO_IN_NEXT_S, RTC_GPIO_INDEX + RTC_GPIO_IN_NEXT_S),
  I_ANDI(R0, R0, 1),
  I_MOVR(R2, R0),

  //── 2) Rising-edge detect (raw_bit - prev_state == 1) ─────
  I_MOVI(R1, ULP_PREV_STATE),
  I_LD(R0, R1, 0),
  I_SUBR(R0, R2, R0),
  I_ST(R2, R1, 0),
  M_BL(ULP_NO_EDGE, 1),
  M_BG(ULP_NO_EDGE, 1),

  //── 3) Conditional bump ─────────────────────────────────────
  I_RD_REG(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_MAIN_STATE_IN_IDLE_S, RTC_CNTL_MAIN_STATE_IN_IDLE_S),
  M_BG(ULP_CPU_IS_AWAKE, 0),  // if flag == 1, CPU is awake, use pending count

  // ── CPU idle → bump 32-bit COUNT (HI:LO) ──

  I_MOVI(R1, ULP_COUNT_LO),
  I_LD(R3, R1, 0),
  I_ADDI(R3, R3, 1),
  I_ST(R3, R1, 0),
  M_BXF(ULP_BUMP_HI_EDGE),
  M_BX(ULP_AFTER_COUNT),

  M_LABEL(ULP_BUMP_HI_EDGE),
  I_MOVI(R1, ULP_COUNT_HI),
  I_LD(R3, R1, 0),
  I_ADDI(R3, R3, 1),
  I_ST(R3, R1, 0),
  M_BX(ULP_AFTER_COUNT),

  M_LABEL(ULP_CPU_IS_AWAKE),
  // CPU is active → bump ULP_COUNT_PENDING
  I_MOVI(R1, ULP_COUNT_PENDING),
  I_LD(R3, R1, 0),
  I_ADDI(R3, R3, 1),
  I_ST(R3, R1, 0),
  M_BX(ULP_NO_WAKE),

  M_LABEL(ULP_AFTER_COUNT),

  //── 4) Snapshot ULP_TIMER → Δ lo/hi/pop ───────────────────
  I_MOVI(R1, ULP_TIMER_LO),
  I_LD(R0, R1, 0),
  I_MOVI(R1, ULP_TS_DELTA_LO),
  I_ST(R0, R1, 0),
  I_MOVI(R1, ULP_TIMER_HI),
  I_LD(R0, R1, 0),
  I_MOVI(R1, ULP_TS_DELTA_HI),
  I_ST(R0, R1, 0),
  I_MOVI(R1, ULP_TICK_POP),
  I_LD(R0, R1, 0),
  I_MOVI(R1, ULP_TS_DELTA_TICK_POP),
  I_ST(R0, R1, 0),
  // clear timer & pop

  I_MOVI(R0, 0),
  I_MOVI(R1, ULP_TIMER_LO),
  I_ST(R0, R1, 0),
  I_MOVI(R1, ULP_TIMER_HI),
  I_ST(R0, R1, 0),
  I_MOVI(R1, ULP_TICK_POP),
  I_ST(R0, R1, 0),

  // ── Wake logic: use LO only (diff16 = LO - LAST_SENT) ──
  I_MOVI(R1, ULP_COUNT_LO),
  I_LD(R0, R1, 0),  // R0 = LO
  I_MOVI(R1, ULP_LAST_SENT),
  I_LD(R1, R1, 0),     // R1 = last_sent (16-bit)
  I_SUBR(R0, R0, R1),  // diff16
  I_MOVI(R2, ULP_DEBUG_PIN_STATE),
  I_ST(R0, R2, 0),
  I_MOVI(R1, ULP_WAKE_THRESHOLD),
  I_LD(R1, R1, 0),
  I_SUBR(R0, R1, R0),  // threshold - diff
  M_BG(ULP_NO_WAKE, 0),

  // only wake if main CPU idle
  I_RD_REG(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_MAIN_STATE_IN_IDLE_S, RTC_CNTL_MAIN_STATE_IN_IDLE_S),
  M_BG(ULP_NO_WAKE, 0),  // skip if CPU awake

  I_WAKE(),

  M_LABEL(ULP_NO_WAKE),
  M_LABEL(ULP_NO_EDGE),
  M_BX(ULP_ENTRY_LABEL),
};

static void rs485_spin() {
  for (;;) {
    RS485Send(0);
    delay(1000);
  }
}
static bool load_cfg_into_rtc_on_coldboot() {

  if (!prefs.begin(NS, /*readOnly=*/true)) {  // NS == "cfg"
    Serial.println("[CFG] prefs.begin failed");
    return false;
  }

  // ---- schema / identity ----
  g_schema_ver = prefs.getUChar(K_SCHEMA, 0);
  g_device_role = prefs.getUChar(K_ROLE, 0);
  g_uplink_fport = prefs.getUChar(K_FPORT, 0);
  g_lake_type = prefs.getUChar(K_LAKE_TYP, 0);

  if (g_schema_ver == 0) {
    g_schema_ver = CFG_SCHEMA_VERSION;
    Serial.printf("[CFG] schema_ver missing (using default %u, not stored)\n",
                  (unsigned)g_schema_ver);
  }
  // ---- existing config you already use ----
  g_lake_depth_mm = prefs.getUShort(K_LAKE_MM, g_lake_depth_mm);

  uint16_t th = prefs.getUShort(K_WAKE_TH, PULSE_THRESHOLD);
  RTC_SLOW_MEMORY[ULP_WAKE_THRESHOLD] = th;

  char tmp[13] = {};
  if (prefs.isKey(K_NAME)) {
    prefs.getString(K_NAME, tmp, sizeof(tmp));
    if (tmp[0]) {
      strncpy(g_name, tmp, sizeof(g_name) - 1);
      g_name[sizeof(g_name) - 1] = '\0';
    }
  }

  inv_m_u32 = prefs.getUInt(K_INV_M, inv_m_u32);
  b_x10 = prefs.getInt(K_BX10, b_x10);

  // ---- valve behavior (now kept in cfg) ----
  //if (!prefs.isKey(K_V0_ACT)) prefs.putUShort(K_V0_ACT, g_v0_act_ms);
  //if (!prefs.isKey(K_V1_ACT)) prefs.putUShort(K_V1_ACT, g_v1_act_ms);
  g_v0_act_ms = prefs.getUShort(K_V0_ACT, g_v0_act_ms);
  g_v1_act_ms = prefs.getUShort(K_V1_ACT, g_v1_act_ms);

  //if (!prefs.isKey(K_V0_FWD)) prefs.putBool(K_V0_FWD, g_v0_open_fwd);
  //if (!prefs.isKey(K_V1_FWD)) prefs.putBool(K_V1_FWD, g_v1_open_fwd);
  g_v0_open_fwd = prefs.getBool(K_V0_FWD, g_v0_open_fwd);
  g_v1_open_fwd = prefs.getBool(K_V1_FWD, g_v1_open_fwd);

  prefs.end();

  // ---- sanity / clamps (keep your existing clamp style) ----
  g_v0_act_ms = (g_v0_act_ms < 50) ? 50 : (g_v0_act_ms > 1000) ? 1000
                                                               : g_v0_act_ms;
  g_v1_act_ms = (g_v1_act_ms < 50) ? 50 : (g_v1_act_ms > 1000) ? 1000
                                                               : g_v1_act_ms;
  if (!inv_m_u32) inv_m_u32 = 1;

  // ---- mandatory provisioning check ----
  if (g_device_role == 0 || g_uplink_fport == 0) {
    Serial.printf("[CFG] INVALID: role=%u fport=%u (node not provisioned?)\n",
                  (unsigned)g_device_role, (unsigned)g_uplink_fport);
    return false;
  }

  // fPort becomes your application port
  appPort = g_uplink_fport;

  // ---- LoRaWAN OTAA identity lives in NVS namespace "lorawan" ----
  if (!prefs.begin(NS_LORAWAN, /*readOnly=*/true)) {
    Serial.println("[LORAWAN] prefs.begin failed");
    return false;
  }

  // Optional: honor your provisioning marker
  if (!prefs.getBool(K_PROV, false)) {
    Serial.println("[LORAWAN] not provisioned");
    prefs.end();
    return false;
  }

  size_t n = 0;

  n = prefs.getBytes(K_DEVEUI, devEui, 8);
  if (n != 8) {
    Serial.printf("[LORAWAN] missing/short devEui (got %u bytes)\n", (unsigned)n);
    prefs.end();
    return false;
  }

  n = prefs.getBytes(K_APPKEY, appKey, 16);
  if (n != 16) {
    Serial.printf("[LORAWAN] missing/short appKey (got %u bytes)\n", (unsigned)n);
    prefs.end();
    return false;
  }

  prefs.end();

  Serial.printf("[LORAWAN] DevEui loaded: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
                devEui[0], devEui[1], devEui[2], devEui[3],
                devEui[4], devEui[5], devEui[6], devEui[7]);

  // ---- one-line boot summary ----
  Serial.printf("[CFG] schema=%u role=%u fport=%u lake_type=%u name='%s'\n",
                (unsigned)g_schema_ver,
                (unsigned)g_device_role,
                (unsigned)g_uplink_fport,
                (unsigned)g_lake_type,
                g_name);

  Serial.printf("[CFG] lake_depth_mm=%u  wake_th=%u  inv_m=%u  b_x10=%d\n",
                (unsigned)g_lake_depth_mm,
                (unsigned)RTC_SLOW_MEMORY[ULP_WAKE_THRESHOLD],
                (unsigned)inv_m_u32,
                (int)b_x10);

  Serial.printf("[CFG] valve: v0=%u ms (%s)  v1=%u ms (%s)\n",
                (unsigned)g_v0_act_ms, g_v0_open_fwd ? "FWD" : "REV",
                (unsigned)g_v1_act_ms, g_v1_open_fwd ? "FWD" : "REV");

  return true;
}

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println("Booting...");

  if (g_uiSem == NULL) {
    g_uiSem = xSemaphoreCreateBinary();
    configASSERT(g_uiSem);  // if this trips, creation failed (RAM)
  }

  hardware_pins_init();  //
  setPowerEnable(1);
  delay(50);  // rail settle

  // retry-probe window (~300 ms)
  uint32_t deadline = millis() + 300;
  bool found = false;
  do {
    Wire.end();
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.beginTransmission(0x68);
    if (Wire.endTransmission() == 0) {
      found = true;
      break;
    }

    // simple bus recovery if SDA stuck low
    pinMode(PIN_SDA, INPUT_PULLUP);  // <-- add this line (re-claim SDA as GPIO)
    pinMode(PIN_SCL, OUTPUT);
    for (int i = 0; i < 9 && digitalRead(PIN_SDA) == LOW; ++i) {
      digitalWrite(PIN_SCL, LOW);
      delayMicroseconds(5);
      digitalWrite(PIN_SCL, HIGH);
      delayMicroseconds(5);
    }
    pinMode(PIN_SCL, INPUT_PULLUP);
    delay(20);
  } while ((int32_t)(millis() - deadline) < 0);

  i2c_ready = found;
  Serial.printf("I2C 0x68 %s\n", i2c_ready ? "OK" : "NACK");
  TxDutyCycle_hold = (TxDutyCycle_hold == 0) ? 3600000 : TxDutyCycle_hold;

  // configure RTC GPIO, enable ULP wake up.
  ESP_ERROR_CHECK(rtc_gpio_hold_dis(RTC_GPIO_SENSOR_PIN));
  ESP_ERROR_CHECK(rtc_gpio_init(RTC_GPIO_SENSOR_PIN));
  ESP_ERROR_CHECK(rtc_gpio_set_direction(RTC_GPIO_SENSOR_PIN, RTC_GPIO_MODE_INPUT_ONLY));
  ESP_ERROR_CHECK(rtc_gpio_set_direction_in_sleep((gpio_num_t)RTC_GPIO_SENSOR_PIN, RTC_GPIO_MODE_INPUT_ONLY));
  ESP_ERROR_CHECK(rtc_gpio_pullup_dis((gpio_num_t)RTC_GPIO_SENSOR_PIN));
  ESP_ERROR_CHECK(rtc_gpio_pulldown_dis((gpio_num_t)RTC_GPIO_SENSOR_PIN));
  ESP_ERROR_CHECK(rtc_gpio_hold_en(RTC_GPIO_SENSOR_PIN));
  ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

  // figure out why we’re here
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  switch (cause) {
    case ESP_SLEEP_WAKEUP_ULP:
      {
        uint32_t count = read_count32();
        (void)count;
        xSemaphoreGive(g_uiSem);
      }
      break;

    case ESP_SLEEP_WAKEUP_TIMER:
      {
        uint32_t count = read_count32();
        if (initialCycleFast > 0) {
          initialCycleFast--;
        }
        Serial.printf("Woke by RTC TIMER, pulse count = %u\n", count);
        xSemaphoreGive(g_uiSem);
      }
      break;

    case ESP_SLEEP_WAKEUP_UNDEFINED:
      {  // THIS IS COLD RESTART
        Serial.printf("WAKEUP_BY_RESTART:::\n");
        memset(RTC_SLOW_MEM, 0, CONFIG_ULP_COPROC_RESERVE_MEM);

        // Load cfg once into RTC, and set appPort from cfg/uplink_fport
        if (!load_cfg_into_rtc_on_coldboot()) {
          Serial.println("[CFG] load failed - halting");
          while (true) delay(1000);
        }
        // --- Try loading LoRaWAN identity from NVS ---
        bool nvs_ok = load_lorawan_identity_from_nvs();

        if (nvs_ok) {
          Serial.println("[NVS] LoRaWAN identity loaded from NVS");
        } else {
          Serial.println("[NVS] Using compiled-in LoRaWAN keys");
        }
        initialCycleFast = FAST_BURST_COUNT;

        TxDutyCycle_hold = appTxDutyCycle;
        xSemaphoreGive(g_uiSem);

        valveState->onA = 0;
        valveState->onB = 0;

        size_t sz = sizeof(ulp_program) / sizeof(ulp_insn_t);
        esp_err_t r = ulp_process_macros_and_load(ULP_PROG_START, ulp_program, &sz);
        Serial.printf("load - %s, %u instructions\n", esp_err_to_name(r), (unsigned)sz);
        ESP_ERROR_CHECK(ulp_run(ULP_PROG_START));
        if (g_device_role == ROLE_VALVE) {
          if (g_coldboot_valve_init_done == 0) {
            for (int t = 0; t < 1; t++) {
              delay(500);
              Serial.printf("set valves to on (cycle %d)...\n", t + 1);
              controlValve(0, 1);
              delay(500);
              controlValve(1, 1);
              delay(500);
              Serial.printf("set valves to off (cycle %d/2)...\n", t + 1);
              controlValve(0, 0);
              delay(500);
              controlValve(1, 0);
            }
            delay(3000);
            g_coldboot_valve_init_done = 1;
          } else {
            Serial.println("valve init skipped (already done this power-up)");
          }
        }
      }
      break;

    default:
      {
        uint32_t count = read_count32();
        Serial.printf("Woke by DEFAULT, pulse count = %u\n", count);
        xSemaphoreGive(g_uiSem);
      }
      break;
  }  // end switch(cause)


  static uint16_t loop_count = 0;
  static uint16_t raw_pressure = 0;

#if defined(CALIBRATION_MODE)
  //#############################################test pressire every 5 seconds for calibration, not for dep;loyment
  while (1) {
    if (i2c_ready) {
      raw_pressure = readMCP3421avg_cont();
      pressResult = depth_m_from_raw(raw_pressure);
      Serial.printf("raw : %u\n", raw_pressure);
      Serial.printf("pressure * 100 : %i\n", (int)((float)pressResult / 100.0f));
      loop_count++;
      switch (loop_count % 4) {
        case (0):
          controlValve(0, 1);
          Serial.printf("A : on\n");
          break;
        case (1):
          controlValve(1, 1);
          Serial.printf("B : on\n");
          break;
        case (2):
          controlValve(0, 0);
          Serial.printf("A : off\n");
          break;
        case (3):
          controlValve(1, 0);
          Serial.printf("B : off\n");
          break;
      }
    }
    delay(5000);
  }
//############################################################################
#endif

  if (g_device_role == ROLE_VALVE) {
    tick_timers(valveState);
    uint32_t t_end = millis() + 200;
    while ((int32_t)(millis() - t_end) < 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  vTaskDelay(pdMS_TO_TICKS(100));
  static bool displayInited = false;
  if (!displayInited) {
    display.init();
    displayInited = true;

    display.screenRotate(ANGLE_180_DEGREE);
    display.setFont(ArialMT_Plain_24);
    vTaskDelay(pdMS_TO_TICKS(50));  // short settle; NOT seconds
    display_status();
    //Serial.println("BOOT: after HELTEC_B start, Serial ready");
    if (!adc.begin(ADC_ADDR, &Wire)) {
      Serial.println("MCP3421 not found on Wire");
    } else Serial.println("MCP3421 found");
  }
}  // of setup function


void loop() {
  static eDeviceState_LoraWan prevState = (eDeviceState_LoraWan)-1;
  if (prevState != deviceState) {
    Serial.printf("STATE -> %s\n", deviceStateToString(deviceState));
    prevState = deviceState;
  }

  switch (deviceState) {
    case DEVICE_STATE_INIT:
      {
        LoRaWAN.init(loraWanClass, loraWanRegion);
        LoRaWAN.setDefaultDR(3);
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        // STOCK: let the MAC handle RX windows
        Serial.println("[SM] ENTER JOIN -> calling LoRaWAN.join()");
        join_inflight = true;  // [CHANGE] flag OTAA in progress
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        //  Join succeeded → clear inflight so UI can run again safely
        join_inflight = false;
        //Serial.println("In SEND");
        prepareTxFrame(appPort);  //  has a pop_data() calll in it
        LoRaWAN.send();
        g_last_tx_ms = millis();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        //Serial.println("In CYCLE");
        schedule_next_cycle();  //  LoRaWAN.cycle() that allows for fast initial cycles to figure our ADR and make settings
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        if (join_inflight) {
          LoRaWAN.sleep(loraWanClass);    // <-- DO NOT skip this during join
          vTaskDelay(pdMS_TO_TICKS(10));  // yield, keep pumping loop
          break;
        }
        // Apply valve cmd as soon as safe, then start a 30 s settle window.
        // if (g_need_vlv_update && rx_windows_clear() && !join_inflight) {
        if (g_need_vlv_update) {
          Serial.println("[VL] applying queued valve cmd (from SLEEP)");
          apply_downlink_snapshot();  // actuate valve NOW
          g_send_after_settle = true;
          g_send_due_ms = millis() + 30000;  // <-- THIS is the missing piece
          break;                             // optional but recommended: don't do UI work this loop
        }

        // Optional UI update (doesn't affect LoRaWAN timing)
        if ((xSemaphoreTake(g_uiSem, 0) == pdTRUE) && rx_windows_clear() && !join_inflight) {
          display_status();
        }

        // If we’re waiting to re-uplink, stay awake until due, then SEND exactly once.
        if (g_send_after_settle) {
          if ((int32_t)(millis() - g_send_due_ms) >= 0) {
            g_send_after_settle = false;
            deviceState = DEVICE_STATE_SEND;
            break;
          }
          vTaskDelay(pdMS_TO_TICKS(50));  // don’t spin and don’t re-read sensors
          break;
        }

        // Normal behavior when there's no valve event pending
        LoRaWAN.sleep(loraWanClass);
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}  // of loop function
