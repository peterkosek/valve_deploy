#include "projdefs.h"
// valve_logic.cpp
#include "valve_logic.h"
#include "LoRaWan_APP.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static portMUX_TYPE g_vlvMux = portMUX_INITIALIZER_UNLOCKED;
#define ENTER_CRIT() portENTER_CRITICAL(&g_vlvMux)
#define EXIT_CRIT() portEXIT_CRITICAL(&g_vlvMux)
// move your current set_vlv_status(), printValveState(), displayPacketBits() here unchanged

// Take a one-shot snapshot and CLEAR it so it won't re-fire next tick
void post_cmd(const volatile ValveCmd_t* in) {
  noInterrupts();
  g_cmd.flags = in->flags;
  g_cmd.unitsA = in->unitsA;
  g_cmd.unitsB = in->unitsB;
  interrupts();
}

// Bit masks for readability
enum { startTimedA = 1 << 0,
       startTimedB = 1 << 1,
       pendLatchA = 1 << 2,
       pendLatchB = 1 << 3,
       offA = 1 << 4,
       offB = 1 << 5 };

// Atomically read+clear from the mailbox, then act
void apply_downlink_snapshot(void) {
  uint8_t f, uA, uB;

  noInterrupts();
  f = g_cmd.flags;
  uA = g_cmd.unitsA;
  uB = g_cmd.unitsB;
  g_cmd.flags = 0;
  g_cmd.unitsA = 0;
  g_cmd.unitsB = 0;
  interrupts();

  // --- HARD OFF overrides ---

  if (f & OFF_A) {
    controlValve(0, 0);
    valveState->onA = 0;
    valveState->latchA = 0;
    valveState->timeA = 0;
  }

  if (f & OFF_B) {
    controlValve(1, 0);
    valveState->onB = 0;
    valveState->latchB = 0;
    valveState->timeB = 0;
  }

  // --- LATCH (force ON, no timer) ---

  if (f & LATCH_A) {
    Serial.printf("A latch ON\n");
    controlValve(0, 1);
    valveState->onA = 0;  // "on" is for timed, latch is separate
    valveState->latchA = 1;
    valveState->timeA = 0;
  }

  if (f & LATCH_B) {
    Serial.printf("B latch ON\n");
    controlValve(1, 1);
    valveState->onB = 0;
    valveState->latchB = 1;
    valveState->timeB = 0;
  }

  // --- TIMED (turn ON, clear latch, set ticks from units) ---

  if (f & START_TIMED_A) {
    controlValve(0, 1);
    valveState->onA = 1;
    valveState->latchA = 0;
    valveState->timeA = uA;  // 10-minute units from your downlink
  }

  if (f & START_TIMED_B) {
    controlValve(1, 1);
    valveState->onB = 1;
    valveState->latchB = 0;
    valveState->timeB = uB;
  }

  // NOTE:
  // We do NOT call LoRaWAN.cycle() here anymore.
  // The main loop's DEVICE_STATE_SEND -> DEVICE_STATE_CYCLE path
  // will schedule the next wake using valveState->timeA/timeB.
}

#include "valve_logic.h"

void show_vlv_status(uint8_t vlv) {
  switch (vlv) {
    case 0:
      if (valveState->onA) {
        unsigned mins = (unsigned)(valveState->timeA * 10);
        snprintf(buffer, sizeof(buffer), "A %u min\n", mins);
      } else if (valveState->latchA) {
        snprintf(buffer, sizeof(buffer), "A latched\n");
      } else {
        snprintf(buffer, sizeof(buffer), "A off");
      }
      break;

    case 1:
      if (valveState->onB) {
        unsigned mins = (unsigned)(valveState->timeB * 10);
        snprintf(buffer, sizeof(buffer), "B %u min\n", mins);
      } else if (valveState->latchB) {
        snprintf(buffer, sizeof(buffer), "B latched\n");
      } else {
        snprintf(buffer, sizeof(buffer), "B off");
      }
      break;
  }
}

// run every tick (e.g., 10-min), AFTER apply_downlink and any UI button handling
void tick_timers(volatile ValveState_t* v) {
  // decrement once per wake cycle if requested
  if (v->onA && !v->latchA && !g_skip_next_decrement && v->timeA) {
    v->timeA--;
  }
  if (v->onB && !v->latchB && !g_skip_next_decrement && v->timeB) {
    v->timeB--;
  }

  // timed A expired -> turn OFF
  if (v->onA && !v->latchA && v->timeA == 0) {
    controlValve(0, 0);
    v->onA = 0;

    // Optional: trigger an immediate status uplink so server sees OFF quickly
    xSemaphoreGive(g_uiSem);
    pop_data();
    vTaskDelay(pdMS_TO_TICKS(3000));
    deviceState = DEVICE_STATE_SEND;
  }

  // timed B expired -> turn OFF
  if (v->onB && !v->latchB && v->timeB == 0) {
    controlValve(1, 0);
    v->onB = 0;

    // Optional: immediate status uplink
    xSemaphoreGive(g_uiSem);
    pop_data();
    vTaskDelay(pdMS_TO_TICKS(3000));
    deviceState = DEVICE_STATE_SEND;
  }

  g_skip_next_decrement = true;  // only once per wake cycle
}
