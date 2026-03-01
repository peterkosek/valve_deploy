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
  f = g_pending_cmd.flags;
  uA = g_pending_cmd.unitsA;
  uB = g_pending_cmd.unitsB;
  g_pending_cmd.flags = 0;
  g_pending_cmd.unitsA = 0;
  g_pending_cmd.unitsB = 0;
  interrupts();

  Serial.printf("[DL] applied valve cmd: flags=0x%02X uA=%u uB=%u\n",
                f,
                uA,
                uB);
  // --- HARD OFF overrides ---

  if (f & FLAG_OFF_A) {
    controlValve(0, 0);
    valveState->onA = 0;
    valveState->latchA = 0;
    valveState->timeA = 0;
  }

  if (f & FLAG_OFF_B) {
    controlValve(1, 0);
    valveState->onB = 0;
    valveState->latchB = 0;
    valveState->timeB = 0;
  }

  // --- LATCH (force ON, no timer) ---

  if (f & FLAG_LATCH_A) {
    Serial.printf("A latch ON\n");
    controlValve(0, 1);
    valveState->onA = 0;
    valveState->latchA = 1;
    valveState->timeA = 0;
  }

  if (f & FLAG_LATCH_B) {
    Serial.printf("B latch ON\n");
    controlValve(1, 1);
    valveState->onB = 0;
    valveState->latchB = 1;
    valveState->timeB = 0;
  }

  // --- TIMED (turn ON, clear latch, set ticks from units) ---

  if (f & FLAG_ON_A) {
    controlValve(0, 1);
    valveState->onA = 1;
    valveState->latchA = 0;
    valveState->timeA = uA;
  }

  if (f & FLAG_ON_B) {
    controlValve(1, 1);
    valveState->onB = 1;
    valveState->latchB = 0;
    valveState->timeB = uB;
  }
  g_awake_until_ms = millis() + 35000;
  g_need_vlv_update = false;  //  wait 30 seconds and send an uplink before sleep
}


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
    g_send_after_settle = true;
    g_send_due_ms = millis() + 30000;

    xSemaphoreGive(g_uiSem);
  }

  // timed B expired -> turn OFF
  if (v->onB && !v->latchB && v->timeB == 0) {
    controlValve(1, 0);
    v->onB = 0;
    g_send_after_settle = true;
    g_send_due_ms = millis() + 30000;

    xSemaphoreGive(g_uiSem);
  }

  g_skip_next_decrement = true;  // only once per wake cycle
}
