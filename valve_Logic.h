// valve_logic.h
#pragma once
#include "sensor_solenoid.h"

extern bool g_skip_next_decrement;
extern volatile ValveState_t *valveState;
extern volatile ValveCmd_t g_cmd;
extern volatile uint8_t dl_flags, dl_unitsA, dl_unitsB;
extern volatile bool g_need_display;
extern SemaphoreHandle_t g_uiSem;
extern volatile uint32_t TxDutyCycle_hold;
extern uint32_t appTxDutyCycle;
extern char buffer[64];
extern enum eDeviceState_LoraWan deviceState;

void post_cmd(const volatile ValveCmd_t*);
void apply_downlink_snapshot(void);
void show_vlv_status(uint8_t vlv);
void tick_timers(volatile ValveState_t *v);
void set_cycle_for_irrigation(bool set_it);
void pop_data(void);