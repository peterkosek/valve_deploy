#pragma once

#include <Arduino.h>
#include <Preferences.h>

// Generic typed write helpers
bool pref_put_ushort(Preferences &prefs, const char *key, uint16_t value);
bool pref_put_uint(Preferences &prefs, const char *key, uint32_t value);
bool pref_put_int(Preferences &prefs, const char *key, int32_t value);
bool pref_put_bool(Preferences &prefs, const char *key, bool value);
bool pref_put_string(Preferences &prefs, const char *key, const char *value);

bool pref_get_uchar(Preferences &prefs, const char *key, uint8_t default_value, uint8_t &out);
bool pref_get_ushort(Preferences &prefs, const char *key, uint16_t default_value, uint16_t &out);
bool pref_get_uint(Preferences &prefs, const char *key, uint32_t default_value, uint32_t &out);
bool pref_get_int(Preferences &prefs, const char *key, int32_t default_value, int32_t &out);
bool pref_get_bool(Preferences &prefs, const char *key, bool default_value, bool &out);
bool pref_get_string(Preferences &prefs, const char *key, char *buf, size_t buf_size);

// Higher-level save helpers
bool save_valve_cfg(Preferences &prefs,
                    const char *ns_name,
                    uint16_t v0_act_ms,
                    uint16_t v1_act_ms,
                    bool v0_open_fwd,
                    bool v1_open_fwd,
                    const char *k_v0_act,
                    const char *k_v1_act,
                    const char *k_v0_fwd,
                    const char *k_v1_fwd);

bool save_calibration_cfg(Preferences &prefs,
                          const char *ns_name,
                          uint32_t inv_m_u32,
                          int32_t b_x10,
                          const char *k_inv_m,
                          const char *k_bx10);

bool save_device_name_cfg(Preferences &prefs,
                          const char *ns_name,
                          const char *name,
                          const char *k_name);
						  
						  
bool load_valve_cfg(Preferences &prefs,
                    uint16_t &v0_act_ms,
                    uint16_t &v1_act_ms,
                    bool &v0_open_fwd,
                    bool &v1_open_fwd,
                    const char *k_v0_act,
                    const char *k_v1_act,
                    const char *k_v0_fwd,
                    const char *k_v1_fwd);

bool load_calibration_cfg(Preferences &prefs,
                          uint32_t &inv_m_u32,
                          int32_t &b_x10,
                          const char *k_inv_m,
                          const char *k_bx10);