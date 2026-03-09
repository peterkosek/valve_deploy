#include "prefs_helpers.h"

bool pref_put_ushort(Preferences &prefs, const char *key, uint16_t value) {
  return prefs.putUShort(key, value) == sizeof(uint16_t);
}

bool pref_put_uint(Preferences &prefs, const char *key, uint32_t value) {
  return prefs.putUInt(key, value) == sizeof(uint32_t);
}

bool pref_put_int(Preferences &prefs, const char *key, int32_t value) {
  return prefs.putInt(key, value) == sizeof(int32_t);
}

bool pref_put_bool(Preferences &prefs, const char *key, bool value) {
  return prefs.putBool(key, value);
}

bool pref_put_string(Preferences &prefs, const char *key, const char *value) {
  return prefs.putString(key, value) > 0;
}

bool save_valve_cfg(Preferences &prefs,
                    const char *ns_name,
                    uint16_t v0_act_ms,
                    uint16_t v1_act_ms,
                    bool v0_open_fwd,
                    bool v1_open_fwd,
                    const char *k_v0_act,
                    const char *k_v1_act,
                    const char *k_v0_fwd,
                    const char *k_v1_fwd) {
  if (!prefs.begin(ns_name, false)) return false;

  bool ok = true;
  ok &= pref_put_ushort(prefs, k_v0_act, v0_act_ms);
  ok &= pref_put_ushort(prefs, k_v1_act, v1_act_ms);
  ok &= pref_put_bool(prefs, k_v0_fwd, v0_open_fwd);
  ok &= pref_put_bool(prefs, k_v1_fwd, v1_open_fwd);

  prefs.end();
  return ok;
}

bool save_calibration_cfg(Preferences &prefs,
                          const char *ns_name,
                          uint32_t inv_m_u32,
                          int32_t b_x10,
                          const char *k_inv_m,
                          const char *k_bx10) {
  if (!prefs.begin(ns_name, false)) return false;

  bool ok = true;
  ok &= pref_put_uint(prefs, k_inv_m, inv_m_u32);
  ok &= pref_put_int(prefs, k_bx10, b_x10);

  prefs.end();
  return ok;
}

bool save_device_name_cfg(Preferences &prefs,
                          const char *ns_name,
                          const char *name,
                          const char *k_name) {
  if (!prefs.begin(ns_name, false)) return false;

  bool ok = pref_put_string(prefs, k_name, name);

  prefs.end();
  return ok;
}

bool pref_get_uchar(Preferences &prefs, const char *key, uint8_t default_value, uint8_t &out) {
  out = prefs.getUChar(key, default_value);
  return true;
}

bool pref_get_ushort(Preferences &prefs, const char *key, uint16_t default_value, uint16_t &out) {
  out = prefs.getUShort(key, default_value);
  return true;
}

bool pref_get_uint(Preferences &prefs, const char *key, uint32_t default_value, uint32_t &out) {
  out = prefs.getUInt(key, default_value);
  return true;
}

bool pref_get_int(Preferences &prefs, const char *key, int32_t default_value, int32_t &out) {
  out = prefs.getInt(key, default_value);
  return true;
}

bool pref_get_bool(Preferences &prefs, const char *key, bool default_value, bool &out) {
  out = prefs.getBool(key, default_value);
  return true;
}

bool pref_get_string(Preferences &prefs, const char *key, char *buf, size_t buf_size) {
  if (!prefs.isKey(key)) return false;
  prefs.getString(key, buf, buf_size);
  return buf[0] != '\0';
}

bool load_valve_cfg(Preferences &prefs,
                    uint16_t &v0_act_ms,
                    uint16_t &v1_act_ms,
                    bool &v0_open_fwd,
                    bool &v1_open_fwd,
                    const char *k_v0_act,
                    const char *k_v1_act,
                    const char *k_v0_fwd,
                    const char *k_v1_fwd) {
  pref_get_ushort(prefs, k_v0_act, v0_act_ms, v0_act_ms);
  pref_get_ushort(prefs, k_v1_act, v1_act_ms, v1_act_ms);
  pref_get_bool(prefs, k_v0_fwd, v0_open_fwd, v0_open_fwd);
  pref_get_bool(prefs, k_v1_fwd, v1_open_fwd, v1_open_fwd);
  return true;
}

bool load_calibration_cfg(Preferences &prefs,
                          uint32_t &inv_m_u32,
                          int32_t &b_x10,
                          const char *k_inv_m,
                          const char *k_bx10) {
  pref_get_uint(prefs, k_inv_m, inv_m_u32, inv_m_u32);
  pref_get_int(prefs, k_bx10, b_x10, b_x10);
  return true;
}

