#pragma once

#include <cstdint>

// WuK SITL Global Variables
// These variables allow ArduCopter FC to communicate morph state to SITL
// Defined in libraries/SITL/WuK_SITL_Globals.cpp, used in multiple targets

extern float g_wuk_target_angle;      // Target morph angle (0-90 degrees)
extern bool g_wuk_morph_active;       // Whether morphing is currently active
extern uint32_t g_wuk_last_update_ms; // Last update timestamp
