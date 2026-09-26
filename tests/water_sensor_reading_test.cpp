// Compile-time regression tests against the actual sketch header (no hardware).
// Compile with -std=c++17 -fsyntax-only -I <water_level_st7789>.
#include "sensor_reading.h"
using S = WaterSampleStatus;
static_assert(classifyWaterSample(-1, 0, 0, 120, false) == S::NoEcho);
static_assert(classifyWaterSample(-1, 0, 0, 120, true) == S::NoEcho);
static_assert(classifyWaterSample(80, 1, 0, 120, true) == S::UnstableEcho);
static_assert(classifyWaterSample(80, 2, 0, 120, true) == S::UnstableEcho);
static_assert(classifyWaterSample(80, 3, 0, 120, true) == S::OK);
static_assert(classifyWaterSample(80, 5, 0, 120, false) == S::NotConfigured);
static_assert(classifyWaterSample(10, 5, 0, 120, false) == S::BelowBlindZone);
static_assert(classifyWaterSample(24.9f, 5, 50, 120, true) == S::BelowBlindZone);
static_assert(classifyWaterSample(25, 5, 0, 120, true) == S::OK);
static_assert(classifyWaterSample(501, 5, -50, 500, true) == S::OutOfRange);
static_assert(classifyWaterSample(500, 5, 0, 500, true) == S::OK);
static_assert(classifyWaterSample(130, 5, 0, 120, true) == S::OK);
static_assert(classifyWaterSample(131, 5, 0, 120, true) == S::OutsideTank);
static_assert(classifyWaterSample(30, 5, -10, 120, true) == S::OutsideTank);
static_assert(classifyWaterSample(100, 5, 5, 120, true) == S::OK);
static_assert(classifyWaterSample(__builtin_nanf(""), 5, 0, 120, true) == S::OutOfRange);
static_assert(classifyWaterSample(80, 5, __builtin_nanf(""), 120, true) == S::OutsideTank);
