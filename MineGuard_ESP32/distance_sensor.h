/*
 * MineGuard — Member 2's distance sensor logic
 * Rewritten to be NON-BLOCKING so it can share the ESP32 with
 * Member 1's motor control + web server (see distance_sensor.cpp
 * for why this was necessary).
 */
#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include <Arduino.h>

enum SensorStatus {
  STATUS_OK,
  STATUS_WARNING,
  STATUS_ERROR
};

// Call once from setup()
void beginDistanceSensor();

// Call every pass of loop(). Internally does nothing most of the
// time, and only takes an actual sensor reading roughly every
// 50ms -- so it never slows down the motor web server.
void updateDistanceSensor();

// Read the most recent result (safe to call anytime)
float getDistanceMeters();
SensorStatus getDistanceStatus();
const char *distanceStatusToString(SensorStatus status);

#endif
