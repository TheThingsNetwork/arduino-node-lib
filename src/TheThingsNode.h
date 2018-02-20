// Copyright © 2017 The Things Network
// Use of this source code is governed by the MIT license that can be found in the LICENSE file.

#ifndef _THETHINGSNODE_H_
#define _THETHINGSNODE_H_

#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <Hackscribble_MCP9804.h>
#include "TheThingsNetwork.h"

#define TTN_LDR_INPUT 10
#define TTN_LDR_GAIN1 12
#define TTN_LDR_GAIN2 4
#define TTN_RED_LED 13
#define TTN_GREEN_LED 5
#define TTN_BLUE_LED 6
#define TTN_BUTTON 16
#define TTN_LORA_RESET 21 /* Hardware reset pin of LoRa module */
#define TTN_LORA_SERIAL_RX_INT INT2 /* Serial RX Interrupt number (to be waked by RN2483 or RN2903 module) */
#define TTN_VBAT_MEAS_EN A2
#define TTN_VBAT_MEAS 1
#define TTN_TEMPERATURE_SENSOR_ADDRESS 0x18
#define TTN_TEMPERATURE_ALERT 14
#define TTN_ACCELEROMETER_INT2 9

#define TTN_ADDR_ACC 0x1D
#define TTN_DR 5  // active data rate
#define TTN_SR 3  // sleep data rate
#define TTN_SC 4  // sleep delay
#define TTN_MT 4  // 0.063g/LSB
#define TTN_MDC 2 // debounce delay in samples
#define TTN_SYSMOD 0x0B
#define TTN_FF_MT_CFG 0x15
#define TTN_FF_MT_SRC 0x16
#define TTN_FF_MT_THS 0x17
#define TTN_FF_MT_COUNT 0x18
#define TTN_TRANSIENT_CFG 0x1D
#define TTN_TRANSIENT_SRC 0x1E
#define TTN_TRANSIENT_THS 0x1F
#define TTN_TRANSIENT_COUNT 0x20
#define TTN_ASLP_CNT 0x29
#define TTN_CTRL_REG1 0x2A
#define TTN_CTRL_REG2 0x2B
#define TTN_CTRL_REG3 0x2C
#define TTN_CTRL_REG4 0x2D
#define TTN_CTRL_REG5 0x2E

#define TTN_WAKE_TEMPERATURE  0x0001
#define TTN_WAKE_MOTION_START 0x0002
#define TTN_WAKE_MOTION_STOP  0x0004
#define TTN_WAKE_BTN_PRESS    0x0008
#define TTN_WAKE_BTN_RELEASE  0x0010
#define TTN_WAKE_LORA         0x0020
#define TTN_WAKE_WATCHDOG     0x0040
#define TTN_WAKE_INTERVAL     0x0080

#define TTN_WAKE_ANY          ( TTN_WAKE_TEMPERATURE | \
                                TTN_WAKE_MOTION_START | TTN_WAKE_MOTION_STOP | \
                                TTN_WAKE_BTN_PRESS | TTN_WAKE_BTN_RELEASE | \
                                TTN_WAKE_LORA | \
                                TTN_WAKE_WATCHDOG )

enum ttn_color : byte
{
  TTN_RED,
  TTN_GREEN,
  TTN_BLUE,
  TTN_YELLOW,
  TTN_CYAN,
  TTN_MAGENTA,
  TTN_WHITE,
  TTN_BLACK
};

class TheThingsNode
{
private:
  TheThingsNode();
  TheThingsNode(TheThingsNode const &);
  void operator=(TheThingsNode const &);

  TheThingsNetwork *ttn;

  bool intervalEnabled;
  uint32_t intervalMs;
  uint32_t intervalSince;
  bool lightEnabled;
  uint8_t lightGain;
  bool temperatureEnabled;
  bool temperatureSleep;
  bool motionStarted;
  unsigned long motionStartedAt;
  bool motionEnabled;
  bool buttonEnabled;
  bool buttonPressed;
  unsigned long buttonPressedAt;
  bool wasUSBDisconnected;
  bool USBDeepSleep;
  bool wdtStarted;

  void (*wakeCallback)(uint8_t wakeStatus);
  void (*sleepCallback)(void);
  void (*temperatureCallback)(void);
  void (*motionStartCallback)(void);
  void (*motionStopCallback)(unsigned long duration);
  void (*buttonPressCallback)(void);
  void (*buttonReleaseCallback)(unsigned long duration);
  void (*intervalCallback)(void);

  void wakeTemperature();
  void sleepTemperature();
  void wakeMotion();
  void sleepMotion();
  void writeMotion(unsigned char REG_ADDRESS, unsigned char DATA);
  uint8_t readMotion(unsigned char REG_ADDRESS);
  void WDT_start();
  void WDT_stop();
  void deepSleep(void);

public:
  static TheThingsNode *setup()
  {
    static TheThingsNode node;
    return &node;
  };

  void onWake(void (*callback)(uint8_t wakeStatus));
  void loop();
  void onSleep(void (*callback)(void));

  void showStatus();

  void onInterval(void (*callback)(void));
  void configInterval(bool enabled, uint32_t ms);
  void configInterval(bool enabled);
  void configInterval(TheThingsNetwork *ttn, uint32_t ms);

  void configLight(bool enabled, uint8_t gain);
  void configLight(bool enabled);
  uint16_t getLight();

  int8_t getTemperatureAsInt();
  float getTemperatureAsFloat();
  void configTemperature(bool enabled, MCP9804_Resolution resolution, int8_t lower, int8_t upper, int8_t critical, MCP9804_Hysteresis hysteresis);
  void configTemperature(bool enabled, MCP9804_Resolution resolution);
  void configTemperature(bool enabled);
  void onTemperature(void (*callback)(void));
  bool hasTemperatureAlert();

  void onMotionStart(void (*callback)(void));
  void onMotionStop(void (*callback)(unsigned long duration));
  bool isMoving();
  void configMotion(bool enabled);
  void getAcceleration(float *x, float *y, float *z);

  void onButtonPress(void (*callback)(void));
  void onButtonRelease(void (*callback)(unsigned long duration));
  bool isButtonPressed();
  void configButton(bool enabled);

  void setColor(ttn_color color);
  void setRGB(bool red = false, bool green = false, bool blue = false);
  void setRed(bool on = true);
  void setGreen(bool on = true);
  void setBlue(bool on = true);
  ttn_color getColor();
  String colorToString(ttn_color color);
  bool getRed();
  bool getGreen();
  bool getBlue();

  bool isUSBConnected();
  void configUSB(bool deepSleep);

  uint16_t getBattery();
};

#endif
