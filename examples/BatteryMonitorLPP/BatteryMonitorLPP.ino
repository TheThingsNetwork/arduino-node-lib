// **********************************************************************************
// Ultra Low Power BatteryMonitorLPP for TheThingsNode device
// **********************************************************************************
//
// This program just send lora packet containing Battery info to myDevices portal
// if button is pressed the batterey packet is send and the push button duration
// is also added 
//
// This example also take into account of setting Lora Module to sleep mode this 
// allow to reduce consumption in sleep mode from 3.2mA to 280µA
//
// Written by Charles-Henri Hallard (CH2i) 
//
// History : V1.00 2018-02-11 - First release based on TTN CayenneLPP example code
//           V1.01 2018-02-12 - Added Hard reset of Lora module at startup
//                            - Fixed issue https://github.com/TheThingsNetwork/arduino-node-lib/issues/9
//                            - Enable Low Power of Lora Module
//
// **********************************************************************************

#include <TheThingsNode.h>
#include <CayenneLPP.h>

// Set your AppEUI and AppKey
const char *appEui = "0000000000000000";
const char *appKey = "00000000000000000000000000000000";

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan REPLACE_ME

#define loraSerial Serial1
#define debugSerial Serial

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);
TheThingsNode *node;
CayenneLPP lpp(16);

#define PORT_SETUP 1
#define PORT_INTERVAL 2
#define PORT_MOTION 3
#define PORT_BUTTON 4

// Interval between send in seconds, so 300s = 5min
#define CONFIG_INTERVAL 300

void sendData(uint8_t port=PORT_SETUP, uint32_t duration=0);

// This is called on each interval we defined so mainly
// this is where we need to do our job
void interval()
{
  node->setColor(TTN_BLUE);
  debugSerial.println(F("-- SEND: INTERVAL"));
  sendData(PORT_INTERVAL);
}

// This is called on each wake, every 8S or by an sensor/button interrupt
// if it's watchdog, we mainly do nothing (core IRQ, we don't have choice)
// but if it's an interupt it will ne bone ine loop
void wake()
{
  debugSerial.println(F("-- WAKE"));

  // Just if you want to see this IRQ with a LED, remove for LOW power
  //node->setColor(TTN_GREEN);
  //delay(50);
  //node->setColor(TTN_BLACK);
  //delay(100);
}

void sleep()
{
  node->setColor(TTN_BLACK);

  // Just in case, disable all sensors
  // this couldn't hurt except time to do job
  node->configMotion(false);
  node->configLight(false);
  node->configTemperature(false);
}

void onButtonRelease(unsigned long duration)
{
  uint16_t adc_value;
  uint32_t timepressed = (uint32_t) duration;

  node->setColor(TTN_BLUE);

  debugSerial.print(F("-- SEND: BUTTON "));
  debugSerial.print(timepressed);
  debugSerial.println(F(" ms"));

  sendData(PORT_BUTTON, timepressed);
}

void sendData(uint8_t port, uint32_t value)
{
  // Wake RN2483 
  ttn.wake();

  // Read battery voltage
  uint16_t vbat = node->getBattery();

  debugSerial.print(F("Battery:\t"));
  debugSerial.print(vbat);
  debugSerial.println(F("mV"));

  // Just send battery voltage 
  lpp.reset();
  lpp.addAnalogInput(4, vbat/1000.0);

  // If button pressed, send press duration
  // please myDeviceCayenne add counter value type to
  // avoid us using analog values to send counters
  if (port == PORT_BUTTON) {
	  debugSerial.print(F("Button:\t"));
	  debugSerial.print(value);
	  debugSerial.println(F("ms"));
	  lpp.addAnalogInput(7, value/1000.0);
  }

  ttn.sendBytes(lpp.getBuffer(), lpp.getSize(), port);

  // Set RN2483 to sleep mode
  ttn.sleep( (uint32_t) (CONFIG_INTERVAL*1000) );

  // This one is not optionnal, remove it
  // and say bye bye to RN2983 sleep mode
  delay(50);
}

void setup()
{
  loraSerial.begin(57600);
  debugSerial.begin(9600);

  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000)
    ;

  // Config Node, Disable all sensors 
  // Check node schematics here
  // https://github.com/TheThingsProducts/node
  node = TheThingsNode::setup();

  // Each interval
  node->configInterval(true, CONFIG_INTERVAL*1000); 
  node->onWake(wake);
  node->onInterval(interval);
  node->onSleep(sleep);

  // We monitor just button release
  node->onButtonRelease(onButtonRelease);

  // Test sensors and set LED to GREEN if it works
  node->showStatus();
  node->setColor(TTN_GREEN);

  debugSerial.println(F("-- TTN: STATUS"));
  ttn.showStatus();

  debugSerial.println(F("-- TTN: JOIN"));
  ttn.join(appEui, appKey);

  debugSerial.println(F("-- SEND: SETUP"));
  sendData(PORT_SETUP);
}

void loop()
{
  node->loop();
}
