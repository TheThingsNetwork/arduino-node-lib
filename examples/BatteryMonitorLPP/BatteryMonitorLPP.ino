#include <TheThingsNode.h>
#include <TheThingsNetwork.h>
#include <CayenneLPP.h>

// Set your AppEUI and AppKey
const char *appEui = "0000000000000000";
const char *appKey = "00000000000000000000000000000000";

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan REPLACE_ME

#define loraSerial Serial1
#define debugSerial Serial

#define PORT_SETUP 1
#define PORT_INTERVAL 2
#define PORT_MOTION 3
#define PORT_BUTTON 4

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);
TheThingsNode *node;
CayenneLPP lpp(16);

// Interval between send in seconds, so 300s = 5min
#define CONFIG_INTERVAL ((uint32_t)300)

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
void wake(uint8_t wakeReason)
{
  debugSerial.print(F("-- WAKE 0x"));
  debugSerial.print(wakeReason, HEX);
  uint8_t ledcolor = TTN_BLACK;
  uint8_t ledblink = 0 ;

  if (wakeReason&(TTN_WAKE_WATCHDOG|TTN_WAKE_INTERVAL)) {
    ledcolor = TTN_YELLOW;
    ledblink = 1;
    if (wakeReason&TTN_WAKE_WATCHDOG) {
      debugSerial.print(F(" Watchdog"));
    }
    if (wakeReason&TTN_WAKE_INTERVAL) {
      debugSerial.print(F(" INTERVAL"));
      ledblink++;
    }
  }

  if (wakeReason&TTN_WAKE_LORA) {
    debugSerial.print(F(" LoRa"));
    ledblink = 1;
    ledcolor = TTN_GREEN;
  }

  if (wakeReason&TTN_WAKE_BTN_PRESS) {
    debugSerial.print(F(" PRESS"));
  }
  
  if (wakeReason&TTN_WAKE_BTN_RELEASE) {
    debugSerial.print(F(" RELEASE"));
  }

  if (wakeReason&TTN_WAKE_MOTION_START) {
    ledblink = 1;
    ledcolor = TTN_RED;
    debugSerial.print(F(" MOTION_START"));
  }
  if (wakeReason&TTN_WAKE_MOTION_STOP)  {
    ledblink = 2;
    ledcolor = TTN_RED;
    debugSerial.print(F(" MOTION_STOP"));
  }

  if (wakeReason&TTN_WAKE_TEMPERATURE) {
    debugSerial.print(F(" TEMPERATURE"));
  }
  
  debugSerial.println();

  // Just if you want to see this IRQ with a LED, remove for LOW power
  /*
  while (ledblink--) {
    node->setColor(ledcolor);
    delay(50);
    node->setColor(TTN_BLACK);
    delay(333);
  }
  */

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

  debugSerial.print(F("Bat: "));
  debugSerial.print(vbat);
  debugSerial.println(F("mV"));

  // This one is usefull when battery < 2.5V  below reference ADC 2.52V
  // because in this case reading are wrong, but you can use it 
  // as soon as VCC < 3.3V, 
  // when above 3.3V; since regulator fix 3.3V you should read 3300mV
  uint16_t vcc = node->getVcc();
  debugSerial.print(F("Vcc: "));
  debugSerial.print(vcc);
  debugSerial.println(F("mV"));

/*
  uint16_t vrn = ttn.getVDD();
  debugSerial.print(F("VRN: "));
  debugSerial.print(vrn);
  debugSerial.println(F("mV"));
  */

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
}

void setup()
{
  loraSerial.begin(57600);
  debugSerial.begin(9600);

  // Wait a maximum of 5s for Serial Monitor
  while (!debugSerial && millis() < 5000) {
    node->setColor(TTN_RED);
    delay(20);
    node->setColor(TTN_BLACK);
    delay(480);
  };

  // Config Node, Disable all sensors 
  // Check node schematics here
  // https://github.com/TheThingsProducts/node
  node = TheThingsNode::setup();

  node->onWake(wake);
  node->onInterval(interval);
  node->onSleep(sleep);

  // We monitor just button release
  node->onButtonRelease(onButtonRelease);

  // Test sensors and set LED to GREEN if it works
  node->showStatus();

  debugSerial.println(F("-- TTN: STATUS"));
  ttn.showStatus();

  // Each interval (with watchdog)
  //node->configInterval(true, CONFIG_INTERVAL*1000); 

  // Each interval (with Lora Module and Serial IRQ)
  // Take care this one need to be called after any
  // first call to ttn.* so object has been instancied
  // if not &ttn will be null and watchdog
  node->configInterval(&ttn, CONFIG_INTERVAL*1000); 

  debugSerial.println(F("-- TTN: JOIN"));
  node->setColor(TTN_MAGENTA);
  if (ttn.join(appEui, appKey)) {
    node->setColor(TTN_GREEN);
  } else {
    node->setColor(TTN_RED);
  }

  debugSerial.println(F("-- SEND: SETUP"));
  sendData(PORT_SETUP);

  // Enable USB deepsleep
  //node->configUSB(true);
}

void loop()
{
  node->loop();
}
