// This sketch use advanced Ultra Low Power techniques
// for this it disable all unneeded peripherals during sleep mode, including
// USB Management, this mean you won't be able to upload anymore if the node is sleeping
// when wake up, Lora transmission is approx 3s (including receive windows) this means
// that you have 3 seconds windows to upload, so unless you're lucky, it's almost impossible
// to sync Arduino compilation and upload. But to avoid this, you can press the node button
// for more than 2s, then the led will yellow blink quickly for 60s, letting you time to upload 

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

#define PORT_SETUP         1
#define PORT_BTN_PRESS    10
#define PORT_BTN_RELEASE  11
#define PORT_MOTION_START 20
#define PORT_MOTION_END   21
#define PORT_WATCHDOG     30
#define PORT_INTERVAL     31
#define PORT_LORA         32
#define PORT_TEMPERATURE  33

// Interval between send in seconds, so 300s = 5min
#define CONFIG_INTERVAL ((uint32_t)300)

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);
TheThingsNode *node;
CayenneLPP lpp(24);

uint8_t fport = PORT_SETUP; // LoRaWAN port used 

char buf[24]; // For printf 

void sendData(uint8_t port=PORT_SETUP, uint32_t duration=0);

// This is called on each interval we defined so mainly
// this is where we need to do our job
void interval(uint8_t wakeReason)
{
  uint8_t fport = PORT_INTERVAL;
  snprintf_P(buf, sizeof(buf), PSTR("-- SEND: INTERVAL 0x%02X"), wakeReason);
  debugSerial.println(buf);

  if (wakeReason&TTN_WAKE_LORA) 
  {
    fport = PORT_LORA;
  }

  sendData(fport);
}

// This is called on each wake, every 8S or by an sensor/button interrupt
// if it's watchdog, we mainly do nothing (core IRQ, we don't have choice)
// but if it's an interupt it will ne bone ine loop
void wake(uint8_t wakeReason)
{
  ttn_color ledcolor = TTN_BLACK;
  uint8_t ledblink = 0 ;

  snprintf_P(buf, sizeof(buf), PSTR("-- WAKE 0x%02X"), wakeReason);
  debugSerial.println(buf);

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
    debugSerial.print(F(" LoRa RNxxxx"));
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

  // Just if you want to see this IRQ with a LED
  // just uncomment, but take care, not LOW power
  //while (ledblink--) {
  //  node->setColor(ledcolor);
  //  delay(50);
  //  node->setColor(TTN_BLACK);
  //  delay(333);
  //}
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
  uint32_t timepressed = (uint32_t) duration;

  snprintf_P(buf, sizeof(buf), PSTR("-- SEND: BUTTON %d ms"), timepressed);
  debugSerial.println(buf);

  // If button was pressed for more then 2 seconds
  if (timepressed > 2000) {
    // blink yellow led for 60 seconds
    // this will let us to upload new sketch if needed
    for (uint8_t i=0 ; i<60 ; i++) {
      node->setColor(TTN_YELLOW);
      delay(30);
      node->setColor(TTN_BLACK);
      delay(470);
    }
  }

  // then send data
  sendData(PORT_BTN_RELEASE, timepressed);
}

void sendData(uint8_t port, uint32_t value)
{
  uint16_t volt;

  // Wake RN2483 
  ttn.wake();

  // Prepare cayenne payload
  lpp.reset();

  // Read battery voltage
  volt = node->getBattery();
  snprintf_P(buf, sizeof(buf), PSTR("Bat:\t %dmV"), volt);
  debugSerial.println(buf);
  lpp.addAnalogInput(4, volt/1000.0);

  // This one is usefull when battery < 2.5V  below reference ADC 2.52V
  // because in this case reading are wrong, but you can use it 
  // as soon as VCC < 3.3V, 
  // when above 3.3V, since regulator fix 3.3V you should read 3300mV
  volt = node->getVcc();
  snprintf_P(buf, sizeof(buf), PSTR("Vcc:\t %dmV"), volt);
  debugSerial.println(buf);
  lpp.addAnalogInput(5, volt/1000.0);

  // Read value returned by RN2483 module
  volt = ttn.getVDD();
  snprintf_P(buf, sizeof(buf), PSTR("Vrn:\t %dmV"), volt);
  debugSerial.println(buf);
  lpp.addAnalogInput(6, volt/1000.0);

  // If button pressed, send press duration
  // please myDeviceCayenne add counter value type to
  // avoid us using analog values to send counters
  if (value) {
    snprintf_P(buf, sizeof(buf), PSTR("Btn:\t %dms"), value);
    debugSerial.println(buf);
	  lpp.addAnalogInput(10, value/1000.0);
  }

  node->setColor(TTN_BLUE);
  // Send data
  //ttn.sendBytes(lpp.getBuffer(), lpp.getSize(), port);
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

  // Test sensors 
  node->showStatus();

  debugSerial.println(F("-- TTN: STATUS"));
  ttn.showStatus();

  // Each interval (with watchdog)
  //node->configInterval(true, CONFIG_INTERVAL*1000); 

  // Each interval (with Lora Module and Serial IRQ)
  // Take care this one need to be called after any
  // first call to ttn.* so object has been instancied
  // if not &ttn will be null and watchdog will wake us
  node->configInterval(&ttn, CONFIG_INTERVAL*1000); 

  // Magenta during join, is joined then green else red
  debugSerial.println(F("-- TTN: JOIN"));
  node->setColor(TTN_BLUE);
  if (ttn.join(appEui, appKey)) {
    node->setColor(TTN_GREEN);
  } else {
    node->setColor(TTN_RED);
  }

  debugSerial.println(F("-- SEND: SETUP"));
  sendData(PORT_SETUP);

  // Enable sleep even connected with USB cable
  node->configUSB(true);
}

void loop()
{
  node->loop();
}
