// This sketch use advanced Ultra Low Power techniques
// for this it disable all unneeded peripherals during sleep mode, including
// USB Management, this mean you won't be able to upload anymore if the node is sleeping
// when wake up, Lora transmission is approx 3s (including receive windows) this means
// that you have 3 seconds windows to upload, so unless you're lucky, it's almost impossible
// to sync Arduino compilation and upload. But to avoid this, you can press the node button
// for more than 2s, then the led will yellow blink quickly for 60s, letting you time to upload 

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
CayenneLPP lpp(51);

char buf[24]; // For printf 

#define PORT_SETUP 1
#define PORT_INTERVAL 2
#define PORT_MOTION 3
#define PORT_BUTTON 4

// Interval between send in seconds, so 300s = 5min
#define CONFIG_INTERVAL ((uint32_t)300)

void sendData(uint8_t port=PORT_SETUP, uint32_t duration=0);

void setup()
{
  loraSerial.begin(57600);
  debugSerial.begin(9600);

  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000) {
    node->setColor(TTN_RED);
    delay(20);
    node->setColor(TTN_BLACK);
    delay(480);
  };

  // Config Node
  node = TheThingsNode::setup();
  node->configLight(true);
  node->configTemperature(true);
  node->onWake(wake);
  node->onInterval(interval);
  node->onSleep(sleep);
  node->onMotionStart(onMotionStart);
  node->onButtonRelease(onButtonRelease);

  // Test sensors 
  node->showStatus();

  debugSerial.println("-- TTN: STATUS");
  ttn.showStatus();

  // Each interval (with Lora Module and Serial IRQ)
  // Take care this one need to be called after any
  // first call to ttn.* so object has been instancied
  // if not &ttn will be null and watchdog will wake us
  node->configInterval(&ttn, CONFIG_INTERVAL*1000); 

  // Magenta during join, if joined then green else red
  debugSerial.println(F("-- TTN: JOIN"));
  node->setColor(TTN_MAGENTA);
  if (ttn.join(appEui, appKey)) {
    node->setColor(TTN_GREEN);
  } else {
    node->setColor(TTN_RED);
  }

  debugSerial.println("-- SEND: SETUP");
  sendData(PORT_SETUP);

  // Enable sleep even connected with USB cable
  node->configUSB(true);
}

void loop()
{
  node->loop();
}

void interval(uint8_t wakeReason)
{
  snprintf_P(buf, sizeof(buf), PSTR("-- SEND: INTERVAL 0x%02X"), wakeReason);
  debugSerial.println(buf);
  sendData(PORT_INTERVAL);
}

void wake(uint8_t wakeReason)
{
  snprintf_P(buf, sizeof(buf), PSTR("-- WAKE 0x%02X"), wakeReason);
  debugSerial.println(buf);
  node->setColor(TTN_GREEN);
}

void sleep()
{
  node->setColor(TTN_BLACK);
}

void onMotionStart()
{
  node->setColor(TTN_RED);

  debugSerial.print("-- SEND: MOTION");
  sendData(PORT_MOTION);
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
  sendData(PORT_BUTTON, timepressed);
}

void sendData(uint8_t port, uint32_t duration)
{
  uint16_t bat = node->getBattery();

  // when battery < 3.3V (regulator does not regule output, so 
  // we use another method to read VCC of the device and this will 
  // also avoid error if VCC <  ADC reference (2.52V)
  if (bat <= 3300) 
  {
    bat = node->getVcc();
  }

  // Wake RN2483
  ttn.wake();

  printSensors();

  lpp.reset();
  lpp.addDigitalInput(1, node->isButtonPressed());
  lpp.addDigitalInput(2, node->isUSBConnected());
  lpp.addDigitalInput(3, node->isMoving());
  lpp.addAnalogInput(4, bat/1000.0);
  lpp.addTemperature(5, node->getTemperatureAsFloat());
  lpp.addLuminosity(6, node->getLight());
  
  float x,y,z;
  node->getAcceleration(&x, &y, &z);
  lpp.addAccelerometer(7, x, y, z);

  // If button pressed, send press duration
  // please myDeviceCayenne add counter value type to
  // avoid us using analog values to send counters
  if (duration) {
    snprintf_P(buf, sizeof(buf), PSTR("Btn:\t %dms"), duration);
    debugSerial.println(buf);
    lpp.addAnalogInput(10, duration/1000.0);
  }
  
  node->setColor(TTN_BLUE);
  ttn.sendBytes(lpp.getBuffer(), lpp.getSize(), port);
}

void printSensors()
{
  debugSerial.println();
  debugSerial.println("SENSORS:");
  debugSerial.println("--------");
  
  debugSerial.print("LED Colour:\t");
  debugSerial.println(node->colorToString(node->getColor()));

  debugSerial.print("Temperature:\t");
  debugSerial.print(node->getTemperatureAsFloat());
  debugSerial.println("°C");

  debugSerial.print("Light Sensor:\t");
  debugSerial.print(node->getLight());
  debugSerial.println("lux");
  
  debugSerial.print("Moving now:\t");
  if(node->isMoving())
  {
    debugSerial.println("yes");
  }
  else
  {
    debugSerial.println("no");
  }
  
  debugSerial.print("Button pressed:\t");
  if(node->isButtonPressed())
  {
    debugSerial.println("yes");
  }
  else
  {
    debugSerial.println("no");
  }

  debugSerial.print("USB connected:\t");
  if(node->isUSBConnected())
  {
    debugSerial.println("yes");
  }
  else
  {
    debugSerial.println("no");
  }

  debugSerial.print("Battery:\t");
  debugSerial.print(node->getBattery());
  debugSerial.println("mV");

  float x,y,z;
  node->getAcceleration(&x, &y, &z);

  debugSerial.print("Acceleration:\tx=");
  debugSerial.print(x);
  debugSerial.print("g\n\t\ty=");
  debugSerial.print(y);
  debugSerial.print("g\n\t\tz=");
  debugSerial.print(z);
  debugSerial.println("g");

  debugSerial.println("--------");
  debugSerial.println();
  
}

