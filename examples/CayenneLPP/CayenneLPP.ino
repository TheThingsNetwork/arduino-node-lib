#include <TheThingsNode.h>
#include <CayenneLPP.h>

// Set your AppEUI and AppKey
const char *appEui = "0000000000000000";
const char *appKey = "00000000000000000000000000000000";

#define loraSerial Serial1
#define debugSerial Serial

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915
#define freqPlan REPLACE_ME

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);
TheThingsNode *node;
CayenneLPP lpp(51);

#define PORT_SETUP 1
#define PORT_INTERVAL 2
#define PORT_MOTION 3
#define PORT_BUTTON 4

void setup()
{
  loraSerial.begin(57600);
  debugSerial.begin(9600);

  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000)
    ;

  // Config Node
  node = TheThingsNode::setup();
  node->configLight(true);
  node->configInterval(true, 60000);
  node->configTemperature(true);
  node->onWake(wake);
  node->onInterval(interval);
  node->onSleep(sleep);
  node->onMotionStart(onMotionStart);
  node->onButtonRelease(onButtonRelease);

  // Test sensors and set LED to GREEN if it works
  node->showStatus();
  node->setColor(TTN_GREEN);

  debugSerial.println("-- TTN: STATUS");
  ttn.showStatus();

  debugSerial.println("-- TTN: JOIN");
  ttn.join(appEui, appKey);

  debugSerial.println("-- SEND: SETUP");
  sendData(PORT_SETUP);
}

void loop()
{
  node->loop();
}

void interval()
{
  node->setColor(TTN_BLUE);

  debugSerial.println("-- SEND: INTERVAL");
  sendData(PORT_INTERVAL);
}

void wake()
{
  node->setColor(TTN_GREEN);
}

void sleep()
{
  node->setColor(TTN_BLACK);
}

void onMotionStart()
{
  node->setColor(TTN_BLUE);

  debugSerial.print("-- SEND: MOTION");
  sendData(PORT_MOTION);
}

void onButtonRelease(unsigned long duration)
{
  node->setColor(TTN_BLUE);

  debugSerial.print("-- SEND: BUTTON");
  debugSerial.println(duration);

  sendData(PORT_BUTTON);
}

void sendData(uint8_t port)
{
  printSensors();

  lpp.reset();
  lpp.addDigitalInput(1, node->isButtonPressed());
  lpp.addDigitalInput(2, node->isUSBConnected());
  lpp.addDigitalInput(3, node->isMoving());
  lpp.addAnalogInput(4, node->getBattery()/1000.0);
  lpp.addTemperature(5, node->getTemperatureAsFloat());
  lpp.addLuminosity(6, node->getLight());
  
  float x,y,z;
  node->getAcceleration(&x, &y, &z);
  lpp.addAccelerometer(7, x, y, z);
  
  ttn.sendBytes(lpp.getBuffer(), lpp.getSize());
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

