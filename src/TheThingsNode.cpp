// Copyright © 2017 The Things Network
// Use of this source code is governed by the MIT license that can be found in the LICENSE file.

#include <Arduino.h>
#include <TheThingsNode.h>

Hackscribble_MCP9804 TTN_TEMPERATURE_SENSOR(TTN_TEMPERATURE_SENSOR_ADDRESS);

volatile uint16_t wakeStatus;
volatile uint32_t TTN_INTERVAL = 0;
volatile uint8_t  adcIRQCnt;

void TTN_TEMPERATURE_FN()
{
  wakeStatus |= TTN_WAKE_TEMPERATURE;
  TTN_TEMPERATURE_SENSOR.clearAlert();
}

void TTN_MOTION_FN()
{
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(TTN_ACCELEROMETER_INT2));
  // Reset according bits
  wakeStatus &= ~ (TTN_WAKE_MOTION_START|TTN_WAKE_MOTION_STOP);

  if (trigger == RISING)
  {
    wakeStatus |= TTN_WAKE_MOTION_START;
  }
  else if (trigger == FALLING)
  {
    wakeStatus |= TTN_WAKE_MOTION_STOP;
  }
}

void TTN_BUTTON_FN()
{
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(TTN_BUTTON));
  if (trigger == FALLING)
  {
    wakeStatus |= TTN_WAKE_BTN_PRESS;
  }

  // Be sure main loop ACK press button before rising the release 
  if (trigger == RISING /*&& !(wakeStatus&TTN_WAKE_BTN_PRESS)*/) 
  {
    wakeStatus |= TTN_WAKE_BTN_RELEASE;
  }
}

void TTN_SERIAL_LORA_FN()
{
  // Just used to wake up we can now mask this
  // interrupt because serial will become verbose
  wakeStatus |= TTN_WAKE_LORA;
  EIMSK &= ~(1 << INT2);
}

ISR(WDT_vect)
{
  wakeStatus |= TTN_WAKE_WATCHDOG;
  TTN_INTERVAL = TTN_INTERVAL + 8000;
}

ISR(ADC_vect)  
{
  // Increment ADC counter
  adcIRQCnt++;
}




/******************************************************************************
 * PUBLIC
 *****************************************************************************/

void TheThingsNode::onWake(void (*callback)(uint8_t wakeStatus))
{
  this->wakeCallback = callback;
}

void TheThingsNode::loop()
{
  // USB is connected and last time we checked it wasn't or visa versa
  if (this->isUSBConnected() == this->wasUSBDisconnected)
  {
    this->wasUSBDisconnected = !this->isUSBConnected();

    // It is no longer connected
    if (this->wasUSBDisconnected)
    {
      Serial.end();
      WDT_start();
    }
    else
    {

      // Stop watchdog unless deep sleep is enabled for USB
      if (!this->USBDeepSleep)
      {
        WDT_stop();
      }

      // Restore communication
      if (Serial)
      {
        Serial.begin(9600);
      }
    }
  }

  if (TTN_INTERVAL >= this->intervalMs) {
    wakeStatus |= TTN_WAKE_INTERVAL;
  }

  if (this->wakeCallback)
  {
    this->wakeCallback(wakeStatus);
  }

  if (wakeStatus & TTN_WAKE_BTN_PRESS)
  {
    if (!this->buttonPressed)
    {
      this->buttonPressedAt = millis();
      if (this->buttonEnabled && this->buttonPressCallback)
      {
        this->buttonPressCallback();
      }
      this->buttonPressed = true;
    }
    // ACK our interrupt
    wakeStatus &= ~TTN_WAKE_BTN_PRESS;
  } 
  // At least one loop instance between press and release so else if here
  else if (wakeStatus & TTN_WAKE_BTN_RELEASE)
  {
    if (this->buttonPressed)
    {
      if (this->buttonEnabled && this->buttonReleaseCallback)
      {
        this->buttonReleaseCallback(millis() - this->buttonPressedAt);
      }
      this->buttonPressed = false;
    }
    // ACK our interrupt
    wakeStatus &= ~TTN_WAKE_BTN_RELEASE;
  }

  if (wakeStatus & TTN_WAKE_MOTION_START)
  {
    if (!this->motionStarted)
    {
      this->motionStartedAt = millis();
      if (this->motionEnabled && this->motionStartCallback)
      {
        this->motionStartCallback();
      }
      this->motionStarted = true;
    }
    // ACK our interrupt
    wakeStatus &= ~TTN_WAKE_MOTION_START;
  }

  if (wakeStatus & TTN_WAKE_MOTION_STOP)
  {
    if (this->motionStarted)
    {
      if (this->motionEnabled && this->motionStopCallback)
      {
        this->motionStopCallback(millis() - this->motionStartedAt);
      }
      this->motionStarted = false;
    }
    // ACK our interrupt
    wakeStatus &= ~TTN_WAKE_MOTION_STOP;
  }

  if (wakeStatus & TTN_WAKE_TEMPERATURE)
  {
    if (this->temperatureEnabled && this->temperatureCallback)
    {
      this->temperatureCallback();
    }
    // ACK our interrupt
    wakeStatus &= ~TTN_WAKE_TEMPERATURE;
  }

  if (wakeStatus & TTN_WAKE_INTERVAL)  
  {
    if (this->intervalEnabled && this->intervalCallback)
    {
      this->intervalCallback();
    }
    // Ack our watchdog and interval interrupt
    wakeStatus &= ~(TTN_WAKE_WATCHDOG|TTN_WAKE_INTERVAL);
    TTN_INTERVAL = 0;
  }

  if (wakeStatus & TTN_WAKE_LORA)
  {
    if (this->pttn && this->intervalCallback)
    {
      this->intervalCallback();
    }
    // ACK our interrupt
    wakeStatus &= ~TTN_WAKE_LORA;
  }

  if (this->sleepCallback)
  {
    this->sleepCallback();
  }

  // If button pushed manage loop faster 
  uint16_t dly = this->buttonPressed ? 10 : 100;

  if (this->isUSBConnected() && !this->USBDeepSleep)
  {

    while (!(wakeStatus&TTN_WAKE_ANY) && TTN_INTERVAL < this->intervalMs)
    {
      delay(dly);
      TTN_INTERVAL = TTN_INTERVAL + dly;
    }
  }
  else
  {
    // Don't go to sleep mode while button is still pressed
    // because if so, timer of ms will be stopped and duration
    // of pressed button will not work, this acting like a debounce
    if (this->buttonPressed) {
      delay(dly);
      TTN_INTERVAL = TTN_INTERVAL + dly;

    } else {
      Serial.flush();
      deepSleep();
    }
  }
}

void TheThingsNode::onSleep(void (*callback)(void))
{
  this->sleepCallback = callback;
}

void TheThingsNode::showStatus()
{
  Serial.print(F("Light: "));
  if (this->lightEnabled)
  {
    Serial.println(String(getLight()));
  }
  else
  {
    Serial.println(F("disabled"));
  }
  Serial.print(F("Temperature: "));
  Serial.println(String(getTemperatureAsFloat()) + F(" C"));
  Serial.print(F("Temperature alert: "));
  if (this->temperatureEnabled)
  {
    Serial.println(hasTemperatureAlert() ? F("Yes") : F("No"));
  }
  else
  {
    Serial.println(F("disabled"));
  }
  Serial.print(F("Moving: "));
  if (this->motionEnabled)
  {
    Serial.println(isMoving() ? F("Yes") : F("No"));
  }
  else
  {
    Serial.println(F("disabled"));
  }
  Serial.print(F("Button pressed: "));
  if (this->buttonEnabled)
  {
    Serial.println(isButtonPressed() ? F("Yes") : F("No"));
  }
  else
  {
    Serial.println(F("disabled"));
  }
  Serial.print(F("Color: "));
  Serial.println(colorToString(getColor()));
  Serial.print(F("USB connected: "));
  Serial.println(isUSBConnected() ? F("Yes") : F("No"));
  Serial.print(F("Battery voltage: "));
  Serial.println(String(getBattery()) + F(" MV"));
}

/******************************************************************************
 * INTERVAL
 */

void TheThingsNode::configInterval(bool enabled, uint32_t ms)
{
  this->intervalMs = ms;
  configInterval(enabled);
}

void TheThingsNode::configInterval(TheThingsNetwork *ttn, uint32_t ms)
{
  this->intervalMs = ms;
  this->pttn = ttn;
  this->intervalEnabled = true;
}

void TheThingsNode::configInterval(bool enabled)
{
  this->pttn = NULL;
  this->intervalEnabled = enabled;
}

void TheThingsNode::onInterval(void (*callback)(void))
{
  this->intervalCallback = callback;

  configInterval(true);
}

/******************************************************************************
 * LIGHT
 */

void TheThingsNode::configLight(bool enabled, uint8_t gain)
{
  this->lightGain = gain;

  configLight(enabled);
}

void TheThingsNode::configLight(bool enabled)
{
  if (enabled == this->lightEnabled)
  {
    return;
  }

  // Ok be sure to set it to low power mode
  if (!enabled)
  {
    digitalWrite(TTN_LDR_GAIN1, LOW);
    digitalWrite(TTN_LDR_GAIN2, LOW);
    // Just to be sure, see datasheet, at least 1.5ms to enable Low Power
    delay(2);
  }

  this->lightEnabled = enabled;
}

uint16_t TheThingsNode::getLight()
{
  uint16_t value = 0;

  if ( this->lightEnabled)
  {
    switch (this->lightGain)
    {
    case 0:
      digitalWrite(TTN_LDR_GAIN1, LOW);
      digitalWrite(TTN_LDR_GAIN2, LOW);
      break;
    case 1:
      digitalWrite(TTN_LDR_GAIN1, HIGH);
      digitalWrite(TTN_LDR_GAIN2, LOW);
      break;
    case 2:
      digitalWrite(TTN_LDR_GAIN1, LOW);
      digitalWrite(TTN_LDR_GAIN2, HIGH);
      break;
    case 3:
      digitalWrite(TTN_LDR_GAIN1, HIGH);
      digitalWrite(TTN_LDR_GAIN2, HIGH);
      break;
    }
    // Wait to settle
    delay(1);

    // Read value
    value = analogRead(TTN_LDR_INPUT);

    // Go back to sleep mode
    digitalWrite(TTN_LDR_GAIN1, LOW);
    digitalWrite(TTN_LDR_GAIN2, LOW);
    // Just to be sure, see datasheet, at least 1.5ms to enable Low Power
    delay(2);
  }

  return value;
}

/******************************************************************************
 * TEMPERATURE
 */

void TheThingsNode::configTemperature(bool enabled, MCP9804_Resolution resolution, int8_t lower, int8_t upper, int8_t critical, MCP9804_Hysteresis hysteresis)
{
  wakeTemperature();
  TTN_TEMPERATURE_SENSOR.setTLOWER(lower);
  TTN_TEMPERATURE_SENSOR.setTUPPER(upper);
  TTN_TEMPERATURE_SENSOR.setTCRIT(critical);
  TTN_TEMPERATURE_SENSOR.setHysteresis(hysteresis);
  configTemperature(enabled, resolution);
  sleepTemperature();
}

void TheThingsNode::configTemperature(bool enabled, MCP9804_Resolution resolution)
{
  wakeTemperature();
  TTN_TEMPERATURE_SENSOR.setResolution(resolution);
  configTemperature(enabled);
  sleepTemperature();
}

void TheThingsNode::configTemperature(bool enabled)
{
  if (this->temperatureEnabled == enabled)
  {
    return;
  }

  wakeTemperature();

  if (enabled)
  {
    TTN_TEMPERATURE_SENSOR.configureAlert(true);
    attachPCINT(digitalPinToPCINT(TTN_TEMPERATURE_ALERT), TTN_TEMPERATURE_FN, FALLING);

    if (hasTemperatureAlert() && this->temperatureCallback)
    {
      this->temperatureCallback();
    }
  }
  else
  {
    TTN_TEMPERATURE_SENSOR.configureAlert(false);

    detachPCINT(digitalPinToPCINT(TTN_TEMPERATURE_ALERT));

    sleepTemperature();
  }

  this->temperatureEnabled = enabled;
}

void TheThingsNode::onTemperature(void (*callback)(void))
{
  this->temperatureCallback = callback;

  if (this->temperatureEnabled)
  {
    if (hasTemperatureAlert())
    {
      callback();
    }
  }
  else
  {
    configTemperature(true);
  }
}

int8_t TheThingsNode::getTemperatureAsInt()
{
  wakeTemperature();
  int8_t value = TTN_TEMPERATURE_SENSOR.getTAInteger();
  sleepTemperature();
  return value;
}

float TheThingsNode::getTemperatureAsFloat()
{
  wakeTemperature();
  float value = TTN_TEMPERATURE_SENSOR.getTA();
  sleepTemperature();
  return value;
}

bool TheThingsNode::hasTemperatureAlert()
{
  // From testing, it appears that if Ta is already out of range of the defined thresholds at startup, the Alert output is not activated.
  int8_t value = getTemperatureAsInt();
  return ((value < TTN_TEMPERATURE_SENSOR.getTLOWER()) || (value > TTN_TEMPERATURE_SENSOR.getTUPPER()) || (value > TTN_TEMPERATURE_SENSOR.getTCRIT()));
}

/******************************************************************************
 * MOTION
 */

void TheThingsNode::configMotion(bool enabled)
{
  if (this->motionEnabled == enabled)
  {
    return;
  }

  if (enabled)
  {
    wakeMotion();
    attachPCINT(digitalPinToPCINT(TTN_ACCELEROMETER_INT2), TTN_MOTION_FN, CHANGE);
  }
  else
  {
    detachPCINT(digitalPinToPCINT(TTN_ACCELEROMETER_INT2));
    sleepMotion();
  }

  this->motionEnabled = enabled;
}

void TheThingsNode::onMotionStart(void (*callback)(void))
{
  configMotion(true);

  this->motionStartCallback = callback;
}

void TheThingsNode::onMotionStop(void (*callback)(unsigned long duration))
{
  configMotion(true);

  this->motionStopCallback = callback;
}

bool TheThingsNode::isMoving()
{
  return this->motionStarted;
}

/******************************************************************************
 * BUTTON
 */

void TheThingsNode::configButton(bool enabled)
{
  if (this->buttonEnabled == enabled)
  {
    return;
  }

  if (enabled)
  {
    attachPCINT(digitalPinToPCINT(TTN_BUTTON), TTN_BUTTON_FN, CHANGE);
  }
  else
  {
    detachPCINT(digitalPinToPCINT(TTN_BUTTON));
  }

  this->buttonEnabled = enabled;
}

void TheThingsNode::onButtonPress(void (*callback)(void))
{
  this->buttonPressCallback = callback;

  configButton(true);
}

void TheThingsNode::onButtonRelease(void (*callback)(unsigned long duration))
{
  this->buttonReleaseCallback = callback;

  configButton(true);
}

bool TheThingsNode::isButtonPressed()
{
  return this->buttonPressed;
}

/******************************************************************************
 * LED
 */

bool TheThingsNode::getRed()
{
  return (digitalRead(TTN_RED_LED) == LOW);
}

bool TheThingsNode::getGreen()
{
  return (digitalRead(TTN_GREEN_LED) == LOW);
}

bool TheThingsNode::getBlue()
{
  return (digitalRead(TTN_BLUE_LED) == LOW);
}

ttn_color TheThingsNode::getColor()
{
  bool red = getRed();
  bool green = getGreen();
  bool blue = getBlue();

  if (red && green && blue)
  {
    return TTN_WHITE;
  }
  else if (red && green)
  {
    return TTN_YELLOW;
  }
  else if (red && blue)
  {
    return TTN_MAGENTA;
  }
  else if (green && blue)
  {
    return TTN_CYAN;
  }
  else if (red)
  {
    return TTN_RED;
  }
  else if (green)
  {
    return TTN_GREEN;
  }
  else if (blue)
  {
    return TTN_BLUE;
  }
  else
  {
    return TTN_BLACK;
  }
}

String TheThingsNode::colorToString(ttn_color color)
{
  switch (color)
  {
  case TTN_RED:
    return String("Red");
    break;
  case TTN_GREEN:
    return String("Green");
    break;
  case TTN_BLUE:
    return String("Blue");
    break;
  case TTN_YELLOW:
    return String("Yellow");
    break;
  case TTN_CYAN:
    return String("Cyan");
    break;
  case TTN_MAGENTA:
    return String("Magenta");
    break;
  case TTN_WHITE:
    return String("White");
    break;
  case TTN_BLACK:
    return String("Black");
    break;
  }
}

void TheThingsNode::setRGB(bool red, bool green, bool blue)
{
  setRed(red);
  setGreen(green);
  setBlue(blue);
}

void TheThingsNode::setRed(bool on)
{
  digitalWrite(TTN_RED_LED, on ? LOW : HIGH);
}

void TheThingsNode::setGreen(bool on)
{
  digitalWrite(TTN_GREEN_LED, on ? LOW : HIGH);
}

void TheThingsNode::setBlue(bool on)
{
  digitalWrite(TTN_BLUE_LED, on ? LOW : HIGH);
}

void TheThingsNode::setColor(ttn_color color)
{
  switch (color)
  {
  case TTN_RED:
    setRGB(true, false, false);
    break;
  case TTN_GREEN:
    setRGB(false, true, false);
    break;
  case TTN_BLUE:
    setRGB(false, false, true);
    break;
  case TTN_YELLOW:
    setRGB(true, true, false);
    break;
  case TTN_CYAN:
    setRGB(false, true, true);
    break;
  case TTN_MAGENTA:
    setRGB(true, false, true);
    break;
  case TTN_WHITE:
    setRGB(true, true, true);
    break;
  case TTN_BLACK:
    setRGB(false, false, false);
    break;
  }
}

/******************************************************************************
 * BATTERY
 */
uint16_t TheThingsNode::readADCLowNoise(bool average)
{
  uint8_t low, high;
  uint16_t sum = 0;
  
  // Start 1st Conversion, but ignore it, can be hazardous
  ADCSRA |= _BV(ADSC); 
  
  // wait for first dummy conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Init our measure counter
  adcIRQCnt = 0;

  // We want to have a interrupt when the conversion is done
  ADCSRA |= _BV( ADIE );

  // Loop thru samples
  // 8 samples (we don't take the 1st one)
  do {
    // Enable Noise Reduction Sleep Mode
    set_sleep_mode( SLEEP_MODE_ADC );
    sleep_enable();

    // Wait until conversion is finished 
    do {
      // enabled IRQ before sleeping
      sei();
      sleep_cpu();
      cli();
    }
    // Check is done with interrupts disabled to avoid a race condition
    while (bit_is_set(ADCSRA,ADSC));

    // No more sleeping
    sleep_disable();
    sei();
    
    // read low first
    low  = ADCL;
    high = ADCH;
    
    // Sum the total
    sum += ((high << 8) | low);
  }
  while (adcIRQCnt<8);
  
  // No more interrupts needed for this
  // we finished the job
  ADCSRA &= ~ _BV( ADIE );
  
  // Return the average divided by 8 (8 samples)
  return ( average ? sum >> 3 : sum );
}

/* ======================================================================
Function: getVcc
Purpose : Read and Calculate V powered, the Voltage on Arduino VCC pin
Input   : -
Output  : value readed
Comments: ADC Channel input is modified
====================================================================== */
uint16_t TheThingsNode::getVcc() 
{
  uint16_t value; 
  uint16_t vcc; 

  // Enable ADC (just in case going out of low power)
  power_adc_enable();
  ADCSRA |= _BV(ADEN)  ;

  // Read 1.1V reference against AVcc
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
  // MUX4 MUX3 MUX2 MUX1 MUX0  --> 011110 1.1V (VBG)        -Selects channel 14, bandgap voltage, to measure
  ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);

  // Take care, changing reference from VCC to 1.1V bandgap can take some time, this is due
  // to the fact that the capacitor on aref pin need to discharge
  // or to charge when we're just leaving power down mode
  // power down does not hurt and 15ms strong enough for ADC setup
  delay(15);  

  // read value
  value = readADCLowNoise(true);

  // Vcc reference in millivolts
  vcc =  ( 1023L * 1100L) / value ; 
  
  // Operating range of ATMega
  if (vcc < 1800 ) vcc = 1800 ;
  if (vcc > 5500 ) vcc = 5500 ;
    
  // Vcc in millivolts
  return ( vcc ); 
}


uint16_t TheThingsNode::getBattery()
{
  digitalWrite(TTN_VBAT_MEAS_EN, LOW);
  uint16_t val = analogRead(TTN_VBAT_MEAS);
  digitalWrite(TTN_VBAT_MEAS_EN, HIGH);
  uint16_t batteryVoltage = map(val, 0, 1024, 0, 3300) * 2; // *2 for voltage divider
  return batteryVoltage;
}





/******************************************************************************
 * USB
 */

void TheThingsNode::configUSB(bool deepSleep)
{
  this->USBDeepSleep = deepSleep;

  if (this->USBDeepSleep || !isUSBConnected())
  {
    WDT_start();
  }
  else
  {
    WDT_stop();
  }
}

bool TheThingsNode::isUSBConnected()
{
  return USBSTA & (1 << VBUS);
}

/******************************************************************************
 * PRIVATE
 *****************************************************************************/

TheThingsNode::TheThingsNode()
{
  configInterval(false, 60000);

  pinMode(TTN_LDR_GAIN1, OUTPUT);
  pinMode(TTN_LDR_GAIN2, OUTPUT);
  configLight(false, 1);

  configTemperature(false, R_DEGREES_0_0625, 0, 30, 55, H_DEGREES_0_0);

  configMotion(false);

  pinMode(TTN_BUTTON, INPUT);
  digitalWrite(TTN_BUTTON, HIGH);

  pinMode(TTN_RED_LED, OUTPUT);
  pinMode(TTN_GREEN_LED, OUTPUT);
  pinMode(TTN_BLUE_LED, OUTPUT);
  setColor(TTN_BLACK);

  // reset RN2483 module, this allow to reset module on sketch upload also
#ifdef TTN_LORA_RESET
  pinMode(TTN_LORA_RESET, OUTPUT);
  digitalWrite(TTN_LORA_RESET, LOW);
  delay(100);
  digitalWrite(TTN_LORA_RESET, HIGH);
#endif

  // TODO: Can we enable/disable this at will to save memory?
  USBCON |= (1 << OTGPADE);

  pinMode(TTN_VBAT_MEAS_EN, OUTPUT);
  digitalWrite(TTN_VBAT_MEAS_EN, HIGH);

  if (!isUSBConnected() || this->USBDeepSleep)
  {
    WDT_start();
  }
}

void TheThingsNode::sleepTemperature()
{
  if (this->temperatureSleep || this->temperatureEnabled)
  {
    return;
  }

  TTN_TEMPERATURE_SENSOR.setMode(MODE_SHUTDOWN);

  this->temperatureSleep = true;
}

void TheThingsNode::wakeTemperature()
{
  if (!this->temperatureSleep)
  {
    return;
  }

  TTN_TEMPERATURE_SENSOR.setMode(MODE_CONTINUOUS);

  this->temperatureSleep = false;
}

void TheThingsNode::sleepMotion()
{
  writeMotion(TTN_CTRL_REG1, (0 | TTN_DR << 3) | TTN_SR << 6); //Put ACC in standby mode
}

void TheThingsNode::wakeMotion()
{
  // describes motion interrupt setup for low power:
  // http://arduino.stackexchange.com/questions/1475/setting-up-the-mma8452-to-trigger-interrupt
  writeMotion(TTN_CTRL_REG1, (0 | TTN_DR << 3) | TTN_SR << 6); //DR and SR defined data rate and sleep rate
  writeMotion(TTN_CTRL_REG2, 0x1F);                            //LP mode in sleep and active, autosleep on
  writeMotion(TTN_ASLP_CNT, TTN_SC);                           //defined sleep count
  writeMotion(TTN_CTRL_REG3, 0x42);                            //Transient interrupt
  writeMotion(TTN_CTRL_REG4, 0x20);                            //Transient interrupt source on
  writeMotion(TTN_CTRL_REG5, 0x20);                            //Transient to pin INT1, the rest to INT2
  writeMotion(TTN_TRANSIENT_CFG, 0x0E);                        //Flag latch disabled, motion on all axes
  writeMotion(TTN_TRANSIENT_THS, TTN_MT);                      //Motion threshold, debounce in inc/dec mode
  writeMotion(TTN_TRANSIENT_COUNT, TTN_MDC);                   //Motion delay

  writeMotion(TTN_CTRL_REG1, (0x01 | TTN_DR << 3) | TTN_SR << 6); //Put ACC in active mode
  delay(300);
  readMotion(TTN_TRANSIENT_SRC);
}

void TheThingsNode::writeMotion(unsigned char REG_ADDRESS, unsigned char DATA) //SEND data to MMA8652
{
  Wire.beginTransmission(TTN_ADDR_ACC);
  Wire.write(REG_ADDRESS);
  Wire.write(DATA);
  Wire.endTransmission();
}

uint8_t TheThingsNode::readMotion(unsigned char REG_ADDRESS)
{
  uint8_t resp;
  Wire.beginTransmission(TTN_ADDR_ACC);
  Wire.write(REG_ADDRESS);
  Wire.endTransmission(false);
  Wire.requestFrom(TTN_ADDR_ACC, 1);
  resp = Wire.read();
  return resp;
}

void TheThingsNode::getAcceleration(float *x, float *y, float *z)
{
  // Resource: https://github.com/sparkfun/MMA8452_Accelerometer/blob/master/Libraries/Arduino/src/SparkFun_MMA8452Q.cpp
  // Read the acceleration from registers 1 through 6 of the MMA8452 accelerometer.
  // 2 registers per axis, 12 bits per axis.
  // Bit-shifting right does sign extension to preserve negative numbers.
  *x = ((short)(readMotion(1)<<8 | readMotion(2))) >> 4;
  *y = ((short)(readMotion(3)<<8 | readMotion(4))) >> 4;
  *z = ((short)(readMotion(5)<<8 | readMotion(6))) >> 4;

  // Scale 12 bit signed values to units of g. The default measurement range is ±2g.
  // That is 11 bits for positive values and 11 bits for negative values.
  // value = (value / (2^11)) * 2
  *x = (float)*x / (float)(1<<11) * (float)(2);
  *y = (float)*y / (float)(1<<11) * (float)(2);
  *z = (float)*z / (float)(1<<11) * (float)(2);
}

void TheThingsNode::WDT_start()
{
  if (this->wdtStarted)
  {
    return;
  }

  cli();
  MCUSR &= ~(1 << WDRF);
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = 1 << WDP0 | 0 << WDP1 | 0 << WDP2 | 1 << WDP3; /* 8.0 seconds */
  WDTCSR |= _BV(WDIE);
  sei();

  this->wdtStarted = true;
}

void TheThingsNode::WDT_stop()
{
  if (!this->wdtStarted)
  {
    return;
  }

  cli();
  MCUSR &= ~(1 << WDRF);
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = 0x00;
  WDTCSR = 0 << WDP0 | 1 << WDP1 | 0 << WDP2 | 0 << WDP3;
  sei();

  this->wdtStarted = true;
}

void TheThingsNode::deepSleep(void)
{
  // We want to be awake by LoRa module ?
  if (this->pttn) {
    // watchdog Not needed, avoid wake every 8S
    WDT_stop(); 

  // Set LoRa module sleep mode
  //  ttn.sleep( CONFIG_INTERVAL*1000 );
    this->pttn->sleep(this->intervalMs);
    // This one is not optionnal, remove it
    // and say bye bye to RN2483 or RN2903 sleep mode
    delay(50);

    // Module need to wake us with interrupt
    attachInterrupt(TTN_LORA_SERIAL_RX_INT, TTN_SERIAL_LORA_FN, FALLING);

    // switch all interrupts off while messing with their settings  
    cli();  
    bitSet(EIFR,INTF2); // clear any pending interrupts for serial RX pin (INT2 D0)
    sei();

  } else {
    // watchdog needed for wakeup
    WDT_start(); 
  }

  ADCSRA &= ~_BV(ADEN);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  MCUCR |= (1 << JTD);
  USBCON |= (1 << FRZCLK);
  //USBCON &= ~_BV(USBE);
  PLLCSR &= ~_BV(PLLE);
  sleep_enable();
  sleep_mode(); //Sweet dreams!

  //wake up, after ISR we arrive here ->
  sleep_disable();
  PLLCSR |= (1 << PLLE);
  power_all_enable();
  //USBCON |= (1 << USBE);
  USBCON &= ~_BV(FRZCLK);
  ADCSRA |= (1 << ADEN);
}
