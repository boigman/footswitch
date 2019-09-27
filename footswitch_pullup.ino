/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
  This example shows how to send HID (keyboard/mouse/etc) data via BLE
  Note that not all devices support BLE keyboard! BLE Keyboard != Bluetooth Keyboard
*/

#include <Arduino.h>
#include "LowPower.h"
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#define KEY_SYSRQ 0x46 // Keyboard Print Screen
#define KEY_SCROLLLOCK 0x47 // Keyboard Scroll Lock
#define KEY_PAUSE 0x48 // Keyboard Pause
#define KEY_INSERT 0x49 // Keyboard Insert
#define KEY_HOME 0x4a // Keyboard Home
#define KEY_PAGEUP 0x4b // Keyboard Page Up
#define KEY_DELETE 0x4c // Keyboard Delete Forward
#define KEY_END 0x4d // Keyboard End
#define KEY_PAGEDOWN 0x4e // Keyboard Page Down
#define KEY_RIGHT 0x4f // Keyboard Right Arrow
#define KEY_LEFT 0x50 // Keyboard Left Arrow
#define KEY_DOWN 0x51 // Keyboard Down Arrow
#define KEY_UP 0x52 // Keyboard Up Arrow

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "DISABLE"

/*=========================================================================*/


// Create the bluefruit object, either software serial...uncomment these lines

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
/**/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


const int button_count = 4;
uint8_t button_history[button_count];
int buttonPin[button_count];
int pressCount[button_count];
int releaseCount[button_count];
char* buttonNames[button_count]={"Home", "Left", "Right","End"};
char* keyCodes[button_count]={"AT+BLEKEYBOARDCODE=00-00-4B-00-00-00-00" //PageUp
, "AT+BLEKEYBOARDCODE=00-00-50-00-00-00-00" //LeftArrow
, "AT+BLEKEYBOARDCODE=00-00-4F-00-00-00-00" //RightArrow
, "AT+BLEKEYBOARDCODE=00-00-4E-00-00-00-00" //PageDown
};
const int chkInterval = 10;
unsigned long nextHomeTime;
unsigned long nextEndTime;
unsigned long nextMillis;
unsigned long nextSleep;
const int idleMin = 5;  //idle minutes until sleep
char outbuffer [80];
const int buttonHome = 2; 
const int buttonLeft = 3; 
const int buttonRight = 4;
const int buttonEnd = 5;
const int pinLED = 7;

const int keyHome = 0x60;  //Keyboard UpArrow
const int keyLeft = 0x50;  //Keyboard LeftArrow
const int keyRight = 0x4F;  //Keyboard RightArrow
const int keyDown = 0x51;  //Keyboard DownArrow

int buttonState = LOW;
int stateHome = LOW;         // pushbutton status for Up button
int stateLeft = LOW;       // pushbutton status for Left button
int stateRight = LOW;      // pushbutton status for Right button
int stateEnd = LOW;       // pushbutton status for Down button
int lastStateHome = LOW;         // pushbutton status for Up button
int lastStateLeft = LOW;       // pushbutton status for Left button
int lastStateRight = LOW;      // pushbutton status for Right button
int lastStateEnd = LOW;       // pushbutton status for Down button
int countKeyHome = 0;
int countKeyEnd = 0;
int countKeyLeft = 0;
int countKeyRight = 0;

long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 100;    // the debounce time; increase if the output flickers

/**/


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{

  pinMode(buttonHome, INPUT_PULLUP);           // set pin to input with internal pullup resistor
  pinMode(buttonEnd, INPUT_PULLUP);           // set pin to input with internal pullup resistor
  pinMode(buttonLeft, INPUT_PULLUP);           // set pin to input with internal pullup resistor
  pinMode(buttonRight, INPUT_PULLUP);           // set pin to input with internal pullup resistor
  pinMode(pinLED, OUTPUT);

  digitalWrite(pinLED, HIGH);
  delay(1000);
  digitalWrite(pinLED, LOW);
  
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Bluefruit HID Footswitch"));
  Serial.println(F("-------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'Bluefruit Footboard': "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=Bluefruit Footboard" )) ) {
    error(F("Could not set device name?"));
  }

  /* Enable HID Service */
  Serial.println(F("Enable HID Service (including Keyboard): "));
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    if ( !ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ))) {
      error(F("Could not enable Keyboard"));
    }
  }else
  {
    if (! ble.sendCommandCheckOK(F( "AT+BleKeyboardEn=On"  ))) {
      error(F("Could not enable Keyboard"));
    }
  }

  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }

  /* Turn off Bluefruit LEDs */
 // Serial.println(F("Turning off Bluefruit LEDs"));
 // if ( !ble.sendCommandCheckOK(F( "AT+HWMODELED=MANUAL,TOGGLE" ))) {
 //   error(F("Could not turn off Bluefruit LED"));
 // }


  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }

  Serial.println();
  Serial.println(F("Go to your phone's Bluetooth settings to pair your device"));
  Serial.println(F("then open an application that accepts keyboard input"));

  Serial.println();
  Serial.println(F("Press Footswitches to send characters:"));
  Serial.println(F("- (far left)   - PageUp"));
  Serial.println(F("- (near left)  - LeftArrow"));
  Serial.println(F("- (near right) - RightArrow"));
  Serial.println(F("- (far right)  - PageDown"));

  Serial.println();
  buttonPin[0] = buttonHome;
  buttonPin[1] = buttonLeft;
  buttonPin[2] = buttonRight;
  buttonPin[3] = buttonEnd;
  for(int ii=0; ii<button_count; ii++) {
    pinMode(buttonPin[ii], INPUT_PULLUP);           // set pin to input
    pressCount[ii] = 0;
    button_history[ii]=0b11111111;
  }
  nextHomeTime = millis()+(60ul * 60ul * 1000ul);
  nextEndTime = millis()+(60ul * 60ul * 1000ul);
  nextMillis = millis()+chkInterval;
  nextSleep = millis()+(idleMin * 60ul * 1000ul);

  attachInterrupt(1,wakeUp,FALLING);
  
}


void update_button(int p_count){
    button_history[p_count] = button_history[p_count] << 1;
    button_history[p_count] |= digitalRead(buttonPin[p_count]);
//      Serial.print("p_count=");
//      Serial.print(p_count);
//      Serial.print(", button_history[p_count]=");
//      Serial.println(button_history[p_count], BIN);
}

// button_released profile: 0xx11111
uint8_t is_button_released(int p_count){
    uint8_t released = 0; 
    if(releaseCount[p_count]==0) {  
 }
    if ((button_history[p_count] /*| 0b01100000*/ ) == 0b01111111){ 
        released = 1;
        button_history[p_count] = 0b11111111;
    }
    return released;
}

// button_pressed profile: 1xxx0000
uint8_t is_button_pressed(int p_count){
    uint8_t pressed = 0;
    if(p_count==0) {
    }
  if ((button_history[p_count] /*& 0b10001111 */) == 0b10000000){ 
      pressed = 1;
      button_history[p_count] = 0b00000000;
  }
  return pressed;
}

uint8_t is_button_up(int p_count){
        return (button_history[p_count] == 0b11111111);
}
uint8_t is_button_down(int p_count){
        return (button_history[p_count] == 0b00000000);
}


/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{

    int n;
    if(millis()>nextMillis) {
      nextMillis = millis()+chkInterval;
      for(int ii=0; ii<button_count; ii++) {
        update_button(ii);
        if (is_button_pressed(ii)){
          pressCount[ii]++;
          if(ii==1) nextHomeTime = millis()+750;  //set next Home command for 3/4 second from now
          if(ii==2) nextEndTime = millis()+750;  //set next End command for 3/4 second from now
//          n=sprintf(outbuffer,"Button: %s pressed",buttonNames[ii]);
//          Serial.print(outbuffer);
//          Serial.print(", pressCount[");
//          Serial.print(ii);
//          Serial.print("]=");
//          Serial.println(pressCount[ii]);
          n=sprintf(outbuffer,"\nSending %s: ", buttonNames[ii]);
          Serial.print(outbuffer);
          Serial.println(pressCount[ii]);
          ble.println(keyCodes[ii]);
          ble.println("AT+BLEKEYBOARDCODE=00-00");
          if( ble.waitForOK() )
            {
              Serial.println( F("OK!") );
            }else
            {
              Serial.println( F("FAILED!") );
            }
          nextSleep = millis()+(idleMin * 60ul * 1000ul); //activity resets sleep time
        }
        if (is_button_released(ii)){
          releaseCount[ii]++;
          if(ii==1) nextHomeTime = millis()+(60ul * 60ul * 1000ul); //set next Home command for an hour from now
          if(ii==2) nextEndTime = millis()+(60ul * 60ul * 1000ul);  //set next End command for an hour from now
          nextSleep = millis()+(idleMin * 60ul * 1000ul); //activity resets sleep time
        }
        if (is_button_down(ii)){
          if(ii==1 && (millis()>nextHomeTime)) {
            //send home command
            Serial.print(F("Home command sent\n"));
            ble.println(keyCodes[0]);
            ble.println(F("AT+BLEKEYBOARDCODE=00-00"));
            if( ble.waitForOK() )
              {
                Serial.println( F("OK!") );
              }else
              {
                Serial.println( F("FAILED!") );
              }
            nextHomeTime = millis()+(60ul * 60ul * 1000ul); //set next Home command for an hour from now (so it won't retrigger)
          }
          if(ii==2 && (millis()>nextEndTime)) {
            //send end command
            Serial.print(F("End command sent\n"));
            ble.println(keyCodes[3]);
            ble.println(F("AT+BLEKEYBOARDCODE=00-00"));
            if( ble.waitForOK() )
              {
                Serial.println( F("OK!") );
              }else
              {
                Serial.println( F("FAILED!") );
              }
            nextEndTime = millis()+(60ul * 60ul * 1000ul);  //set next End command for an hour from now (so it won't retrigger)
          }
          nextSleep = millis()+(idleMin * 60ul * 1000ul); //activity resets sleep time
        }
      }
    }
    if(millis()>nextSleep) {
       Serial.print(F("Idle time exceeded. Sleeping now\n"));
       delay(2000);
       LowPower.powerDown(SLEEP_FOREVER , ADC_OFF, BOD_OFF);
       Serial.print(F("Waking Up\n"));
       nextSleep = millis()+(idleMin * 60ul * 1000ul); //activity resets sleep time     
       button_history[2] = 0b11111111;   
    }

/*
  buttonState = debouncedRead(stateHome, buttonHome);
  if (!buttonState==stateHome) {
    if(stateHome==LOW) {
      stateHome = HIGH;
      Serial.println("PageUp State=HIGH:");
    } else {
      stateHome = LOW;
      countKeyHome++;
      Serial.print("\nSending PageUp:");
      Serial.println(countKeyHome);
      ble.println(F("AT+BLEKEYBOARDCODE=00-00-4B-00-00-00-00"));    //Page-Up
//      ble.println(F("AT+BLEKEYBOARDCODE=00-00-52-00-00-00-00"));  //Key-up
      ble.println(F("AT+BLEKEYBOARDCODE=00-00"));
    
      if( ble.waitForOK() )
        {
          Serial.println( F("OK!") );
        }else
        {
          Serial.println( F("FAILED!") );
        }
            
    }
  }

  buttonState = debouncedRead(stateLeft, buttonLeft);
  if (!buttonState==stateLeft) {
    if(stateLeft==LOW) {
      stateLeft = HIGH;
    } else {
      stateLeft = LOW;
      countKeyLeft++;
      Serial.print("\nSending keyLeft:");
      Serial.println(countKeyLeft);
      ble.println(F("AT+BLEKEYBOARDCODE=00-00-50-00-00-00-00"));
      ble.println(F("AT+BLEKEYBOARDCODE=00-00"));
    
      if( ble.waitForOK() )
        {
          Serial.println( F("OK!") );
        }else
        {
          Serial.println( F("FAILED!") );
        }
            
    }
  }

  buttonState = debouncedRead(stateRight, buttonRight);
  if (buttonState!=stateRight) {
    if(stateRight==LOW) {
      stateRight = HIGH;
    } else {
      stateRight = LOW;
      countKeyRight++;
      Serial.print("\nSending keyRight:");
      Serial.println(countKeyRight);
      ble.println(F("AT+BLEKEYBOARDCODE=00-00-4F-00-00-00-00"));
      ble.println(F("AT+BLEKEYBOARDCODE=00-00"));
    
      if( ble.waitForOK() )
        {
          Serial.println( F("OK!") );
        }else
        {
          Serial.println( F("FAILED!") );
        }
            
    }
  }

  buttonState = debouncedRead(stateEnd, buttonEnd);
  if (!buttonState==stateEnd) {
    if(stateEnd==LOW) {
      stateEnd = HIGH;
    } else {
      stateEnd = LOW;
      countKeyEnd++;
      Serial.print("\nSending PageDown:");
      Serial.println(countKeyEnd);
//      ble.println(F("AT+BLEKEYBOARDCODE=00-00-51-00-00-00-00"));  //Key-Down
      ble.println(F("AT+BLEKEYBOARDCODE=00-00-4E-00-00-00-00"));  //Page-Down
      ble.println(F("AT+BLEKEYBOARDCODE=00-00"));
    
      if( ble.waitForOK() )
        {
          Serial.println( F("OK!") );
        }else
        {
          Serial.println( F("FAILED!") );
        }
            
    }
  }
*/
  // Display prompt
//  Serial.print(F("keyboard > "));

  // Check for user input and echo it back if anything was found
//  char keys[BUFSIZE+1];
//  getUserInput(keys, BUFSIZE);

//  Serial.print("\nSending ");
//  Serial.println(keys);

//  ble.print("AT+BleKeyboard=");
//  ble.println(keys);

//  if( ble.waitForOK() )
//  {
//    Serial.println( F("OK!") );
//  }else
//  {
//    Serial.println( F("FAILED!") );
//  }
}

void wakeUp()
{
  
}


/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
void getUserInput(char buffer[], uint8_t maxSize)
{
  memset(buffer, 0, maxSize);
  while( Serial.available() == 0 ) {
    delay(1);
  }

  uint8_t count=0;

  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && !(Serial.available() == 0) );
}

int debouncedRead(int pLastState, int pButton) 
{
  int lastDebounceTime = 0;
  int lastButtonState = pLastState;
  int buttonState = digitalRead(pButton);
  if (buttonState != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  while ((millis() - lastDebounceTime) < debounceDelay) {
    lastButtonState = buttonState;
    buttonState = digitalRead(pButton);
    if (buttonState != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
  }
  return buttonState;
  
}
