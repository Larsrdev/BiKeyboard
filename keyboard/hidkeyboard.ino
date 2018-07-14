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
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"
#include "keycode.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

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
/*=========================================================================*/


// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


#define NUMBER_OF_ACTIVE_KEYS         12
#define NUMBER_OF_KEYCODES            42
#define SCANNING_PERIOD_MS            10
#define LONGPRESS_THRESHOLD_MS        400

int currentPressedTime = 0;
bool longPressActive = false;
int previousPressedKey = -1;
int previousRowNumber = -1;
bool multipressOn = false;
// {thumb_l, UNUSED, little_l, ring_l, middle_l, index_l, thumb_r, index_r, middle_r, ring_r, little_r, UNUSED}
int multipress_A[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,0,0,1,0,0,0,0};
int multipress_B[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,1,0,0,0,1,0,0,0};
int multipress_C[NUMBER_OF_ACTIVE_KEYS] =       {1,0,0,1,1,0,0,0,0,0,0,0};
int multipress_D[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,1,0,1,0,0,0,0};
int multipress_E[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,1,0,0,0,0,0,0};
int multipress_F[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,1,0,0,0,0,0,1,0,0};
int multipress_G[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,0,0,0,0,1,1,0};
int multipress_H[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,1,1,0,0,0,0,0,0,0};
int multipress_I[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,0,1,0,0,0,0,0};
int multipress_J[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,0,0,1,1,1,0,0};
int multipress_K[NUMBER_OF_ACTIVE_KEYS] =       {1,0,0,0,0,0,0,0,0,1,0,0};
int multipress_L[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,0,1,0,1,0,0,0};
int multipress_M[NUMBER_OF_ACTIVE_KEYS] =       {0,0,1,1,0,0,0,0,0,0,0,0};
int multipress_N[NUMBER_OF_ACTIVE_KEYS] =       {1,0,0,0,0,0,0,0,0,0,0,0};
int multipress_O[NUMBER_OF_ACTIVE_KEYS] =       {1,0,0,0,0,0,1,0,0,0,0,0};
int multipress_P[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,1,0,0,1,0,0,0,0,0};
int multipress_Q[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,0,0,0,1,1,1,0};
int multipress_R[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,0,0,0,1,0,0,0};
int multipress_S[NUMBER_OF_ACTIVE_KEYS] =       {0,0,1,0,0,0,0,0,0,0,0,0};
int multipress_T[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,0,0,0,0,0,1,0};
int multipress_U[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,0,0,0,0,1,0,0};
int multipress_V[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,1,1,1,0,0,0,0,0,0};
int multipress_W[NUMBER_OF_ACTIVE_KEYS] =       {0,0,1,0,0,0,0,0,0,0,1,0};
int multipress_X[NUMBER_OF_ACTIVE_KEYS] =       {1,0,0,0,1,1,0,0,0,0,0,0};
int multipress_Y[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,0,1,1,1,0,0,0};
int multipress_Z[NUMBER_OF_ACTIVE_KEYS] =       {0,0,1,1,1,0,0,0,0,0,0,0};
int multipress_DOT[NUMBER_OF_ACTIVE_KEYS] =     {0,0,1,1,1,1,0,0,0,0,0,0};
int multipress_COMMA[NUMBER_OF_ACTIVE_KEYS] =   {0,0,0,0,0,0,0,1,1,1,1,0};
int multipress_OE[NUMBER_OF_ACTIVE_KEYS] =      {0,0,0,0,1,1,0,1,1,0,0,0};
int multipress_AE[NUMBER_OF_ACTIVE_KEYS] =      {0,0,0,1,1,0,0,0,1,1,0,0};
int multipress_UE[NUMBER_OF_ACTIVE_KEYS] =      {0,0,1,1,0,0,0,0,0,1,1,0};
int multipress_MODE_A[NUMBER_OF_ACTIVE_KEYS] =  {0,0,0,0,0,0,1,1,1,1,1,0};
int multipress_MODE_B[NUMBER_OF_ACTIVE_KEYS] =  {1,0,1,1,1,1,0,0,0,0,0,0};
int multipress_MODE_C[NUMBER_OF_ACTIVE_KEYS] =  {1,0,1,1,1,1,1,1,1,1,1,0};
int multipress_0[NUMBER_OF_ACTIVE_KEYS] =       {0,0,1,0,0,0,0,0,0,0,0,0};
int multipress_1[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,0,1,0,0,0,0,0};
int multipress_2[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,0,0,1,0,0,0,0};
int multipress_3[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,0,0,0,1,0,0,0};
int multipress_4[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,0,0,0,0,1,0,0};
int multipress_5[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,0,0,0,0,0,1,0};
int multipress_6[NUMBER_OF_ACTIVE_KEYS] =       {1,0,0,0,0,0,0,0,0,0,0,0};
int multipress_7[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,0,1,0,0,0,0,0,0};
int multipress_8[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,0,1,0,0,0,0,0,0,0};
int multipress_9[NUMBER_OF_ACTIVE_KEYS] =       {0,0,0,1,0,0,0,0,0,0,0,0};
int multipress_UP[NUMBER_OF_ACTIVE_KEYS] =      {0,0,0,0,0,0,1,0,0,0,0,0};
int multipress_DOWN[NUMBER_OF_ACTIVE_KEYS] =    {1,0,0,0,0,0,0,0,0,0,0,0};
int multipress_LEFT[NUMBER_OF_ACTIVE_KEYS] =    {0,0,0,0,0,1,0,0,0,0,0,0};
int multipress_RIGHT[NUMBER_OF_ACTIVE_KEYS] =   {0,0,0,0,0,0,0,1,0,0,0,0};
int multipress_ENTER[NUMBER_OF_ACTIVE_KEYS] =   {0,0,0,0,0,0,0,1,1,1,1,0};
int multipress_DELETE[NUMBER_OF_ACTIVE_KEYS] =  {0,0,1,1,1,1,0,0,0,0,0,0};
int multipress_INSERT[NUMBER_OF_ACTIVE_KEYS] =  {0,0,0,0,0,0,0,0,0,1,1,0};
int multipress_PAGE_UP[NUMBER_OF_ACTIVE_KEYS] = {0,0,0,0,0,0,1,1,0,0,0,0};
int multipress_PAGE_DW[NUMBER_OF_ACTIVE_KEYS] = {1,0,0,0,1,0,0,0,0,0,0,0};
int multipress_SHIFT[NUMBER_OF_ACTIVE_KEYS] =   {0,0,1,0,0,0,0,0,0,0,0,0};
int multipress_TAB[NUMBER_OF_ACTIVE_KEYS] =     {0,0,0,1,0,0,0,0,0,0,0,0};
int multipress_ESC[NUMBER_OF_ACTIVE_KEYS] =     {0,0,1,1,1,1,0,1,1,1,1,0};

//Input pins:
int inputPins[NUMBER_OF_ACTIVE_KEYS]     =                     {A0,             A1,         A2,         A3,         A4,         A5,         5,              6,          9,          10,         11,         12};
int multipressOnSwitch = 13;

//Current active pin:
int hid_keyboard_current_pressed_key[NUMBER_OF_ACTIVE_KEYS] =  {0,              0,          0,          0,          0,          0,          0,              0,          0,          0,          0,          0};


int hid_keyboard_row0[NUMBER_OF_ACTIVE_KEYS] =                 { HID_KEY_NONE,  HID_KEY_Q,  HID_KEY_W,  HID_KEY_E,  HID_KEY_R,  HID_KEY_T,  HID_KEY_NONE ,  HID_KEY_Z,  HID_KEY_U,  HID_KEY_I,  HID_KEY_O,  HID_KEY_P };
int hid_keyboard_longpress_row0[NUMBER_OF_ACTIVE_KEYS] =       { HID_KEY_NONE,  HID_KEY_AT,  HID_KEY_CARET,  HID_KEY_EURO,  HID_KEY_DEGREE,  HID_KEY_NONE,  HID_KEY_NONE ,  HID_KEY_BACKSLASH,  HID_KEY_UE,  HID_KEY_NONE,  HID_KEY_OE,  HID_KEY_NONE };

int hid_keyboard_row1[NUMBER_OF_ACTIVE_KEYS] =                 { HID_KEY_NONE,  HID_KEY_A,  HID_KEY_S,  HID_KEY_D,  HID_KEY_F,  HID_KEY_G,  HID_KEY_NONE ,  HID_KEY_H,  HID_KEY_J,  HID_KEY_K,  HID_KEY_L,  HID_KEY_SPACE };
int hid_keyboard_longpress_row1[NUMBER_OF_ACTIVE_KEYS] =       { HID_KEY_NONE,  HID_KEY_AE,  HID_KEY_SZ,  HID_KEY_DOUBLECROSS,  HID_KEY_QESTIONMARK,  HID_KEY_APOSTROPHE,  HID_KEY_NONE ,  HID_KEY_COMMA,  HID_KEY_SEMICOLON,  HID_KEY_DOT,  HID_KEY_DOUBLEDOT,  HID_KEY_NONE };

int hid_keyboard_row2[NUMBER_OF_ACTIVE_KEYS] =                 { HID_KEY_NONE,  HID_KEY_Y,  HID_KEY_X,  HID_KEY_C,  HID_KEY_V,  HID_KEY_B,  HID_KEY_NONE ,  HID_KEY_N,  HID_KEY_M,  HID_KEY_NONE,  HID_KEY_BACKSPACE,  HID_KEY_INSERT  };
int hid_keyboard_longpress_row2[NUMBER_OF_ACTIVE_KEYS] =       { HID_KEY_NONE,  HID_KEY_LESSERTHAN,  HID_KEY_GREATERTHAN,  HID_KEY_NONE,  HID_KEY_MINUS,  HID_KEY_NONE,  HID_KEY_NONE ,  HID_KEY_NONE,  HID_KEY_NONE,  HID_KEY_NONE,  HID_KEY_BACKSPACE,  HID_KEY_INSERT};

int hid_keyboard_row3[NUMBER_OF_ACTIVE_KEYS] =                 { HID_KEY_NONE,  HID_KEY_1,  HID_KEY_2,  HID_KEY_3,  HID_KEY_4,  HID_KEY_5,  HID_KEY_NONE ,  HID_KEY_6,  HID_KEY_7,  HID_KEY_8,  HID_KEY_9,  HID_KEY_0};
int hid_keyboard_longpress_row3[NUMBER_OF_ACTIVE_KEYS] =       { HID_KEY_NONE,  HID_KEY_NONE,  HID_KEY_QUOTATIONMARK,  HID_KEY_NONE,  HID_KEY_EURO,  HID_KEY_PERCENT,  HID_KEY_NONE ,  HID_KEY_AND,  HID_KEY_SLASH,  HID_KEY_ROUNDBRACKETOPEN,  HID_KEY_ROUNDBRACKETCLOSED,  HID_KEY_EQUALS};

int inputKeycodes[NUMBER_OF_KEYCODES] = {
  HID_KEY_A,
  HID_KEY_B,
  HID_KEY_C,
  HID_KEY_D,
  HID_KEY_E,
  HID_KEY_F,
  HID_KEY_G,
  HID_KEY_H,
  HID_KEY_I,
  HID_KEY_J,
  HID_KEY_K,
  HID_KEY_L,
  HID_KEY_M,
  HID_KEY_N,
  HID_KEY_O,
  HID_KEY_P,
  HID_KEY_Q,
  HID_KEY_R,
  HID_KEY_S,
  HID_KEY_T,
  HID_KEY_U,
  HID_KEY_V,
  HID_KEY_W,
  HID_KEY_X,
  HID_KEY_Y,
  HID_KEY_Z,
  HID_KEY_1,
  HID_KEY_2,
  HID_KEY_3,
  HID_KEY_4,
  HID_KEY_5,
  HID_KEY_6,
  HID_KEY_7,
  HID_KEY_8,
  HID_KEY_9,
  HID_KEY_0,
  HID_KEY_RETURN,
  HID_KEY_ESCAPE,
  HID_KEY_BACKSPACE,
  HID_KEY_TAB,
  HID_KEY_SPACE,
  HID_KEY_MINUS
};

typedef struct
{
  uint8_t modifier;   /**< Keyboard modifier keys  */
  uint8_t reserved;   /**< Reserved for OEM use, always set to 0. */
  uint8_t keycode[8]; /**< Key codes of the currently pressed keys. */
} hid_keyboard_report_t;

// Report that send to Central every scanning period
hid_keyboard_report_t keyReport = { 0, 0, { 0 } };

// Report sent previously. This is used to prevent sending the same report over time.
// Notes: HID Central intepretes no new report as no changes, which is the same as
// sending very same report multiple times. This will help to reduce traffic especially
// when most of the time there is no keys pressed.
// - Init to different with keyReport
hid_keyboard_report_t previousReport = { 0, 0, { 1 } };

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void){
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("BiKeyboard"));
  Serial.println(F("---------------------------------------"));

  if ( !ble.begin(VERBOSE_MODE) ){
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  if ( FACTORYRESET_ENABLE ){
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset ... "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'BiKeyboard' ... "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=BiKeyboard" )) ) {
    error(F("Could not set device name?"));
  }

  /* Enable HID Service */
  Serial.println(F("Enable HID Service (including Keyboard) ... "));
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) ){
    if ( !ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ))) {
      error(F("Could not enable Keyboard"));
    }
  }else {
    if (! ble.sendCommandCheckOK(F( "AT+BleKeyboardEn=On"  ))) {
      error(F("Could not enable Keyboard"));
    }
  }

  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }

  Serial.println();
  Serial.println(F("Go to your phone's Bluetooth settings to pair your device"));
  Serial.println(F("then open an application that accepts keyboard input"));
  Serial.println();

  // Set up input Pins
  for(int i=0; i < NUMBER_OF_ACTIVE_KEYS; i++){
    pinMode(inputPins[i], INPUT_PULLUP);
    pinMode(multipressOnSwitch, INPUT_PULLUP);
  }
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new input
*/
/**************************************************************************/
void loop(void){
  if ( ble.isConnected() ){
    getBoardInput();
    if(multipressOn){
      mapMultiPressInput();
    }else{
      mapLongPressInput();
    }
    sendInput();

    delay(SCANNING_PERIOD_MS);
  }
}

/**************************************************************************/
/*!
    @brief  Checks for user input (via Pin Input)
*/
/**************************************************************************/
void getBoardInput(void){
  // https://learn.adafruit.com/custom-wireless-bluetooth-cherry-mx-gamepad?view=all 
  if(digitalRead(multipressOnSwitch) == LOW){
    multipressOn = false;
    
  }else{
    multipressOn = true;
  }
   
  for(int i=0; i<NUMBER_OF_ACTIVE_KEYS; i++){
    // GPIO is active low
    if( digitalRead(inputPins[i]) == LOW ){
      hid_keyboard_current_pressed_key[i] = 1;
    }else{
      hid_keyboard_current_pressed_key[i] = 0;
    }
  }
}

void mapMultiPressInput(void){
  resetKeyReport();
  comparePressedKeysAndMapToHIDKey(multipress_A, HID_KEY_A);
  comparePressedKeysAndMapToHIDKey(multipress_B, HID_KEY_B);
  comparePressedKeysAndMapToHIDKey(multipress_C, HID_KEY_C);
  comparePressedKeysAndMapToHIDKey(multipress_D, HID_KEY_D);
  comparePressedKeysAndMapToHIDKey(multipress_E, HID_KEY_E);
  comparePressedKeysAndMapToHIDKey(multipress_F, HID_KEY_F);
  comparePressedKeysAndMapToHIDKey(multipress_G, HID_KEY_G);
  comparePressedKeysAndMapToHIDKey(multipress_H, HID_KEY_H);
  comparePressedKeysAndMapToHIDKey(multipress_I, HID_KEY_I);
  comparePressedKeysAndMapToHIDKey(multipress_J, HID_KEY_J);
  comparePressedKeysAndMapToHIDKey(multipress_K, HID_KEY_K);
  comparePressedKeysAndMapToHIDKey(multipress_L, HID_KEY_L);
  comparePressedKeysAndMapToHIDKey(multipress_M, HID_KEY_M);
  comparePressedKeysAndMapToHIDKey(multipress_N, HID_KEY_N);
  comparePressedKeysAndMapToHIDKey(multipress_O, HID_KEY_O);
  comparePressedKeysAndMapToHIDKey(multipress_P, HID_KEY_P);
  comparePressedKeysAndMapToHIDKey(multipress_Q, HID_KEY_Q);
  comparePressedKeysAndMapToHIDKey(multipress_R, HID_KEY_R);
  comparePressedKeysAndMapToHIDKey(multipress_S, HID_KEY_S);
  comparePressedKeysAndMapToHIDKey(multipress_T, HID_KEY_T);
  comparePressedKeysAndMapToHIDKey(multipress_U, HID_KEY_U);
  comparePressedKeysAndMapToHIDKey(multipress_V, HID_KEY_V);
  comparePressedKeysAndMapToHIDKey(multipress_W, HID_KEY_W);
  comparePressedKeysAndMapToHIDKey(multipress_X, HID_KEY_X);
  comparePressedKeysAndMapToHIDKey(multipress_Y, HID_KEY_Y);
  comparePressedKeysAndMapToHIDKey(multipress_Z, HID_KEY_Z);
  // comparePressedKeysAndMapToHIDKey(multipress_DOT
  // comparePressedKeysAndMapToHIDKey(multipress_COMMA
  // comparePressedKeysAndMapToHIDKey(multipress_OE
  // comparePressedKeysAndMapToHIDKey(multipress_AE
  // comparePressedKeysAndMapToHIDKey(multipress_UE
  // comparePressedKeysAndMapToHIDKey(multipress_MODE_A
  // comparePressedKeysAndMapToHIDKey(multipress_MODE_B
  // comparePressedKeysAndMapToHIDKey(multipress_MODE_C
  // comparePressedKeysAndMapToHIDKey(multipress_0
  // comparePressedKeysAndMapToHIDKey(multipress_1
  // comparePressedKeysAndMapToHIDKey(multipress_2
  // comparePressedKeysAndMapToHIDKey(multipress_3
  // comparePressedKeysAndMapToHIDKey(multipress_4
  // comparePressedKeysAndMapToHIDKey(multipress_5
  // comparePressedKeysAndMapToHIDKey(multipress_6
  // comparePressedKeysAndMapToHIDKey(multipress_7
  // comparePressedKeysAndMapToHIDKey(multipress_8
  // comparePressedKeysAndMapToHIDKey(multipress_9
  // comparePressedKeysAndMapToHIDKey(multipress_UP
  // comparePressedKeysAndMapToHIDKey(multipress_DOWN
  // comparePressedKeysAndMapToHIDKey(multipress_LEFT
  // comparePressedKeysAndMapToHIDKey(multipress_RIGHT
  // comparePressedKeysAndMapToHIDKey(multipress_ENTER
  // comparePressedKeysAndMapToHIDKey(multipress_DELETE
  // comparePressedKeysAndMapToHIDKey(multipress_INSERT
  // comparePressedKeysAndMapToHIDKey(multipress_PAGE_UP
  // comparePressedKeysAndMapToHIDKey(multipress_PAGE_DW
  // comparePressedKeysAndMapToHIDKey(multipress_SHIFT
  // comparePressedKeysAndMapToHIDKey(multipress_TAB
  // comparePressedKeysAndMapToHIDKey(multipress_ESC
}

void resetKeyReport(void){
  keyReport.keycode[1] = 0;
}

void comparePressedKeysAndMapToHIDKey(int keyMap[], int key){
  for(int i=0; i<NUMBER_OF_ACTIVE_KEYS; i++){
    if(hid_keyboard_current_pressed_key[i] != keyMap[i]){
      return;
    }
  }
  Serial.print("Multipress erkannt: ");
  Serial.print(key);
  Serial.println();
  keyReport.keycode[1] = key;
}

void mapLongPressInput(void) {
  int offset = 0;
  if(hid_keyboard_current_pressed_key[0] == 1 && !hid_keyboard_current_pressed_key[6] == 1){
    //ROW: QWERTZUI...
    checkPressedKeys(hid_keyboard_row1, 12, 1, hid_keyboard_longpress_row1);
  }else if(!hid_keyboard_current_pressed_key[0] == 1 && hid_keyboard_current_pressed_key[6] == 1){
    //ROW: YXCVBNM...
    checkPressedKeys(hid_keyboard_row3, 36, 3, hid_keyboard_longpress_row3);
  }else if(hid_keyboard_current_pressed_key[0] == 1 && hid_keyboard_current_pressed_key[6] == 1){
    //ROW: 1234567...
    checkPressedKeys(hid_keyboard_row0, 0, 0, hid_keyboard_longpress_row0);
  }else{ //!hid_keyboard_current_pressed_key[0] == 1 && !hid_keyboard_current_pressed_key[6] == 1
    //ROW: ASDFGHJKL...
    checkPressedKeys(hid_keyboard_row2, 24, 2, hid_keyboard_longpress_row2);
  }
}

void checkPressedKeys(int row[ ], int offset, int rowNumber, int longPressRow[ ]){
  // search for pressed keys
  for(int i=0; i<NUMBER_OF_ACTIVE_KEYS; i++){

    // current pressed key found:
    if(hid_keyboard_current_pressed_key[i] == 1){

      // save keycode only if not thumb key
      if(row[i] != HID_KEY_NONE){
        // Modulo 6 to match bluetooth send size?! - Only way to get it work 
        keyReport.keycode[i%6] = row[i];

        // \start longpress check
        if(rowNumber == previousRowNumber && i == previousPressedKey){
          if(currentPressedTime >= LONGPRESS_THRESHOLD_MS){
            keyReport.keycode[i%6] = longPressRow[i];
          }
          currentPressedTime = currentPressedTime + SCANNING_PERIOD_MS;
        }else {
          currentPressedTime = 0;
          previousPressedKey = i;
          previousRowNumber = rowNumber;
        }
        // \end longpress check
        
        break;
      }
    }
    // nothing or only thumb keys pressed -> send nothing
    keyReport.keycode[i%6] = 0;
  }
}

void sendInput(void){
  // Only send if it is not the same as previous report
  if ( memcmp(&previousReport, &keyReport, 8) ){
    
    // Send keyboard report
    ble.atcommand("AT+BLEKEYBOARDCODE", (uint8_t*) &keyReport, 8);

    // copy to previousReport
    memcpy(&previousReport, &keyReport, 8);
  }
}

