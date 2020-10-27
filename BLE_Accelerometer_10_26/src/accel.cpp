// This program is used to send accelerometer data from MMA8451 through Bluefruit LE using Teensy 4.0 //

// Headers for Teensy & Accelerometer //
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>
#include <Adafruit_BluefruitLE_SPI.h>


// Accelerator Class //
Adafruit_MMA8451 mma = Adafruit_MMA8451();
int ledPin = 11;
#define MMA8451_SAMPLERATE_DELAY_MS (800)

// Define pins for bluefruit serial output //
#define BLUEFRUIT_UART_MODE_PIN  9
Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

#define VERBOSE_MODE true

#define ACCEL_SERVICE_UUID128        "b6-f8-d5-00-b0-c9-11-ea-b3-de-02-42-ac-13-00-04"
#define ACCEL_MEASURE_CHAR_UUID128   "b6-f8-d8-ac-b0-c9-11-ea-b3-de-02-42-ac-13-00-04"


void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

int32_t accelServiceId;
int32_t accelMeasureCharId;

void setup(void)
{

  while (!Serial); // required for Flora & Micro
  delay(500);

  boolean success;

  Serial.begin(115200); // Serial connection to USB
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Setting device name to 'Bluefruit ACCEL': "));

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Bluefruit ACCEL")) ) {
    error(F("Could not set device name?"));
  }

  /* Add the Heart Rate Service definition */
  /* Service ID should be 1 */
  Serial.println(F("Adding the accelerometer service definition: "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID128=" ACCEL_SERVICE_UUID128), &accelServiceId);
  if (! success) {
    error(F("Could not add accel service"));
  }

  /* Add the Heart Rate Measurement characteristic */
  /* Chars ID for Measurement should be 1 */
  Serial.println(F("Adding the Accelerometer Measurement characteristic: "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=" ACCEL_MEASURE_CHAR_UUID128 ", PROPERTIES=0x12, MIN_LEN=1, MAX_LEN=10"), &accelMeasureCharId);
    if (! success) {
    error(F("Could not add measurement characteristic"));
  }

  // Serial.println(F("Performing a factory reset: "));
  // if(!ble.factoryReset())
  // {
  //   Serial.println("");
  //   Serial.println(F("Couldn't reset module."));
  //   while(1);
  // }
  // Serial.println(F("Done!"));
  //
  // Serial.print(F("Changing device name..."));
  // ble.print("AT+GAPDEVNAME=");
  // ble.println("BLE Accelerometer");
  // if (!ble.waitForOK())
  // {
  //   Serial.println(F("Could not change name."));
  //   while(1);
  // }
  // ble.reset();
  // Serial.println(F("Done!"));
  ble.setMode(BLUEFRUIT_MODE_DATA);

  // Start accelerometer
  if (! mma.begin())
  {
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("MMA8451 found!");
  // Set G-range for accelerometer //
  mma.setRange(MMA8451_RANGE_2_G);
  Serial.print("Range = ");
  Serial.print(2 << mma.getRange());
  Serial.println("G");
}

void loop(void)
  {
  /* Get a new sensor event */
  sensors_event_t event;
  mma.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print(event.acceleration.x,4);
  // Serial.print("\t");
  // ble.print(event.acceleration.x,4); // Send data to Bluetooth
  // ble.print("\t");

  // Serial.print(event.acceleration.y,4);
  // Serial.print("\t");
  // ble.print(event.acceleration.y,4); // Send data to Bluetooth
  // ble.print("\t");

  // Serial.print(event.acceleration.z,4);
  // Serial.print("\t");
  ble.print(event.acceleration.z,4); // Send data to Bluetooth
  ble.print("\t");

  ble.println();
  // Serial.println();

  delay(MMA8451_SAMPLERATE_DELAY_MS);
}
