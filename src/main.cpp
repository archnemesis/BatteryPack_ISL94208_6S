/**
 * Power Bank Prototype 1
 * Battery Management System Firmware
 * Version 1.0
 * 
 * Copyright (C) 2019 by R Gingras <robin@robingingras.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include <Wire.h>
#include <ISL94208.h>
#include <math.h>

#define ISL94208_ADDR 0x28

#define DEBUG 1

#define VIN 3.333
#define TEMP_R1 46400
#define TEMP_B 3988
#define TEMP_T0 298.15
#define TEMP_R0 10000

#define PIN_LED_D3 3
#define PIN_LED_D2 2
#define PIN_VPACK_ENABLE 8
#define PIN_VPACK_SENSE A3
#define PIN_ISL94208_AO A0
#define PIN_CHARGE_IND A1
#define PIN_DISCHARGE_IND A2

#define CONF_CHARGE_START_THRESHOLD 25.1  // don't start charge above this voltage
#define CONF_CHARGE_CUTOFF_THRESH 25.2    // charge cutoff voltage
#define CONF_DISCHARGE_CUTOFF_THRESH 15.6 // discharge cutoff voltage
#define CONF_BAD_CELL_THRESH 2.0          // min cell voltage before pack shutdown
#define CONF_SLEEP_TIMEOUT 9000           // put ISL94208 to sleep after this many millis
#define CONF_BAL_CHARGE 1                 // balance cells on charge cycle
#define CONF_BAL_DISCHARGE 0              // balance cells on discharge cycle
#define CONF_OC_CHARGE_THRESH ISL94208_OC_CHG_THRESH_120MV
#define CONF_OC_DISCHARGE_THRESH ISL94208_OC_CHG_THRESH_120MV

ISL94208 isl(ISL94208_ADDR);

unsigned long int counter = 0;
bool charger_connected = false;
bool charge_started = false;
bool shutoff = false;
bool error_light = false;

float adc2voltage(uint16_t adc)
{
  float voltage = (3.3 / 1024.0) * adc;
  return voltage;
}

void expireTimeout()
{
  counter = millis();
}

void enableVPack()
{
  digitalWrite(PIN_VPACK_ENABLE, HIGH);
}

void disableVPack()
{
  digitalWrite(PIN_VPACK_ENABLE, LOW);
}

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("");
  Serial.println("BatteryPack_ISL94208_6S V1.0");
  Serial.println("(C) 2019 by R. Gingras");
#endif

  pinMode(PIN_LED_D2, OUTPUT);
  pinMode(PIN_LED_D3, OUTPUT);
  pinMode(PIN_VPACK_ENABLE, OUTPUT);
  pinMode(PIN_CHARGE_IND, INPUT);
  pinMode(PIN_DISCHARGE_IND, INPUT);

  digitalWrite(PIN_LED_D2, LOW);
  digitalWrite(PIN_LED_D3, LOW);
  digitalWrite(PIN_VPACK_ENABLE, LOW);

#ifdef DEBUG
  Serial.println("Connecting to ISL94208...");
#endif

  if (isl.readCheckBit() != 1) {
#ifdef DEBUG
    Serial.println("Could not verify ISL94208 connection: check bit error!");
#endif
    while (true) {
      digitalWrite(PIN_LED_D2, HIGH);
      delay(500);
      digitalWrite(PIN_LED_D3, LOW);
      delay(500);
    }
  }

  /*
   * Configure ISL94208
   */
  isl.enableChargeSetWrites(true);
  isl.setOverCurrentChargeThreshold(ISL94208_OC_CHG_THRESH_120MV);
  isl.enableChargeSetWrites(false);

  isl.enableDischargeSetWrites(true);
  isl.setOverCurrentDischargeThreshold(ISL94208_OC_CHG_THRESH_120MV);
  isl.setShortCircuitDischargeThreshold(ISL94208_SHORT_CIRCUIT_THRESH_650MV);
  isl.enableDischargeSetWrites(false);
  
#ifdef DEBUG
  /*
   * Helpful to reset ISL94208 state when debugging.
   */
  isl.enableCFET(false);
  isl.enableDFET(false);
  isl.disableAllCB();
  delay(100);
#endif

  /*
   * Determine if the charger was present on wakeup
   */
  charger_connected = isl.readWkupFlag();

  expireTimeout();
}

// the loop function runs over and over again forever
void loop() {
  float v_pack = 0.0;
  float v_sensed = 0.0;
  float v_top = 0.0;
  float v_cell = 0.0;
  float temperature = 0.0;
  uint8_t top_cell = 0;
  uint16_t adc_val = 0;

  digitalWrite((error_light == true ? PIN_LED_D2 : PIN_LED_D3), HIGH);
  delay(100);
  digitalWrite((error_light == true ? PIN_LED_D2 : PIN_LED_D3), LOW);
  delay(100);         

  isl.disableAllCB();
  delay(100);

  for (uint8_t i = 1; i < 7; i++) {
    isl.selectAnalogOutput(i);
    delay(100);

    v_cell = adc2voltage(analogRead(PIN_ISL94208_AO)) * 2.0;

#ifdef DEBUG
    Serial.print("VCELL");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(v_cell);
#endif

    if (v_cell < CONF_BAD_CELL_THRESH) {
      // shut down FETs, show error light, sleep
    }

    v_pack += v_cell;

    if (v_cell > v_top) {
      top_cell = i;
      v_top = v_cell;
    }
  }

  isl.selectAnalogOutput(8);
  delay(100);
  adc_val = analogRead(PIN_ISL94208_AO);
  float adc_volts = VIN / 1024.0;
  float resistance = (TEMP_R1 * (adc_volts * adc_val)) / (VIN - adc_volts);
  temperature = ((TEMP_T0 * TEMP_B) / (TEMP_T0 * log(resistance/TEMP_R0) + TEMP_B)) - 273.15;

#ifdef DEBUG
  Serial.print("Temperature: ");
  Serial.println(temperature);
#endif

  isl.selectAnalogOutput(0);

#ifdef DEBUG
  Serial.print("Pack Voltage (Computed): ");
  Serial.println(v_pack);
#endif

  enableVPack();
  delay(100);
  v_sensed = ((3.3 / 1024.0) * (float)analogRead(PIN_VPACK_SENSE)) * 16.0;
  disableVPack();

#ifdef DEBUG
  Serial.print("Pack Voltage (Sensed): ");
  Serial.println(v_sensed);
#endif
  
  /* CHARGING */
  if (charger_connected) {
    if (digitalRead(PIN_CHARGE_IND)) {
      Serial.println("Charge State: Charging");
      
      /*
       * Keep looping and cell balancing while we
       * are still drawing charge current and VBAT is
       * below charge threshold.
       */
      if (v_pack < CONF_CHARGE_CUTOFF_THRESH) {
        if (top_cell > 0) {
          isl.enableCB(top_cell, true);
#ifdef DEBUG
          Serial.print("Enabling CB for cell ");
          Serial.println(top_cell);
#endif
        }
        expireTimeout();
      }
      else {
        /*
        * Stop charging
        */
        isl.enableCFET(false);
        isl.enableDFET(false);
        delay(100);
      }
    }
    else {
#ifdef DEBUG
      Serial.println("Charger Connected");
#endif

      /*
       * Charger is connected and voltage is below full charge
       * threshold, so start charging.
       */
      if (charge_started == false) {
        if (v_pack < CONF_CHARGE_START_THRESHOLD) {
          isl.enableCFET(true);
          isl.enableDFET(true);
          delay(100);
          expireTimeout();
          charge_started = true;
        }
      }
    }
  }
  /* DISCHARGING */
  else if (digitalRead(PIN_DISCHARGE_IND)) {
    if (v_pack < CONF_DISCHARGE_CUTOFF_THRESH) {
#ifdef DEBUG
      Serial.println("Discharge State: Shutdown (discharge threshold)");
#endif

      isl.enableCFET(false);
      isl.enableDFET(false);
      shutoff = true;
      error_light = true;
      delay(100);
    }
    else {
#ifdef DEBUG
      Serial.println("Discharge State: Discharging");
#endif

      if (top_cell > 0 && CONF_BAL_DISCHARGE == 1) {
        isl.enableCB(top_cell, true);
#ifdef DEBUG
        Serial.print("Enabling CB for cell ");
        Serial.println(top_cell);
#endif
      }

      /*
      * There is a load so keep looping while it is
      * drawing current.
      */
      expireTimeout();
    }
  }
  else if (shutoff == false) {
#ifdef DEBUG
    Serial.println("Discharge State: Load Test");
#endif

    if (v_pack > CONF_DISCHARGE_CUTOFF_THRESH) {
      /*
      * Disable CFET and enable DFET, if there is a load
      * then the discharge pin will be pulled high and
      * keep the loop going.
      */
      isl.enableCFET(false);
      isl.enableDFET(true);

      delay(100);
    }
    else {
#ifdef DEBUG
      Serial.println("Discharge State: Under-volt Lockout");
      shutoff = true;
      error_light = true;
#endif
    }
  }

  if ((millis() - counter) > CONF_SLEEP_TIMEOUT) {
#ifdef DEBUG
    Serial.println("Timed out, sleeping...");
#endif
    delay(1000);
    isl.sleep();
  }
}