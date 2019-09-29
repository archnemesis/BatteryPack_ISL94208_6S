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
#include <Timer.h>
#include <EEPROM.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define ISL94208_ADDR 0x28

#define DEBUG 1

#define VIN 3.333
#define TEMP_R1 46400
#define TEMP_B 3988
#define TEMP_T0 298.15
#define TEMP_R0 10000

#define PIN_LED_D3 3
#define PIN_LED_D2 2
#define PIN_LED_PL1 4
#define PIN_LED_PL2 5
#define PIN_LED_PL3 9
#define PIN_LED_PL4 10 
#define PIN_VPACK_ENABLE 8
#define PIN_ISL94208_AO 0
#define PIN_CHARGE_SENSE 1
#define PIN_DISCHARGE_SENSE 2
#define PIN_VPACK_SENSE 3

#define CONF_CHARGE_CURRENT_MIN 0.1       // minimum charge current to keep alive
#define CONF_DISCHARGE_CURRENT_MIN 0.1    // minimum discharge current to keep alive
#define CONF_CHARGE_START_THRESHOLD 25.1  // don't start charge above this voltage
#define CONF_CHARGE_CUTOFF_THRESH 25.452    // charge cutoff voltage
#define CONF_DISCHARGE_CUTOFF_THRESH 15.6 // discharge cutoff voltage
#define CONF_BAD_CELL_THRESH 2.0          // min cell voltage before pack shutdown
#define CONF_SLEEP_TIMEOUT 9000           // put ISL94208 to sleep after this many millis
#define CONF_BAL_CHARGE 1                 // balance cells on charge cycle
#define CONF_BAL_DISCHARGE 0              // balance cells on discharge cycle
#define CONF_OC_CHARGE_THRESH ISL94208_OC_CHG_THRESH_120MV
#define CONF_OC_DISCHARGE_THRESH ISL94208_OC_CHG_THRESH_120MV
#define CONF_DISCHARGE_CURRENT_OFFSET -0.01

struct eeprom_data_t {
  unsigned char a;
  uint32_t cycle_count;
  uint32_t current_charge;
  double discharge_offset;
  unsigned char b;
} eeprom_data;

struct cell_voltage_t {
  uint8_t cell;
  double voltage;
};

ISL94208 isl(ISL94208_ADDR);

Timer timer;

unsigned long int counter = 0;
bool charger_connected = false;
bool charge_started = false;
bool shutoff = false;
bool error_light = false;
bool in_overcurrent = false;
uint8_t adc_channel = 0;
uint16_t adc_samples[4] = { 0 };
double charge_coulomb = 0.0;
double discharge_coulomb = 0.0;
double discharge_offset = 0.0;
double v_sensed = 0.0;

bool loadEEPROM();
void saveEEPROM();
void calibrateDischargeSense();
void measureCurrent();
void expireTimeout();
void enableVPack();
void disableVPack();
void sleep();
void setup();
void loop();
void control();

bool loadEEPROM()
{
  EEPROM.get(0, eeprom_data);

  if (eeprom_data.a == 'A' && eeprom_data.b == 'B') {
    return true;
  }
  else {
    memset((void *)&eeprom_data, 0, sizeof(eeprom_data_t));
    return false;
  }
}

void saveEEPROM()
{
  eeprom_data.a = 'A';
  eeprom_data.b = 'B';
  EEPROM.put(0, eeprom_data);
}

void measureCurrent()
{
  double v;
  double i;
  double c;

  enableVPack();
  v_sensed = ((3.3 / 1024.0) * (float)analogRead(PIN_VPACK_SENSE)) * 16.0;
  disableVPack();

  v = (3.3 / 1024.0) * analogRead(PIN_CHARGE_SENSE);
  i = (v / 20) / 0.1;
  c = i * 0.01;
  charge_coulomb += c;
  
  v = ((3.3 / 1024.0) * analogRead(PIN_DISCHARGE_SENSE));
  i = ((v / (1+(91.0/4.7))) / 0.01) - discharge_offset;
  c = i * 0.01;
  discharge_coulomb += c;
}

void calibrateDischargeSense()
{
  double v, i, avg = 0;
  
#ifdef DEBUG
  Serial.println("Calibrating Discharge Offset...");
#endif

  for (int j = 0; j < 10; j++) {
    v = ((3.3 / 1024.0) * analogRead(PIN_DISCHARGE_SENSE));
    i = ((v / (1+(91.0/4.7))) / 0.01);
    avg += i;
  }

  avg /= 10;
  
#ifdef DEBUG
  Serial.print("Discharge Sense Offset: ");
  Serial.println(avg);
#endif

  discharge_offset = avg;
}

int compareCellVoltages(const void *a, const void *b)
{
  if (((struct cell_voltage_t *)a)->voltage < ((struct cell_voltage_t *)b)->voltage) return -1;
  else if (((struct cell_voltage_t *)a)->voltage == ((struct cell_voltage_t *)b)->voltage) return 0;
  else return 1;
}

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

void sleep()
{
  saveEEPROM();
  
  isl.enableCFET(false);
  isl.enableDFET(false);
  isl.sleep(false);
  isl.sleep();
}

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("");
  Serial.println("BatteryPack_ISL94208_6S V1.0");
  Serial.println("(C) 2019 by R. Gingras");
#endif

  pinMode(PIN_LED_D2, OUTPUT);
  pinMode(PIN_LED_D3, OUTPUT);
  pinMode(PIN_VPACK_ENABLE, OUTPUT);
  pinMode(PIN_LED_PL1, OUTPUT);
  pinMode(PIN_LED_PL2, OUTPUT);
  pinMode(PIN_LED_PL3, OUTPUT);
  pinMode(PIN_LED_PL4, OUTPUT);

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
  
#ifdef DEBUG
  /*
   * Helpful to reset ISL94208 state when debugging.
   */
  isl.enableCFET(false);
  isl.enableDFET(false);
  isl.disableAllCB();
  delay(500);
#endif

  //isl.setUserFlag0(false);

  if (isl.readUserFlag0() == 0) {
#ifdef DEBUG
    Serial.println("UserFlag0 not set, performing first-run intialization...");
#endif
    calibrateDischargeSense();
    
    eeprom_data.current_charge = 0;
    eeprom_data.cycle_count = 0;
    eeprom_data.discharge_offset = discharge_offset;
    saveEEPROM();

    isl.setUserFlag0();
  }
  else {
    if (!loadEEPROM()) {
  #ifdef DEBUG
      Serial.println("No data present or invalid data.");
  #endif
    }

    discharge_offset = eeprom_data.discharge_offset;
  }

#ifdef DEBUG
  Serial.println("Loading EEPROM data...");
#endif


#ifdef DEBUG
  Serial.print("Current Charge Level: ");
  Serial.println(eeprom_data.current_charge);
  Serial.print("Cycle Count: ");
  Serial.println(eeprom_data.cycle_count);
#endif

  timer.every(500, control);
  timer.every(10, measureCurrent);

  /*
   * Configure ISL94208
   */
  isl.enableChargeSetWrites(true);
  isl.setOverCurrentChargeThreshold(ISL94208_OC_CHG_THRESH_160MV);
  isl.enableChargeSetWrites(false);

  isl.enableDischargeSetWrites(true);
  isl.setOverCurrentDischargeThreshold(ISL94208_OC_CHG_THRESH_120MV);
  isl.setShortCircuitDischargeThreshold(ISL94208_SHORT_CIRCUIT_THRESH_650MV);
  isl.enableDischargeSetWrites(false);

  /*
   * Determine if the charger was present on wakeup
   */
  charger_connected = isl.readWkupFlag();

  expireTimeout();
}

void loop() {
  timer.update();
}

// the loop function runs over and over again forever
void control() {
  float v_pack = 0.0;
  float v_cell = 0.0;
  float temperature = 0.0;
  uint16_t adc_val = 0;
  struct cell_voltage_t cell_list[6] = { 0 };

  isl.disableAllCB();

  for (uint8_t i = 0; i < 6; i++) {
    isl.selectAnalogOutput(i+1);
    v_cell = adc2voltage(analogRead(PIN_ISL94208_AO)) * 2.0;

    if (v_cell < CONF_BAD_CELL_THRESH) {
      // shut down FETs, show error light, sleep
    }

    cell_list[i].cell = i+1;
    cell_list[i].voltage = v_cell;
    v_pack += v_cell;
  }

  qsort(cell_list, 6, sizeof(struct cell_voltage_t), compareCellVoltages);

#ifdef DEBUG
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print("VCELL");
    Serial.print(cell_list[i].cell);
    Serial.print(": ");
    Serial.println(cell_list[i].voltage);
  }
#endif

  isl.selectAnalogOutput(8);
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

#ifdef DEBUG
  Serial.print("Pack Voltage (Sensed): ");
  Serial.println(v_sensed);
  
  Serial.print("Charge In (Amp-Seconds): ");
  Serial.println(charge_coulomb);

  Serial.print("Charge Out (Amp-Seconds): ");
  Serial.println(discharge_coulomb);
#endif

  digitalWrite(PIN_LED_PL1, (v_sensed > 15.0));
  digitalWrite(PIN_LED_PL2, (v_sensed > 17.55));
  digitalWrite(PIN_LED_PL3, (v_sensed > 20.1));
  digitalWrite(PIN_LED_PL4, (v_sensed > 22.65));

  if (isl.readDischargeOCFlag() == 1) {
    error_light = true;
    in_overcurrent = true;
    isl.enableVMON();
    delay(1);
  }

  if (isl.readChargeOCFlag()) {
    Serial.println("Overcurrent!");
    error_light = true;
  }
  else if (in_overcurrent) {
#ifdef DEBUG
    Serial.println("Discharge Over-Current Condition Detected");
#endif

    if (isl.readLDFailFlag() == 1) {
#ifdef DEBUG
      Serial.println("LDFAIL: load still present");
#endif
    }
    else {
#ifdef DEBUG
      Serial.println("LDFAIL: load apparently removed");
#endif
      in_overcurrent = false;
      error_light = false;
    }
  }

  uint16_t adc_charge_sense = analogRead(PIN_CHARGE_SENSE);
  double v_charge = (3.3 / 1024) * adc_charge_sense;
  double i_charge = (v_charge / 20.0) / 0.1;

#ifdef DEBUG
  Serial.print("Charge Current: ");
  Serial.println(i_charge);
#endif

  uint16_t adc_discharge_sense = analogRead(PIN_DISCHARGE_SENSE);
  double v_discharge = ((3.3 / 1024) * adc_discharge_sense);
  double i_discharge = ((v_discharge / (1+(91.0/4.7))) / 0.01) - discharge_offset;

#ifdef DEBUG
  Serial.print("Discharge Current: ");
  Serial.println(i_discharge);
#endif
  
  /* CHARGING */
  if (charger_connected) {
    if (i_charge > CONF_CHARGE_CURRENT_MIN) {
#ifdef DEBUG
      Serial.println("Charge State: Charging");
#endif

      digitalWrite(PIN_LED_D3, HIGH);
      digitalWrite(PIN_LED_D2, LOW);
      
      /*
       * Keep looping and cell balancing while we
       * are still drawing charge current and VBAT is
       * below charge threshold.
       */
      if (v_sensed < CONF_CHARGE_CUTOFF_THRESH) {
        isl.enableCB(cell_list[5].cell, true);
        isl.enableCB(cell_list[4].cell, true);
#ifdef DEBUG
        Serial.print("Enabling CB for cells ");
        Serial.print(cell_list[5].cell);
        Serial.print(" and ");
        Serial.println(cell_list[4].cell);
#endif
        expireTimeout();
      }
      else {
#ifdef DEBUG
      Serial.println("Charge State: Complete");
#endif
        /*
        * Stop charging
        */
        isl.enableCFET(false);
        isl.enableDFET(false);
        digitalWrite(PIN_LED_D3, LOW);
        eeprom_data.current_charge += (uint32_t)round(charge_coulomb);
        sleep();
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
        if (v_sensed < CONF_CHARGE_START_THRESHOLD) {
          isl.enableCFET(true);
          isl.enableDFET(true);
          delay(1);
          expireTimeout();
          charge_started = true;
        }
      }
      /*
       * Power was removed or charge current dropped below threshold.
       */
      else {
#ifdef DEBUG
      Serial.println("Charge State: Stopped");
#endif

        isl.enableCFET(false);
        isl.enableDFET(false);
        digitalWrite(PIN_LED_D3, LOW);
        eeprom_data.current_charge += (uint32_t)round(charge_coulomb);
        sleep();
      }
    }
  }
  /* DISCHARGING */
  else if (i_discharge > CONF_DISCHARGE_CURRENT_MIN) {
    if (v_pack < CONF_DISCHARGE_CUTOFF_THRESH) {
#ifdef DEBUG
      Serial.println("Discharge State: Shutdown (discharge threshold)");
#endif

      isl.enableCFET(false);
      isl.enableDFET(false);
      digitalWrite(PIN_LED_D2, LOW);
      eeprom_data.current_charge = 0;
      sleep();
    }
    else {
#ifdef DEBUG
      Serial.println("Discharge State: Discharging");
      Serial.print("Charge Level: ");
      Serial.println(eeprom_data.current_charge - (uint32_t)round(discharge_coulomb));
#endif
      digitalWrite(PIN_LED_D2, HIGH);
      digitalWrite(PIN_LED_D3, LOW);

      /*
      * There is a load so keep looping while it is
      * drawing current.
      */
      expireTimeout();
    }
  }
  else if (in_overcurrent == false) {
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

      delay(1);
    }
    else {
#ifdef DEBUG
      Serial.println("Discharge State: Under-volt Lockout");
      isl.enableCFET(false);
      isl.enableDFET(false);
      digitalWrite(PIN_LED_D2, LOW);
      sleep();
#endif
    }
  }

  if ((millis() - counter) > CONF_SLEEP_TIMEOUT) {
#ifdef DEBUG
    Serial.println("Timed out, sleeping...");
#endif

    eeprom_data.current_charge -= discharge_coulomb;
    sleep();
  }
}