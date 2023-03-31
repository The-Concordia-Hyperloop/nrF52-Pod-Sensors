#include <ArduinoBLE.h>
#include <Wire.h>
#include <SPI.h>
#include <LSM6DS3.h>
#include <Adafruit_BMP280.h>

#define DEBUG false
#define PIN_VBAT (32u)
const double vRef = 3.3; // 3.3V regulator output is ADC reference voltage

BLEService stm32ServerService; 
BLECharacteristic stm32ServerCharacteristicImu; 
BLECharacteristic stm32ServerCharacteristicPressure; 
BLECharacteristic stm32ServerCharacteristicBattery; 
const String STM_SERVER_NAME = "STM32 Hyperloop"; 
const char STM_SERVICE_UUID[] = "a000";
const char IMU_CHARACTERISTIC_UUID[] = "a001";
const char PRESSURE_CHARACTERISTIC_UUID[] = "a002";
const char BATT_CHARACTERISTIC_UUID[] = "a003";
Adafruit_BMP280 bmp;
bool temp = false;

LSM6DS3Core myIMU(I2C_MODE, 0x6A);          //I2C device address 0x6A
uint8_t dataToWrite = 0;                    //Temporary variable
uint16_t errorsAndWarnings = 0;
u_int8_t test = 0x04;
u_int8_t pressure;
u_int8_t battery;
int reset_counter = 0;


void setCharacteristics(BLEService service) {
  for (int i = 0; i < service.characteristicCount(); i++) {
    BLECharacteristic characteristic = service.characteristic(i);
    if (strcmp(IMU_CHARACTERISTIC_UUID, characteristic.uuid()) == 0) {
      stm32ServerCharacteristicImu = characteristic;
      if (DEBUG) {
        Serial.println("-----------------------------------");
        Serial.println("Set stm32ServerCharacteristicImu.");
        Serial.println("-----------------------------------");
      }
    }
    if (strcmp(PRESSURE_CHARACTERISTIC_UUID, characteristic.uuid()) == 0) {
      stm32ServerCharacteristicPressure = characteristic;
      if (DEBUG) {
        Serial.println("-----------------------------------");
        Serial.println("Set stm32ServerCharacteristicPressure.");
        Serial.println("-----------------------------------");
      }
    }
    if (strcmp(BATT_CHARACTERISTIC_UUID, characteristic.uuid()) == 0) {
      stm32ServerCharacteristicBattery = characteristic;
      if (DEBUG) {
        Serial.println("-----------------------------------");
        Serial.println("Set stm32ServerCharacteristicBattery.");
        Serial.println("-----------------------------------");
      }
    }
  }
}

void setServices(BLEDevice peripheral) {
  if (DEBUG) {
    Serial.println("Connecting ...");
  }

  if (peripheral.connect()) {
    if (DEBUG) {
      Serial.println("Connected");
    }
  } else {
    if (DEBUG) {
      Serial.println("Error connecting.");
    }    
    return;
  }

  if (peripheral.discoverAttributes()) {
    if (DEBUG) {
      Serial.println("Attributes discovered");
    }
  } else {
    if (DEBUG) {
      Serial.println("Attribute discovery failed!");
    }
    peripheral.disconnect();
    return;
  }

  /*
  loop the services of the peripheral and explore each, 
  when the match is found for the service we expect then assign it to stm32ServerService
  */ 
  for (int i = 0; i < peripheral.serviceCount(); i++) {
    BLEService service = peripheral.service(i);
    if (strcmp(STM_SERVICE_UUID, service.uuid()) == 0) {
      stm32ServerService = service;
      if (DEBUG) {
        Serial.println("-----------------------------------");
        Serial.println("Set stm32ServerService.");
        Serial.println("-----------------------------------");
      }
      return;
    }
  }
}

int16_t* getIMUData() {
    int16_t temp;
    static int16_t data[3] = { 0, 0, 0 }; // Initialize array with some values
    //Acelerometer axis X
    if (myIMU.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTX_L_XL) != 0) {
        errorsAndWarnings++;
    }
    data[0] = temp;
    //Acelerometer axis Y
    if (myIMU.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTY_L_XL) != 0) {
        errorsAndWarnings++;
    }
    data[1] = temp;


    //Acelerometer axis Z
    if (myIMU.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTZ_L_XL) != 0) {
        errorsAndWarnings++;
    }
    data[2] = temp;

    return data;
}

void sendImuData(int16_t arr[]) {
    // Converts u_int16_t to byte array
    uint8_t bytes[6];
    for (int i = 0; i < 3; i++) {
        bytes[i*2] = arr[i] & 0xFF;
        bytes[i*2 + 1] = (arr[i] >> 8) & 0xFF;
    }

    // Send byte array using characteristic write
    stm32ServerCharacteristicImu.writeValue(bytes, 6);
}

double getBatteryVoltage() {
  unsigned int adcVal = analogRead(PIN_VBAT);
  double adcVoltage = (adcVal * vRef) / 1024; // (1024 = 2^10 (10 bit adc))
  return adcVoltage*1510.0/510.0; 
}

int getBatteryPercentage() {
  double maxVoltage = 4.20;
  double currentVoltage = getBatteryVoltage();
  battery = (int) currentVoltage/maxVoltage * 100;
  return (int) currentVoltage/maxVoltage * 100;
}

void softReset() {
    NVIC_SystemReset();
}

void setup() {

if (DEBUG){
  Serial.begin(9600);
  while (!Serial);
}
pinMode(LED_BUILTIN, OUTPUT);
digitalWrite(LED_BUILTIN, LOW);

  if (!bmp.begin(0x77)) {
    if (DEBUG) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    }
  }

      //Call .beginCore() to configure the IMU
    if (myIMU.beginCore() != 0) {
        if (DEBUG) {
        Serial.print("\nDevice Error.\n");
        }
    } else {
        if (DEBUG) {
        Serial.print("\nDevice Good.\n");
        }
    }

    //Setup the accelerometer******************************
    dataToWrite = 0; //Start Fresh!
    dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
    dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
    dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;

    //Now, write the patched together data
    errorsAndWarnings += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  // begin initialization
  while (!BLE.begin()) {
    if (DEBUG) {
        Serial.print("\nFailed to startup!.\n");
    }
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }

  BLE.scan();
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);
  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    reset_counter++;
    if (DEBUG) {
      Serial.print("Searching... ");
      Serial.println(peripheral.address());
    }
    if (reset_counter > 1000) {
      for (int i = 0; i < 20; i++) {
          digitalWrite(LED_BUILTIN, LOW);
          delay(25);
          digitalWrite(LED_BUILTIN, HIGH);
          delay(25);
      }
      NVIC_SystemReset();
    }
    if (peripheral.localName() == STM_SERVER_NAME) {
      BLE.stopScan();
      setServices(peripheral);                                          // <-- sets the service
      setCharacteristics(stm32ServerService);                           // <-- sets the characteristic 
      digitalWrite(LED_BUILTIN, HIGH);
      while (1) {
        pressure = (bmp.readPressure() / 100.0F)*0.1;
        getBatteryPercentage();
          if (stm32ServerCharacteristicPressure.canWrite()) {
              stm32ServerCharacteristicPressure.writeValue(pressure);
          }       
          
          if (stm32ServerCharacteristicImu.canWrite()) {
              sendImuData(getIMUData());
          }   

          if (stm32ServerCharacteristicBattery.canWrite()) {
              stm32ServerCharacteristicBattery.writeValue(battery);
          }   

          delay(20);      // ~50Hz
          if (!BLE.connected()) {
            NVIC_SystemReset();
      }
    }
  }
}
}