// Heltec WiFi Kit 32
#include "BluetoothSerial.h"
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>
#include <utility/imumaths.h>
#define NMEA_END_CHAR_1 '\n'
#define NMEA_MAX_LENGTH 70

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define MAX(x, y) (x > y ? x : y)
#define MIN(x, y) (x < y ? x : y)
#define RESTRICT_RANGE(x, low, high) (MIN(MAX(x, low), high))
#define APROX_ZERO(x) (x > 0.001 || x < -0.001)

typedef struct index_buffer {
  char buf[128];
  int idx;
} index_buffer_t;

BluetoothSerial SerialBT;

Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
// Adafruit_ADS1015 ads;     /* Use thi for the 12-bit version */

Preferences Calibration;

// the OLED used
// U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /*
// reset=*/ 16);

const static int DIR = 12;                  // direction yellow wire
const static int PWM = 13;                  // PWM green wire
const static int RM_DRIVE_PLUS_PIN = 27;     // yellow to blue/white wire
const static int RM_DRIVE_MINUS_PIN = 26;    // green to green/white wire
const static int RM_CLUTCH_INPIN = 25;      // white to brown/white wire
const static int CLUTCH_OUTPIN = 33;        // blue wire
const static int CLUTCH_ENABLED_IN_PIN = 34; //
// seems to be a don't care, high or low works, gray wire
const static int CLUTCH_DIR_PIN = 32;
const static int ALERT_PIN = 35; // ADS1115 ALRT pin
const static int FREQUENCY = 10 * 1000;
const static int PWM_CLANNEL = 0;
const static int PWM_RESOLUTION = 8;

const char compile_date[] = __DATE__ " " __TIME__;

static int dutyCycle = 128;
static int loopCount = 0;
static float oldAngle = -45.0;
static float new_angle = 0.0;
static int old_clutch = LOW;
static String inString = "";             // string to hold input
static unsigned long previousMillis = 0; //
static float oldCompass = -360.0;
static float RM_gain = 0.1;

/* create a server and listen on port 8088 */
// WiFiServer server(tcpPort);

// Use WiFiClient class to create TCP connections
//    WiFiClient client;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (50)

// Check I2C device address and correct line below (by default address is 0x29
// or 0x28)
//                                   id, address
// Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
Adafruit_BNO055 bno = Adafruit_BNO055(1390);

void setup(void) {

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(CLUTCH_OUTPIN, OUTPUT);
  pinMode(CLUTCH_DIR_PIN, OUTPUT);
  pinMode(RM_DRIVE_PLUS_PIN, INPUT);
  pinMode(RM_DRIVE_MINUS_PIN, INPUT);
  pinMode(RM_CLUTCH_INPIN, INPUT);
  pinMode(CLUTCH_ENABLED_IN_PIN, INPUT);
  pinMode(ALERT_PIN, INPUT);
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(PWM, LOW);
  digitalWrite(DIR, LOW);
  digitalWrite(CLUTCH_DIR_PIN, LOW);
  digitalWrite(CLUTCH_OUTPIN, HIGH);

  // configure PWM functionalitites
  ledcSetup(PWM_CLANNEL, FREQUENCY, PWM_RESOLUTION);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(PWM, PWM_CLANNEL);
  ledcWrite(PWM_CLANNEL, 0);

  Serial.begin(115200);
  Serial2.begin(38400);
  SerialBT.begin("ESP32test"); // Bluetooth device name

  Serial.println("The device started, now you can pair it with bluetooth!");
  Serial.println("Hello!");
  Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015
  //                                                                ADS1115
  //                                                                -------
  //                                                                -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV 0.1875mV
  // (default)
  ads.setGain(GAIN_ONE); // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV 0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV
  // 0.03125mV ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit =
  // 0.25mV   0.015625mV ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V
  // 1 bit = 0.125mV  0.0078125mV

  ads.begin(); // Configures the ADS1115 to the default mode

  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print(
        "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  Calibration.begin("offsets", false);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  int32_t bnoID = Calibration.getInt("bnoID");
  bool foundCalib = false;
  /*
     Look for the sensor's unique ID at the beginning oF EEPROM.
     This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id) {
    Serial.println("\nNo Calibration Data for this sensor exists in NVS");
    delay(500);
  } else {
    Serial.println("\nFound Calibration for this sensor in NVS.");

    // Accelerometer offsets
    calibrationData.accel_offset_x = Calibration.getInt("acc_off_x");
    calibrationData.accel_offset_y = Calibration.getInt("acc_off_y");
    calibrationData.accel_offset_z = Calibration.getInt("acc_off_z");

    // Gyroscrope offsets
    calibrationData.gyro_offset_x = Calibration.getInt("gyr_off_x");
    calibrationData.gyro_offset_y = Calibration.getInt("gyr_off_y");
    calibrationData.gyro_offset_z = Calibration.getInt("gyr_off_z");

    // Magnetometer offsets
    calibrationData.mag_offset_x = Calibration.getInt("mag_off_x");
    calibrationData.mag_offset_y = Calibration.getInt("mag_off_y");
    calibrationData.mag_offset_z = Calibration.getInt("mag_off_z");

    displaySensorOffsets(calibrationData);

    Serial.println("\n\nRestoring Calibration data to the BNO055...");
    bno.setSensorOffsets(calibrationData);

    Serial.println("\n\nCalibration data loaded into BNO055");
    foundCalib = true;
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println(
      "Calibration status values: 0=uncalibrated, 3=fully calibrated");

  delay(1000);
}

void loop(void) {
  char hdgBuf[128];
  index_buffer_t nmeaBuf = {0};
  float angle = getRudderAngle();
  unsigned long currentMillis = millis();
  while (Serial2.available()) {
    if (getNMEA(&nmeaBuf) && strncmp(nmeaBuf.buf, "$HCHDG", 6) == 0) {
      char nmeaOutBuf[128];
      sprintf(nmeaOutBuf, "$CHRSA,%f,A,,*", getRudderAngle());
      uint8_t chk = nmea_get_checksum(nmeaOutBuf);
      sprintf(nmeaOutBuf, "%s%x", nmeaOutBuf, chk);
      Serial.println(nmeaOutBuf); // debug
      Serial2.println(nmeaOutBuf);
    }
  }

  if (currentMillis - previousMillis >= BNO055_SAMPLERATE_DELAY_MS) {
    previousMillis = currentMillis;
    float compass = compassAngle();
    float rateOfTurn = oldCompass - compass;

    if (APROX_ZERO(rateOfTurn)) {
      Serial.print("ROT\t");
      Serial.print(compass);
      Serial.print("\t");
      Serial.print(oldCompass);
      Serial.print("\t");
      Serial.println(rateOfTurn);
    }
    oldCompass = compass;
  }

  int pos = 0;
  char command;
  while (SerialBT.available() > 0) {
    int inChar = SerialBT.read();

    if (inChar != '\n') {
      // As long as the incoming byte is not a newline, convert the incoming
      // byte to a char and add it to the string
      if (pos == 0) {
        command = (char)inChar;
        pos++;
      } else {
        inString += (char)inChar;
        pos++;
      }
    }
    // if you get a newline, print the string,
    // then the string's value as a float:
    else {
      Serial.print("Command: ");
      Serial.print(command);
      Serial.print("\tInput string: ");
      Serial.print(inString);
      Serial.print("\tAfter conversion to float:");
      float num_arg = inString.toFloat();
      Serial.println(num_arg, 4);

      switch (command) {
      case 'V':
        SerialBT.print("Build date: ");
        SerialBT.println(compile_date);
        break;
      case 'G':
        RM_gain = num_arg;
        break;
      case 'R':
        Serial.println(getRudderAngle());
        Serial.println(compassAngle());
        SerialBT.println(getRudderAngle());
        SerialBT.println(compassAngle());
        break;
      case 'A':
        new_angle = num_arg;
        break;
      case 'D':
        new_angle += num_arg;
        break;
      // Drive to GPS heading.
      // Stay on present compass heading.
      // Stay on present GPS heading.
      // Stay on present wind heading.
      // Tack left 120 degrees.
      // Tack right 120 defrees.
      // Drive to GPS waypoint.
      }

      // clear the string for new input:
      inString = "";
    }
  }

  // Serial.println(digitalRead(RM_CLUTCH_INPIN));
  if (digitalRead(RM_CLUTCH_INPIN) == HIGH) {
    if (digitalRead(RM_DRIVE_PLUS_PIN) == HIGH)
      new_angle = new_angle + RM_gain;
    if (digitalRead(RM_DRIVE_MINUS_PIN) == HIGH)
      new_angle = new_angle - RM_gain;
    Serial.print("Angle, new angle: ");
    Serial.println(angle, 4);
    Serial.println(new_angle, 4);
  }

  new_angle = RESTRICT_RANGE(new_angle, -30.0, 30.0);

  int clutch = digitalRead(CLUTCH_ENABLED_IN_PIN);
  if (!clutch)
    ledcWrite(PWM_CLANNEL, 0);
  if (old_clutch == LOW && clutch == HIGH) {
    new_angle = getRudderAngle();
    old_clutch = clutch;
  }
  if (old_clutch == HIGH && clutch == LOW) {
    old_clutch = clutch;
  }
  digitalWrite(LED_BUILTIN, clutch);

  float over_under = 0.5;
  float drive_over_under = 0.25;
  if (oldAngle != new_angle) {
    over_under = drive_over_under;
  }

  dutyCycle = abs((int)angle - (int)new_angle);
  dutyCycle = dutyCycle * loopCount / 5;
  dutyCycle = RESTRICT_RANGE(dutyCycle, 32, 255);

  if (abs(angle - new_angle) > over_under) {
    digitalWrite(DIR, angle > new_angle ? HIGH : LOW);
    if (clutch) {
      ledcWrite(PWM_CLANNEL, dutyCycle);
    }
    delay(20);
    loopCount++;
  } else {
    ledcWrite(PWM_CLANNEL, 0);
    loopCount = 0;
    oldAngle = new_angle;
  }

  /*
  if (angle > new_angle + abs(over_under)) {
    digitalWrite(DIR, HIGH);
    if (clutch)
      ledcWrite(PWM_CLANNEL, dutyCycle);
    delay(20);
    loopCount++;
  } else if (angle < new_angle - abs(over_under)) {
    digitalWrite(DIR, LOW);
    if (clutch)
      ledcWrite(PWM_CLANNEL, dutyCycle);
    delay(20);
    loopCount++;
  } else {
    // Serial.println("d3");
    // digitalWrite(PWM, LOW);
    ledcWrite(PWM_CLANNEL, 0);
    loopCount = 0;
    oldAngle = new_angle;
    // oldAngle = new_angle;
  }
  */

  delay(2);
}

float compassAngle(void) {
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  /*Serial.print("X: ");
    Serial.print(euler.x());
    Serial.print(" Y: ");
    Serial.print(euler.y());
    Serial.print(" Z: ");
    Serial.println(euler.z());*/
  //  Serial.print("\t\t");

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  /*Serial.print("CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.println(mag, DEC);*/

  //  char buf[16];

  //  sprintf(buf, "%d.%d.%d.%d", system, gyro, accel, mag );

  //  u8x8.drawString(0, 1, buf);

  // delay(BNO055_SAMPLERATE_DELAY_MS);
  return euler.x();
}

float getRudderAngle(void) {
  int16_t results;

  /* Be sure to update this value based on the IC and the gain settings! */
  // float   multiplier = 3.0F;    /* ADS1015 @ +/- 6.144V gain (12-bit results)
  // */
  float multiplier = 0.125F; /* ADS1115  @ +/- 4.096V gain (16-bit results) */

  results = ads.readADC_Differential_0_1();
  float p3 = 0.00000000008584467813;
  float p2 = -0.000005696669841;
  float p1 = 0.1359565313;
  float p0 = -1147.171616;
  p3 = p3 * results * results * results;
  p2 = p2 * results * results;
  p1 = p1 * results;
  //  angle = 8.584467813*10^-11*results^3 - 5.696669841*10^-6*results^2
  //  + 1.359565313*10^-1*results - 1147.171616
  return (p3 + p2 + p1 + p0 +
          4.9); // need to fix. Not sure why I now need to add 4.9 degrees
  //  angle = pow(x, y);
}
/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void) {
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
*/
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData) {
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x);
  Serial.print(" ");
  Serial.print(calibData.accel_offset_y);
  Serial.print(" ");
  Serial.print(calibData.accel_offset_z);
  Serial.print(" ");

  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x);
  Serial.print(" ");
  Serial.print(calibData.gyro_offset_y);
  Serial.print(" ");
  Serial.print(calibData.gyro_offset_z);
  Serial.print(" ");

  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x);
  Serial.print(" ");
  Serial.print(calibData.mag_offset_y);
  Serial.print(" ");
  Serial.print(calibData.mag_offset_z);
  Serial.print(" ");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.print(calibData.mag_radius);
}

bool getNMEA(index_buffer_t *nmeaBuf) {
  char nmea_in = Serial2.read();
  if (nmea_in == '$') {
    nmeaBuf->idx = 0;
  }
  nmeaBuf->buf[nmeaBuf->idx++] = nmea_in;
  if (nmea_in == '*') {
    nmeaBuf->buf[nmeaBuf->idx] = '\0';
    return true;
  } else {
    return false;
  }
}

uint8_t nmea_get_checksum(const char *sentence) {
  const char *n = sentence + 1; // Plus one, skip '$'
  uint8_t chk = 0;

  /* While current char isn't '*' or sentence ending (newline) */
  while ('*' != *n && NMEA_END_CHAR_1 != *n) {
    if ('\0' == *n || n - sentence > NMEA_MAX_LENGTH) {
      /* Sentence too long or short */
      return 0;
    }
    chk ^= (uint8_t)*n;
    n++;
  }

  return chk;
}
