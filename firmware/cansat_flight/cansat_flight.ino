#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <PWMServo.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define MPU_ADDR 0x68
#define SERVO_PIN 2

// Must match TEAM_ID in gs_logic.py exactly, or the ground station discards
// every packet as "foreign team id".
#define TEAM_ID "2026-IN-SPACe-CAN-7USAT-056"

// Battery sense. Set to 1 once a divider is wired to VBAT_PIN; until then the
// packet carries 0.00 V rather than a floating-pin reading.
#define USE_VBAT_SENSE 0
#define VBAT_PIN       A0
#define VBAT_DIVIDER   2.0     // (R1+R2)/R2 of the divider feeding VBAT_PIN
#define ADC_REF_V      3.3

Adafruit_BMP280 bmp(&Wire1);
PWMServo payloadServo;

unsigned long bootTime = 0;
bool actionTriggered = false;

unsigned long packetCount = 0;
unsigned long lastTxTime = 0;
const unsigned long txInterval = 500; // 2Hz Transmission

// Ground reference: ALTITUDE is transmitted relative to the launch site, so
// the sea-level altitude at boot is subtracted from every reading.
float groundAltitudeM = 0.0;

// Flight state code 0-7, matching STATE_NUMBER in gs_logic.py:
// 0 BOOT, 1 TEST_MODE, 2 LAUNCH_PAD, 3 ASCENT, 4 ROCKET_DEPLOY,
// 5 DESCENT, 6 AEROBREAK_RELEASE, 7 IMPACT
uint8_t flightState = 0;
float lastAltitudeM = 0.0;
float apogeeM = 0.0;

// Raw IMU variables
int16_t rawAccX, rawAccY, rawAccZ;
int16_t rawGyroX, rawGyroY, rawGyroZ;

// GNSS Data
String gpsTime = "00:00:00";
String gpsLatitude = "0.000000";
String gpsLongitude = "0.000000";
String gpsAltitude = "0.0";
int gpsSatellites = 0;
String nmeaBuffer = "";

void wakeUpMPU() {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x6B);
  Wire1.write(0x00);
  Wire1.endTransmission();
}

// Read one big-endian 16-bit word. The two Wire1.read() calls are sequenced
// through named locals on purpose: written as (read() << 8 | read()) the
// argument evaluation order is unspecified and the bytes can come out swapped.
int16_t readWord() {
  uint8_t hi = Wire1.read();
  uint8_t lo = Wire1.read();
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

void readRawMPU() {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x3B);
  Wire1.endTransmission(false);
  Wire1.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  if (Wire1.available() >= 14) {
    rawAccX  = readWord();
    rawAccY  = readWord();
    rawAccZ  = readWord();
    readWord();                 // on-chip temperature, unused
    rawGyroX = readWord();
    rawGyroY = readWord();
    rawGyroZ = readWord();
  }
}

void parseNMEA(String sentence) {
  if (sentence.startsWith("$GNGGA") || sentence.startsWith("$GPGGA")) {
    int commaIndex[15];
    int count = 0;

    for (int i = 0; i < sentence.length(); i++) {
      if (sentence.charAt(i) == ',') {
        commaIndex[count] = i;
        count++;
        if (count >= 15) break;
      }
    }

    if (count >= 10) {
      String rawTime = sentence.substring(commaIndex[0] + 1, commaIndex[1]);
      String rawLat = sentence.substring(commaIndex[1] + 1, commaIndex[2]);
      String latDir = sentence.substring(commaIndex[2] + 1, commaIndex[3]);
      String rawLon = sentence.substring(commaIndex[3] + 1, commaIndex[4]);
      String lonDir = sentence.substring(commaIndex[4] + 1, commaIndex[5]);
      String sats   = sentence.substring(commaIndex[6] + 1, commaIndex[7]);
      String alt    = sentence.substring(commaIndex[8] + 1, commaIndex[9]);

      // hhmmss.ss -> hh:mm:ss, the GNSS_TIME field the ground station logs.
      if (rawTime.length() >= 6) {
        gpsTime = rawTime.substring(0, 2) + ":" +
                  rawTime.substring(2, 4) + ":" +
                  rawTime.substring(4, 6);
      }

      if (rawLat.length() > 0 && rawLon.length() > 0) {
        float degLat = rawLat.substring(0, 2).toFloat();
        float minLat = rawLat.substring(2).toFloat();
        float decLat = degLat + (minLat / 60.0);
        if (latDir == "S") decLat = -decLat;
        gpsLatitude = String(decLat, 6);

        float degLon = rawLon.substring(0, 3).toFloat();
        float minLon = rawLon.substring(3).toFloat();
        float decLon = degLon + (minLon / 60.0);
        if (lonDir == "W") decLon = -decLon;
        gpsLongitude = String(decLon, 6);
      }

      if (alt.length() > 0) {
        gpsAltitude = String(alt.toFloat(), 1);
      }

      if (sats.length() > 0) {
        gpsSatellites = sats.toInt();
      }
    }
  }
}

void readGNSS() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == '\n') {
      parseNMEA(nmeaBuffer);
      nmeaBuffer = "";
    } else if (c != '\r') {
      nmeaBuffer += c;
    }
  }
}

float readBatteryVolts() {
#if USE_VBAT_SENSE
  return (analogRead(VBAT_PIN) / 1023.0) * ADC_REF_V * VBAT_DIVIDER;
#else
  return 0.0;
#endif
}

// Minimal altitude-driven state machine. The thresholds below are starting
// values — tune them to the actual flight profile before a graded flight.
void updateFlightState(float altitudeM, float velocityMS) {
  if (altitudeM > apogeeM) apogeeM = altitudeM;

  switch (flightState) {
    case 0:                                   // BOOT -> LAUNCH_PAD
      if (millis() - bootTime > 3000) flightState = 2;
      break;
    case 2:                                   // LAUNCH_PAD -> ASCENT
      if (altitudeM > 10.0 && velocityMS > 2.0) flightState = 3;
      break;
    case 3:                                   // ASCENT -> DESCENT past apogee
      if (velocityMS < -2.0 && altitudeM < apogeeM - 5.0) flightState = 5;
      break;
    case 5:                                   // DESCENT -> IMPACT
      if (altitudeM < 10.0 && velocityMS > -1.0) flightState = 7;
      break;
    default:
      break;
  }
}

void setup() {
  bootTime = millis();

  Serial.begin(9600);
  Serial1.begin(115200); // ESP32-CAM Trigger on Pins 0 (RX1) & 1 (TX1)
  Serial2.begin(9600);   // Quectel L89 GNSS on Pins 7 (RX2) & 8 (TX2)
  Serial5.begin(9600);   // XBee Pro S2C on Pins 20 (TX5) & 21 (RX5)

  while (!Serial && millis() < 3000);

  Wire1.begin();
  Wire1.setClock(100000);

  payloadServo.attach(SERVO_PIN);
  payloadServo.write(0);

  pinMode(LED_BUILTIN, OUTPUT);
  delay(500);

  Serial.println("\n--- CANSAT FLIGHT SYSTEM BOOTING ---");

  if (!bmp.begin(0x76)) {
    Serial.println("[FAIL] BMP280 not responding!");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH); delay(100);
      digitalWrite(LED_BUILTIN, LOW);  delay(100);
    }
  }
  Serial.println("[SUCCESS] BMP280 Online!");

  // Average a few samples so the ground reference is not set from one noisy read.
  float sum = 0.0;
  for (int i = 0; i < 10; i++) {
    sum += bmp.readAltitude(SEALEVELPRESSURE_HPA);
    delay(50);
  }
  groundAltitudeM = sum / 10.0;
  Serial.print("[INFO] Ground reference altitude: ");
  Serial.println(groundAltitudeM, 2);

  wakeUpMPU();
  Serial.println("[SUCCESS] MPU9250 Woken Up!");
  Serial.println("[SUCCESS] GNSS & Telemetry Interfaces Online!");
}

void loop() {
  readGNSS();

  // Trigger Servo 180 degrees & Camera Recording at T+5 seconds
  if (!actionTriggered && (millis() - bootTime >= 5000)) {
    payloadServo.write(180);
    Serial1.print('C');
    actionTriggered = true;
    Serial.println("[TRIGGER] Servo rotated 180 deg & ESP32-CAM video capture started!");
  }

  // Telemetry loop (2Hz)
  if (millis() - lastTxTime >= txInterval) {
    lastTxTime = millis();
    packetCount++;

    float missionTimeS = (millis() - bootTime) / 1000.0;
    float tempC        = bmp.readTemperature();
    float pressurePa   = bmp.readPressure();
    float altitudeM    = bmp.readAltitude(SEALEVELPRESSURE_HPA) - groundAltitudeM;
    float voltage      = readBatteryVolts();

    float velocityMS = (altitudeM - lastAltitudeM) / (txInterval / 1000.0);
    lastAltitudeM = altitudeM;
    updateFlightState(altitudeM, velocityMS);

    readRawMPU();
    float accX = rawAccX / 16384.0;
    float accY = rawAccY / 16384.0;
    float accZ = rawAccZ / 16384.0;
    float gyroX = rawGyroX / 131.0;
    float gyroY = rawGyroY / 131.0;
    float gyroZ = rawGyroZ / 131.0;

    // Field order must match _REQUIRED_FIELDS + _OPTIONAL_FIELDS in gs_logic.py.
    String telemetry = String(TEAM_ID) + ","          //  1. TEAM_ID
                     + String(missionTimeS, 1) + ","  //  2. TIME_STAMPING (s)
                     + String(packetCount) + ","      //  3. PACKET_COUNT
                     + String(altitudeM, 1) + ","     //  4. ALTITUDE (m, rel. ground)
                     + String(pressurePa, 0) + ","    //  5. PRESSURE (Pa, not hPa)
                     + String(tempC, 1) + ","         //  6. TEMP (C)
                     + String(voltage, 2) + ","       //  7. VOLTAGE (V)
                     + gpsTime + ","                  //  8. GNSS_TIME
                     + gpsLatitude + ","              //  9. GNSS_LATITUDE
                     + gpsLongitude + ","             // 10. GNSS_LONGITUDE
                     + gpsAltitude + ","              // 11. GNSS_ALTITUDE (m)
                     + String(gpsSatellites) + ","    // 12. GNSS_SATS
                     + String(accX, 2) + ","          // 13. ACC_R
                     + String(accY, 2) + ","          // 14. ACC_P
                     + String(accZ, 2) + ","          // 15. ACC_Y
                     + String(gyroX, 2) + ","         // 16. GYRO_R
                     + String(gyroY, 2) + ","         // 17. GYRO_P
                     + String(gyroZ, 2) + ","         // 18. GYRO_Y
                     + String(flightState) + ","      // 19. FLIGHT_SOFTWARE_STATE
                     + "0,"                           // 20. TVOC (ppb)
                     + "400,"                         // 21. eCO2 (ppm)
                     + "0.0"                          // 22. GYRO_SPIN_RATE (deg/s)
                     + "\r\n";
    // Transmit over Serial5
    Serial5.print(telemetry);

    Serial.print("Transmitted: ");
    Serial.print(telemetry);

    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
  }
}
