
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <SD.h>

const int GPS_RX_PIN = 4;
const int GPS_TX_PIN = 3;    // unused
const int PPS_PIN    = 5;
const int LASER_PIN  = 7;
const int SD_CS_PIN  = 10;
const unsigned long LASER_BLINK_MS = 50;

// *** VERIFIED SETTINGS: inverted polarity, 9600 baud ***
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN, true);
const long GPS_BAUD = 9600;

TinyGPSPlus gps;

const int MPU_ADDR = 0x68;
float tiltX, tiltY, tiltZ, spinX, spinY, spinZ;

File logFile;
bool sdOK = false;
unsigned long blinkNumber = 0;
bool laserOn = false;
unsigned long laserOnTime = 0;
int lastPPS = LOW;

void setup() {
  Serial.begin(115200);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  pinMode(PPS_PIN, INPUT);

  gpsSerial.begin(GPS_BAUD);

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  if (SD.begin(SD_CS_PIN)) {
    sdOK = true;
    Serial.println(F("SD ready."));
    logFile = SD.open("txlog.csv", FILE_WRITE);
    if (logFile) {
      logFile.println(F("blink_number,gps_time,gps_date,lat,lon,fix,tilt_x,tilt_y,tilt_z,spin_x,spin_y,spin_z,laser_on"));
      logFile.flush();
    }
  } else {
    Serial.println(F("SD FAILED."));
  }

  Serial.println(F("Transmitter ready. 9600 baud, inverted."));
}

void readIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  tiltX = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  tiltY = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  tiltZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  spinX = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
  spinY = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
  spinZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
}

void fireAndLog() {
  digitalWrite(LASER_PIN, HIGH);
  laserOn = true;
  laserOnTime = millis();

  readIMU();

  bool haveFix = gps.location.isValid();
  double lat = haveFix ? gps.location.lat() : 0.0;
  double lon = haveFix ? gps.location.lng() : 0.0;

  char t[12];
  if (gps.time.isValid())
    sprintf(t, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
  else
    strcpy(t, "00:00:00");

  char d[12];
  if (gps.date.isValid())
    sprintf(d, "%04d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day());
  else
    strcpy(d, "0000-00-00");

  blinkNumber++;

  if (sdOK && logFile) {
    logFile.print(blinkNumber); logFile.print(',');
    logFile.print(t);           logFile.print(',');
    logFile.print(d);           logFile.print(',');
    logFile.print(lat, 7);      logFile.print(',');
    logFile.print(lon, 7);      logFile.print(',');
    logFile.print(haveFix ? 1 : 0); logFile.print(',');
    logFile.print(tiltX, 4);    logFile.print(',');
    logFile.print(tiltY, 4);    logFile.print(',');
    logFile.print(tiltZ, 4);    logFile.print(',');
    logFile.print(spinX, 3);    logFile.print(',');
    logFile.print(spinY, 3);    logFile.print(',');
    logFile.print(spinZ, 3);    logFile.print(',');
    logFile.println('1');
    logFile.flush();
  }

  Serial.print(F("BLINK #")); Serial.print(blinkNumber);
  Serial.print(F("  "));      Serial.print(t);
  Serial.print(F("  lat="));  Serial.print(lat, 6);
  Serial.print(F(" lon="));   Serial.print(lon, 6);
  Serial.println(haveFix ? F("   FIX OK") : F("   [no fix]"));
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  int pps = digitalRead(PPS_PIN);
  if (pps == HIGH && lastPPS == LOW) {
    fireAndLog();
  }
  lastPPS = pps;

  if (laserOn && (millis() - laserOnTime >= LASER_BLINK_MS)) {
    digitalWrite(LASER_PIN, LOW);
    laserOn = false;
  }
}
