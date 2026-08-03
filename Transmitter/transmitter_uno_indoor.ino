/*
 * Asynchronous Lidar — Transmitter INDOOR TEST (no GPS)
 * ======================================================
 * Tests IMU + SD card + laser INDOORS. Fires the laser once per second
 * using the Uno's clock, reads the IMU, and logs to the SD card with
 * PLAIN-ENGLISH column names.
 *
 * WIRING (no GPS):
 *   MPU6050 IMU:  VCC->5V  GND->GND  SDA->A4  SCL->A5
 *   SD MODULE:    VCC->5V  GND->GND  MOSI->11  MISO->12  SCK->13  CS->10
 *   MOSFET:       SIG->pin7  VCC->5V  GND->GND
 *                 V+/V- -> 12V charger   OUT -> laser
 *
 * COLUMN NAMES IN THE FILE (plain English):
 *   blink_number   = which laser blink (1, 2, 3...)
 *   time_seconds   = seconds since the Uno turned on
 *   tilt_x         = tilt left/right   (from accelerometer)
 *   tilt_y         = tilt forward/back
 *   tilt_z         = up/down (about 0.98 when flat & still = gravity)
 *   spin_x         = how fast spinning left/right (from gyroscope)
 *   spin_y         = how fast spinning forward/back
 *   spin_z         = how fast spinning around
 *   laser_on       = 1 means the laser fired that moment
 */
#include <Wire.h>
#include <SD.h>

const int LASER_PIN = 7;
const int SD_CS_PIN = 10;

const unsigned long FIRE_INTERVAL_MS = 1000;  // fire every 1 second
const unsigned long LASER_BLINK_MS   = 50;    // laser on 50ms each fire

const int MPU_ADDR = 0x68;
float tiltX, tiltY, tiltZ;    // accelerometer (tilt)
float spinX, spinY, spinZ;    // gyroscope (spin rate)

File logFile;
bool sdOK = false;
unsigned long blinkNumber = 0;
unsigned long lastFire = 0;
bool laserOn = false;
unsigned long laserOnTime = 0;

void setup() {
  Serial.begin(115200);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.println("IMU started.");

  if (SD.begin(SD_CS_PIN)) {
    sdOK = true;
    Serial.println("SD card ready.");
    logFile = SD.open("indoor.csv", FILE_WRITE);
    if (logFile) {
      // Plain-English header row
      logFile.println("blink_number,time_seconds,"
                      "tilt_x,tilt_y,tilt_z,"
                      "spin_x,spin_y,spin_z,laser_on");
      logFile.flush();
    }
  } else {
    Serial.println("SD card FAILED — check wiring. Will still print to serial.");
  }

  Serial.println("INDOOR TEST ready. Laser fires every 1 second.\n");
}

void readIMU() {
  // Accelerometer -> tilt
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  tiltX = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  tiltY = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
  tiltZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;

  // Gyroscope -> spin rate
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  spinX = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
  spinY = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
  spinZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
}

void loop() {
  unsigned long now = millis();

  if (now - lastFire >= FIRE_INTERVAL_MS) {
    lastFire = now;

    digitalWrite(LASER_PIN, HIGH);
    laserOn = true;
    laserOnTime = now;

    readIMU();
    blinkNumber++;

    if (sdOK && logFile) {
      logFile.print(blinkNumber);     logFile.print(",");
      logFile.print(now / 1000.0, 2); logFile.print(",");  // seconds
      logFile.print(tiltX, 4);        logFile.print(",");
      logFile.print(tiltY, 4);        logFile.print(",");
      logFile.print(tiltZ, 4);        logFile.print(",");
      logFile.print(spinX, 3);        logFile.print(",");
      logFile.print(spinY, 3);        logFile.print(",");
      logFile.print(spinZ, 3);        logFile.print(",");
      logFile.println("1");
      logFile.flush();
    }

    Serial.print("BLINK #");    Serial.print(blinkNumber);
    Serial.print("  tilt=(");   Serial.print(tiltX,2);
    Serial.print(",");          Serial.print(tiltY,2);
    Serial.print(",");          Serial.print(tiltZ,2);
    Serial.print(")  spin=(");  Serial.print(spinX,1);
    Serial.print(",");          Serial.print(spinY,1);
    Serial.print(",");          Serial.print(spinZ,1);
    Serial.print(")");
    if (!sdOK) Serial.print("  [SD not saving]");
    Serial.println();
  }

  if (laserOn && (now - laserOnTime >= LASER_BLINK_MS)) {
    digitalWrite(LASER_PIN, LOW);
    laserOn = false;
  }
}
