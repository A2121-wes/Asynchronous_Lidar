/*
 * GARMIN GPS DIAGNOSTIC SCANNER
 * ==============================
 * This sketch does ONE job: find the correct baud rate and polarity
 * for your Garmin GPS, by trying every combination and showing you
 * the RAW characters it receives.
 *
 * It tries:  4800, 9600, 19200, 38400 baud
 *      x     normal polarity AND inverted polarity
 *      = 8 combinations, ~5 seconds each
 *
 * WHAT TO LOOK FOR IN THE SERIAL MONITOR:
 *   The WINNING combination will show readable text like:
 *       $GPGGA,123519,4807.038,N,01131.000,E,1,08,...
 *       $GPRMC,123519,A,4807.038,N,...
 *   The wrong ones show garbage symbols or nothing.
 *
 * WIRING: only the GPS is needed for this test.
 *   Garmin Red   -> 5V
 *   Garmin Black -> GND
 *   Garmin White -> Uno pin 4
 *   (Yellow/PPS not needed for this test)
 *
 * Open Serial Monitor at 115200. Note which combo shows "$GP" text.
 */

#include <SoftwareSerial.h>

const int GPS_RX_PIN = 4;
const int GPS_TX_PIN = 3;

long bauds[] = {4800, 9600, 19200, 38400};
bool polarities[] = {false, true};   // false = normal, true = inverted

const unsigned long TEST_MS = 5000;  // 5 seconds per combination

void testCombo(long baud, bool inverted) {
  Serial.println();
  Serial.println(F("=================================================="));
  Serial.print(F("TESTING: "));
  Serial.print(baud);
  Serial.print(F(" baud, polarity = "));
  Serial.println(inverted ? F("INVERTED") : F("NORMAL"));
  Serial.println(F("--------------------------------------------------"));

  SoftwareSerial *ss = new SoftwareSerial(GPS_RX_PIN, GPS_TX_PIN, inverted);
  ss->begin(baud);

  unsigned long start = millis();
  unsigned long total = 0;
  unsigned long printable = 0;
  unsigned long dollars = 0;

  while (millis() - start < TEST_MS) {
    while (ss->available() > 0) {
      char c = ss->read();
      total++;
      if (c == '$') dollars++;
      // Print readable characters so we can see real NMEA text
      if (c >= 32 && c <= 126) {
        printable++;
        Serial.print(c);
      } else if (c == '\n') {
        Serial.println();
      }
    }
  }

  ss->end();
  delete ss;

  Serial.println();
  Serial.println(F("--------------------------------------------------"));
  Serial.print(F("RESULT: total chars = ")); Serial.print(total);
  Serial.print(F(" | readable = "));         Serial.print(printable);
  Serial.print(F(" | '$' found = "));        Serial.println(dollars);

  if (dollars > 0 && printable > total / 2) {
    Serial.println(F(">>> THIS LOOKS CORRECT! Use this combination. <<<"));
  } else if (total == 0) {
    Serial.println(F("    (no data at all on this setting)"));
  } else {
    Serial.println(F("    (garbage - wrong setting)"));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println(F("\n\n########################################"));
  Serial.println(F("   GARMIN GPS BAUD + POLARITY SCANNER"));
  Serial.println(F("########################################"));
  Serial.println(F("Trying 8 combinations, 5 seconds each."));
  Serial.println(F("Look for readable $GPGGA / $GPRMC text.\n"));
}

void loop() {
  for (int p = 0; p < 2; p++) {
    for (int b = 0; b < 4; b++) {
      testCombo(bauds[b], polarities[p]);
    }
  }

  Serial.println(F("\n\n#### SCAN COMPLETE — restarting in 5s ####\n"));
  delay(5000);
}
