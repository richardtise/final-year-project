// ============================================================
//   BIOGAS GREENHOUSE CONTROLLER — ESP32
//   Patched Version: latched safety timeouts, ADC averaging,
//   CO2 hysteresis, range checks, interlock, 2-page LCD
// ============================================================

#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <LiquidCrystal.h>

// ==================== PIN DEFINITIONS ====================
#define ONE_WIRE_BUS        5
#define DHT_PIN             4
#define MQ135_PIN           35
#define MQ4_PIN             34
#define WATER_DIGESTER_PIN  32
#define WATER_HYDRO_PIN     33

// Relays (Active LOW)
// NOTE: GPIO12 removed (strapping pin — can break boot/flash
// voltage if pulled by relay board). Spares moved to 21/22.
#define RELAY_HEATER_DIGESTER   25
#define RELAY_HEATER_CO2        26
#define RELAY_VALVE_WATER_DIG   27
#define RELAY_VALVE_DIG_HYDRO   13
#define RELAY_VALVE_DIG_GAS     21   // Spare (was 14)
#define RELAY_VALVE_DIG_CO2     22   // Spare (was 12 — BOOT PIN, do not use 12!)

// 16x2 LCD (RS, E, D4, D5, D6, D7)
// NOTE: D7 on GPIO15 is a strapping pin. It works once wired,
// but on a fresh PCB rev move LCD off 15 (e.g. to 21/22 area).
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);

// ==================== SENSOR OBJECTS ====================
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
DHT dht(DHT_PIN, DHT22);

DeviceAddress addrDigester, addrCO2Water;

// ==================== THRESHOLDS ====================
// Digester temperature (mesophilic range)
const float DIG_TEMP_LOW     = 36.0;
const float DIG_TEMP_HIGH    = 40.0;

// CO2 stripping water temperature
const float CO2_TEMP_LOW     = 55.0;
const float CO2_TEMP_HIGH    = 62.0;

// Safety over-temperature
const float MAX_SAFE_TEMP    = 72.0;

// DS18B20 sanity range (reject 85.0C POR value + outliers)
const float TEMP_MIN_SANE    = -10.0;
const float TEMP_MAX_SANE    = 85.0;

// Water level thresholds (0-4095)
const int WATER_DIG_LOW      = 1100;
const int WATER_DIG_OK       = 1600;
const int WATER_HYDRO_LOW    = 1000;
const int WATER_HYDRO_OK     = 1500;
// Interlock margin: don't feed hydroponics unless digester has this much
const int WATER_DIG_INTERLOCK = WATER_DIG_LOW + 200;

// CO2 threshold with hysteresis (MQ-135 raw, averaged)
const int CO2_ON             = 1500;  // below this → need CO2, may heat
const int CO2_OFF            = 1700;  // above this → enough CO2, stop heating

// Timing
const unsigned long READ_INTERVAL_MS = 5000;   // 5 seconds
const unsigned long LCD_PAGE_MS      = 3000;   // LCD page alternate period

// Safety timeouts (milliseconds)
const unsigned long MAX_HEATER_ON_MS = 1800000;  // 30 minutes
const unsigned long MAX_VALVE_ON_MS  = 300000;   // 5 minutes

// ==================== GLOBALS ====================
unsigned long lastReadTime = 0;

// Actuator time tracking — init to 0, but timeout is only
// evaluated while the actuator is actually ON (see loop),
// so a 0 init can never cause a spurious trip after 30 min uptime.
unsigned long heaterDigOnTime  = 0;
unsigned long heaterCO2OnTime  = 0;
unsigned long valveWaterOnTime = 0;
unsigned long valveHydroOnTime = 0;

// Latched safety lockouts: once tripped, stay OFF until the
// underlying demand genuinely clears (temp recovered / tank full).
bool heaterDigLockout  = false;
bool heaterCO2Lockout  = false;
bool valveWaterLockout = false;
bool valveHydroLockout = false;

// Persistent CO2 demand state (hysteresis memory)
bool co2Demand = false;

// Previous states (for change detection)
bool lastHeaterDig  = false;
bool lastHeaterCO2  = false;
bool lastValveWater = false;
bool lastValveHydro = false;

// Sensor fault flags (shown on LCD page 1 + serial)
bool faultDigTemp   = false;
bool faultCO2Temp   = false;
bool faultDHT       = false;
bool faultWaterDig  = false;
bool faultWaterHyd  = false;

// LCD paging
unsigned long lastPageSwitch = 0;
uint8_t lcdPage = 0;
char lcdBuf[2][17];  // last-drawn content per line, for flicker-free updates

// ==================== HELPER FUNCTIONS ====================
void relaySet(uint8_t pin, bool on) {
  digitalWrite(pin, on ? LOW : HIGH);
}

bool relayIsOn(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

// Averaged ADC read: ESP32 ADC is noisy, never trust one sample.
// 16 samples ≈ a few ms, kills most jitter.
int analogAvg(uint8_t pin, uint8_t samples = 16) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  return (int)(sum / samples);
}

void printAddress(const DeviceAddress &a) {
  for (uint8_t i = 0; i < 8; i++) {
    if (a[i] < 16) Serial.print("0");
    Serial.print(a[i], HEX);
    if (i < 7) Serial.print(":");
  }
}

// Write one LCD line only if content changed (no flicker).
// Always exactly 16 chars — fixed width, so no ghost digits.
void lcdWriteLine(uint8_t row, const char *s) {
  if (strncmp(lcdBuf[row], s, 16) != 0) {
    strncpy(lcdBuf[row], s, 16);
    lcdBuf[row][16] = '\0';
    lcd.setCursor(0, row);
    lcd.print(s);
  }
}

// Two alternating pages (switches every LCD_PAGE_MS):
//   Page 0 — process values: digester temp, CO2-water temp, GH temp/hum, CO2 raw
//   Page 1 — actuator + fault states + water levels
void updateLCD(float tDig, float tCO2, float ghTemp, float ghHum, int co2Raw,
               int wDig, int wHydro, unsigned long now) {
  if (now - lastPageSwitch >= LCD_PAGE_MS) {
    lcdPage = 1 - lcdPage;
    lastPageSwitch = now;
    lcd.clear();                 // clean switch, then force rewrite below
    lcdBuf[0][0] = '\0';
    lcdBuf[1][0] = '\0';
  }

  char l0[17], l1[17];
  if (lcdPage == 0) {
    // Fixed-width %5.1f so "5.0" can't leave ghosts of "36.5".
    // Every line is exactly 16 chars (count them!) — no ghost digits.
    snprintf(l0, sizeof(l0), "D:%5.1f W:%5.1f ", tDig, tCO2);   // 2+5+1+2+5+1 = 16
    // Humidity clamped to 0-99 for display: 100% would be 3 chars and
    // overflow the 16-col line. Serial still logs the true value.
    int humD = constrain((int)ghHum, 0, 99);
    snprintf(l1, sizeof(l1), "H:%5.1f %2d%% %4d", ghTemp, humD, co2Raw); // 2+5+1+2+1+1+4 = 16
  } else {
    snprintf(l0, sizeof(l0), "D:%d C:%d W:%d H:%d ",   // 3+1+3+1+3+1+3+1 = 16
             lastHeaterDig ? 1 : 0, lastHeaterCO2 ? 1 : 0,
             lastValveWater ? 1 : 0, lastValveHydro ? 1 : 0);
    if (faultDigTemp || faultCO2Temp || faultDHT || faultWaterDig || faultWaterHyd) {
      // Fault line wins over water levels — you need to SEE faults
      snprintf(l1, sizeof(l1), "FAULT D%dC%dH%dW%d%d ",  // 15 + 1 pad = 16
               faultDigTemp ? 1 : 0, faultCO2Temp ? 1 : 0, faultDHT ? 1 : 0,
               faultWaterDig ? 1 : 0, faultWaterHyd ? 1 : 0);
    } else {
      snprintf(l1, sizeof(l1), "WD:%4d WH:%4d ", wDig, wHydro);  // 7+1+7+1 = 16
    }
  }
  lcdWriteLine(0, l0);
  lcdWriteLine(1, l1);
}

// ==================== SETUP ====================
void setup() {
  // Relays FIRST, before Serial/delays, so the pins are never left
  // floating (high-Z) on an active-low board. digitalWrite precedes
  // pinMode deliberately: writing the latch before enabling the output
  // avoids the brief LOW (= relay ON) glitch that pinMode-then-write
  // would produce.
  uint8_t relayPins[] = {RELAY_HEATER_DIGESTER, RELAY_HEATER_CO2,
                         RELAY_VALVE_WATER_DIG, RELAY_VALVE_DIG_HYDRO,
                         RELAY_VALVE_DIG_GAS, RELAY_VALVE_DIG_CO2};
  for (uint8_t p : relayPins) {
    digitalWrite(p, HIGH);   // relay OFF (active LOW) ...
    pinMode(p, OUTPUT);      // ... then drive it
  }

  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Biogas Greenhouse Controller (Patched) ===");

  // ADC: full 0-3.3V range for MQ + level sensors
  analogSetAttenuation(ADC_11db);
  // analogReadResolution(12);  // default on ESP32-Arduino, left explicit

  ds18b20.begin();
  // 10-bit = 187 ms conversion instead of 750 ms at 12-bit.
  // requestTemperatures() still blocks, but ~187 ms total for 2 sensors
  // on the same bus (one conversion for all) instead of ~750 ms.
  // 0.25 C steps are plenty for this plant.
  ds18b20.setResolution(10);
  dht.begin();
  lcd.begin(16, 2);
  lcd.print("Booting...");
  memset(lcdBuf[0], 0, sizeof(lcdBuf[0]));
  memset(lcdBuf[1], 0, sizeof(lcdBuf[1]));
  delay(2000);
  lcd.clear();

  int count = ds18b20.getDeviceCount();
  Serial.print("DS18B20 found: ");
  Serial.println(count);
  if (count >= 1) {
    ds18b20.getAddress(addrDigester, 0);
    Serial.print("  [0] digester: "); printAddress(addrDigester); Serial.println();
  }
  if (count >= 2) {
    ds18b20.getAddress(addrCO2Water, 1);
    Serial.print("  [1] co2water: "); printAddress(addrCO2Water); Serial.println();
  }
  if (count < 2) Serial.println("WARNING: Only 1 or 0 DS18B20 detected!");
  Serial.println("TIP: copy the addresses above into fixed bindings for defense.");

  Serial.println("System Ready!\n");
}

// Validate a DS18B20 reading: disconnected OR 85C power-on-reset OR out of range
bool tempSane(float t) {
  if (t == DEVICE_DISCONNECTED_C) return false;
  if (t == 85.0f) return false;   // DS18B20 POR value = sensor didn't convert
  if (t < TEMP_MIN_SANE || t > TEMP_MAX_SANE) return false;
  return true;
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long now = millis();
  if (now - lastReadTime < READ_INTERVAL_MS) return;
  lastReadTime = now;

  // ---------- 1. READ SENSORS WITH ERROR HANDLING ----------
  ds18b20.requestTemperatures();   // blocking ~187 ms at 10-bit (see setup)
  // no extra delay needed — library waits for conversion

  float rawDig = ds18b20.getTempC(addrDigester);
  float rawCO2 = ds18b20.getTempC(addrCO2Water);

  float tDig, tCO2;
  faultDigTemp = !tempSane(rawDig);
  if (faultDigTemp) {
    Serial.println("[ERROR] Digester temp sensor fault (disconnected/85C/range)!");
    tDig = DIG_TEMP_HIGH;   // fail-safe: above threshold → heater stays OFF
  } else {
    tDig = rawDig;
  }
  faultCO2Temp = !tempSane(rawCO2);
  if (faultCO2Temp) {
    Serial.println("[ERROR] CO2 water temp sensor fault!");
    tCO2 = CO2_TEMP_HIGH;
  } else {
    tCO2 = rawCO2;
  }

  float ghTemp = dht.readTemperature();
  float ghHum  = dht.readHumidity();
  faultDHT = false;
  if (isnan(ghTemp) || ghTemp < -20 || ghTemp > 80) {
    Serial.println("[ERROR] DHT22 temp read failed/range!");
    ghTemp = 25.0;   // display placeholder only — not used for control
    faultDHT = true;
  }
  if (isnan(ghHum) || ghHum < 0 || ghHum > 100) {
    Serial.println("[ERROR] DHT22 humidity read failed/range!");
    ghHum = 50.0;
    faultDHT = true;
  }

  int co2Raw    = analogAvg(MQ135_PIN);
  int methane   = analogAvg(MQ4_PIN);    // demo only, not used in control
  int wDig      = analogAvg(WATER_DIGESTER_PIN);
  int wHydro    = analogAvg(WATER_HYDRO_PIN);

  // ADC open/short detection: railed at either end = wiring fault
  faultWaterDig = (wDig <= 5 || wDig >= 4090);
  faultWaterHyd = (wHydro <= 5 || wHydro >= 4090);
  if (faultWaterDig) Serial.println("[ERROR] Digester level sensor railed (open/short)!");
  if (faultWaterHyd) Serial.println("[ERROR] Hydro level sensor railed (open/short)!");

  // ---------- CO2 demand with hysteresis (update BEFORE heater logic) ----------
  if (co2Raw < CO2_ON) co2Demand = true;
  else if (co2Raw > CO2_OFF) co2Demand = false;
  // else: inside deadband → keep previous co2Demand (no chatter)

  // ---------- 2. SAFETY OVER-TEMPERATURE ----------
  bool overTemp = false;
  if (tDig > MAX_SAFE_TEMP || tCO2 > MAX_SAFE_TEMP) {
    overTemp = true;
    relaySet(RELAY_HEATER_DIGESTER, false);
    relaySet(RELAY_HEATER_CO2, false);
    lastHeaterDig = false;
    lastHeaterCO2 = false;
    Serial.println("[SAFETY] Over-temperature! Heaters forced OFF.");
  }

  // ---------- 3. DIGESTER HEATER (hysteresis + LATCHED timeout) ----------
  bool wantHeaterDig = false;
  if (!overTemp && !faultDigTemp) {
    if (tDig <= DIG_TEMP_LOW) wantHeaterDig = true;
    else if (tDig >= DIG_TEMP_HIGH) wantHeaterDig = false;
    else wantHeaterDig = relayIsOn(RELAY_HEATER_DIGESTER);   // stay in deadband
  }
  // Latch: only evaluated while actually ON, and once tripped it
  // holds OFF until the tank genuinely recovers (>= HIGH), not next cycle.
  if (heaterDigLockout) {
    wantHeaterDig = false;
    // Guard !faultDigTemp: while faulted, tDig is a fail-safe placeholder
    // (40.0) that must NOT clear the lockout.
    if (!faultDigTemp && tDig >= DIG_TEMP_HIGH) {
      heaterDigLockout = false;
      Serial.println("[RECOVER] Digester heater lockout cleared (temp recovered).");
    }
  } else if (wantHeaterDig && lastHeaterDig &&
             (now - heaterDigOnTime > MAX_HEATER_ON_MS)) {
    heaterDigLockout = true;
    wantHeaterDig = false;
    Serial.println("[TIMEOUT] Digester heater LOCKED OUT (30 min limit). Recovers at >= 40C.");
  }
  if (wantHeaterDig != lastHeaterDig) {
    relaySet(RELAY_HEATER_DIGESTER, wantHeaterDig);
    lastHeaterDig = wantHeaterDig;
    if (wantHeaterDig) {
      heaterDigOnTime = now;
      Serial.println("[ACTION] Digester heater ON");
    } else {
      Serial.println("[ACTION] Digester heater OFF");
    }
  }

  // ---------- 4. CO2 WATER HEATER (hysteresis + LATCHED timeout) ----------
  bool wantHeaterCO2 = false;
  if (!overTemp && !faultCO2Temp) {
    if (co2Demand && tCO2 <= CO2_TEMP_LOW) wantHeaterCO2 = true;
    else if (tCO2 >= CO2_TEMP_HIGH || !co2Demand) wantHeaterCO2 = false;
    else wantHeaterCO2 = relayIsOn(RELAY_HEATER_CO2);
  }
  if (heaterCO2Lockout) {
    wantHeaterCO2 = false;
    if (!faultCO2Temp && (tCO2 >= CO2_TEMP_HIGH || !co2Demand)) {
      heaterCO2Lockout = false;
      Serial.println("[RECOVER] CO2 heater lockout cleared.");
    }
  } else if (wantHeaterCO2 && lastHeaterCO2 &&
             (now - heaterCO2OnTime > MAX_HEATER_ON_MS)) {
    heaterCO2Lockout = true;
    wantHeaterCO2 = false;
    Serial.println("[TIMEOUT] CO2 heater LOCKED OUT (30 min limit).");
  }
  if (wantHeaterCO2 != lastHeaterCO2) {
    relaySet(RELAY_HEATER_CO2, wantHeaterCO2);
    lastHeaterCO2 = wantHeaterCO2;
    if (wantHeaterCO2) {
      heaterCO2OnTime = now;
      Serial.println("[ACTION] CO2 water heater ON");
    } else {
      Serial.println("[ACTION] CO2 water heater OFF");
    }
  }

  // ---------- 5. WATER REFILL VALVE (tank → digester, LATCHED timeout) ----------
  bool wantValveWater = false;
  if (!faultWaterDig) {
    if (wDig < WATER_DIG_LOW) wantValveWater = true;
    else if (wDig > WATER_DIG_OK) wantValveWater = false;
    else wantValveWater = relayIsOn(RELAY_VALVE_WATER_DIG);
  }
  if (valveWaterLockout) {
    wantValveWater = false;
    if (!faultWaterDig && wDig > WATER_DIG_OK) {
      valveWaterLockout = false;
      Serial.println("[RECOVER] Water->digester valve lockout cleared (tank full).");
    }
  } else if (wantValveWater && lastValveWater &&
             (now - valveWaterOnTime > MAX_VALVE_ON_MS)) {
    valveWaterLockout = true;
    wantValveWater = false;
    Serial.println("[TIMEOUT] Water->digester valve LOCKED OUT (5 min limit). Check for leak/float stuck!");
  }
  if (wantValveWater != lastValveWater) {
    relaySet(RELAY_VALVE_WATER_DIG, wantValveWater);
    lastValveWater = wantValveWater;
    if (wantValveWater) {
      valveWaterOnTime = now;
      Serial.println("[ACTION] Water->digester valve OPEN");
    } else {
      Serial.println("[ACTION] Water->digester valve CLOSED");
    }
  }

  // ---------- 6. DIGESTATE VALVE (digester → hydroponics, LATCHED + INTERLOCK) ----------
  bool wantValveHydro = false;
  if (!faultWaterHyd && !faultWaterDig) {
    if (wHydro < WATER_HYDRO_LOW) wantValveHydro = true;
    else if (wHydro > WATER_HYDRO_OK) wantValveHydro = false;
    else wantValveHydro = relayIsOn(RELAY_VALVE_DIG_HYDRO);
    // INTERLOCK: never drain the digester to feed hydroponics
    if (wantValveHydro && wDig < WATER_DIG_INTERLOCK) {
      wantValveHydro = false;
      Serial.println("[INTERLOCK] Hydro feed blocked: digester level too low!");
    }
  }
  if (valveHydroLockout) {
    wantValveHydro = false;
    if (!faultWaterHyd && wHydro > WATER_HYDRO_OK) {
      valveHydroLockout = false;
      Serial.println("[RECOVER] Hydroponics valve lockout cleared (tank full).");
    }
  } else if (wantValveHydro && lastValveHydro &&
             (now - valveHydroOnTime > MAX_VALVE_ON_MS)) {
    valveHydroLockout = true;
    wantValveHydro = false;
    Serial.println("[TIMEOUT] Hydroponics valve LOCKED OUT (5 min limit).");
  }
  if (wantValveHydro != lastValveHydro) {
    relaySet(RELAY_VALVE_DIG_HYDRO, wantValveHydro);
    lastValveHydro = wantValveHydro;
    if (wantValveHydro) {
      valveHydroOnTime = now;
      Serial.println("[ACTION] Hydroponics refill valve OPEN");
    } else {
      Serial.println("[ACTION] Hydroponics refill valve CLOSED");
    }
  }

  // ---------- 7. UPDATE LCD (2 pages, flicker-free, fixed width) ----------
  updateLCD(tDig, tCO2, ghTemp, ghHum, co2Raw, wDig, wHydro, now);

  // ---------- 8. SERIAL DEBUG (one line per cycle, now with states) ----------
  Serial.printf("Td:%.1f Tw:%.1f CO2:%d CH4:%d WD:%d WH:%d Hd:%d Hc:%d Vw:%d Vh:%d Lk:%d%d%d%d\n",
                tDig, tCO2, co2Raw, methane, wDig, wHydro,
                lastHeaterDig ? 1 : 0, lastHeaterCO2 ? 1 : 0,
                lastValveWater ? 1 : 0, lastValveHydro ? 1 : 0,
                heaterDigLockout ? 1 : 0, heaterCO2Lockout ? 1 : 0,
                valveWaterLockout ? 1 : 0, valveHydroLockout ? 1 : 0);
}
