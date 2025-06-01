#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <SPI.h>
#include <max6675.h>
#include <PID_v1.h>
#include <HTTPClient.h>
#include <LiquidCrystal_I2C.h>
#include <esp_log.h>

// ── PIN ASSIGNMENTS ─────────────────────────────────────────────────────────────
const int thermoDO     = 19;
const int thermoCS     = 5;
const int thermoCLK    = 18;

#define SSR_PIN        25
const int MOTOR_RPWM   = 26;

// ── MANUAL CONTROLS ─────────────────────────────────────────────────────────────
#define PIN_ONOFF    15  // toggle manual control (active LOW)
#define PIN_BTN      2  // “enter” button (active LOW)
#define PIN_ENC_CLK  32  // encoder CLK (interrupt)
#define PIN_ENC_DT   33  // encoder DT


// ── NETWORK & AUTH ──────────────────────────────────────────────────────────────
const char* SSID = "Mommy Boots";
const char* PASS = "Sakurabigas";
const char* ADMIN_USER = "admin";
const char* ADMIN_PASS = "secret";

// ── GLOBAL OBJECTS ───────────────────────────────────────────────────────────────
unsigned long motorStartTime = 0;
bool motorStartTimerActive = false;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
WebServer server(80);
LiquidCrystal_I2C lcd(0x27, 20, 4);

// PID
double currentTemp, setpoint, pidOutput;
PID myPID(&currentTemp, &pidOutput, &setpoint, 1.0, 0.5, 0.2, DIRECT);

// Motor/RPM
int targetRpm = 0;

// ── MANUAL STATE ────────────────────────────────────────────────────────────────
volatile int8_t  encoderDelta = 0;
bool             encoderMoved = false;
bool             manualEnabled = false;
uint8_t          adjustMode   = 0;  // 0=temp, 1=rpm

// debounce
int     lastOnoffReading = HIGH;
uint32_t lastOnoffDebounce = 0;
int     lastBtnReading = HIGH;
uint32_t lastBtnDebounce = 0;
const uint32_t DEBOUNCE_MS = 50;

// timing
unsigned long lastTempRead = 0;
unsigned long lastPrint    = 0;


// Interrupt Service Routine for rotary encoder input.
// Determines direction of rotation and updates encoderDelta.
void IRAM_ATTR onEncoder() {
  bool clk = digitalRead(PIN_ENC_CLK);
  bool dt  = digitalRead(PIN_ENC_DT);
  encoderDelta += (clk == dt) ? +1 : -1;
  encoderMoved = true;
}

// Arduino setup function - runs once at boot.
// Initializes peripherals, PID, WiFi, server routes, LCD, etc.
void setup() {
  Serial.begin(115200);
  esp_log_level_set("i2c.master", ESP_LOG_NONE);
  esp_log_level_set("i2c",        ESP_LOG_NONE);

  // LCD init
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Set Value:     RPM:");
  lcd.setCursor(0,2);
  lcd.print("Current Temp:");

  // SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  // pin modes
  pinMode(SSR_PIN,    OUTPUT);
  pinMode(MOTOR_RPWM, OUTPUT);
  pinMode(PIN_ONOFF,  INPUT_PULLUP);
  pinMode(PIN_BTN,    INPUT_PULLUP);
  pinMode(PIN_ENC_CLK,INPUT_PULLUP);
  pinMode(PIN_ENC_DT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), onEncoder, CHANGE);

  // PID
  setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);

  WiFi.begin(SSID, PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.printf("\nConnected! IP=%s\n", WiFi.localIP().toString().c_str());

  // notify backend
  sendUniqueCodeToPHP();

  // HTTP endpoints
  server.on("/data",     HTTP_GET,  handleData);
  server.on("/set",      HTTP_POST, handleSet);
  server.on("/setMotor", HTTP_GET,  handleSetMotor);
  server.on("/tune",     HTTP_GET,  handleGetTunings);
  server.on("/tune",     HTTP_POST, handleSetTunings);

  // serve dashboard
  server.on("/", []( ){
    File f = SPIFFS.open("/index.html", "r");
    server.streamFile(f, "text/html");
    f.close();
  });
  server.on("/dashboard", []( ){
    File f = SPIFFS.open("/index.html", "r");
    server.streamFile(f, "text/html");
    f.close();
  });
  server.serveStatic("/images", SPIFFS, "/images");
  server.begin();
}

// Main loop - runs continuously after setup.
// Handles UI, temperature reading, PID control, motor control,
// debounce logic, and HTTP requests.
void loop() {
  server.handleClient();

  // —— debounce on/off
  int onoff = digitalRead(PIN_ONOFF);
  if (onoff != lastOnoffReading) {
    lastOnoffDebounce = millis();
    lastOnoffReading = onoff;
  }
  if (millis() - lastOnoffDebounce > DEBOUNCE_MS) {
    bool nowManual = (onoff == LOW);
    if (nowManual != manualEnabled) {
      manualEnabled = nowManual;
      Serial.printf("Manual Control %s\n", manualEnabled ? "ENABLED" : "DISABLED");
      // reset mode & encoder state
      adjustMode = 0;
      encoderDelta = 0;
      encoderMoved = false;
    }
  }

  // —— debounce enter button
  int btn = digitalRead(PIN_BTN);
  if (btn != lastBtnReading) {
    lastBtnDebounce = millis();
    lastBtnReading = btn;
  }
  if (millis() - lastBtnDebounce > DEBOUNCE_MS) {
    if (btn == LOW && manualEnabled) {
      adjustMode = 1 - adjustMode;
      Serial.printf("Switched to %s mode\n", adjustMode==0?"TEMPERATURE":"RPM");
      // wait release
      while (digitalRead(PIN_BTN)==LOW) delay(5);
    }
  }

  // —— apply encoder
  if (manualEnabled && encoderMoved) {
    if (adjustMode == 0) {
      double old = setpoint;
      setpoint = constrain(setpoint + (encoderDelta * 5.0), 0.0, 210.0);
      Serial.printf("Manual Temp: %.1f -> %.1f °C\n", old, setpoint);
    } else {
      int old = targetRpm;
      targetRpm = constrain(targetRpm + encoderDelta, 0, 80);
      Serial.printf("Manual RPM: %d -> %d rpm\n", old, targetRpm);
    }
    encoderDelta = 0;
    encoderMoved = false;
    updateLCD();
  }

  // —— read temp & update LCD every 0.5s
  if (millis() - lastTempRead >= 500) {
    currentTemp = thermocouple.readCelsius();
    lastTempRead = millis();
    updateLCD();
  }

  // —— PID & heater
  if (!isnan(currentTemp)) {
    myPID.Compute();
    int out = (currentTemp < setpoint) ? (int)pidOutput : 0;
    analogWrite(SSR_PIN, out);
  }

  // —— motor drive
int pwmVal = 0;
if (motorStartTimerActive && millis() - motorStartTime >= 360000) { // 6 mins = 360,000 ms
  pwmVal = map(targetRpm, 0, 80, 0, 255);
}
analogWrite(MOTOR_RPWM, pwmVal);

  // —— periodic serial status
  if (millis() - lastPrint >= 300) {
    Serial.printf("Temp: %.1f°C | Set: %.1f°C | RPMreq: %d | Manual: %s\n",
                  currentTemp, setpoint, targetRpm,
                  manualEnabled ? "ON" : "OFF");
    lastPrint = millis();
  }
}

// Updates the LCD display with current setpoint, target RPM,
// and measured temperature.
void updateLCD() {
  lcd.setCursor(0,1); lcd.print("                    ");
  lcd.setCursor(0,1);
  lcd.print(setpoint,1); lcd.print(" C");

  lcd.setCursor(15,1); lcd.print("   ");
  lcd.setCursor(15,1);
  lcd.print(targetRpm);

  lcd.setCursor(0,3); lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print(currentTemp,1); lcd.print(" C");
}

// HTTP GET /data
// Sends current temperature, setpoint, PID output, and RPM
// as a JSON response to the frontend.
void handleData() {
  char buf[128];
  snprintf(buf, sizeof(buf),
           "{\"temp\":%.1f,\"set\":%.1f,\"output\":%.1f,\"rpm\":%d}",
           currentTemp, setpoint, pidOutput, targetRpm);
  server.send(200, "application/json", buf);
}


// HTTP POST /set?set=VALUE
// Updates the temperature setpoint from frontend input.
// Also initiates motor timer if heating starts.
void handleSet() {
  if (server.hasArg("set")) {
    double newSetpoint = server.arg("set").toFloat();
    if (setpoint == 0 && newSetpoint > 0 && !motorStartTimerActive) {
      motorStartTime = millis();
      motorStartTimerActive = true;
      Serial.println("Motor start timer initiated.");
    }
    setpoint = newSetpoint;
  }
  handleData();
}

// HTTP GET /setMotor?rpm=VALUE
// Sets the target RPM and applies mapped PWM to motor pin.
// Responds with JSON confirmation of RPM value.
void handleSetMotor() {
  if (server.hasArg("rpm")) {
    targetRpm = constrain(server.arg("rpm").toInt(), 0, 80);
  }
int pwm = map(targetRpm, 0, 80, 0, 255);
  analogWrite(MOTOR_RPWM, pwm);
  server.send(200, "application/json",
              String("{\"rpm\":") + targetRpm + "}");
}

void handleGetTunings() {
  if (!server.hasArg("user") || !server.hasArg("pass")
      || server.arg("user") != ADMIN_USER
      || server.arg("pass") != ADMIN_PASS) {
    server.send(401, "application/json", "{\"error\":\"unauthorized\"}");
    return;
  }
  server.send(200, "application/json",
              String("{\"kp\":1.00,\"ki\":0.50,\"kd\":0.20}"));
}

// HTTP GET /tune?user=admin&pass=secret
// Returns the current PID tuning constants (kp, ki, kd).
// Requires correct admin credentials.
void handleSetTunings() {
  if (!server.hasArg("user") || !server.hasArg("pass")
      || server.arg("user") != ADMIN_USER
      || server.arg("pass") != ADMIN_PASS) {
    server.send(401, "application/json", "{\"error\":\"unauthorized\"}");
    return;
  }
  double Kp = server.arg("kp").toFloat();
  double Ki = server.arg("ki").toFloat();
  double Kd = server.arg("kd").toFloat();
  myPID.SetTunings(Kp, Ki, Kd);
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// Sends a unique identifier code to a PHP backend server via
// HTTP POST. Used for registration, logging, or monitoring.
void sendUniqueCodeToPHP() {  
  const char* uniqueCode = "ESP32CAPSTONE";
  HTTPClient http;
  http.begin("http://172.20.10.8/capstone/send_code.php");
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  String payload = String("unique_code=") + uniqueCode;
  http.POST(payload);
  http.end();
}