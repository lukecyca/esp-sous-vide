#include <ESP8266WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Ticker.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Define these in the config.h file
//#define WIFI_SSID "yourwifi"
//#define WIFI_PASSWORD "yourpassword"
//#define INFLUX_HOSTNAME "data.example.com"
//#define INFLUX_PORT 8086
//#define INFLUX_PATH "/write?db=<database>&u=<user>&p=<pass>"
//#define WEBSERVER_USERNAME "something"
//#define WEBSERVER_PASSWORD "something"
#include "config.h"

#define DEVICE_NAME "dcc-05v2-sous-vide"

#define RED_LED_PIN 16
#define GREEN_LED_PIN 14
#define RELAY_PIN 13
#define ONE_WIRE_PIN 2

#define N_SENSORS 2
#define BOARD_SENSOR 0
#define WATER_TEMP_SENSOR 1
byte sensorAddr[][8] = {
  {0x28, 0x64, 0xF7, 0x7F, 0x06, 0x00, 0x00, 0x15}, // (board)
  {0x28, 0xFF, 0xE2, 0x4A, 0x01, 0x15, 0x04, 0x33}, // (probe)
};
char * sensorNames[] = {
  "board",
  "probe",
};

#define SETTINGS_VERSION "99D3"
struct Settings {
  float boardOverTemp;
  double kp;
  double ki;
  double kd;
  double aTuneStep;
  double aTuneNoise;
  unsigned int aTuneLookBack;
  char aTuneType;             // 0=PI; 1=PID
} settings = {
  60.0, 220.0, 0.1, 0.1, 500., 1., 20, 1
};

String formatSettings() {
  return \
    String("boardOverTemp=") + String(settings.boardOverTemp, 3) + \
    String(",kp=") + String(settings.kp, 3) + \
    String(",ki=") + String(settings.ki, 3) + \
    String(",kd=") + String(settings.kd, 3) + \
    String(",aTuneStep=") + String(settings.aTuneStep, 3) + \
    String(",aTuneNoise=") + String(settings.aTuneNoise, 3) + \
    String(",aTuneLookBack=") + settings.aTuneLookBack + \
    String(",aTuneType=") + int(settings.aTuneType);
}

#include "libdcc/webserver.h"
#include "libdcc/onewire.h"
#include "libdcc/settings.h"
#include "libdcc/influx.h"
#include "libdcc/display.h"

void handleSettings() {
  REQUIRE_AUTH;

  for (int i=0; i<server.args(); i++) {
    if (server.argName(i).equals("boardOverTemp")) {
      settings.boardOverTemp = server.arg(i).toFloat();
    } else if (server.argName(i).equals("kp")) {
      settings.kp = server.arg(i).toFloat();
    } else if (server.argName(i).equals("ki")) {
      settings.ki = server.arg(i).toFloat();
    } else if (server.argName(i).equals("kd")) {
      settings.kd = server.arg(i).toFloat();
    } else if (server.argName(i).equals("aTuneStep")) {
      settings.aTuneStep = server.arg(i).toFloat();
    } else if (server.argName(i).equals("aTuneNoise")) {
      settings.aTuneNoise = server.arg(i).toFloat();
    } else if (server.argName(i).equals("aTuneLookBack")) {
      settings.aTuneLookBack = server.arg(i).toInt();
    } else if (server.argName(i).equals("aTuneType")) {
      settings.aTuneType = server.arg(i).toInt();
    } else {
      Serial.println("Unknown argument: " + server.argName(i) + ": " + server.arg(i));
    }
  }

  saveSettings();

  String msg = String("Settings saved: ") + formatSettings();
  Serial.println(msg);
  server.send(200, "text/plain", msg);
}


double setPoint;
double currentTemp;
double pidOutput;
volatile long onTime = 0;
Ticker pidTicker;
PID myPID(&currentTemp, &pidOutput, &setPoint, settings.kp, settings.ki, settings.kd, DIRECT);

// 10 second Time Proportional Output window
#define WINDOW_SIZE (10000)
unsigned long windowStartTime;

boolean tuning = false;
boolean overTemp = false;

PID_ATune aTune(&currentTemp, &pidOutput);

LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

#define DISPLAY_FREQ 100
#define SENSOR_FREQ 1000
#define UPLOAD_FREQ 10000
unsigned long displayUpdateIteration;
unsigned long lastSensorIteration;
unsigned long lastUploadIteration;


void setup() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(A0, INPUT);

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  server.on("/settings", handleSettings);
  server.on("/restart", handleRestart);
  server.on("/status", handleStatus);
  server.on("/sensors", handleSensors);
  server.on("/autotune", handleAutotune);
  server.onNotFound(handleNotFound);
  server.begin();

  loadSettings();
  Serial.println(formatSettings());

  Wire.begin(4, 5); // SDA=4, SCL=5
  lcd.begin(16, 2);
  lcd.backlight();
  loadCustomChars(lcd);

  lcd.home();
  lcd.print("   Sous Vide!");
  delay(2000);
  lcd.clear();

  myPID.SetTunings(settings.kp, settings.ki, settings.kd);
  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, WINDOW_SIZE);

  // Start the PID
  windowStartTime = millis();
  myPID.SetMode(AUTOMATIC);
  pidTicker.attach(0.015, pidTick);

  // Initialize zeroth iteration
  takeAllMeasurementsAsync();
  lastSensorIteration = millis();
  // Offset upload events from sensor events
  lastUploadIteration = millis() + SENSOR_FREQ / 2;
}


WiFiClient client;

void loop() {
  server.handleClient();

  // Copy HTTP client response to Serial
  while (client.connected() && client.available()) {
    Serial.print(client.readStringUntil('\r'));
  }

  // If we are NOT ready to do a display update iteraction, return early
  if (millis() < displayUpdateIteration + DISPLAY_FREQ) {
    return;
  }
  displayUpdateIteration = millis();

  // NB: Fudge factors are calibrated to my potentiometer and resistors
  // We snap it to an integer because it's nicer UX with the potentiometer
  setPoint = int(analogRead(A0) / 1024. * 62.6 + 19.8);

  // Update Display
  lcd.setCursor(0, 0);
  lcd.print("Wtr: " + leftPad(String(currentTemp, 1), 4) + char(3) + "C");
  lcd.setCursor(0, 1);
  lcd.print("Set: " + leftPad(String(setPoint, 1), 4) + char(3) + "C");

  // If we are NOT ready to do a sensor iteraction, return early
  if (millis() < lastSensorIteration + SENSOR_FREQ) {
    return;
  }
  lastSensorIteration = millis();

  float temp[N_SENSORS];
  String sensorBody = String(DEVICE_NAME) + " uptime=" + String(millis()) + "i";
  for (int i=0; i<N_SENSORS; i++) {
    Serial.print("Temperature sensor ");
    Serial.print(i);
    Serial.print(": ");
    if (readTemperature(sensorAddr[i], &temp[i])) {
      Serial.print(temp[i]);
      Serial.println();
      sensorBody += String(",") + sensorNames[i] + "=" + String(temp[i], 3);
    } else {
      temp[i] = NAN;
    }
  }

  // If internal temperature is too high, or sensors are not responding,
  // go into error state (and shut off the relay).
  overTemp = ((temp[BOARD_SENSOR] > settings.boardOverTemp) || \
              isnan(temp[WATER_TEMP_SENSOR]) || \
              isnan(temp[BOARD_SENSOR]));

  currentTemp = temp[WATER_TEMP_SENSOR];

  if (overTemp) {
    tuning = false;
    aTune.Cancel();
    myPID.SetMode(MANUAL);
  } else if (tuning) {
    if (aTune.Runtime()) {
      finishAutoTune();
    }
  } else {
    myPID.SetTunings(settings.kp, settings.ki, settings.kd);
    myPID.SetMode(AUTOMATIC);
    myPID.Compute();
  }
  onTime = pidOutput;

  sensorBody += String(",pidOutput=") + String(float(pidOutput) / WINDOW_SIZE * 100, 3);
  Serial.println(sensorBody);

  // Instruct sensors to take measurements for next iteration
  takeAllMeasurementsAsync();

  lcd.setCursor(13, 1);
  if (overTemp) {
    lcd.print("ERR");
  } else if (tuning) {
    lcd.print("TUN");
  } else {
    lcd.print("   ");
  }

  if (WiFi.status() == WL_CONNECTED) {
    lcd.setCursor(15, 0);
    lcd.print(char(0));
  } else {
    lcd.setCursor(15, 0);
    lcd.print(char(1));
  }

  // If we are ready to do an upload iteration, do that now
  if (millis() > lastUploadIteration + UPLOAD_FREQ) {
    Serial.println("Wifi connected to " + WiFi.SSID() + " IP:" + WiFi.localIP().toString());
    client.connect(INFLUX_HOSTNAME, INFLUX_PORT);
    postRequestAsync(sensorBody, client);
    lastUploadIteration = millis();
  }
}


void pidTick(void) {
  if (overTemp) {
     digitalWrite(RELAY_PIN, LOW);
     return;
  }

  long now = millis();

  // Set the output
  // "on time" is proportional to the PID output
  if (now - windowStartTime > WINDOW_SIZE) { //time to shift the Relay Window
     windowStartTime += WINDOW_SIZE;
  }
  if ((onTime > 100) && (onTime > (now - windowStartTime))) {
     digitalWrite(RELAY_PIN, HIGH);
  } else {
     digitalWrite(RELAY_PIN, LOW);
  }
}


void handleAutotune() {
  REQUIRE_AUTH;

  aTune.SetControlType(settings.aTuneType);
  aTune.SetNoiseBand(settings.aTuneNoise);
  aTune.SetOutputStep(settings.aTuneStep);
  aTune.SetLookbackSec((int)settings.aTuneLookBack);
  tuning = true;
  Serial.println("Autotune started");

  server.send(200, "text/plain", "Autotune enabled");
}

void finishAutoTune() {
  tuning = false;

  settings.kp = aTune.GetKp();
  settings.ki = aTune.GetKi();
  settings.kd = aTune.GetKd();
  Serial.println(String("Autotune complete: ") + \
    String(settings.kp, 3) + ", " + \
    String(settings.ki, 3) + ", " + \
    String(settings.kd, 3));

  myPID.SetTunings(settings.kp, settings.ki, settings.kd);
  myPID.SetMode(AUTOMATIC);

  saveSettings();
}


