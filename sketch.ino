// ===================== Libraries =====================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// ===================== OLED SETUP =====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===================== Servo ==========================
Servo servoA, servoB, servoC;

// ===================== Stepper ========================
// STEP = GPIO 13, DIR = GPIO 12
#define STEPPER_STEP_PIN 13
#define STEPPER_DIR_PIN 12

// ===================== Pin Assignments (SESUAI JSON) =================
#define LED_IDLE_PIN    40
#define LED_PROCESS_PIN 39
#define LED_DONE_PIN    38
#define BUZZ_PIN        4
#define START_BUTTON_PIN 5
#define POT_A_PIN       1
#define POT_B_PIN       2
#define POT_C_PIN       3
#define SERVO_A_PIN     17
#define SERVO_B_PIN     16
#define SERVO_C_PIN     15
#define CLK_PIN         18
#define DT_PIN          19

// ===================== Mode Pencampuran =================
const char* modes[] = {"Cepat", "Bertahap", "Otomatis"};
const int NUM_MODES = 3;
volatile int modeIndex = 0;
volatile int lastCLK = 0;

// ===================== Konfigurasi Global =================
struct MixingConfig {
  int volA;
  int volB;
  int volC;
  int mode;
};

MixingConfig currentConfig;
SemaphoreHandle_t configMutex;
SemaphoreHandle_t startSemaphore;

enum SystemState { IDLE, PROCESSING, DONE };
volatile SystemState systemState = IDLE;

// ===================== Helper =====================
int readVolume(int pin) {
  int raw = analogRead(pin);
  return map(raw, 0, 4095, 0, 100);
}

// ===================== OLED Update =====================
void updateOLED(int volA, int volB, int volC, int modeIdx, SystemState state) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.print("A: "); display.print(volA); display.println("%");
  display.print("B: "); display.print(volB); display.println("%");
  display.print("C: "); display.print(volC); display.println("%");
  display.print("Mode: "); display.println(modes[modeIdx]);

  if (state == IDLE)       display.println("Status: Siap");
  else if (state == PROCESSING) display.println("Status: Proses");
  else if (state == DONE)  display.println("Status: Selesai");

  display.display();
}

// ===================== Buzzer Square Wave =====================
void beep(int freq, int duration_ms) {
  int halfPeriod = 1000000 / (freq * 2);
  unsigned long start = millis();

  while (millis() - start < duration_ms) {
    digitalWrite(BUZZ_PIN, HIGH);
    delayMicroseconds(halfPeriod);
    digitalWrite(BUZZ_PIN, LOW);
    delayMicroseconds(halfPeriod);
  }
}

// =======================================================
// ====================== TASKS ==========================
// =======================================================

// ---------- INPUT & UI TASK (CORE 0) ----------
void TaskInputUI(void *pvParameters) {
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(LED_IDLE_PIN, OUTPUT);
  pinMode(LED_PROCESS_PIN, OUTPUT);
  pinMode(LED_DONE_PIN, OUTPUT);

  lastCLK = digitalRead(CLK_PIN);
  int lastButtonState = HIGH;

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED Gagal!");
    vTaskDelete(NULL);
  }

  systemState = IDLE;
  digitalWrite(LED_IDLE_PIN, HIGH);
  digitalWrite(LED_PROCESS_PIN, LOW);
  digitalWrite(LED_DONE_PIN, LOW);

  while (1) {
    int clk = digitalRead(CLK_PIN);
    if (clk != lastCLK) {
      int dt = digitalRead(DT_PIN);
      if (dt != clk) modeIndex = (modeIndex + 1) % NUM_MODES;
      else modeIndex = (modeIndex - 1 + NUM_MODES) % NUM_MODES;
      lastCLK = clk;
    }

    int volA = readVolume(POT_A_PIN);
    int volB = readVolume(POT_B_PIN);
    int volC = readVolume(POT_C_PIN);

    updateOLED(volA, volB, volC, modeIndex, systemState);

    if (systemState == IDLE) {
      digitalWrite(LED_IDLE_PIN, HIGH);
      digitalWrite(LED_PROCESS_PIN, LOW);
      digitalWrite(LED_DONE_PIN, LOW);
    } else if (systemState == PROCESSING) {
      digitalWrite(LED_IDLE_PIN, LOW);
      digitalWrite(LED_PROCESS_PIN, HIGH);
      digitalWrite(LED_DONE_PIN, LOW);
    } else if (systemState == DONE) {
      digitalWrite(LED_IDLE_PIN, LOW);
      digitalWrite(LED_PROCESS_PIN, LOW);
      digitalWrite(LED_DONE_PIN, HIGH);
    }

    int button = digitalRead(START_BUTTON_PIN);
    if (button == LOW && lastButtonState == HIGH && systemState == IDLE) {
      xSemaphoreTake(configMutex, portMAX_DELAY);
      currentConfig.volA = volA;
      currentConfig.volB = volB;
      currentConfig.volC = volC;
      currentConfig.mode = modeIndex;
      xSemaphoreGive(configMutex);

      systemState = PROCESSING;
      xSemaphoreGive(startSemaphore);
    }
    lastButtonState = button;

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// ---------- MIXING EXECUTION TASK (CORE 1) ----------
void TaskMixing(void *pvParameters) {
  servoA.setPeriodHertz(50);
  servoB.setPeriodHertz(50);
  servoC.setPeriodHertz(50);
  servoA.attach(SERVO_A_PIN, 500, 2400);
  servoB.attach(SERVO_B_PIN, 500, 2400);
  servoC.attach(SERVO_C_PIN, 500, 2400);

  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);

  pinMode(BUZZ_PIN, OUTPUT);

  MixingConfig cfg;

  while (1) {
    if (xSemaphoreTake(startSemaphore, portMAX_DELAY) == pdTRUE) {
      beep(800, 200);
      xSemaphoreTake(configMutex, portMAX_DELAY);
      cfg = currentConfig;
      xSemaphoreGive(configMutex);

      Serial.printf("Mulai pencampuran: A=%d%%, B=%d%%, C=%d%%, Mode=%s\n",
                    cfg.volA, cfg.volB, cfg.volC, modes[cfg.mode]);

      int durA = cfg.volA * 10;
      int durB = cfg.volB * 10;
      int durC = cfg.volC * 10;

      servoA.write(90); delay(durA); servoA.write(0);
      servoB.write(90); delay(durB); servoB.write(0);
      servoC.write(90); delay(durC); servoC.write(0);

      int stirDuration = 2000;
      if (cfg.mode == 0) stirDuration = 1000;
      else if (cfg.mode == 1) stirDuration = 3000;
      else if (cfg.mode == 2) stirDuration = 2000;

      digitalWrite(STEPPER_DIR_PIN, HIGH);
      unsigned long startTime = millis();
      while (millis() - startTime < stirDuration) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(1000);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(1000);
      }

      digitalWrite(STEPPER_STEP_PIN, LOW);
      digitalWrite(STEPPER_DIR_PIN, LOW);

      // ================================
      // FIX: BUZZER BENAR-BENAR BUNYI
      // ================================
      beep(800, 200);

      systemState = DONE;
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      systemState = IDLE;
    }
  }
}

// =======================================================
// ======================= SETUP =========================
// =======================================================
void setup() {
  Serial.begin(115200);

  configMutex = xSemaphoreCreateMutex();
  startSemaphore = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(TaskInputUI, "InputUI", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskMixing, "Mixing", 4096, NULL, 1, NULL, 1);
}

void loop() {
}
