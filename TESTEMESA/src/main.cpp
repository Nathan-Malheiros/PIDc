#include <Arduino.h>

#define XP 4
#define XM 5
#define YP 6
#define YM 7

// Calibracao: rode sem tocar e depois tocando nos extremos para ajustar
#define X_RAW_MIN  200
#define X_RAW_MAX  3800
#define Y_RAW_MIN  200
#define Y_RAW_MAX  3800

#define SCREEN_W_MM 187.0f
#define SCREEN_H_MM 141.0f

int readX() {
  pinMode(YP, OUTPUT); pinMode(YM, OUTPUT);
  digitalWrite(YP, HIGH); digitalWrite(YM, LOW);
  pinMode(XP, INPUT); pinMode(XM, INPUT);
  delayMicroseconds(100);
  int sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(XP);
  return sum / 4;
}

int readY() {
  pinMode(XP, OUTPUT); pinMode(XM, OUTPUT);
  digitalWrite(XP, HIGH); digitalWrite(XM, LOW);
  pinMode(YP, INPUT); pinMode(YM, INPUT);
  delayMicroseconds(100);
  int sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(YP);
  return sum / 4;
}

bool isTouched() {
  pinMode(XP, OUTPUT); pinMode(XM, OUTPUT);
  digitalWrite(XP, HIGH); digitalWrite(XM, LOW);
  pinMode(YP, INPUT_PULLDOWN); pinMode(YM, INPUT);
  delayMicroseconds(50);
  return (analogRead(YP) > 2000);
}

float toMM(int raw, int rawMin, int rawMax, float mmMax) {
  float norm = (float)(raw - rawMin) / (float)(rawMax - rawMin);
  if (norm < 0.0f) norm = 0.0f;
  if (norm > 1.0f) norm = 1.0f;
  return norm * mmMax;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  Serial.println("Teste touch resistivo");
}

void loop() {
  static int noTouchCount = 0;
  static bool touching = false;

  if (isTouched()) {
    noTouchCount = 0;
    if (!touching) touching = true;

    float x = toMM(readX(), X_RAW_MIN, X_RAW_MAX, SCREEN_W_MM);
    float y = toMM(readY(), Y_RAW_MIN, Y_RAW_MAX, SCREEN_H_MM);

    Serial.print("X:"); Serial.print(x, 1);
    Serial.print(",Y:"); Serial.println(y, 1);
  } else {
    noTouchCount++;
    if (noTouchCount >= 3) {
      if (touching) {
        touching = false;
        Serial.println("NONE");
      }
      noTouchCount = 3;
    }
  }

  delay(50);
}
