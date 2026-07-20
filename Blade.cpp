#include "Blade.h"

#include <Arduino.h>
#include <esp_arduino_version.h>

#include "Config.h"
#include "Panel.h"

BladePosition bladePosition = BladePosition::RETRACTED;
bool bladePwmReady = false;

namespace {

unsigned long bladeDriveMs(float seconds) {
  return static_cast<unsigned long>(seconds * 1000.0f);
}

void setBladePwm(uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(BLADE_PWM_CHANNEL, duty);
#else
  ledcWrite(BLADE_PWM_CHANNEL, duty);
#endif
}

bool driveBlade(uint8_t direction, float seconds) {
  if (!relayConnected) {
    Serial.println("ERROR_BLADE_RELAY_OFF: turn machine power ON first");
    return false;
  }
  if (!bladePwmReady) {
    Serial.println("ERROR_BLADE_PWM_NOT_READY");
    return false;
  }
  digitalWrite(BLADE_DIR_PIN, direction);
  setBladePwm(BLADE_PWM_DUTY);
  Serial.print("BLADE_DRIVE PWM_GPIO=");
  Serial.print(BLADE_PWM_PIN);
  Serial.print(" DIR_GPIO=");
  Serial.print(BLADE_DIR_PIN);
  Serial.print(" DIR=");
  Serial.print(direction == HIGH ? "HIGH" : "LOW");
  Serial.print(" DUTY=");
  Serial.print(BLADE_PWM_DUTY);
  Serial.print(" MS=");
  Serial.println(bladeDriveMs(seconds));
  delay(bladeDriveMs(seconds));
  stopBlade();
  return true;
}

}  // namespace

void bladeSetup() {
  pinMode(BLADE_DIR_PIN, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  bladePwmReady = ledcAttachChannel(
      BLADE_PWM_PIN, BLADE_PWM_FREQUENCY_HZ, BLADE_PWM_RESOLUTION_BITS,
      BLADE_PWM_CHANNEL);
#else
  ledcSetup(BLADE_PWM_CHANNEL, BLADE_PWM_FREQUENCY_HZ, BLADE_PWM_RESOLUTION_BITS);
  ledcAttachPin(BLADE_PWM_PIN, BLADE_PWM_CHANNEL);
  bladePwmReady = true;
#endif
  digitalWrite(BLADE_DIR_PIN, BLADE_RETRACT_DIRECTION);
  stopBlade();
}

void stopBlade() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (bladePwmReady) ledcWriteChannel(BLADE_PWM_CHANNEL, 0);
#else
  if (bladePwmReady) ledcWrite(BLADE_PWM_CHANNEL, 0);
#endif
}

bool bladeIsDown() { return bladePosition == BladePosition::DOWN; }

void bladeDown(bool force) {
  if (!force && bladePosition == BladePosition::DOWN) return;
  if (driveBlade(BLADE_DOWN_DIRECTION, BLADE_DOWN_SECONDS)) {
    bladePosition = BladePosition::DOWN;
    Serial.println("BLADE_DOWN");
  }
}

void bladeRetracted(bool force) {
  if (!force && bladePosition == BladePosition::RETRACTED) return;
  if (driveBlade(BLADE_RETRACT_DIRECTION, BLADE_RETRACT_SECONDS)) {
    bladePosition = BladePosition::RETRACTED;
    Serial.println("BLADE_RETRACTED");
  }
}

void printBladeStatus() {
  Serial.print("BLADE_STATUS pwm_ready=");
  Serial.print(bladePwmReady ? "YES" : "NO");
  Serial.print(" relay=");
  Serial.print(relayConnected ? "ON" : "OFF");
  Serial.print(" pwm_gpio=");
  Serial.print(BLADE_PWM_PIN);
  Serial.print(" channel=");
  Serial.print(BLADE_PWM_CHANNEL);
  Serial.print(" dir_gpio=");
  Serial.print(BLADE_DIR_PIN);
  Serial.print(" duty=");
  Serial.print(BLADE_PWM_DUTY);
  Serial.print(" position=");
  Serial.println(bladeIsDown() ? "DOWN" : "RETRACTED");
}
