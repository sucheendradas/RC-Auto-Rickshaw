
//Control motor speed and servo angle remotely using Blynk sliders on an ESP32, with real-time feedback and MCPWM precision.
// Perfect for DIY automation projects that blend hardware finesse with app-based interactivity.
#define BLYNK_TEMPLATE_ID "TMPL33YDLlsfw"
#define BLYNK_TEMPLATE_NAME "Vu meter"
#define BLYNK_AUTH_TOKEN "Rl6OCkEJ28EOqISz4VHwPCQLC8f5yuWY"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <driver/mcpwm.h>  // Built-in ESP32 MCPWM driver
#include <ESP32Servo.h>

// Wi-Fi credentials
char ssid[] = "eDragon";
char pass[] = "$123@Mac";

// Motor pins
#define MOTOR_PWM_PIN 12  // MCPWM output
#define MOTOR_DIR_PIN 13  // Direction control

// Servo
Servo myServo;
#define SERVO_PIN 14

// MCPWM configuration
#define MOTOR_FREQ 1000  // 1kHz

BlynkTimer timer;

void setup() {
  Serial.begin(115200);
  
  // Setup motor direction pin
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, LOW);
  
  // Setup MCPWM for motor
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_PWM_PIN);
  mcpwm_config_t pwm_config;
  pwm_config.frequency = MOTOR_FREQ;
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  
  // Setup servo
  ESP32PWM::allocateTimer(0);
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN);
  myServo.write(90); // Center position
  
  // Connect to Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  
  timer.setInterval(1000L, checkBlynkConnection);
  Serial.println("Setup complete");
}

// Motor control (V21 slider: 0-255)
BLYNK_WRITE(V21) {
  int sliderValue = param.asInt(); // 0-255
  
  if (sliderValue > 0) {
    digitalWrite(MOTOR_DIR_PIN, HIGH); // Set direction
    // Convert 0-255 to 0-100% for MCPWM
    float duty = (sliderValue / 255.0) * 100.0;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
    Blynk.virtualWrite(V22, "Motor: Running");
  } else {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    digitalWrite(MOTOR_DIR_PIN, LOW); // Stop motor completely
    Blynk.virtualWrite(V22, "Motor: Stopped");
  }
  
  Serial.print("Motor speed: ");
  Serial.println(sliderValue);
}

// Servo control (V23 slider: 0-255, but we'll map to 0-180)
BLYNK_WRITE(V23) {
  int sliderValue = param.asInt(); // 0-255 from slider
  
  // Map 0-255 to 0-180 degrees
  int servoAngle = map(sliderValue, 0, 255, 0, 180);
  
  // Constrain to valid range
  servoAngle = constrain(servoAngle, 0, 180);
  
  myServo.write(servoAngle);
  Blynk.virtualWrite(V24, "Servo: " + String(servoAngle) + "°");
  
  Serial.print("Servo position: ");
  Serial.print(servoAngle);
  Serial.println(" degrees");
}

void checkBlynkConnection() {
  if (!Blynk.connected()) {
    Serial.println("Reconnecting to Blynk...");
    Blynk.connect();
  }
}

void loop() {
  Blynk.run();
  timer.run();
}