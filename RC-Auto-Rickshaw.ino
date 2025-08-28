//arduino nano hc05
#include <Servo.h>

#define IN1 5  // Motor driver input 1
#define IN2 6  // Motor driver input 2
#define SERVO_PIN 10 // Servo motor control pin

// Lights & Buzzer Pins
#define HEADLIGHT_PIN 7
#define TAILLIGHT_PIN 8
#define HORN_PIN 9

Servo steeringServo;

void setup() {
    Serial.begin(9600);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    steeringServo.attach(SERVO_PIN);
    steeringServo.write(90); // Center the steering initially

    // Initialize light & buzzer pins
    pinMode(HEADLIGHT_PIN, OUTPUT);
    pinMode(TAILLIGHT_PIN, OUTPUT);
    pinMode(HORN_PIN, OUTPUT);

    // Turn off all lights & horn initially
    digitalWrite(HEADLIGHT_PIN, LOW);
    digitalWrite(TAILLIGHT_PIN, LOW);
    digitalWrite(HORN_PIN, LOW);
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n'); // Read complete command
        input.trim(); // Remove unwanted spaces or newlines

        if (input.length() == 6) {
            char moveDir = input[0];
            int moveSpeed = input.substring(1, 3).toInt();
            char steerDir = input[3];
            int steerAngle = input.substring(4, 6).toInt();
            processMotion(moveDir, moveSpeed, steerDir, steerAngle);
        }
        else if (input.length() == 1) {
            char functionCommand = input[0];
            processFunction(functionCommand);
        }
    }
}

void processMotion(char moveDir, int moveSpeed, char steerDir, int steerAngle) {
    int pwmValue = map(moveSpeed, 0, 99, 0, 255);

    if (moveDir == 'F') {
        analogWrite(IN1, pwmValue);
        digitalWrite(IN2, LOW);
    } else if (moveDir == 'B') {
        analogWrite(IN2, pwmValue);
        digitalWrite(IN1, LOW);
    }

    int servoPosition = (steerDir == 'R') ? (90 + steerAngle) : (90 - steerAngle);
    steeringServo.write(servoPosition);
}

void processFunction(char command) {
    switch (command) {
        case 'Y':
            digitalWrite(HORN_PIN, HIGH);
            delay(500);
            digitalWrite(HORN_PIN, LOW);
            break;
        case 'U':
            digitalWrite(HEADLIGHT_PIN, HIGH);
            break;
        case 'u':
            digitalWrite(HEADLIGHT_PIN, LOW);
            break;
        case 'V':
            digitalWrite(TAILLIGHT_PIN, HIGH);
            break;
        case 'v':
            digitalWrite(TAILLIGHT_PIN, LOW);
            break;
        case 'W':
            digitalWrite(HEADLIGHT_PIN, HIGH);
            digitalWrite(TAILLIGHT_PIN, HIGH);
            break;
        case 'w':
            digitalWrite(HEADLIGHT_PIN, LOW);
            digitalWrite(TAILLIGHT_PIN, LOW);
            break;
        case 'X':
            for (int i = 0; i < 3; i++) {
                digitalWrite(TAILLIGHT_PIN, HIGH);
                delay(500);
                digitalWrite(TAILLIGHT_PIN, LOW);
                delay(500);
            }
            break;
        case 'x':
            break;
        case 'Z':
            Serial.println("Some Other Function");
            break;
        default:
            Serial.println(">__");
            break;
    }
}


