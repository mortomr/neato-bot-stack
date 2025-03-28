// # motor_and_encoder_test.ino
// Arduino Uno version: ROS2-ready motor + encoder control

// ==== Pin Definitions ====
const int PWM_LEFT = 3;   // D3 (PWM)
const int DIR_LEFT = 4;   // D4
const int PWM_RIGHT = 5;  // D5 (PWM)
const int DIR_RIGHT = 6;  // D6

const int ENC_LEFT = 2;   // D2 (INT0)
const int ENC_RIGHT = 7;  // D7 (polled)

// ==== Encoder State ====
volatile long left_ticks = 0;
volatile long right_ticks = 0;

// ==== Setup ====
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(DIR_LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);

  // Encoder pins
  pinMode(ENC_LEFT, INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT), countLeft, RISING);

  Serial.println("status:ready");
}

// ==== Encoder ISR ====
void countLeft() {
  left_ticks++;
}

// ==== Main Loop ====
void loop() {
  static unsigned long last_publish = 0;
  unsigned long now = millis();

  // Poll right encoder
  static int lastState = HIGH;
  int currentState = digitalRead(ENC_RIGHT);
  if (lastState == HIGH && currentState == LOW) {
    right_ticks++;
  }
  lastState = currentState;

  // Handle incoming serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleCommand(cmd);
  }

  // Publish encoder data at 10Hz
  if (now - last_publish >= 100) {
    Serial.print("encoder_l:"); Serial.print(left_ticks);
    Serial.print(" encoder_r:"); Serial.println(right_ticks);
    last_publish = now;
  }
}

// ==== Command Handler ====
void handleCommand(String cmd) {
  if (cmd.length() == 0) return;  // Ignore blank lines
  if (cmd.startsWith("SET_LPWM ")) {
    int val = cmd.substring(9).toInt();
    setMotor(PWM_RIGHT, DIR_RIGHT, -val);  // ← Right motor, flipped
  } else if (cmd.startsWith("SET_RPWM ")) {
    int val = cmd.substring(9).toInt();
    setMotor(PWM_LEFT, DIR_LEFT, -val);   // ← Left motor, flipped
  } else if (cmd == "STOP_MOTORS") {
    setMotor(PWM_LEFT, DIR_LEFT, 0);
    setMotor(PWM_RIGHT, DIR_RIGHT, 0);
    Serial.println("status:motors_stopped");
  } else if (cmd == "RESET_ENCODERS") {
    left_ticks = 0;
    right_ticks = 0;
    Serial.println("status:encoders_reset");
  } else {
    Serial.println("error:unknown_command");
  }
}


// ==== Motor Driver ====
void setMotor(int pwmPin, int dirPin, int power) {
  bool dir = (power >= 0);
  if (pwmPin == PWM_LEFT) dir = !dir; // Flip left motor direction
  if (pwmPin == PWM_LEFT || pwmPin == PWM_RIGHT) dir = !dir;
  int pwm = constrain(abs(power), 0, 255);
  digitalWrite(dirPin, dir ? HIGH : LOW);
  analogWrite(pwmPin, pwm);
} // power: -255 to +255
