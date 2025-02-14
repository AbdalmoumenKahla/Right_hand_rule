#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X(); // Left  1
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X(); // Front 2
Adafruit_VL53L0X sensor3 = Adafruit_VL53L0X(); // Right 3
#define SHUT1 5  // Left  1
#define SHUT2 16  // Front 2
#define SHUT3 17  // Right 3

// ========= Pin Definitions for Motors =========
#define ENA 12  // Motor A Speed (Enable A) - PWM-capable
#define ENB 14  // Motor B Speed (Enable B) - PWM-capable
#define IN1 25  // Motor A Control Pin 1
#define IN2 26  // Motor A Control Pin 2
#define IN3 32  // Motor B Control Pin 1
#define IN4 33  // Motor B Control Pin 2

// ========= Encoder Pins =========
#define ENCODER_A_PIN_A 18
#define ENCODER_A_PIN_B 19
#define ENCODER_B_PIN_A 2
#define ENCODER_B_PIN_B 4

// ========= Global Variables for Encoders =========
volatile long encoderCountA = 0;  
volatile long encoderCountB = 0;  

// ========= Constants for Distance =========
const float WHEEL_DIAMETER = 4.5;  // cm
const float TICKS_PER_REV = 160;   // Encoder ticks per revolution
const float CM_PER_TICK = (PI * WHEEL_DIAMETER) / TICKS_PER_REV;

// PID Constants for forward/backward
float Kp = 1.2;  
float Ki = 0.1;   
float Kd = 0.05;  

float targetDistance = 18.0; 
float previousError = 0;
float integral = 0;

// ========= MPU6050 Setup =========
#define MPU_ADDR 0x68 

// We now separate the "raw" yaw angle from the "offset" used to zero it
float rawYawAngle = 0.0;   // Integrated yaw from gyro (unbounded)
float yawOffset   = 0.0;   // Offset to treat initial heading as zero

// For timekeeping in gyro integration
unsigned long prev_time = 0; 

// ========= Maze Representation =========
const int MAZE_SIZE = 8;
int maze[MAZE_SIZE][MAZE_SIZE];
int currentX = 0, currentY = 0;
int goalX = MAZE_SIZE - 1, goalY = MAZE_SIZE - 1;

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA = GPIO21, SCL = GPIO22
  
  // Initialize SHUT pins
  pinMode(SHUT1, OUTPUT);
  pinMode(SHUT2, OUTPUT);
  pinMode(SHUT3, OUTPUT);

  // Disable all sensors
  digitalWrite(SHUT1, LOW);
  digitalWrite(SHUT2, LOW);
  digitalWrite(SHUT3, LOW);
  
  delay(10);

  // Initialize sensor 1
  digitalWrite(SHUT1, HIGH);
  delay(10);  // Wait for sensor to power up
  if (sensor1.begin()) {
    sensor1.setAddress(0x30);  // Assign new address to sensor 1
    Serial.println("Sensor 1 initialized at 0x30");
  }

  // Initialize sensor 2
  digitalWrite(SHUT2, HIGH);
  delay(10);  // Wait for sensor to power up
  if (sensor2.begin()) {
    sensor2.setAddress(0x31);  // Assign new address to sensor 2
    Serial.println("Sensor 2 initialized at 0x31");
  }

  // Initialize sensor 3
  digitalWrite(SHUT3, HIGH);
  delay(10);  // Wait for sensor to power up
  if (sensor3.begin()) {
    sensor3.setAddress(0x32);  // Assign new address to sensor 3
    Serial.println("Sensor 3 initialized at 0x32");
  }

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Encoder pins
  pinMode(ENCODER_A_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_A_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN_B, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN_A), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN_A), encoderB_ISR, CHANGE);

  // Initialize MPU6050 (wake up)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Clear the sleep bit
  Wire.endTransmission(true);

  // Initialize motors off
  stopMotors();

  // Initialize time for gyro integration
  prev_time = millis();

  // ========= Set Yaw Offset to Make Initial Orientation = 0° =========
  // 1) Let the sensor stabilize for a short moment
  delay(1000);

  // 2) Make a few initial gyro reads to start integration
  for (int i = 0; i < 50; i++) {
    getYawAngle();  // updates rawYawAngle
    delay(10);
  }

  // 3) The current rawYawAngle is our "starting orientation" => offset
  yawOffset = rawYawAngle;
  Serial.print("Initial yawOffset set to: ");
  Serial.println(yawOffset);

  // Initialize maze
  initializeMaze();
}

// ========== LOOP ==========
void loop() {
  // Update maze with sensor data
  updateMaze();

  // Determine next move using Right-Hand Rule
  int nextMove = getNextMoveRightHandRule();

  // Execute move
  executeMove(nextMove);

  // Check if goal is reached
  if (currentX == goalX && currentY == goalY) {
    Serial.println("Goal reached!");
    while (1) { 
      stopMotors();
    }
  }
}

// ========== RIGHT-HAND RULE FUNCTIONS ==========
int getNextMoveRightHandRule() {
  // Priority: Right -> Straight -> Left -> Turn Around
  if (!isWall(3)) { // No wall on the right
    return 2; // Turn right
  } else if (!isWall(2)) { // No wall in front
    return 0; // Move forward
  } else if (!isWall(1)) { // No wall on the left
    return 1; // Turn left
  } else {
    return 3; // Turn around (180 degrees)
  }
}

void executeMove(int move) {
  switch (move) {
    case 0: // Move forward
      moveForwardPID(targetDistance, 100); // Max speed = 100
      updatePositionAfterMove(0);
      break;
    case 1: // Turn left
      turnRelativeAngle(-90, 100); // Max speed = 100
      updatePositionAfterMove(1);
      break;
    case 2: // Turn right
      turnRelativeAngle(90, 100); // Max speed = 100
      updatePositionAfterMove(2);
      break;
    case 3: // Turn around (180 degrees)
      turnRelativeAngle(180, 100); // Max speed = 100
      updatePositionAfterMove(3);
      break;
    default:
      Serial.println("Invalid move!");
      break;
  }
}

void updatePositionAfterMove(int move) {
  // Update the robot's position based on the move and current orientation
  float currentAngle = getYawAngle();
  switch (move) {
    case 0: // Forward
      if (abs(currentAngle - 0) <= 45) currentX--; // Facing north
      else if (abs(currentAngle - 90) <= 45) currentY++; // Facing east
      else if (abs(currentAngle - 180) <= 45 || abs(currentAngle + 180) <= 45) currentX++; // Facing south
      else if (abs(currentAngle + 90) <= 45) currentY--; // Facing west
      break;
    case 1: // Left
      // Update orientation (subtract 90 degrees)
      rawYawAngle -= 90;
      if (rawYawAngle < -180) rawYawAngle += 360;
      break;
    case 2: // Right
      // Update orientation (add 90 degrees)
      rawYawAngle += 90;
      if (rawYawAngle > 180) rawYawAngle -= 360;
      break;
    case 3: // Turn around
      // Update orientation (add 180 degrees)
      rawYawAngle += 180;
      if (rawYawAngle > 180) rawYawAngle -= 360;
      break;
  }
}

// ========== MAZE FUNCTIONS ==========
void initializeMaze() {
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      maze[i][j] = 0; // 0 represents unknown
    }
  }
}

void updateMaze() {
  // Update left wall
  if (isWall(1)) {
    maze[currentX][currentY] |= 0x01; // Set left wall
  }
  // Update front wall
  if (isWall(2)) {
    maze[currentX][currentY] |= 0x02; // Set front wall
  }
  // Update right wall
  if (isWall(3)) {
    maze[currentX][currentY] |= 0x04; // Set right wall
  }
}

// ========== GYRO UTILS ==========
int16_t readSensorData(uint8_t highReg, uint8_t lowReg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(highReg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  return (Wire.read() << 8 | Wire.read());
}

float getYawAngle() {
  int16_t gyro_z = readSensorData(0x47, 0x48);
  float gyro_z_dps = gyro_z / 131.0;
  unsigned long current_time = millis();
  float dt = (current_time - prev_time) / 1000.0;
  prev_time = current_time;
  rawYawAngle += gyro_z_dps * dt;
  float adjustedYaw = rawYawAngle - yawOffset;
  if (adjustedYaw > 180.0)  adjustedYaw -= 360.0;
  if (adjustedYaw < -180.0) adjustedYaw += 360.0;
  return adjustedYaw;
}

// ========== TURNING FUNCTION WITH FINE-TUNING ==========
void turnRelativeAngle(float relativeAngle, int maxSpeed) {
  float startAngle = getYawAngle();
  float targetAngle = startAngle + relativeAngle;
  float error = 0;
  float previousError = 0;
  float integral = 0;
  float Kp_turn = 1.2;
  float Ki_turn = 0.01;
  float Kd_turn = 0.2;

  // Main turn
  while (true) {
    float currentAngle = getYawAngle();
    error = targetAngle - currentAngle;
    if (error > 180)  error -= 360;
    if (error < -180) error += 360;
    integral += error;
    float derivative = error - previousError;
    previousError = error;
    float output = (Kp_turn * error) + (Ki_turn * integral) + (Kd_turn * derivative);
    int speed = constrain((int)output, 0, maxSpeed);

    if (error > 0) {
      analogWrite(ENA, speed);
      analogWrite(ENB, speed);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else if (error < 0) {
      analogWrite(ENA, speed);
      analogWrite(ENB, speed);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }

    // Stop if within ±1° of the target angle
    if (abs(error) <= 1.0) {
      stopMotors();
      break;
    }
    delay(10);
  }

  // Fine-tuning after the main turn
  fineTuneTurn(targetAngle, maxSpeed / 2); // Use half speed for fine-tuning
}

// ========== FINE-TUNING FUNCTION ==========
void fineTuneTurn(float targetAngle, int fineTuneSpeed) {
  float error = 0;
  float previousError = 0;
  float integral = 0;
  float Kp_fine = 1.0; // Lower gains for fine-tuning
  float Ki_fine = 0.01;
  float Kd_fine = 0.1;

  while (true) {
    float currentAngle = getYawAngle();
    error = targetAngle - currentAngle;
    if (error > 180)  error -= 360;
    if (error < -180) error += 360;
    integral += error;
    float derivative = error - previousError;
    previousError = error;
    float output = (Kp_fine * error) + (Ki_fine * integral) + (Kd_fine * derivative);
    int speed = constrain((int)output, 0, fineTuneSpeed);

    if (error > 0) {
      analogWrite(ENA, speed);
      analogWrite(ENB, speed);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else if (error < 0) {
      analogWrite(ENA, speed);
      analogWrite(ENB, speed);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }

    // Stop if within ±0.5° of the target angle
    if (abs(error) <= 0.5) {
      stopMotors();
      break;
    }
    delay(10);
  }
}

// ========== FORWARD/BACKWARD PID ==========
void moveForwardPID(float distance, int maxSpeed) {
  encoderCountA = 0;
  encoderCountB = 0;
  previousError = 0;
  integral = 0;

  while (true) {
    float currentDistanceA = encoderCountA * CM_PER_TICK;
    float currentDistanceB = encoderCountB * CM_PER_TICK;
    float averageDistance = -1*((currentDistanceA + currentDistanceB) / 2.0);

    if (averageDistance >= distance) {
      stopMotors();
      break;
    }

    float error = distance - averageDistance;
    integral += error;
    float derivative = error - previousError;
    previousError = error;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    int speed = constrain((int)output, 0, maxSpeed);

    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    delay(10);
  }
}

void moveBackwardPID(float distance, int maxSpeed) {
  encoderCountA = 0;
  encoderCountB = 0;
  previousError = 0;
  integral = 0;

  while (true) {
    float currentDistanceA = encoderCountA * CM_PER_TICK;
    float currentDistanceB = encoderCountB * CM_PER_TICK;
    float averageDistance = (currentDistanceA + currentDistanceB) / 2.0);

    if (averageDistance >= distance) {
      stopMotors();
      break;
    }

    float error = distance - averageDistance;
    integral += error;
    float derivative = error - previousError;
    previousError = error;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    int speed = constrain((int)output, 0, maxSpeed);

    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    delay(10);
  }
}

//ENCODER ISRs
void encoderA_ISR() {
  bool pinA = digitalRead(ENCODER_A_PIN_A);
  bool pinB = digitalRead(ENCODER_A_PIN_B);
  if (pinA == pinB)
    encoderCountA++;
  else
    encoderCountA--;
}

void encoderB_ISR() {
  bool pinA = digitalRead(ENCODER_B_PIN_A);
  bool pinB = digitalRead(ENCODER_B_PIN_B);
  if (pinA == pinB)
    encoderCountB++;
  else
    encoderCountB--;
}

// STOP MOTORS 
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

bool isWall(int sensorIndex, int threshold = 150) {
  VL53L0X_RangingMeasurementData_t measure;

  switch (sensorIndex) {
    case 1: // Left sensor
      sensor1.rangingTest(&measure, false);
      break;

    case 2: // Front sensor
      sensor2.rangingTest(&measure, false);
      break;

    case 3: // Right sensor
      sensor3.rangingTest(&measure, false);
      break;

    default:
      Serial.print("Invalid sensor index: ");
      Serial.println(sensorIndex);
      return false; 
  }

  if (measure.RangeStatus == 4) {
    Serial.print("Sensor ");
    Serial.print(sensorIndex);
    Serial.println(" out of range");
    return false;
  }

  if (measure.RangeMilliMeter <= threshold) {
    Serial.print("Sensor ");
    Serial.print(sensorIndex);
    Serial.print(" sees wall at: ");
    Serial.print(measure.RangeMilliMeter);
    Serial.println(" mm");
    return true;
  } else {
    Serial.print("Sensor ");
    Serial.print(sensorIndex);
    Serial.print(" distance: ");
    Serial.print(measure.RangeMilliMeter);
    Serial.println(" mm -> No wall");
    return false;
  }
}