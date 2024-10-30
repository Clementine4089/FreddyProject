#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create an instance of the PCA9685 controller
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver();

// Servo channel assignments
const int blink_channel = 0;    // Blink servo channel
const int horizontal_channel = 1; // Horizontal movement servo channel
const int vertical_channel = 2; // Vertical (Y) movement servo channel

// Servo pulse range limits
const int blinkMin = 630;
const int blinkMax = 450;
const int verticalMin = 150;
const int verticalMax = 300;
const int horizontalMin = 320;
const int horizontalMax = 400;

// Variables to hold the current pulse positions of the servos
int blink_currentPulse = blinkMax;    // Start with the eye open (blinkMax)
int vertical_currentPulse = (verticalMax - verticalMin) / 2 + verticalMin;      // Center position for vertical servo
int horizontal_currentPulse = (horizontalMax - horizontalMin) / 2 + horizontalMin;     // Center position for horizontal servo

// Variables for blinking behavior
unsigned long lastBlinkTime = 0;
unsigned long blinkInterval = 5000;  // Random blink every ~5 seconds
bool blink = false;

// Screen dimensions (example values, adjust as needed)
const int screenWidth = 640;
const int screenHeight = 480;

// Variables for tracking target presence
unsigned long lastTargetTime = 0;
const unsigned long targetTimeout = 2000;  // 2 seconds timeout

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  pca9685.begin();     // Initialize the PCA9685 controller

  // Set the PWM frequency to 50Hz (standard for servos)
  pca9685.setPWMFreq(50);

  // Initialize the servos to their default positions
  pca9685.setPWM(blink_channel, 0, blink_currentPulse);
  pca9685.setPWM(vertical_channel, 0, vertical_currentPulse);
  pca9685.setPWM(horizontal_channel, 0, horizontal_currentPulse);
}

void loop() {
  // Check for incoming serial data to control X and Y movement
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');

    // Parse the X and Y positions
    int commaIndex = data.indexOf(',');
    if (commaIndex != -1) {
      String xStr = data.substring(0, commaIndex);
      String yStr = data.substring(commaIndex + 1);

      int x = xStr.toInt();
      int y = yStr.toInt();

      // Move the servos based on the parsed X and Y positions
      moveEyeTo(x, y);

      // Update the last target time
      lastTargetTime = millis();
    }
  }

  // Check if the target timeout has been reached
  if (millis() - lastTargetTime > targetTimeout) {
    // If no target for 2 seconds, move eye side to side
    moveEyeSideToSide();
  }

  // Handle random blinking
  if (blink || millis() - lastBlinkTime > blinkInterval) {
    blinkEye();
    lastBlinkTime = millis();
    blinkInterval = random(3000, 7000);  // Random interval between 3 and 7 seconds
    blink = false;
  }
}

// Function to blink the eye by moving the blink servo smoothly
void blinkEye() {
  // Close the eye
  smoothMoveTo(blink_channel, blinkMin, blink_currentPulse);
  delay(200);  // Keep eyelid closed for 200ms

  // Open the eye
  smoothMoveTo(blink_channel, blinkMax, blink_currentPulse);
}

// Function to move the eye to a specific screen coordinate
void moveEyeTo(int x, int y) {
  // Map screen coordinates to servo pulse widths
  int targetHorizontalPulse = map(x, 0, screenWidth, horizontalMin, horizontalMax);
  int targetVerticalPulse = map(y, 0, screenHeight, verticalMin, verticalMax);

  // Ensure the target pulses are within bounds
  targetHorizontalPulse = constrain(targetHorizontalPulse, horizontalMin, horizontalMax);
  targetVerticalPulse = constrain(targetVerticalPulse, verticalMin, verticalMax);

  // Move the servos smoothly to the target positions
  smoothMoveTo(horizontal_channel, targetHorizontalPulse, horizontal_currentPulse);
  smoothMoveTo(vertical_channel, targetVerticalPulse, vertical_currentPulse);
}

// Function to move the eye side to side when no target is present
void moveEyeSideToSide() {
  static bool movingRight = true;
  static unsigned long lastMoveTime = 0;
  const unsigned long moveInterval = 1000;  // Move every 1 second

  if (millis() - lastMoveTime > moveInterval) {
    int targetHorizontalPulse = movingRight ? horizontalMax : horizontalMin;

    // Ensure the target pulse is within bounds
    targetHorizontalPulse = constrain(targetHorizontalPulse, horizontalMin, horizontalMax);

    // Move the servo smoothly to the target position
    smoothMoveTo(horizontal_channel, targetHorizontalPulse, 10, horizontal_currentPulse);

    movingRight = !movingRight;
    lastMoveTime = millis();
  }
}

// Function to move a servo smoothly to a target pulse width
void smoothMoveTo(int channel, int targetPulse, int &currentPulse) {
  int step = (targetPulse > currentPulse) ? 1 : -1;  // Determine the direction of movement
  while (currentPulse != targetPulse) {
    currentPulse += step;  // Move in steps of 1 unit
    pca9685.setPWM(channel, 0, currentPulse);  // Update the servo position
    delay(1);  // Control movement speed (adjust delay for faster/slower movement)
  }
}

void smoothMoveTo(int channel, int targetPulse, int speed, int &currentPulse) {
  int step = (targetPulse > currentPulse) ? 1 : -1;  // Determine the direction of movement
  while (currentPulse != targetPulse) {
    currentPulse += step;  // Move in steps of 1 unit
    pca9685.setPWM(channel, 0, currentPulse);  // Update the servo position
    delay(speed);  // Control movement speed (adjust delay for faster/slower movement)
  }
}
