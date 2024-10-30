#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create an instance of the PCA9685 controller
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver();

// Servo channel assignments
const int blink_channel = 0;    // Blink servo channel
const int diagonal_channel = 1; // Diagonal movement servo channel
const int vertical_channel = 2; // Vertical (Y) movement servo channel

// Servo pulse range limits
const int blinkMin = 630;
const int blinkMax = 450;
const int verticalMin = 150;
const int verticalMax = 300;
const int diagonalMin = 400;
const int diagonalMax = 500;
// Variables to hold the current pulse positions of the servos
int blink_currentPulse = blinkMax;    // Start with the eye open (blinkMax)
int vertical_currentPulse = (verticalMax - verticalMin) / 2;      // Center position for vertical servo
int diagonal_currentPulse = (diagonalMax - diagonalMin) / 2;     // Center position for diagonal servo

// Variables for blinking behavior
unsigned long lastBlinkTime = 0;
unsigned long blinkInterval = 5000;  // Random blink every ~5 seconds
bool blink = false;

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  pca9685.begin();     // Initialize the PCA9685 controller

  // Set the PWM frequency to 50Hz (standard for servos)
  pca9685.setPWMFreq(50);

  // Initialize the servos to their default positions
  pca9685.setPWM(blink_channel, 0, blink_currentPulse);
  pca9685.setPWM(vertical_channel, 0, vertical_currentPulse);
  pca9685.setPWM(diagonal_channel, 0, diagonal_currentPulse);
}

void loop() {
  // Check for incoming serial data to control Y movement
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');

    // If we receive a blink command
    if (data.equals("blink")) {
      blink = true;
    }
  }

  // Handle random blinking
  if (blink || millis() - lastBlinkTime > blinkInterval) {
//    /blinkEye();
    lastBlinkTime = millis();
    blinkInterval = random(3000, 7000);  // Random interval between 3 and 7 seconds
    blink = false;
    moveEyeUp();
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


// Function to blink the eye by moving the blink servo smoothly
void moveEyeUp() {
  // Close the eye
  smoothMoveTo(vertical_channel, verticalMin, vertical_currentPulse);
  delay(200);  // Keep eyelid closed for 200ms

  // Open the eye
  smoothMoveTo(vertical_channel, verticalMax, vertical_currentPulse);
}

void moveEyeDiagonal() {
  smoothMoveTo(diagonal_channel, diagonalMin, diagonal_currentPulse);
  delay(200);
  smoothMoveTo(diagonal_channel, diagonalMax, diagonal_currentPulse);
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
