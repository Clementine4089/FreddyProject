#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create an instance of the PCA9685 controller
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver();

// Servo channel assignments
const int blink_channel = 0;    // Blink servo channel
const int diagonal_channel = 1; // Diagonal (X) movement servo channel
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
int vertical_currentPulse = 225;      // Center position for vertical servo
int diagonal_currentPulse = 450;      // Center position for diagonal servo

// Variables for blinking behavior
unsigned long lastBlinkTime = 0;
unsigned long blinkInterval = 5000;  // Random blink every ~5 seconds
bool blink = false;

void setup() {
  Serial.begin(9600);

  // Initialize PCA9685 controller and set frequency for servos
  pca9685.begin();
  pca9685.setPWMFreq(60);  // 60 Hz for servos

  // Move servos to default (centered) positions
  smoothMoveTo(blink_channel, blinkMax, blink_currentPulse); // Open the blink servo
  smoothMoveTo(diagonal_channel, diagonal_currentPulse, diagonal_currentPulse); // Center diagonal servo
  smoothMoveTo(vertical_channel, vertical_currentPulse, vertical_currentPulse); // Center vertical servo
}

void loop() {
  // Check for incoming serial data to control X and Y movement
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');

    // If we receive a blink command
    if (data.equals("blink")) {
      blink = true;
    } else {
      // Parse the X and Y positions
      int commaIndex = data.indexOf(',');
      if (commaIndex != -1) {
        String xStr = data.substring(0, commaIndex);
        String yStr = data.substring(commaIndex + 1);

        int x_pos = xStr.toInt();
        int y_pos = yStr.toInt();

        // Map X and Y positions to pulse ranges for diagonal and vertical servos
        int targetDiagonalPulse = map(x_pos, 0, 180, diagonalMin, diagonalMax);
        int targetVerticalPulse = map(y_pos, 0, 180, verticalMin, verticalMax);

        // Move the eye servos smoothly to the new positions
        smoothMoveTo(diagonal_channel, targetDiagonalPulse, diagonal_currentPulse);
        smoothMoveTo(vertical_channel, targetVerticalPulse, vertical_currentPulse);
      }
    }
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

// Function to move a servo smoothly to a target pulse width
void smoothMoveTo(int channel, int targetPulse, int &currentPulse) {
  int step = (targetPulse > currentPulse) ? 1 : -1;  // Determine the direction of movement
  while (currentPulse != targetPulse) {
    currentPulse += step;  // Move in steps of 1 unit
    pca9685.setPWM(channel, 0, currentPulse);  // Update the servo position
    delay(1);  // Control movement speed (adjust delay for faster/slower movement)
  }
}
