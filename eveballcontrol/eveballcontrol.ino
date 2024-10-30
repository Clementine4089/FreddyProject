#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create an instance of the PCA9685 controller
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver();

// Servo channel assignments
const int blink_channel = 0;    // Blink servo channel
const int vertical_channel = 2; // Vertical (Y) movement servo channel
const int diagonal_channel = 1; // Diagonal movement servo channel

// Servo pulse range limits
const int blinkMin = 630;
const int blinkMax = 450;
const int verticalMin = 150;
const int verticalMax = 300;
const int diagonalMin = 150;
const int diagonalMax = 300;

// Variables to hold the current pulse positions of the servos
int blink_currentPulse = blinkMax;    // Start with the eye open (blinkMax)
int vertical_currentPulse = 225;      // Center position for vertical servo
int diagonal_currentPulse = 225;      // Center position for diagonal servo

// Variables for blinking behavior
unsigned long lastBlinkTime = 0;
unsigned long blinkInterval = 5000;  // Random blink every ~5 seconds
bool blink = false;

// Variables for vertical movement behavior
bool movingUp = true;

// Variables for diagonal movement behavior
bool movingDiagonalUp = true;

void setup() {
  Serial.begin(9600);

  // Initialize PCA9685 controller and set frequency for servos
  pca9685.begin();
  pca9685.setPWMFreq(60);  // 60 Hz for servos

  // Move servos to default (centered) positions
  smoothMoveTo(blink_channel, blinkMax, blink_currentPulse); // Open the blink servo
  smoothMoveTo(vertical_channel, vertical_currentPulse, vertical_currentPulse); // Center vertical servo
  smoothMoveTo(diagonal_channel, diagonal_currentPulse, diagonal_currentPulse); // Center diagonal servo
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
    blinkEye();
    lastBlinkTime = millis();
    blinkInterval = random(3000, 7000);  // Random interval between 3 and 7 seconds
    blink = false;
  }

  // Handle constant up and down movement for vertical servo
  if (movingUp) {
    vertical_currentPulse++;
    if (vertical_currentPulse >= verticalMax) {
      movingUp = false;
    }
  } else {
    vertical_currentPulse--;
    if (vertical_currentPulse <= verticalMin) {
      movingUp = true;
    }
  }
  pca9685.setPWM(vertical_channel, 0, vertical_currentPulse);

  // Handle constant up and down movement for diagonal servo
  if (movingDiagonalUp) {
    diagonal_currentPulse++;
    if (diagonal_currentPulse >= diagonalMax) {
      movingDiagonalUp = false;
    }
  } else {
    diagonal_currentPulse--;
    if (diagonal_currentPulse <= diagonalMin) {
      movingDiagonalUp = true;
    }
  }
  pca9685.setPWM(diagonal_channel, 0, diagonal_currentPulse);

  delay(10);  // Adjust delay for movement speed
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
