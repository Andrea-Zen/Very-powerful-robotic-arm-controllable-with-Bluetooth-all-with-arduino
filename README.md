Arduino Dual Servo Control with Button
This Arduino project controls two servos with a single button. Pressing the button moves the first servo from 0 to 90 degrees and the second servo from 90 to 0 degrees. Releasing the button returns both servos to their initial positions. The movement speed of the servos is adjustable for smoother, faster transitions.

Components
1 x Arduino Uno R3
1 x Push Button
1 x Servo Motor (for pin 9)
1 x Servo Motor (for pin 10)
Jumper Wires
Breadboard (optional)
Power Supply (if required for high-power servos)
Wiring Diagram
Button:

Connect one terminal of the button to pin 2 on the Arduino.
Connect the other terminal to GND.
The internal pull-up resistor on pin 2 is enabled in the code.
Servos:

Connect the control wire of the first servo to pin 9 on the Arduino.
Connect the control wire of the second servo to pin 10.
Connect both servos' power (VCC) and ground (GND) to the Arduino or an external power supply if necessary.
Code
The code gradually moves both servos to opposite positions when the button is pressed. The movement speed is adjustable using the stepDelay variable.

cpp
Copia codice
#include <Servo.h>

Servo servo1;                  // Servo 1 object
Servo servo2;                  // Servo 2 object
const int buttonPin = 2;       // Pin connected to the button
int servo1Position = 0;        // Initial position of servo 1
int servo2Position = 90;       // Initial position of servo 2
int targetPosition1 = 0;       // Target position for servo 1
int targetPosition2 = 90;      // Target position for servo 2
bool buttonState = false;      // Button state
bool lastButtonState = false;  // Last button state
int stepDelay = 5;             // Delay between each servo step (lower = faster)

void setup() {
  servo1.attach(9);              // Attach servo 1 to pin 9
  servo2.attach(10);             // Attach servo 2 to pin 10
  pinMode(buttonPin, INPUT_PULLUP); // Enable pull-up resistor on pin 2
  servo1.write(servo1Position);  // Set initial position for servo 1
  servo2.write(servo2Position);  // Set initial position for servo 2
}

void loop() {
  bool currentButtonState = digitalRead(buttonPin) == LOW; // Read button state

  // Check for button state change (debounce)
  if (currentButtonState != lastButtonState) {
    delay(50); // Debounce delay
    if (currentButtonState == true) { // When button is pressed
      // Toggle target positions
      if (targetPosition1 == 0) {
        targetPosition1 = 90;   // Set target for servo 1 to 90 degrees
        targetPosition2 = 0;    // Set target for servo 2 to 0 degrees
      } else {
        targetPosition1 = 0;    // Reset target for servo 1 to 0 degrees
        targetPosition2 = 90;   // Reset target for servo 2 to 90 degrees
      }

      // Move servos gradually to target positions
      moveServosGradually(servo1Position, targetPosition1, servo1);
      moveServosGradually(servo2Position, targetPosition2, servo2);

      // Update current positions
      servo1Position = targetPosition1;
      servo2Position = targetPosition2;
    }
  }

  lastButtonState = currentButtonState; // Store previous button state
}

// Function to gradually move servos to target position
void moveServosGradually(int startPos, int endPos, Servo &servo) {
  if (startPos < endPos) {
    for (int pos = startPos; pos <= endPos; pos++) {
      servo.write(pos);           // Move servo to current position
      delay(stepDelay);           // Step delay for speed control
    }
  } else {
    for (int pos = startPos; pos >= endPos; pos--) {
      servo.write(pos);           // Move servo to current position
      delay(stepDelay);           // Step delay for speed control
    }
  }
}
Configuration
Adjusting Servo Speed:
The stepDelay variable controls how quickly each servo moves. Lower values make the servos move faster, while higher values slow down the movement.
Initial and Target Positions:
Modify servo1Position, servo2Position, targetPosition1, and targetPosition2 to set the initial and final positions for each servo.
Troubleshooting
Servo Power: If the servos are unresponsive or moving erratically, ensure they have adequate power. For high-power servos, consider using an external power source (e.g., 5V supply) with a common ground with the Arduino.
Debounce Timing: If the button is not responding reliably, you may need to adjust the delay(50); line to a slightly higher or lower value to improve debounce performance.
License
This project is open-source and available for modification and distribution.
