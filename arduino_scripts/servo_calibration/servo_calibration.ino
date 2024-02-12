#include <ServoTimer2.h>

ServoTimer2 servo;  // Create a servo object
float angle = -90;  // Input angle

void setup() {
  servo.attach(3); // Attach the servo on pin 9 to the servo object
  Serial.begin(9600); // Initialize serial communication for debugging
}

void loop() {
  // Example usage: Set PWM directly
  // You might want to replace this part with your own logic to change PWM values
  
  move_servo(angle);
  delay(1000); // Wait for a second
}



float angle2pwm(float angle) {
    // Approximate PWM responsive range: 700 - 2300 µs
    // Linear model angle = m*pwm + c
    // Valid range is 900 - 2100 µs, angle saturates outside these PWMs
    // Model RMS is 0.8 deg, central pwm is 1479
    // m = -0.15474 ± .00037 deg/µs
    // c = 228.89 ± 0.57 deg
    // Additional offset added: o = 1 deg

    float m = -0.15474; // deg/µs, or 6.5 µs/deg
    float c = 228.89;   // deg
    float o = 1;        // deg
    float pwm = (angle-(c+o))/m; // µs

  return pwm;
}

void move_servo(float angle) {
    // Move servo to the desired angle
    float pwm = angle2pwm(angle);
    servo.write(pwm);

    // Print info
    Serial.print("Moving servo to ");
    Serial.print(angle);
    Serial.print(" deg (PWM = ");
    Serial.print(pwm);
    Serial.println(")");
}
