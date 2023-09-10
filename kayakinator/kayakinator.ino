/******************************************
 * LIBRARIES, WIRING AND GLOBAL VARIABLES *
 ******************************************/

// Call the ServoTimer2 library and create Servo objects called "esc_right" and "esc_left".
// Didn't use Servo.h library because it generated an interference with the PWM signal to the motor when the Bluetooth module was connected, making the motor jitter randomly.
#include <ServoTimer2.h>
ServoTimer2 esc_right; 
ServoTimer2 esc_left;

// Call the SoftwareSerial library and create an additional serial port called "kayakinator" refering to our HM-10 BLE device.
// Didn't use the AltSoftSerial library because both that and the Servo.h library used the same Timer in the Arduino, generating a conflict.
// It uses the 8 (TX) and 9 (RX) pins of the Arduino for BLE TX and RX communications by default.
#include <SoftwareSerial.h>
SoftwareSerial kayakinator(8, 9);

// Incoming variables
String message1;
String message2;
char onoff, single_twin;
int autopilot_onoff, autopilot_mode;
float target_power, heading, angular_speed, target_heading;

// Timers and internal variables
boolean newData = false;                // incoming data flag
const int neutral_pwm = 1500;           // stores the neutral PWM value in microseconds
float killswitch_start = millis();      // KILL TIMER: initiates a timer in milliseconds
float killswitch_timer;                 // KILL TIMER: keep track of how long communications have been down
const float killswitch_time_ms = 5000;  // KILL TIMER: defines the kill time of the timer in milliseconds, can be edited by the user
float energy_interval;                  // ENERGY TIMER: keeps track of cycles between motor changes to keep track of energy usage in the app
float energy_start_time;                // ENERGY TIMER: start time of the energy interval timer
float message_interval;                 // MESSAGE TIMER: keeps track of the time between consecutive incoming essages, used for the Kalman filter
float message_start_time;               // MESSAGE TIMER: start time of the message interval timer

// Autopilot and Kalman filter variables
float PowerLevel_right;
float PowerLevel_left;


// Outgoing variables



void setup() {

  // Serial setup. Initializes the serial to communicate at a specific baud rate (9600)
  Serial.begin(9600);

  // For testing purposes
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);
  
  // Bluetooth HM-10 setup. Initializes the port to communicate at a specific baud rate (9600)
  kayakinator.begin(9600);

  // Servo ESC setup: replace the numbers by the Arduino pin number (~) that is being used to connect to the right or left motor's ESC signal port
  esc_right.attach(5); 
  esc_left.attach(6);
  
  // Arm both motors
  arm_motors();

}





/***********************************
 *            MAIN LOOP            *
 ***********************************/


void loop() {

  // Receive and assign the data
  receive_data(2); // set the argument to the number of messages being received
  assign_data(message1, message2);

  // Process the data and command the motors

  //kalmanFilter();
  //autopilot();
  //assignPower();

  // Send out diagnostic data
  send_data();



  print_data();

}


// int autopilot_onoff, autopilot_mode;
// float target_power, heading, angular_speed, target_heading;



void processMessage () {

  if (onoff == "N") {

    // Single engine mode: only give power to the left engine on pin 6
    if (single_twin == "S") {
      set_power(target_power, 0);
    }

    // Twin engine mode: give power to both engines, either with or without autopilot
    else if (single_twin == "T") {


      if (autopilot_onoff == 0) { 
        // Give equal power to both engines
        set_power(target_power, target_power);

      }

      // AUTOPILOT ENGAGED
      else if (autopilot_onoff == 1) { 

      }





    }

  }


  else if (onoff == "F") { // if off, shut down engines
  gradual_shutdown();

  }


  // TODO: Measure the time interval of the loop in order to send back a message


}



/*******************************
 *          AUTOPILOT          *
 *******************************/


void autopilot() {

}






/*****************************
 *       KALMAN FILTER       *
 *****************************/



void kalman_filter() {

}





/*****************************
 *        MOTOR TOOLS        *
 *****************************/



void arm_motors() {

  // Arm motors by sending neutral PWM
  esc_left.write(neutral_pwm);
  esc_right.write(neutral_pwm); 

  // Record the value sent
  PowerLevel_left  = 0;
  PowerLevel_right = 0;

}



void set_power(float newPowerLevel_left, float newPowerLevel_right) {

  // Constrain power levels
  newPowerLevel_left  = constrain(newPowerLevel_left,  0, 100);
  newPowerLevel_right = constrain(newPowerLevel_right, 0, 100);

  // Convert power levels to PWMs
  float pwm_left  = PowerLevel_to_pwm(newPowerLevel_left);
  float pwm_right = PowerLevel_to_pwm(newPowerLevel_right);

  // Send command to motors
  esc_left.write(pwm_left); 
  esc_right.write(pwm_right);

  // Record the value sent
  PowerLevel_left  = newPowerLevel_left;
  PowerLevel_right = newPowerLevel_right;

}



float PowerLevel_to_pwm(float PowerLevel) {
  // Engine power level (0-100) to PWM converter

  PowerLevel = max(0.0, PowerLevel);
  float pwm = 59.331705193777 * pow(PowerLevel, 0.462841587096) + 1500.0;
  pwm = constrain(pwm, 1500.0, 2000.0);
  return pwm;

}


void gradual_shutdown(int delay_ms=50, float step_change=5) {

  // Check if either power level is greater than zero
  while (PowerLevel_left > 0 || PowerLevel_right > 0) {

    // Lower current power level
    PowerLevel_left  = constrain(PowerLevel_left  - step_change, 0, 100);
    PowerLevel_right = constrain(PowerLevel_right - step_change, 0, 100);
    
    // Set new power
    set_power(PowerLevel_left, PowerLevel_right);
    
    // Add a small delay for a gradual shutdown
    delay(delay_ms);

  }

}








/*********************************
 *    DATA COMMUNICATION (IN)    *
 *********************************/



void receive_data(int n_messages) {
  // Receive multiple messages one at a time and set global message variables

  for (int i = 0; i < n_messages; i++) { // do this for each message

    // Checks if the Bluetooth module (kayakinator) is sending data
    if (kayakinator.available() > 0) {

      String receivedString = kayakinator.readStringUntil('>');
      delayMicroseconds(400); // CAREFUL!!!!! Stable above 400 microseconds or more (may be able to go a little less, but not under 100 aprox), less than that generates too many bad/incomplete messages
      newData = true;
    
      // Allocate messages
      if  (receivedString.length() == 18 && receivedString.charAt(0) == '1') {
        message1 = receivedString;
      } 

      else if  (receivedString.length() == 19 && receivedString.charAt(0) == '2') {
        message2 = receivedString;
      }

      // Resets the not available timer to 0, since the status is currently available
      killswitch_start = millis();

    }


    // If the Bluetooth device is not available
    else {

      // Time elapsed in milliseconds since the last loop when the Bluetooth device was available
      killswitch_timer = millis() - killswitch_start;

      // If we have been disconnected long enough, turn off the engines
      if (killswitch_timer > killswitch_time_ms) {
        gradual_shutdown(); // turn off engines gradually
      }

    }


    // Reset the newData variable and empty the serial input buffer
    if (newData == true) {
      newData = false;
      kayakinator.read(); // ensure the Serial input buffer is empty
    }

  }

}



void assign_data(String message1, String message2) {
  // Combine both messages, extract values and assign to global variables

  // Check if the strings are in sync
  if  ( message1.charAt(1) == message2.charAt(1) ) {

    // Unpack messages
    assign_variables(message1, message2);

  }

  else {
    //Serial.println("Bad message");
  }

}



void print_data() {
  // Function to visualise incoming data

  Serial.print("onoff ");
  Serial.print(onoff);
  Serial.print(", single_twin=");
  Serial.print(single_twin);
  Serial.print(", target_power=");
  Serial.print(target_power);
  Serial.print(", autopilot_onoff=");
  Serial.print(autopilot_onoff);
  Serial.print(", autopilot_mode=");
  Serial.print(autopilot_mode);
  Serial.print(", heading=");
  Serial.print(heading);
  Serial.print(", angular_speed=");
  Serial.print(angular_speed, 3);
  Serial.print(", target_heading=");
  Serial.println(target_heading);

}
  


void assign_variables(String message1, String message2) {
  // Read messages and assign variables

  int commaIndex1[5];
  split_by_commas(message1, commaIndex1, 5);
  int commaIndex2[3];
  split_by_commas(message2, commaIndex2, 3);

  // Assign message 1

  String onoff_string = message1.substring(commaIndex1[0] + 1, commaIndex1[1]);
  String single_twin_string = message1.substring(commaIndex1[1] + 1, commaIndex1[2]);
  String target_power_string = message1.substring(commaIndex1[2] + 1, commaIndex1[3]);
  String autopilot_mode_string = message1.substring(commaIndex1[3] + 1, commaIndex1[4]);
  String target_heading_string = message1.substring(commaIndex1[4] + 1, message1.length());

  onoff = onoff_string.charAt(0);
  single_twin = single_twin_string.charAt(0);
  target_power = target_power_string.toFloat();
  autopilot_mode = autopilot_mode_string.toInt();
  target_heading = target_heading_string.toFloat();


  // Assign message 2

  String autopilot_onoff_string = message2.substring(commaIndex2[0] + 1, commaIndex2[1]);
  String angular_speed_string = message2.substring(commaIndex2[1] + 1, commaIndex2[2]);
  String heading_string = message2.substring(commaIndex2[2] + 1, message2.length());

  autopilot_onoff = autopilot_onoff_string.toInt();
  angular_speed = angular_speed_string.toFloat();
  heading = heading_string.toFloat();

}



void split_by_commas(String inputString, int commaIndex[], int max_places) {
  // Split messages by commas and store the positions in commaIndex array.
  int j = 0;
  for (int i = 0; i < inputString.length(); i++) {
    if (inputString.charAt(i) == ',' && j < max_places) {
      commaIndex[j++] = i;
    }
  }
}






/**********************************
 *    DATA COMMUNICATION (OUT)    *
 **********************************/


float calculateBuffer() {

  // Get time interval of loop
  energy_interval = (micros() - energy_start_time)/1000;

  // Restart timing loop
  energy_start_time = micros();

  // Format the float with leading zeros and 1 decimal place and store it in the buffer
  char buffer[10];  // Adjust the buffer size as needed
  dtostrf(energy_interval, 6, 1, buffer);  // Parameters: value, width, precision, buffer
  for (int i = 0; i < strlen(buffer); i++) { // Add leading zeros
    if (buffer[i] == ' ') {
      buffer[i] = '0';
    }
  }

}

void send_data(int left_pwm, int right_pwm, float buffer) {

  // Calculate buffer
  calculateBuffer();

  // Join values in a single string
  String outgoing_message = String(left_pwm) + "," + String(right_pwm) + "," + String(buffer);

  // Send outgoing message
  kayakinator.print(outgoing_message);

}
