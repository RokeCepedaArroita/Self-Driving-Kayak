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
String receivedString;
String message1;
String message2;
char onoff, single_twin;
int target_power, autopilot_onoff, autopilot_mode;
float heading, angular_speed, target_heading;

// Autopilot and Kalman filter variables

// Outgoing variables



void setup() {

  // Serial setup. Initializes the serial to communicate at a specific baud rate (9600)
  Serial.begin(9600);
  
  // Bluetooth HM-10 setup. Initializes the port to communicate at a specific baud rate (9600)
  kayakinator.begin(9600);

  // Initialize incoming data variable
  boolean newData = false;

}





/***********************************
 *            MAIN LOOP            *
 ***********************************/


void loop() {

  receive_data();
  receive_data(); // This function is duplicated to receive both incoming strings, one after the other
  assign_data(message1, message2);
  print_data();

}





/********************************
 *     DATA COMMUNICATION       *
 ********************************/


void receive_data() {
  // Receive multiple messages one at a time and set global message variables

  if (kayakinator.available() > 0) {

    receivedString = kayakinator.readStringUntil('>');
    delayMicroseconds(400); // CAREFUL!!!!! Stable above 400 microseconds or more (may be able to go a little less, but not under 100 aprox), less than that generates too many bad/incomplete messages
    newData = true;
  
    // Allocate messages
    if  (receivedString.length() == 18 && receivedString.charAt(0) == '1') {
      message1 = receivedString;
    } 

    else if  (receivedString.length() == 19 && receivedString.charAt(0) == '2') {
      message2 = receivedString;
    }

  }

  if (newData == true) {
    newData = false;
    kayakinator.read(); // Ensure the Serial input buffer is empty
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
  target_power = target_power_string.toInt();
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
