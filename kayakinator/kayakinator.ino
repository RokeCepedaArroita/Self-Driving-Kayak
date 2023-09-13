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

// Settings
int number_incoming_messages = 3;
int length_message1 = 18;
int length_message2 = 19;
int length_message3 = 6;

// Incoming variables
String message1;
String message2;
String message3;
char onoff, single_twin;
int autopilot_onoff, autopilot_mode;
float target_power, heading, angular_speed, target_heading, kalman_loop_time;

// Timers and internal variables
const int neutral_pwm = 1500;           // stores the neutral PWM value in microseconds
float killswitch_start = millis();      // KILL TIMER: initiates a timer in milliseconds
float killswitch_timer;                 // KILL TIMER: keep track of how long communications have been down
const float killswitch_time_ms = 5000;  // KILL TIMER: defines the kill time of the timer in milliseconds, can be edited by the user
float message_interval;                 // MESSAGE TIMER: keeps track of the time between consecutive incoming essages, used for the Kalman filter
float message_start_time;               // MESSAGE TIMER: start time of the message interval timer

// Autopilot and Kalman filter variables
float PowerLevel_right;
float PowerLevel_left;


// Outgoing variables
float energy_interval;                  // ENERGY TIMER: keeps track of cycles between motor changes to keep track of energy usage in the app
float energy_start_time;                // ENERGY TIMER: start time of the energy interval timer


// TODO: Remove these below
// Internals to test while programming
int bad_messages = 0;
int good_messages = 0;


void setup() {

  // Serial setup. Initializes the serial to communicate at a specific baud rate (115200 recommended, 9600 is too slow - interferes with program)
  Serial.begin(115200);

  // For testing purposes
  Serial.println("");
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
  receive_data(number_incoming_messages); // set the argument to the number of messages being received

  Serial.println(message1);
  //Serial.print(message2);
  //Serial.println(message3);

  // Process the data and command the motors
  process_message();

  //Serial.println(target_power);


  
  
  //print_data();
  // Send out diagnostic data to the app
  //send_data();



  // Serial.println(message1);
  // Serial.println(message2);
  // Serial.println("");

  // Print gyro, compass and message time interval values for noise characterization
  //sensor_test();


  //print_data();

}







/*******************************
 *          AUTOPILOT          *
 *******************************/


void autopilot(float target_power, float target_heading, int autopilot_mode=1) {

}






/*****************************
 *       KALMAN FILTER       *
 *****************************/



void kalman_filter(float previous_heading, float previous_angular_speed, float new_heading, float dt) {

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



void gradual_shutdown_onestep(float step_change=10) {

  // Check if either power level is greater than zero
  if (PowerLevel_left > 0 || PowerLevel_right > 0) {

    // Lower current power level
    PowerLevel_left  = constrain(PowerLevel_left  - step_change, 0, 100);
    PowerLevel_right = constrain(PowerLevel_right - step_change, 0, 100);
    
    // Set new power
    set_power(PowerLevel_left, PowerLevel_right);

  }

}




/***********************************
 *         DATA PROCESSING         *
 ***********************************/


void process_message() {



  if (onoff == 'N') {

    // Single engine mode: only give power to the left engine on pin 6
    if (single_twin == 'S') {
      set_power(target_power, 0);
    }

    // Twin engine mode: give power to both engines, either with or without autopilot
    else if (single_twin == 'T') {


      if (autopilot_onoff == 0) { 
        // Give equal power to both engines
        set_power(target_power, target_power);

      }

      // AUTOPILOT ENGAGED
      else if (autopilot_onoff == 1) { 

        set_power(target_power, target_power);

        //kalman_filter(heading, angular_speed, message_interval);
        //autopilot();
        //set_power(X,Y)

      }

    }
  }


  else if (onoff == 'F') { // if off, shut down engines
    gradual_shutdown_onestep();

  }

}







/*********************************
 *    DATA COMMUNICATION (IN)    *
 *********************************/




void wait_for_bluetooth_message(){
  // Wait until the Bluetooth module (kayakinator) is sending data

  while (kayakinator.available() == 0) {

    // Advance killswitch timer: time elapsed in milliseconds since the last loop when the Bluetooth device was available
    killswitch_timer = millis() - killswitch_start;

    // If we have been disconnected long enough, turn off the engines
    if (killswitch_timer > killswitch_time_ms) {
      gradual_shutdown(); // turn off engines gradually
    }

    // Small delay
    delayMicroseconds(200);

  }

}


String read_and_flush(char ending_character) {

  String receivedString;

  wait_for_bluetooth_message();

  if (kayakinator.available() > 0) {

    receivedString = kayakinator.readStringUntil(ending_character);

    //Serial.println(receivedString);

    // Add small delay to allow the Bluetooth module to catch up
    // CAREFUL!!!!! Stable above 400 microseconds or more (may be able to go a little less, 
    // but not under ~100 as too many bad/incomplete messages will be generated
    delayMicroseconds(400);

  }

  return receivedString;

}


void synchronize_message() {

  boolean ready_to_read = false; // incoming data availability flag

  while (ready_to_read != true) { // while the first message is not seen
    
    // Wait until a message arrives
    wait_for_bluetooth_message();

    // Peek to see if the message starts with a 1
    if (kayakinator.available() > 0) {

      if (kayakinator.peek() == '!') {
        ready_to_read = true;
      }

      else if (kayakinator.peek() == '?') {

        // Erase the message and wait again until #1 appears
        read_and_flush('>');
        read_and_flush('>');

        // Repeat until ready_to_read = true
        ready_to_read = false;
      }


      else if (kayakinator.peek() == '%') {

        // Erase the message and wait again until #1 appears
        read_and_flush('>');

        // Repeat until ready_to_read = true
        ready_to_read = false;
      }


      else {
        // Erase the message and wait again until #1 appears
        read_and_flush('>');

        // Repeat until ready_to_read = true
        ready_to_read = false;

      }

    }
  }
}



void receive_data(int n_messages) {
  // Receive multiple messages one at a time and set global message variables


  // Wait until the first message appears
  synchronize_message();
  

  // Read both messages
  for (int i = 0; i < n_messages; i++) {

    // Receive message one by one
    String receivedString = read_and_flush('>');
  
    // Allocate messages and make sure they are in sync (e.g. each first message has its corresponding second message)
    if (i == 0) { // (receivedString.charAt(0) == '1') {
      message1 = receivedString;
    } 
    

    else if (i == 1) { //  (receivedString.charAt(0) == '2') {
      message2 = receivedString;
    }

    
    else if (i == 2) { //  (receivedString.charAt(0) == '3') {
      message3 = receivedString;
    }

    // Resets the not available timer to 0, since we have just received a complete message
    killswitch_start = millis();

  }

  // Assign the data to variables
  if (check_messages(message1, message2, message3) == true) {
    
    // Unpack messages
    assign_variables(message1, message2, message3);
      
    // Get time interval of loop
    message_interval = (micros() - message_start_time)/1000;

    // Restart timing loop
    message_start_time = micros();

    // REMOVE BELOW
    // Serial.println(message_interval);
    good_messages++;

    

  }

  else {
    // REMOVE BELOW
    bad_messages++;
    // Serial.println("");
    // Serial.println("!! BAD MESSAGES BELOW:");
    // Serial.println(message1);
    // Serial.println(message2);
    // Serial.println(message3);
    // Serial.println("");
  }


}

    




    






boolean check_messages(String message1, String message2, String message3) {
  // Check if the strings are in sync
  
  boolean success = false;

  // Change the expected lengths of each message as necessary
if (message1.length() == length_message1 && 
    message2.length() == length_message2 && 
    message3.length() == length_message3 && 
    message1.charAt(1) == message2.charAt(1) && 
    message2.charAt(1) == message3.charAt(1)) {
  
  success = true;

  }

  return success;
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
  Serial.print(", kalman_loop_time=");
  Serial.println(kalman_loop_time);

}


void sensor_test() {
  // Print sensor data for testing
  //Serial.print(heading);
  //Serial.print(",");
  //Serial.print(angular_speed, 3);
  //Serial.print(",");
  Serial.print(PowerLevel_left);
  Serial.print(" ");
  Serial.print(PowerLevel_right);
  Serial.print(" ");
  Serial.print(target_power);
  //Serial.print(message_interval, 1);
  //Serial.print(" ");
  //Serial.println(kalman_loop_time, 2);
  //Serial.print(",");
  //Serial.print(good_messages);
  //Serial.print(",");
  //Serial.print(bad_messages);
  //Serial.print(",");
  //Serial.print(target_power);
  Serial.println(" ");
}


void assign_variables(String message1, String message2, String message3) {
  // Read messages and assign variables

  // Replace these numbers by the number of commas in the message
  int number_of_commas_message1 = 5;
  int number_of_commas_message2 = 3;
  int number_of_commas_message3 = 1;

  // Split each message by commas
  int commaIndex1[number_of_commas_message1]; 
  split_by_commas(message1, commaIndex1, number_of_commas_message1);
  int commaIndex2[number_of_commas_message2];
  split_by_commas(message2, commaIndex2, number_of_commas_message2);
  int commaIndex3[number_of_commas_message3];
  split_by_commas(message3, commaIndex3, number_of_commas_message3);


  // Assign message 1

  String onoff_string = message1.substring(commaIndex1[0] + 1, commaIndex1[1]);
  String single_twin_string = message1.substring(commaIndex1[1] + 1, commaIndex1[2]);
  String target_power_string = message1.substring(commaIndex1[2] + 1, commaIndex1[3]);
  String autopilot_mode_string = message1.substring(commaIndex1[3] + 1, commaIndex1[4]);
  String target_heading_string = message1.substring(commaIndex1[4] + 1, message1.length());

  onoff = onoff_string[0];
  single_twin = single_twin_string[0];
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


  // Assign message 3

  String kalman_loop_time_string = message3.substring(commaIndex3[0] + 1, message3.length());
  kalman_loop_time = kalman_loop_time_string.toFloat();


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


void update_energy_interval(char* formatted_energy_interval, int buffer_size) {

  // Get time interval of loop
  energy_interval = (micros() - energy_start_time) / 1000;

  // Restart timing loop
  energy_start_time = micros();

  // Format the float with leading zeros and 1 decimal place and store it in the buffer
  dtostrf(energy_interval, 6, 1, formatted_energy_interval); // Parameters: value, width, precision, buffer
  for (int i = 0; i < strlen(formatted_energy_interval); i++) { // Add leading zeros
    if (formatted_energy_interval[i] == ' ') {
      formatted_energy_interval[i] = '0';
    }
  }

}



void wait_until_clear_to_send(){
  // Wait until the Bluetooth module (kayakinator) is sending data


}




void send_data() {
  
  // Calculate energy time interval and format properly
  char formatted_energy_interval[10]; // Adjust the buffer size as needed
  update_energy_interval(formatted_energy_interval, sizeof(formatted_energy_interval));

  // Join values in a single string
  String outgoing_message = String(PowerLevel_left) + "," + String(PowerLevel_right) + "," + String(formatted_energy_interval);

  // Serial.println(outgoing_message);

  // Send outgoing message
  kayakinator.print(outgoing_message);
}
