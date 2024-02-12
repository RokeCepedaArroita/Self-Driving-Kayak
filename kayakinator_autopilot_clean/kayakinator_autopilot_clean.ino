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
#include <AltSoftSerial.h>
AltSoftSerial kayakinator(8, 9);

// Matrix algebra
#include <BasicLinearAlgebra.h>
using namespace BLA;

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


// IMPORTANT AUTOPILOT PARAMS
float integral_activation_angle = 20; // deg, when we are within this error in deg from the setpoint, the integral starts counting
float alpha_fading_memory = 1.1; // memory fading parameter - higher values result in less lag and higher noise, 1.1 is about 1s lag, 1.01 is about 10, 1.3 is less than 0.5 sec but near gyro noise limit
float timestep = 0.122; // s, time equivalent to the control frequency
float smoothing_time_default = 0.1; // s, typical output smoothing time
float derivative_smoothing_time_default = 0.5; // s, typical derivative smoothing time
float engine_offset = 0;

float calm[]               = {4.0,  27,  0.40, 1.1, 0.45, 1.1, 0}; // P, D, I, sigma_compass, sigma_gyro, alpha_fading_memory, engine offset I CHANGED THIS!! IT'S EVEN MORE THAN CALM NO OVERSHOOT
float calm_nolag[]         = {4.0, 10.9, 0.98, 1.1, 0.45, 1.3, 0}; // P, D, I, sigma_compass, sigma_gyro, alpha_fading_memory, engine offset
float calm_noovershoot[]   = {4.0, 16.7, 0.64, 1.1, 0.45, 1.1, 0}; // P, D, I, sigma_compass, sigma_gyro, alpha_fading_memory, engine offset
float rough[]              = {2.0,  7.3, 0.36, 1.1, 0.45, 1.1, 0}; // P, D, I, sigma_compass, sigma_gyro, alpha_fading_memory, engine offset
float rough_nolag[]        = {2.0,  7.1, 0.38, 1.1, 0.45, 1.3, 0}; // P, D, I, sigma_compass, sigma_gyro, alpha_fading_memory, engine offset
float rough_noovershoot[]  = {2.0,  8.9, 0.30, 1.1, 0.45, 1.1, 0}; // P, D, I, sigma_compass, sigma_gyro, alpha_fading_memory, engine offset
float rough_engineoffset[] = {2.0,  7.3, 0.36, 1.1, 0.45, 1.1, 20}; // P, D, I, sigma_compass, sigma_gyro, alpha_fading_memory, engine offset

// Autopilot and Kalman filter variables
float PowerLevel_right;
float PowerLevel_left;
float kp; // proportional gain
float kd; // derivative gain
float ki; // integral gain
float sigma_compass = 1.1; // standard deviation of compass in deg due to noise (environment-dependent)
float sigma_gyro = 0.45; // standard deviation of gyroscope in deg/s due to noise (environment-dependent)
float residual_kalman = 0; // TODO: REMOVE THIS. DIFFERENCE BETWEEN PRIOR AND POSTERIOR
float a_priori_kalman = 0; // TODO: REMOVE THIS. DIFFERENCE BETWEEN PRIOR AND POSTERIOR
BLA::Matrix<2,2,float> Q;
BLA::Matrix<2,2,float> R;
BLA::Matrix<2,2,float> P;
BLA::Matrix<2,2,float> I;
BLA::Matrix<2,1,float> x;
BLA::Matrix<2,1,float> y;
BLA::Matrix<1,1,float> epsilon;
float integral = 0; // integral term
float derivative = 0; // previous derivative term
float left_engine_power = 0; // left power
float right_engine_power = 0; // right power
float previous_left_engine_power = 0; // previous left power
float previous_right_engine_power = 0; // previous right power
float filtered_heading, filtered_angular_velocity; // Filtered readings
float previous_target_heading;
int previous_autopilot_mode;
float smoothing_time = smoothing_time_default; // s, typical output smoothing time
float derivative_smoothing_time = derivative_smoothing_time_default; // s, typical derivative smoothing time
bool integral_is_counting = false;  // whether integral activated

// Outgoing variables
String outgoing_message;                // String to store outgoing message
float energy_interval;                  // ENERGY TIMER: keeps track of cycles between motor changes to keep track of energy usage in the app
float energy_start_time;                // ENERGY TIMER: start time of the energy interval timer

// TODO: Remove these below
// Internals to test while programming
int bad_messages = 0;
int good_messages = 0;
bool message_is_ok = false;

// TODO: REMOVE THIS
float total_time_ms = 0;

void setup() {

  // Serial setup. Initializes the serial to communicate at a specific baud rate (115200 recommended, 9600 is too slow - interferes with program)
  // Serial.begin(115200); //TODO: REMOVE BELOW

  // Bluetooth HM-10 setup. Initializes the port to communicate at a specific baud rate (9600)
  kayakinator.begin(38400);

  // Servo ESC setup: replace the numbers by the Arduino pin number (~) that is being used to connect to the right or left motor's ESC signal port
  esc_right.attach(5);
  esc_left.attach(6);

  // Arm both motors
  arm_motors();

  // Initialize Kalman parameters
  kalman_initialize();

}




/***********************************
 *            MAIN LOOP            *
 ***********************************/


void loop() {

  

  // Receive and assign the data
  receive_data(number_incoming_messages); // set the argument to the number of messages being received

  float process_timer_start = micros();

  // Process the data and command the motors
  process_message();

  // Send out diagnostic data to the app
  send_data();

}







/*******************************
 *          AUTOPILOT          *
 *******************************/



void PID_initialize() {

  // Initialize autopilot and reset integral
  timestep = 0.122; // s, time equivalent to the control frequency
  smoothing_time = smoothing_time_default; // s, typical output smoothing time
  derivative_smoothing_time = derivative_smoothing_time_default; // s, typical derivative smoothing time
  integral = 0; // integral term
  derivative = 0; // previous derivative term
  previous_left_engine_power = 0; // previous left power
  previous_right_engine_power = 0; // previous right power
  filtered_angular_velocity = 0; // reset angular velocity
  filtered_heading = 0; // reset heading
  integral_is_counting == false; // deactivate integral term
  integral_activation_angle = 20; // deg, when we are within this error in deg from the setpoint, the integral starts counting
  alpha_fading_memory = 1.1; // memory fading parameter - higher values result in less lag and higher noise, 1.1 is about 1s lag, 1.01 is about 10, 1.3 is less than 0.5 sec but near gyro noise limit
  engine_offset = 0; // give an engine systematically less power than the other as afeed-forward mechanism (multiplicative)

}


void PID_controller(float current_heading, float angular_velocity, float timestep, float max_power = 100) {

  // Update Kalman filter
  kalman_filter(current_heading, angular_velocity, timestep);

  // Extract the filtered values
  filtered_heading = x(0, 0);
  filtered_angular_velocity = x(1, 0);

  // Constrain filtered heading between 0 and 360
  // Ensure the angle is within the range of 0 to 360 degrees
  filtered_heading = fmod(filtered_heading, 360.0);

  // If the result of the modulo operation is negative, add 360 to make it positive
  if (filtered_heading < 0) {
    filtered_heading += 360;
  }


  // Measure the angle difference, positive means we have to turn right: use filtered readings in PID control loop
  float error = shortest_angle_difference(filtered_heading, target_heading);

  // Check if we are within the integral activation range, and if so, activate it
  if (integral_is_counting == false) {
    if (abs(error) <= integral_activation_angle) {
      integral_is_counting = true;
    }
  }
      
  // Integral calculations
  if (ki != 0 && integral_is_counting == true) {
      if(previous_left_engine_power != max_power && previous_right_engine_power != max_power) {
          integral += error * timestep;
          integral = max(min(integral, max_power/ki), -max_power/ki);
      } else {
          if(integral >= max_power/ki) {
              integral = 0; // reset it
          }
      }
  }

  // Smooth derivative term to reduce noise in the output
  if (derivative_smoothing_time > 0) {
      float alpha = 1 - exp(-timestep / derivative_smoothing_time);
      derivative = alpha * filtered_angular_velocity + (1 - alpha) * derivative;
  } else {
      derivative = filtered_angular_velocity;
  }

  left_engine_power = kp * error - kd * derivative + ki * integral;
  right_engine_power = -kp * error + kd * derivative - ki * integral;

  // if (kp * error > 0) {
  //   Serial.print("P pushing RIGHT, ");
  // }
  // else if (kp * error < 0) {
  //   Serial.print("P pushing LEFT, ");
  // }

  // if (kd * derivative > 0) {
  //   Serial.print("kd pushing LEFT");
  // }
  // else if (kd * derivative < 0) {
  //   Serial.println("kd pushing RIGHT");
  // }


  // Serial.println("");

  // Normalize output
  left_engine_power = max(min(left_engine_power, max_power), 0.0);
  right_engine_power = max(min(right_engine_power, max_power), 0.0);


  // If excess power available, use it
  if(left_engine_power + right_engine_power < target_power*2) {
      float power_deficit = target_power*2 - (left_engine_power + right_engine_power);
      float available_power = max_power - max(left_engine_power, right_engine_power);
      left_engine_power = left_engine_power + min(available_power, power_deficit / 2);
      right_engine_power = right_engine_power + min(available_power, power_deficit / 2);
  }

  // If there is an engine_offset and the motors are not saturated, apply it: more power to left (old), less power to righ (new)
  if (engine_offset != 0 && left_engine_power != max_power && right_engine_power != max_power) {
    left_engine_power  = (1+(engine_offset)/100) * left_engine_power;
    right_engine_power = (1-(engine_offset)/100) * right_engine_power;
  }

  // Smooth output with a low pass filter
  if (smoothing_time > 0) {
      float alpha = 1 - exp(-timestep / smoothing_time);
      left_engine_power = alpha * left_engine_power + (1 - alpha) * previous_left_engine_power;
      right_engine_power = alpha * right_engine_power + (1 - alpha) * previous_right_engine_power;
  }

  // REMOVE THIS: Normalize output again just in case
  left_engine_power = max(min(left_engine_power, max_power), 0.0);
  right_engine_power = max(min(right_engine_power, max_power), 0.0);

  previous_left_engine_power = left_engine_power;
  previous_right_engine_power = right_engine_power;

}


/*****************************
 *       KALMAN FILTER       *
 *****************************/



void kalman_filter(float new_heading, float new_angular_speed, float dt) {

  // Prediction phase
  kalman_predict(dt);

  // Update phase
  kalman_update(new_heading, new_angular_speed);

}



void kalman_initialize() {
  // Initialize matrices
  Q = {0.017 * 0.017 * 0.122, 0, 0, 0.017 * 0.017}; // TODO: fix this so it's not sluggish
  R = {sigma_compass * sigma_compass, 0, 0, sigma_gyro * sigma_gyro};
  P = {1000 * 1000, 0, 0, 1000 * 1000};
  I = {1, 0, 0, 1};
  x = {0, 0};
}


void kalman_predict(float dt) {
  // State transition matrix
  BLA::Matrix<2,2,float> F = {1, dt, 0, 1};

  // Predicted (a priori) state estimate
  x = F * x;

  // Predicted (a priori) estimate covariance
  P = F * P * ~F + Q;

  // Apply fading memory
  P(0,0) = (alpha_fading_memory*alpha_fading_memory) * P(0,0);
  P(0,1) = (alpha_fading_memory*alpha_fading_memory) * P(0,1); 
  P(1,0) = (alpha_fading_memory*alpha_fading_memory) * P(1,0); 
  P(1,1) = (alpha_fading_memory*alpha_fading_memory) * P(1,1); 

}


void kalman_update(float new_current_heading, float new_angular_velocity) {

  BLA::Matrix<2,1,float> z = {new_current_heading, new_angular_velocity};

  // Identidy matrix
  BLA::Matrix<2,2,float> H = {1, 0, 0, 1};

  // Innovation or measurement residual
  y = z - H * x;
  y(0,0) = shortest_angle_difference(x(0,0), z(0,0));  // Adjust the heading difference

  // System uncertainty (Innovation (or residual) covariance)
  Matrix<2,2,float> S = H * P * ~H + R;

  // Invert S
  Matrix<2,2,float> S_inverse = Inverse(S);

  // Calculate Epsilon: the Mahalanobis distance of the measurement residual
  epsilon =  ~y * S_inverse * y;

  // Optimal Kalman gain
  Matrix<2,2,float> K = P * ~H * S_inverse;

  // Updated (a posteriori) state estimate
  x = x + K * y;

  // Updated (a posteriori) estimate covariance
  P = (I - K * H) * P;
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



void gradual_shutdown(int delay_ms=100, float step_change=5) {

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



void set_autopilot_parameters(const float* parameters) {

  kp = parameters[0];
  kd = parameters[1];
  ki = parameters[2];
  sigma_compass = parameters[3];
  sigma_gyro = parameters[4];
  alpha_fading_memory = parameters[5];
  engine_offset = parameters[6];

}





void process_message() {



  if (onoff == 'N') {

    // Single engine mode: only give power to the left engine on pin 6
    if (single_twin == 'S') {
      set_power(target_power, 0);
      
      // Reset integral and kalman filter
      kalman_initialize();
      PID_initialize();
      
    }

    // Twin engine mode: give power to both engines, either with or without autopilot
    else if (single_twin == 'T') {


      if (autopilot_onoff == 0) {
        // Give equal power to both engines
        set_power(target_power, target_power);

        // Reset integral and kalman filter
        kalman_initialize();
        PID_initialize();

      }

      // AUTOPILOT ENGAGED
      else if (autopilot_onoff == 1) {

        // If target_heading or mode changed, reset integral and kalman filter
        if (target_heading != previous_target_heading || autopilot_mode != previous_autopilot_mode) {
          kalman_initialize();
          PID_initialize();
        }

        if        (autopilot_mode == 1) { // Calm
          set_autopilot_parameters(calm);
        } else if (autopilot_mode == 2) { // Calm no lag
          set_autopilot_parameters(calm_nolag);
        } else if (autopilot_mode == 3) { // Calm no overshoot
          set_autopilot_parameters(calm_noovershoot);
        } else if (autopilot_mode == 4) { // Rough
          set_autopilot_parameters(rough);
        } else if (autopilot_mode == 5) { // Rough no lag
          set_autopilot_parameters(rough_nolag);
        } else if (autopilot_mode == 6) { // Rough no overshoot
          set_autopilot_parameters(rough_noovershoot);
        } else if (autopilot_mode == 7) { // Rough engine offset
          set_autopilot_parameters(rough_engineoffset);
        }

        // Update the Kalman filter and choose a power level
        PID_controller(heading, angular_speed, kalman_loop_time/1000);

        // Set the appropiate power
        set_power(left_engine_power, right_engine_power);

        // Save previous variables
        previous_target_heading = target_heading;
        previous_autopilot_mode = autopilot_mode;

      }

    }
  }


  else if (onoff == 'F') { // if off, shut down engines
    gradual_shutdown_onestep();

    // Reset integral and kalman filter
    kalman_initialize();
    PID_initialize();

  }

}



/*******************************
 *        ANGULAR TOOLS        *
 *******************************/


float shortest_angle_difference(float current_heading, float desired_heading) {
  // Ensure both angles are within the range [0, 360)
  current_heading = fmod(current_heading, 360);
  desired_heading = fmod(desired_heading, 360);

  // Calculate the raw difference between the angles
  float raw_difference = desired_heading - current_heading;

  // Adjust the raw difference to the shortest angle (considering the wrap-around)
  float shortest_difference;
  if (abs(raw_difference) <= 180) {
    shortest_difference = raw_difference;
  } else {
    if (raw_difference > 180) {
      shortest_difference = raw_difference - 360;
    } else {
      shortest_difference = raw_difference + 360;
    }
  }
  return shortest_difference;
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

  float start_time_receive = micros();

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
    message_is_ok = true;


  }

  else {
    bad_messages++;
    message_is_ok = false;
  }


}





bool isAsciiString(const String &str) {
  for (int i = 0; i < str.length(); i++) {
    if (!isAscii(str.charAt(i))) {
      // If any character is not ASCII, return false
      return false;
    }
  }
  // All characters are ASCII
  return true;
}




boolean check_messages(String message1, String message2, String message3) {
  // Check if the strings are in sync

  boolean success = false;

  // Change the expected lengths of each message as necessary
  if (message1.length() == length_message1 &&      // Check message lengths
      message2.length() == length_message2 &&
      message3.length() == length_message3 &&
      message1.charAt(1) == message2.charAt(1) &&  // Check synchronicity between messages
      message2.charAt(1) == message3.charAt(1) &&
      isAsciiString(message1) &&  // Check for bad characters
      isAsciiString(message2) &&
      isAsciiString(message3)                      )  {

    // Additional loop time message check since the last item tends to get distorted
    String kalman_time_temp   = message3.substring(3, message3.length());
    String angular_speed_temp = message2.substring(5, 13);
    String heading_temp       = message2.substring(14, message2.length());
    if (kalman_time_temp.toFloat() > 40 && 
        kalman_time_temp.toFloat() < 500 && 
        angular_speed_temp.toFloat() != 0 && 
        heading_temp.toFloat() >= 0 && 
        heading_temp.toFloat() < 360 )  {
      success = true;
    }

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
  //Serial.print(PowerLevel_left);
  //Serial.print(" ");
  //Serial.print(PowerLevel_right);
  //Serial.print(" ");
  //Serial.print(target_power);
  //Serial.print(" ");
  Serial.print(message_interval, 1);
  Serial.print(" ");
  Serial.print(kalman_loop_time, 2);
  //Serial.print(good_messages);
  //Serial.print(",");
  //Serial.print(bad_messages);
  //Serial.print(",");
  //Serial.print(target_power);
  Serial.println("");
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
  angular_speed = -1*(angular_speed_string.toFloat()-0.020); // Remove bias and invert the sign of the angular speed so it matches heading and autopilot convention
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



void format_powerlevel(float powerlevel, char* formatted_power_level, int buffer_size) {

  // Format the float with leading zeros and 1 decimal place and store it in the buffer
  dtostrf(powerlevel, 5, 1, formatted_power_level); // Parameters: value, width, precision, buffer
  for (int i = 0; i < strlen(formatted_power_level); i++) { // Add leading zeros
    if (formatted_power_level[i] == ' ') {
      formatted_power_level[i] = '0';
    }
  }

}


void send_data() {

  // Calculate energy time interval and format properly
  char formatted_energy_interval[10]; // Adjust the buffer size as needed
  update_energy_interval(formatted_energy_interval, sizeof(formatted_energy_interval));

  // Format the power levels with 3 digits and one decimal, with left zeroes if necessary
  char formatted_power_level_left[10]; // Adjust the buffer size as needed
  format_powerlevel(PowerLevel_left, formatted_power_level_left, sizeof(formatted_energy_interval));
  char formatted_power_level_right[10]; // Adjust the buffer size as needed
  format_powerlevel(PowerLevel_right, formatted_power_level_right, sizeof(formatted_energy_interval));

  // Join values in a single string
  outgoing_message = "*#" + String(formatted_power_level_left) + "," + String(formatted_power_level_right) + "," + String(formatted_energy_interval);

  // Serial.println(outgoing_message);

  // Send outgoing message
  kayakinator.print(outgoing_message);
}
