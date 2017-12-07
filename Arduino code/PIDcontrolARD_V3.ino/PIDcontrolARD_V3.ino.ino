#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

// Autonomous Systems Msc Engineering, Autmn 2017, AAU, DK 
// Inspiration for code found in: https://gist.github.com/ShawnHymel/1de08ffaca990b65fade81cb8d01a44a

// -- The following is an implementation of a controller in a differential drive robot operating under a ROS system --
//--------------------------------------------------------------------------------------------------------------------
// Parameters for setting up robot values and speed calculations
const int wheel_dia = 40;                                 // Wheel diameter in mm
const int pulse_per_rev = 900;                            // Number of encoder pulses per revolution of the wheel-axle. (EncoderA pulses (6) * Gearing (150:1)) => 6*150=900
const float wheel_arc = (PI * wheel_dia)/ pulse_per_rev;  //covered distance in mm per encoder pulse

const int enc_L_pin = 2;   // Motor A / LEFT
const int enc_R_pin = 18;  // Motor B / RIGHT
const int pwma_pin = 11;   // PWM value sent to motor A   
const int ain1_pin = 32;   // Logic input for rotational direction of motor A / LEFT
const int ain2_pin = 30;   // Logic input for rotational direction of motor A / LEFT
const int pwmb_pin = 12;   // PWM value sent to motor A
const int bin1_pin = 38;   // Logic input for rotational direction of motor B / RIGHT
const int bin2_pin = 36;   // Logic input for rotational direction of motor B / RIGHT
const int stby_pin = 34;   // Pulled high = takes motor controller out of standby and allows operation

// Values for counting encoder readings. They are volatile because they are used in ISR.
volatile unsigned long enc_L = 0;
volatile unsigned long enc_R = 0;

//-----------------------------------------------------------------------------------------------------------

// Configure PID: Below are the setups and parameters of Left and Right PIDs
// Defines variables to be called in the PIDs
double enc_speed_L, u_speed_L, ROS_speed_L, enc_speed_R, u_speed_R, ROS_speed_R;
  
// Sets controller values
double Kp_L=3, Ki_L=200, Kd_L=0, Kp_R=3, Ki_R=200, Kd_R=0; // PID controller values
double enc_sampletime = 30; // Time that delay uses to sample encoder in milliseconds
double Filterconstant = 0.95; // Meaning that 95 % of the old value will be used and only 5 % of the new 
double enc_speed_L_RAW; // Used for storing the speed before filtering
double enc_speed_R_RAW; // Used for storing the speed before filtering
  

// Specifies controller setup
PID LeftPID(&enc_speed_L, &u_speed_L, &ROS_speed_L, Kp_L, Ki_L, Kd_L, DIRECT);
PID RightPID(&enc_speed_R, &u_speed_R, &ROS_speed_R, Kp_R, Ki_R, Kd_R, DIRECT);

// Logic setup:
double ROS_speed_L_in;
double ROS_speed_R_in;   
   
//------------------------------------------------------------------------------------
// Functions below counts the interrupts from the encoders
void countLeft() {
  enc_L++;
}
void countRight() {
  enc_R++;
}

//--------------------------------------------
// Code for a subcriber function
ros::NodeHandle  nh;

void messageCb( const std_msgs::Float32MultiArray& toggle_msg){
  ROS_speed_L_in = toggle_msg.data[0];
 // // Serial.print(ROS_speed_L_in);
  ROS_speed_R_in = toggle_msg.data[1];
 // // Serial.print(ROS_speed_R_in);

}

ros::Subscriber<std_msgs::Float32MultiArray> sub("motor_control", &messageCb );


//Setup initiates the Arduino before the loop is run
void setup() {
  // Setup of the baud rate
  //Serial.begin(57600);

 // Serial.println("Im in love with tha coco");
  // Setup of Pins and interrupts
  pinMode(enc_L_pin, INPUT_PULLUP);
  pinMode(enc_R_pin, INPUT_PULLUP); 
  pinMode(pwma_pin, OUTPUT);
  pinMode(ain1_pin, OUTPUT);
  pinMode(ain2_pin, OUTPUT);
  pinMode(pwmb_pin, OUTPUT);
  pinMode(bin1_pin, OUTPUT);
  pinMode(bin2_pin, OUTPUT);
  pinMode(stby_pin, OUTPUT);

  // Set up interrupts
  attachInterrupt(digitalPinToInterrupt(enc_L_pin), countLeft, CHANGE);   // Each time the encoder recognizes a change the program will interupt and call the method countLeft
  attachInterrupt(digitalPinToInterrupt(enc_R_pin), countRight, CHANGE);  // Each time the encoder recognizes a change the program will interupt and call the method countRight

  // Pulls the systems out of standby
  digitalWrite(stby_pin, HIGH);

  // Makes the left wheel go forward
  digitalWrite(ain1_pin, LOW);
  digitalWrite(ain2_pin, HIGH);
  // Makes the right wheel go forward
  digitalWrite(bin1_pin, LOW);
  digitalWrite(bin2_pin, HIGH);
  
 
  // PID CONTROL: activates PID control!  
  LeftPID.SetMode(AUTOMATIC);         // The mode options are: DIRECT (like a car) or REVERSE (like a refrigerator)
  LeftPID.SetOutputLimits(70,255);    // The lower limit of 70 ensures that the wheels will turn at low speeds
  LeftPID.SetSampleTime(1);           // The PID will be evaluated every millisecond
  enc_speed_L = 0;                    // Th    e speed is initialized to zero
  
  RightPID.SetMode(AUTOMATIC);        // The mode options are: DIRECT (like a car) or REVERSE (like a refrigerator)
  RightPID.SetOutputLimits(70,255);   // The lower limit of 70 ensures that the wheels will turn at low speeds
  RightPID.SetSampleTime(1);          // The PID will be evaluated every millisecond
  enc_speed_R = 0;                    // The speed is initialized to zero

  // ROS Subcriber 
  pinMode(13, OUTPUT);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  
  
}//VOID SETUP ENDS

void loop() {
  
  // ENCODER READ: the encoders for both motors are read and contained in a value as a enc_speed_L and enc_speed_R in [m/s].
  enc_L = 0; // First, reset the encoder increment value to zero
  enc_R = 0; // First, reset the encoder increment value to zero
  // Delays the program for an amount of miliseconds in which the encoders will still count changes
  delay(enc_sampletime);
    
  // Calculate the measured and unfiltered value as a speed in [mm/s]
  enc_speed_L_RAW = (((wheel_arc * enc_L) / enc_sampletime) * 1000);
  enc_speed_R_RAW = (((wheel_arc * enc_R) / enc_sampletime) * 1000);

  // Filtering of the speed using an RC filter
  enc_speed_L = Filterconstant * enc_speed_L + (1.0f - Filterconstant) * enc_speed_L_RAW;
  enc_speed_R = Filterconstant * enc_speed_R + (1.0f - Filterconstant) * enc_speed_R_RAW;

  // If both input speeds are zero the system will go to standby
  if(ROS_speed_L_in == 0 && ROS_speed_R_in == 0){
    digitalWrite(stby_pin, LOW);
  } else {
    digitalWrite(stby_pin, HIGH);
  }

  // PID Loop
  ROS_speed_L = abs(ROS_speed_L_in);  // gets the actual speed setpoint from ROS  
  LeftPID.Compute();                  // Calls a method in the PID library to compute the values
  
  ROS_speed_R = abs(ROS_speed_R_in);  // gets the actual speed setpoint from ROS
  RightPID.Compute();                 // Calls a method in the PID library to compute the values   

  // Sets the right motor to go forward, backward or stop
  if(ROS_speed_L_in > 0){
    digitalWrite(ain1_pin, LOW);
    digitalWrite(ain2_pin, HIGH);
    // Sends output to PWM pin on motor
    analogWrite(pwma_pin, u_speed_L);
  } else if (ROS_speed_L_in < 0) {
    digitalWrite(ain1_pin, HIGH);
    digitalWrite(ain2_pin, LOW);
    // Sends output to PWM pin on motor
    analogWrite(pwma_pin, u_speed_L);
  } else {
    // Sends output to PWM pin on motor that stops it
    analogWrite(pwma_pin, 0);         
  }

  // Sets the right motor to go forward, backward or stop 
  if(ROS_speed_R_in > 0){
    digitalWrite(bin1_pin, LOW);
    digitalWrite(bin2_pin, HIGH);
    // Sends output to PWM pin on motor
    analogWrite(pwmb_pin, u_speed_R);
  } else if (ROS_speed_R_in < 0) {
    digitalWrite(bin1_pin, HIGH);
    digitalWrite(bin2_pin, LOW);
    // Sends output to PWM pin on motor
    analogWrite(pwmb_pin, u_speed_R);
  } else {
    // Sends output to PWM pin on motor that stops it
    analogWrite(pwmb_pin, 0);
  }
  
  nh.spinOnce();
    
} // VOID LOOP END
