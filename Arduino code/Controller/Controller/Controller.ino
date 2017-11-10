/*
  This is the main controller for the arduino


 */

// Start by importing liberies 
#include <ros.h>
#include <std_msgs/Int16.h>
int timer = 1000;
int tal = 2;

/* An array to go thought 

int timer = 100;                     // The higher the number, the slower the timing.
int ledPins[] = {2, 7, 4, 6, 5, 3};  // an array of numbers
*/


/* A subcriber for later
ros::NodeHandle nh;

void messageCb( const geometry_msgs::TransformStamped& Cordi_New){
  digitalWrite(13, HIGH-digitalRead(13));   
//  Cordi_New.transform.translation.x = 1;
//  Cordi_New.transform.translation.y = 1;
//  Cordi_New.transform.translation.z = 1;
}

ros::Subscriber<geometry_msgs::TransformStamped> sub("/Move_Cordinats", &messageCb );
*/

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:


  if      (tal == 1){
    Serial.println ("Doing 1");

    
  }
  else if (tal == 2){
    Serial.println ("Doing 2");
    delay (timer);

    
  }
  else if (tal == 3){
    Serial.println ("Doing 3");


    
  }
  else if (tal == 4){
    Serial.println ("Doing 4");


    
  }
  else {
    Serial.println ("Red six standing by");

    
  }



  
}
