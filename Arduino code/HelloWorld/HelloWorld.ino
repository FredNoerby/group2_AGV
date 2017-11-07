/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <geometry_msgs/TransformStamped.h>

ros::NodeHandle nh;

//geometry_msgs::TransformStamped Cordi_New;
ros::Publisher chatter("chatter", &Cordi_New);

void messageCb( const geometry_msgs::TransformStamped& Cordi_New){
  digitalWrite(13, HIGH-digitalRead(13));   
//  Cordi_New.transform.translation.x = 1;
//  Cordi_New.transform.translation.y = 1;
//  Cordi_New.transform.translation.z = 1;
}

ros::Subscriber<geometry_msgs::TransformStamped> sub("/Move_Cordinats", &messageCb );



void setup()
{
  nh.initNode();
  pinMode(13, OUTPUT);
  nh.subscribe(sub);
  //nh.advertise(chatter);  

}


void loop()
{
  //chatter.publish( &Cordi_New );
  nh.spinOnce();
  delay(10000);
}
