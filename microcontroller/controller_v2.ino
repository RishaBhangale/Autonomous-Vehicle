#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;
std_msgs::Float32 float_msg;
float offsetFinal = 15000.0;
float right_dir = 2.0;
float left_dir = 0.0;
float left_rev_dir = 0.0;
float right_rev_dir = 0.0;
float k = 0.9;
float k2 = 0.89;
float lft_pwm = 0.0;
float rht_pwm = 0.0;
float pwm =79;
float gp_pwm =100;
float yaw_offset_curr = 0.0;
float yaw_offset_des = 420.0;
float lin_offset = 0.0;
float gpsstop = 0.0;

void avgCallback(const std_msgs::Float32& msg)
{
  offsetFinal = msg.data;
}

void rightCallback(const std_msgs::Float32& msg)
{
  right_dir = msg.data;
}

void leftCallback(const std_msgs::Float32& msg)
{
  left_dir = msg.data;
}

void rightrevCallback(const std_msgs::Float32& msg)
{
  right_rev_dir = msg.data;
}

void leftrevCallback(const std_msgs::Float32& msg)
{
  left_rev_dir = msg.data;
} 

void linCallback(const std_msgs::Float32& msg)
{
  lin_offset = msg.data;
} 

void yawcurrCallback(const std_msgs::Float32& msg)
{
  yaw_offset_curr = msg.data;
} 

void yawdesCallback(const std_msgs::Float32& msg)
{
  yaw_offset_des = msg.data;
}

void stopCallback(const std_msgs::Float32& msg)
{
  gpsstop = msg.data;
} 

ros::Subscriber<std_msgs::Float32> off("/float_data", &avgCallback);
ros::Subscriber<std_msgs::Float32> rht("/robot_right_topic", &rightCallback);
ros::Subscriber<std_msgs::Float32> lft("/robot_left_topic", &leftCallback);
ros::Subscriber<std_msgs::Float32> rht_rev("/robot_right_rev_topic", &rightrevCallback);
ros::Subscriber<std_msgs::Float32> lft_rev("/robot_left_rev_topic", &leftrevCallback);
ros::Subscriber<std_msgs::Float32> yaw_off_curr("/yaw_off_curr", &yawcurrCallback);
ros::Subscriber<std_msgs::Float32> yaw_off_des("/yaw_off_des", &yawdesCallback);
ros::Subscriber<std_msgs::Float32> gps_stop("/gps_stop", &stopCallback);

const int MOTOR_1_PWM = 3;               // To set the speed of the motor (max=255)
const int MOTOR_1_SLEEP_PIN = 4;        // Driver enable pin for motor 1 SLEEP PIN
const int MOTOR_1_DIRECTION_PIN = 2;     // To Turn on THe particular motor  HIGH for one direction and Low for other

const int MOTOR_2_PWM = 5;              // To set the Speed of the motor (max=255)
const int MOTOR_2_SLEEP_PIN = 7;       // To enable Driver for motor 2 SLEEP PIN
const int MOTOR_2_DIRECTION_PIN = 6;  // To Turn ON the particular motor  HIGH for one direction LOW for other

void setup()
{

  pinMode(MOTOR_1_SLEEP_PIN, OUTPUT);
  pinMode(MOTOR_1_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_1_PWM, OUTPUT);
  pinMode(MOTOR_2_SLEEP_PIN, OUTPUT);
  pinMode(MOTOR_2_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_2_PWM, OUTPUT);

  pinMode(12, OUTPUT);

  // TO INITIALIZE THE MOTOR DRIVERS SO THAT IT CAN READ THE VALUES
  digitalWrite(MOTOR_1_SLEEP_PIN, HIGH);
  digitalWrite(MOTOR_2_SLEEP_PIN, HIGH);

  nh.initNode();
  nh.subscribe(off);  
  nh.subscribe(lft);
  nh.subscribe(rht);
  nh.subscribe(lft_rev);
  nh.subscribe(rht_rev);
  nh.subscribe(yaw_off_curr);
  nh.subscribe(yaw_off_des);
  nh.subscribe(gps_stop);
}

void directions()
{
  delay(100);
  digitalWrite(MOTOR_1_DIRECTION_PIN, HIGH);
  digitalWrite(MOTOR_2_DIRECTION_PIN, HIGH);
  analogWrite(MOTOR_1_PWM, rht_pwm);
  analogWrite(MOTOR_2_PWM, lft_pwm);
}

void go(){
//  delay(420);
  digitalWrite(MOTOR_1_DIRECTION_PIN, HIGH);
  digitalWrite(MOTOR_2_DIRECTION_PIN, HIGH);
  analogWrite(MOTOR_1_PWM, 60);
  analogWrite(MOTOR_2_PWM, 60);
}

void stops(){
  delay(100);
  digitalWrite(MOTOR_1_DIRECTION_PIN, HIGH);
  digitalWrite(MOTOR_2_DIRECTION_PIN, HIGH);
  analogWrite(MOTOR_1_PWM, rht_pwm);
  analogWrite(MOTOR_2_PWM, lft_pwm);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
  float diff = yaw_offset_des - yaw_offset_curr;
  float tolerance = 2.0;
  float max_offset = 50;
   if(yaw_offset_des != 420.0){
    offsetFinal = 15000;
       if(diff > 180){
      diff = diff - 360;
      }
      else if (diff < -180){
        diff = diff + 360;
      }
      if(abs(diff) <= tolerance){
        go();
      }
      if(diff > max_offset){
        diff = max_offset;
      } else if (diff < -max_offset){
          diff = -max_offset;
       }
       lft_pwm = gp_pwm - (k * diff);
       rht_pwm = gp_pwm + (k * diff);
       digitalWrite(MOTOR_1_DIRECTION_PIN, HIGH);
       digitalWrite(MOTOR_2_DIRECTION_PIN, HIGH);
       analogWrite(MOTOR_1_PWM, lft_pwm);
       analogWrite(MOTOR_2_PWM, rht_pwm);
    }
   else if(gpsstop == 10000.0001){
    yaw_offset_des = 420.0;
      lft_pwm = pwm + (k2 * offsetFinal);
      rht_pwm = pwm - (k2 * offsetFinal);
      stops();
    }
   else if(offsetFinal != 15000.0){
    lft_pwm = pwm + (k2 * offsetFinal);
    rht_pwm = pwm - (k2 * offsetFinal);
    directions();   
    }
    else{
      lft_pwm = pwm + (k2 * offsetFinal);
      rht_pwm = pwm - (k2 * offsetFinal);
      directions();  
    }
}
