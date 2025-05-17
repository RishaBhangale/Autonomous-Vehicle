 #include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;
std_msgs::Float32 float_msg;
//std_msgs::Float32 heart_msg;

float offsetFinal = 15000.0;
float right_dir = 0.0;
float left_dir = 0.0;
float left_rev_dir = 0.0;
float right_rev_dir = 0.0;
float k = 0.7;
float lft_pwm = 0.0;
float rht_pwm = 0.0;
float pwm = 255;
float yaw_offset_curr = 0.0;
float yaw_offset_des = 0.0;

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

ros::Subscriber<std_msgs::Float32> rht("/robot_right_topic", &rightCallback);
ros::Subscriber<std_msgs::Float32> lft("/robot_left_topic", &leftCallback);
ros::Subscriber<std_msgs::Float32> rht_rev("/robot_right_rev_topic", &rightrevCallback);
ros::Subscriber<std_msgs::Float32> lft_rev("/robot_left_rev_topic", &leftrevCallback);

const int MOTOR_1_PWM = 9;               // To set the speed of the motor (max=255)
const int MOTOR_1_SLEEP_PIN = 8;        // Driver enable pin for motor 1 SLEEP PIN
const int MOTOR_1_DIRECTION_PIN = 10;     // To Turn on THe particular motor  HIGH for one direction and Low for other

const int MOTOR_2_PWM = 11;              // To set the Speed of the motor (max=255)
const int MOTOR_2_SLEEP_PIN = 12;       // To enable Driver for motor 2 SLEEP PIN
const int MOTOR_2_DIRECTION_PIN = 13;    // To Turn ON the particular motor  HIGH for one direction LOW for other


void setup()
{
  Serial.begin(57600);

  pinMode(MOTOR_1_SLEEP_PIN, OUTPUT);
  pinMode(MOTOR_1_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_1_PWM, OUTPUT);
  pinMode(MOTOR_2_SLEEP_PIN, OUTPUT);
  pinMode(MOTOR_2_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_2_PWM, OUTPUT);

  // TO INITIALIZE THE MOTOR DRIVERS SO THAT IT CAN READ THE VALUES
  digitalWrite(MOTOR_1_SLEEP_PIN, HIGH);
  digitalWrite(MOTOR_2_SLEEP_PIN, HIGH);

  nh.initNode();
//  nh.advertise(float_pub);
  nh.subscribe(lft);
  nh.subscribe(rht);
  nh.subscribe(lft_rev);
  nh.subscribe(rht_rev); 
}

void directions()
{
  digitalWrite(MOTOR_1_DIRECTION_PIN, HIGH);
  digitalWrite(MOTOR_2_DIRECTION_PIN, HIGH);
  analogWrite(MOTOR_1_PWM, rht_pwm);
  analogWrite(MOTOR_2_PWM, lft_pwm);
}
//void heart()
//{
//  float_pub.publish(&heart_msg);
//  delay(5000);                        
//}


void loop()
{  
  
  nh.spinOnce();
  delay(1);

    int motorLeft = map(right_dir, 1, -1, 0, pwm);
    int motorRight = map(left_dir, 1, -1, 0, pwm);
    int motorRight_rev = map(left_rev_dir, 0, 1, 0, 255);
    int motorLeft_rev = map(right_rev_dir, 0, 1, 0, 255);
    if (motorLeft > 20 || motorRight > 20) {  
      digitalWrite(MOTOR_1_DIRECTION_PIN, HIGH);
      digitalWrite(MOTOR_2_DIRECTION_PIN, HIGH);
      analogWrite(MOTOR_1_PWM,motorRight );
      analogWrite(MOTOR_2_PWM,motorLeft );
    }
    else if(left_rev_dir == 1 || right_rev_dir == 1){
      digitalWrite(MOTOR_1_DIRECTION_PIN, LOW); 
      digitalWrite(MOTOR_2_DIRECTION_PIN, LOW);
      analogWrite(MOTOR_1_PWM,motorRight_rev );
      analogWrite(MOTOR_2_PWM, motorLeft_rev);
    }    else{
      digitalWrite(MOTOR_1_DIRECTION_PIN, HIGH);
      digitalWrite(MOTOR_2_DIRECTION_PIN, HIGH);
      analogWrite(MOTOR_1_PWM, 0);
      analogWrite(MOTOR_2_PWM, 0);
    }  
      
  }
