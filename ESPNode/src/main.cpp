#include <Arduino.h>
#include "WiFi.h"
#include <ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <ESP32Hardware.h>
#include <Wire.h>
#include <math.h>

#define MOTOR1_PWM_PIN 13     // GPIO pin connected to MOTOR driver PWM input
#define MOTOR1_DIR1_PIN 14
#define MOTOR1_DIR2_PIN 27
#define MOTOR2_PWM_PIN 12     // GPIO pin connected to MOTOR driver PWM input
#define MOTOR2_DIR1_PIN 26
#define MOTOR2_DIR2_PIN 25
#define PWM_CHANNEL 0  // PWM channel
#define PWM_CHANNEL2 1  // PWM channel
#define PWM_FREQ 300000  // PWM frequency in Hz
#define PWM_RES 8      // PWM resolution (8 bits: values from 0 to 255)

#define BUZZER_PIN     32
#define IR_SENSOR_PIN  33
#define ENCODER1_PIN_A 34  // Encoder Channel A connected to GPIO 18
#define ENCODER2_PIN_A 35  // Encoder Channel A connected to GPIO 18
#define PULSES_PER_REV 1045 // Number of pulses per revolution from encoder
#define WHEEL_RADIUS 0.0315  // Radius of the wheel in meters
#define BASE_RADIUS 0.086

/* WiFi credentials and ROS master IP */
const char *ssid = "KM";
//const char *ssid = "Mahmoud";
const char *password = "My.Fuckin.Password1478963'.";
//const char *password = "12345678";
IPAddress server_ip(192,168,1,17);
const uint16_t server_port = 11411;    // Port of ROS master

char wall_Ahead = 0;
char person_Ahead = 0;

/* ros node handle */
ros::NodeHandle_<Esp32Hardware> nh; /* node handle instance for WiFi connection with ros */

geometry_msgs::Pose2D position;                 /* string data type */
ros::Publisher espPub("Encoder", &position); /* Publisher definition */

void callBack( const geometry_msgs::Twist &speed)
{ 
  if (wall_Ahead == 1) {}
  else if (person_Ahead == 1) 
  {
    ledcWrite(PWM_CHANNEL, 0);
    ledcWrite(PWM_CHANNEL2, 0);
    digitalWrite(BUZZER_PIN, HIGH);
  }
  else if (speed.linear.x > 0) {
    digitalWrite(MOTOR1_DIR1_PIN, LOW);
    digitalWrite(MOTOR1_DIR2_PIN, HIGH);
    digitalWrite(MOTOR2_DIR1_PIN, LOW);
    digitalWrite(MOTOR2_DIR2_PIN, HIGH);
    float leftspeed = ((speed.linear.x - speed.angular.z * BASE_RADIUS) / WHEEL_RADIUS)*(30/PI);
    float rightspeed = ((speed.linear.x + speed.angular.z * BASE_RADIUS) / WHEEL_RADIUS)*(30/PI);
    ledcWrite(PWM_CHANNEL, (leftspeed*100)/245);
    ledcWrite(PWM_CHANNEL2, (rightspeed*100)/245);
  } 
  else if (speed.linear.x < 0) {
    digitalWrite(MOTOR1_DIR1_PIN, HIGH);
    digitalWrite(MOTOR1_DIR2_PIN, LOW);
    digitalWrite(MOTOR2_DIR1_PIN, HIGH);
    digitalWrite(MOTOR2_DIR2_PIN, LOW);
    float leftspeed = (((speed.linear.x * -1) - speed.angular.z * BASE_RADIUS) / WHEEL_RADIUS)*(30/PI);
    float rightspeed = (((speed.linear.x * -1) + speed.angular.z * BASE_RADIUS) / WHEEL_RADIUS)*(30/PI);
    ledcWrite(PWM_CHANNEL, (leftspeed*110)/245);
    ledcWrite(PWM_CHANNEL2, (rightspeed*110)/245);
  }
  else {
    ledcWrite(PWM_CHANNEL, 0);
    ledcWrite(PWM_CHANNEL2, 0);
  }
}
ros::Subscriber<geometry_msgs::Twist>espSub("cmd_vel", &callBack);

void callBack2( const std_msgs::Float32MultiArray &trash)
{
  static unsigned short int c0 = 0, c1 = 0, c2 = 0, c3 = 0, c4 = 0, c5 = 0;
  
  if(trash.data[2] == 0)
    person_Ahead = 1;
  else
  {
    person_Ahead = 0;
    if(trash.data[0] > 80)
      c0++;
    else if(trash.data[0] > 70)
      c1++;
    else if(trash.data[0] > 60)
      c2++;
    else if(trash.data[0] > 50)
      c3++;
    else if(trash.data[0] > 40)
      c4++;
    else if(trash.data[0] < 40)
      c5++;
  }

  if(c0 > 30 || c1 > 25 || c2 > 20 || c3 > 15 || c4 > 10 || c5 > 5)
  {
    if(digitalRead(BUZZER_PIN) == HIGH)
      digitalWrite(BUZZER_PIN, LOW);
    else if(digitalRead(BUZZER_PIN) == LOW)
      digitalWrite(BUZZER_PIN, HIGH);
    c0 = 0;
    c1 = 0;
    c2 = 0;
    c3 = 0;
    c4 = 0;
    c5 = 0;
  }
}
ros::Subscriber<std_msgs::Float32MultiArray>espSub2("trash", &callBack2);

volatile int motor1PulseCount = 0; // Pulse counter
volatile int motor2PulseCount = 0; // Pulse counter
unsigned long lastTime = 0;  // To calculate RPM
unsigned long lastTime2 = 0;  // To calculate RPM
float rpm = 0;
float linearVelocity = 0;

float encoderPositionX = 0;
float encoderPositionY = 0;
float encoderPositionTheta = 0;

void IRAM_ATTR motor1CountPulse() {
  motor1PulseCount++;
}

void IRAM_ATTR motor2CountPulse() {
  motor2PulseCount++;
}

void setup()
{
  Serial.begin(115200);
  setCpuFrequencyMhz(80); /* set ESP32 frequency to 80MHz */
  
  // Set Wi-Fi credentials and ROS server details
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
  nh.getHardware()->setConnection(server_ip, server_port); /* Set ROS server details */
    // Set up the PWM channel
    
  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pinMode(MOTOR1_DIR1_PIN, OUTPUT);
  pinMode(MOTOR1_DIR2_PIN, OUTPUT);

  pinMode(MOTOR2_PWM_PIN, OUTPUT);
  pinMode(MOTOR2_DIR1_PIN, OUTPUT);
  pinMode(MOTOR2_DIR2_PIN, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(IR_SENSOR_PIN, INPUT);

  pinMode(ENCODER1_PIN_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_A), motor1CountPulse, CHANGE);

  pinMode(ENCODER2_PIN_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_A), motor2CountPulse, CHANGE);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RES);
  
  // Attach the PWM channel to the GPIO pin
  ledcAttachPin(MOTOR1_PWM_PIN, PWM_CHANNEL);
  ledcAttachPin(MOTOR2_PWM_PIN, PWM_CHANNEL2);

  digitalWrite(MOTOR1_DIR1_PIN, LOW);
  digitalWrite(MOTOR1_DIR2_PIN, HIGH);
  digitalWrite(MOTOR2_DIR1_PIN, LOW);
  digitalWrite(MOTOR2_DIR2_PIN, HIGH);

  digitalWrite(BUZZER_PIN, LOW);
  
  // Initialize ROS
  nh.initNode();        // initialize ROS node
  nh.advertise(espPub); // advertise the publisher
  nh.subscribe(espSub);
  nh.subscribe(espSub2);

 /* Wait till our ESP32 node connects to our ROS Master */
  while (!nh.connected())
  {
    nh.spinOnce();
    delay(100);
  }
}

void loop()
{
  /* Check if ESP32 node is connected to ROS master before publishing */
  if (nh.connected())
  {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastTime;
    unsigned long elapsedTime2 = currentTime - lastTime2;

    if (digitalRead(IR_SENSOR_PIN) == LOW)
    {
      wall_Ahead = 1;
      digitalWrite(MOTOR1_DIR1_PIN, HIGH);
      digitalWrite(MOTOR1_DIR2_PIN, LOW);
      digitalWrite(MOTOR2_DIR1_PIN, HIGH);
      digitalWrite(MOTOR2_DIR2_PIN, LOW);
      ledcWrite(PWM_CHANNEL, 100);
      ledcWrite(PWM_CHANNEL2, 60);
      delay(1500);
      wall_Ahead = 0;
    }
    if (elapsedTime >= 100)
    {
      //Serial.printf("Pulses = %d\n",motor2PulseCount);
      float encoderLeft = ((((float)motor1PulseCount/PULSES_PER_REV)/((float)elapsedTime/60000))*(PI/30))*WHEEL_RADIUS;
      float encoderRight = ((((float)motor2PulseCount/PULSES_PER_REV)/((float)elapsedTime/60000))*(PI/30))*WHEEL_RADIUS;
      
      if (abs(encoderLeft - encoderRight) < 0.03)
        encoderLeft = encoderRight;

      float encoderSpeed = (encoderLeft + encoderRight)/2;
      float encoderOmega = (encoderLeft - encoderRight) / (2 * BASE_RADIUS);


      encoderPositionX += encoderSpeed * cos(encoderPositionTheta) * ((float)elapsedTime/1000);
      encoderPositionY += encoderSpeed * sin(encoderPositionTheta) * ((float)elapsedTime/1000);

      encoderPositionTheta += (encoderOmega) * ((float)elapsedTime/1000);
      encoderPositionTheta = atan2(sin(encoderPositionTheta), cos(encoderPositionTheta));

      Serial.printf("encoderLeft= %f, encoderRight = %f, encoderOmega = %f\n", encoderLeft, encoderRight, encoderOmega);
      Serial.printf("encoderPositionX= %f, encoderPositionY= %f, encoderPositionTheta = %f\n", encoderPositionX, encoderPositionY, encoderPositionTheta);

      position.x = encoderPositionX;
      position.y = encoderPositionY;
      position.theta = encoderPositionTheta;

      espPub.publish(&position); 

      motor1PulseCount = 0;
      motor2PulseCount = 0;
      lastTime = currentTime;
    }
    if (elapsedTime2 >= 3000)
    {
      digitalWrite(BUZZER_PIN, LOW);
      person_Ahead = 0;
      lastTime2 = currentTime;
    }     
  }
  else
  {
    /*
      in case ESP32 Node is not connected to ROS Master
    */
    WiFi.reconnect(); /* try to reconnect */
  }

  nh.spinOnce();
}
