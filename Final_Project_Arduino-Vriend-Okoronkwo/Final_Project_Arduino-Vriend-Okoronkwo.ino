Im/* ECE6311 Intro to Robotics
   Final Project
   Timothy Vriend 12/3/21
   This program is written for Arduino Mega 2560 by ELEGOO
   using DC motor driver L293D by HiLetgo
   with Robot arm kit Owi 535 Robotic Arm


   Motor numbers in this program match motor designators of OWI arm and motor driver shield
   M1 = Gripper
   M2 = joint_4 = wrist rotation
   M3 = joint_3 = elbow rotation (FORWARD = increase in Theta3) (0 degrees = wiper 531 ADC) (90 degrees = 181 ADC) (425 encoder counts for 180 degrees)
   M4 = joint_2 = shoulder rotation (FORWARD = increase in Theta3) (90 degrees = wiper 512 ADC) (180 degrees = 870 ADC)(202 encoder counts for 90 degrees)
   joint_1 = waist rotation

  M3 is 3.889 ADC LSBs per degree rotation & 2.361 encoder counts per degree (theoretical is 3.793 ADC bits per degree and 1.942 encoder counts per degree)
  M4 is 3.978 ADC LSBs per degree rotation & 2.244 encoder counts per degree

  Home position is at z = 5 & x = -15 (M4 = 38.21 deg, M3 = 134.79 deg)


*/

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define M3_wiper_pin  A15
#define M3_fixed_1_pin 42
#define M3_fixed_2_pin 40
#define M3_encoder_pin 20

#define M4_wiper_pin A12
#define M4_fixed_1_pin 39
#define M4_fixed_2_pin 37
#define M4_encoder_pin 21

#define wiper_sensor 0
#define encoder_sensor 1
#define feedback_sensor_type 1 // select which feedback sensor is used for closed loop control (encoder_sensor or wiper_sensor)

#include <AFMotor.h>  // https://learn.adafruit.com/adafruit-motor-shield/af-dcmotor-class 
// https://github.com/adafruit/Adafruit-Motor-Shield-library/blob/master/AFMotor.h


const float M3_wiper_zero_position = 531;
const float M3_wiper_home_position = 0; 
const float M3_wiper_reference_position = 181; 
const float M3_wiper_degrees_per_reference = 90; 
const float M3_wiper_angle_scaling = M3_wiper_degrees_per_reference / (M3_wiper_reference_position - M3_wiper_zero_position) ;
const float M3_wiper_reference_angle = 90; 


const float M3_encoder_home_position = 134.79;
//const float M3_encoder_home_position = get_current_angle(wiper_sensor, 3);;
const float M3_encoder_degrees_per_count = 0.423549; // !!!


const float M4_wiper_zero_position = 512; 
const float M4_wiper_home_position = 0; 
const float M4_wiper_reference_position = 870;
const float M4_wiper_degrees_per_reference = 90; 
const float M4_wiper_angle_scaling = M4_wiper_degrees_per_reference / (M4_wiper_reference_position - M4_wiper_zero_position);
const float M4_wiper_reference_angle = 0; 


const float M4_encoder_home_position = 38.21;  
//const float M4_encoder_home_position = get_current_angle(wiper_sensor, 4);
const float M4_encoder_degrees_per_count = 0.445632; 



const int size_of_trajectory_array = 157;
float tranjectory_array[2][size_of_trajectory_array] = {{41.89854067, 42.0128921, 42.13023434, 42.25052449, 42.37372077, 42.49978248, 42.62867003, 42.76034488, 42.89476957, 43.03190764, 43.17172368, 43.31418329, 43.45925306, 43.60690055, 43.75709428, 43.90980374, 44.06499933, 44.22265239, 44.38273515, 44.54522075, 44.71008318, 44.87729733, 45.04683892, 45.21868452, 45.39281152, 45.56919812, 45.74782336, 45.92866703, 46.11170972, 46.29693279, 46.48431837, 46.67384933, 46.86550928, 47.05928258, 47.25515429, 47.45311021, 47.65313683, 47.85522136, 48.05935168, 48.26551637, 48.4737047, 48.68390659, 48.89611264, 49.11031414, 49.326503, 49.5446718, 49.76481379, 49.98692284, 50.21099348, 50.43702089, 50.66500088, 50.8949299, 51.12680505, 51.36062406, 51.59638529, 51.83408776, 52.07373112, 52.31531565, 52.55884229, 52.80431262, 53.05172887, 53.30109393, 53.55241133, 53.80568528, 54.06092064, 54.31812297, 54.57729849, 54.83845412, 55.10159747, 55.36673686, 55.63388132, 55.90304062, 56.17422526, 56.44744648, 56.7227163, 57.00004751, 57.2794537, 57.56094925, 57.84454939, 58.13027017, 58.41812853, 58.70814228, 59.00033013, 59.29471173, 59.59130768, 59.89013957, 60.19122999, 60.49460256, 60.800282, 61.10829411, 61.41866582, 61.73142526, 62.04660176, 62.36422592, 62.68432964, 63.00694615, 63.33211011, 63.65985762, 63.99022628, 64.32325529, 64.65898545, 64.99745929, 65.3387211, 65.68281704, 66.02979518, 66.37970562, 66.7326006, 67.08853454, 67.44756421, 67.80974879, 68.17515003, 68.54383238, 68.9158631, 69.29131242, 69.67025373, 70.0527637, 70.43892254, 70.8288141, 71.22252618, 71.62015071, 72.02178403, 72.42752711, 72.83748593, 73.25177171, 73.67050133, 74.09379766, 74.52178998, 74.95461447, 75.39241465, 75.83534193, 76.28355623, 76.7372266, 77.19653195, 77.66166184, 78.13281733, 78.61021199, 79.09407295, 79.5846421, 80.08217746, 80.58695463, 81.09926854, 81.6194353, 82.14779438, 82.68471105, 83.23057915, 83.78582428, 84.35090743, 84.9263292, 85.51263466, 86.11041903, 86.72033428, 87.34309698, 87.97949748, 88.63041096, 89.29681062, 89.97978364, 90.6805507}, {130.8237709, 130.474594, 130.1231235, 129.7693815, 129.4133898, 129.0551691, 128.6947394, 128.3321199, 127.967329, 127.6003844, 127.2313029, 126.8601008, 126.4867934, 126.1113954, 125.7339209, 125.354383, 124.9727943, 124.5891667, 124.2035113, 123.8158386, 123.4261584, 123.03448, 122.6408116, 122.2451613, 121.8475362, 121.4479428, 121.046387, 120.6428742, 120.2374089, 119.8299952, 119.4206364, 119.0093355, 118.5960946, 118.1809151, 117.7637982, 117.3447442, 116.9237528, 116.5008232, 116.0759539, 115.6491429, 115.2203877, 114.7896849, 114.3570307, 113.9224206, 113.4858497, 113.0473124, 112.6068022, 112.1643125, 111.7198357, 111.2733638, 110.824888, 110.374399, 109.9218869, 109.4673411, 109.0107503, 108.5521026, 108.0913856, 107.6285859, 107.1636898, 106.6966826, 106.2275491, 105.7562733, 105.2828386, 104.8072276, 104.329422, 103.849403, 103.367151, 102.8826454, 102.3958651, 101.9067879, 101.415391, 100.9216505, 100.4255418, 99.92703943, 99.42611687, 98.92274672, 98.4169006, 97.90854915, 97.39766197, 96.88420759, 96.3681535, 95.84946602, 95.32811036, 94.80405052, 94.27724926, 93.74766811, 93.21526725, 92.68000553, 92.14184039, 91.60072781, 91.05662228, 90.50947671, 89.9592424, 89.40586896, 88.84930426, 88.28949434, 87.72638337, 87.15991352, 86.59002492, 86.01665558, 85.43974123, 84.85921532, 84.27500882, 83.68705018, 83.09526517, 82.49957677, 81.89990504, 81.29616696, 80.68827629, 80.07614341, 79.45967514, 78.83877457, 78.21334081, 77.58326884, 76.94844926, 76.30876801, 75.66410617, 75.01433961, 74.35933874, 73.69896813, 73.03308619, 72.36154477, 71.68418875, 71.00085556, 70.31137474, 69.61556735, 68.91324543, 68.20421132, 67.48825701, 66.76516336, 66.03469926, 65.29662071, 64.55066984, 63.79657375, 63.0340433, 62.26277177, 61.48243328, 60.69268116, 59.89314601, 59.08343362, 58.26312258, 57.43176159, 56.58886645, 55.73391663, 54.86635131, 53.98556499, 53.09090229, 52.18165209, 51.25704064, 50.31622366, 49.35827696, 48.38218554, 47.38683057, 46.370974, 45.33323999, 44.27209247, 43.18580788 }};
//5.0,5.1....24.9,24
//float tranjectory_array[2][size_of_trajectory_array] = { { 45.3469823464183, 48.1027084246821, 58.0207334095287 }, {137.509976220934, 121.82394422595, 100.205014055444 }, }; // 5 10 15

float acceptable_error = 3 ; // degrees
unsigned int minimum_motor_speed = 0; // !!!
unsigned int maximum_motor_speed = 255;

unsigned int max_motor_speed = 255;
unsigned int min_motor_speed = 50;

int M4_encoder_count = 0;
int M3_encoder_count = 0;

int M3_direction;
int M4_direction;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  AF_DCMotor motor3(3, MOTOR34_1KHZ); // The first argument stands for the number of the motors in the shield and the second
  AF_DCMotor motor4(4, MOTOR34_1KHZ); // one stands for the motor speed control frequency.

  pinMode(M3_fixed_1_pin, OUTPUT);
  digitalWrite(M3_fixed_1_pin, HIGH);
  pinMode(M3_fixed_2_pin, OUTPUT);
  digitalWrite(M3_fixed_2_pin, LOW);
  pinMode(M3_encoder_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(M3_encoder_pin), M3_encoder_interrupt, FALLING);

  pinMode(M4_fixed_1_pin, OUTPUT);
  digitalWrite(M4_fixed_1_pin, HIGH);
  pinMode(M4_fixed_2_pin, OUTPUT);
  digitalWrite(M4_fixed_2_pin, LOW);
  pinMode(M4_encoder_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(M4_encoder_pin), M4_encoder_interrupt, FALLING);

  motor3.setSpeed(max_motor_speed);
  motor4.setSpeed(max_motor_speed);

  float goal_angle_M4;
  float current_angle_M4;
  float angle_error_M4;
  float angle_error_magnitude_M4;
  unsigned int M4_speed;


  float goal_angle_M3;
  float current_angle_M3;
  float angle_error_M3;
  float angle_error_magnitude_M3;
  unsigned int M3_speed;

  pinMode(22, INPUT_PULLUP);
  while (digitalRead(22) == 0)
  {
    // wait for jumper to be pulled before doing anything
  }

  for (int i = 0; i < size_of_trajectory_array; i++)
  {
    goal_angle_M4 = tranjectory_array[0][i];
    angle_error_M4 = goal_angle_M4 - get_current_angle(feedback_sensor_type, 4);
    goal_angle_M3 = tranjectory_array[1][i];
    angle_error_M3 = goal_angle_M3 - get_current_angle(feedback_sensor_type, 3);

    while ((abs(angle_error_M4) > acceptable_error) || (abs(angle_error_M3) > acceptable_error))

    {
      current_angle_M4 = get_current_angle(feedback_sensor_type, 4);
      angle_error_M4 = goal_angle_M4 - current_angle_M4;
      angle_error_magnitude_M4 = abs(angle_error_M4);

      current_angle_M3 = get_current_angle(feedback_sensor_type, 3);
      angle_error_M3 = goal_angle_M3 - current_angle_M3;
      angle_error_magnitude_M3 = abs(angle_error_M3);

      Serial.print(current_angle_M4);
      Serial.print(",");
      Serial.print(goal_angle_M4);
      Serial.print(",");
      Serial.print(current_angle_M3);
      Serial.print(",");
      Serial.print(goal_angle_M3);
      Serial.println();

      if ((angle_error_M4 > 0) && (abs(angle_error_M4) > acceptable_error))
      {
        M4_direction = FORWARD;
      }
      else if ((angle_error_M4 < 0) && (abs(angle_error_M4) > acceptable_error))
      {
        M4_direction = BACKWARD;
      }
      else
      {
        M4_direction = RELEASE;
      }
      motor4.run(M4_direction);


      if ((angle_error_M3 > 0) && (abs(angle_error_M3) > acceptable_error))
      {
        M3_direction = FORWARD;
      }
      else if ((angle_error_M3 < 0) && (abs(angle_error_M3) > acceptable_error))
      {
        M3_direction = BACKWARD;
      }
      else
      {
        M3_direction = RELEASE;
      }
      motor3.run(M3_direction);
    }
  }
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void loop() {
  // put your main code here, to run repeatedly:

}

float get_current_angle(bool sensor_type, unsigned int motor_number)
{
  float current_motor_angle;

  switch (sensor_type)
  {
    case wiper_sensor:
      switch (motor_number)
      {
        case 3:
          //current_motor_angle = M3_wiper_degrees_per_reference * (analogRead(M3_wiper_pin) - M3_wiper_zero_position) / (M3_wiper_reference_position - M3_wiper_zero_position);
          current_motor_angle = M3_wiper_angle_scaling * (analogRead(M3_wiper_pin) - M3_wiper_zero_position);
          break;
        case 4:
          //current_motor_angle = 90 + M4_wiper_degrees_per_reference * (analogRead(M4_wiper_pin) - M4_wiper_zero_position) / (M4_wiper_reference_position - M4_wiper_zero_position);
          current_motor_angle = 90 + M4_wiper_angle_scaling * (analogRead(M4_wiper_pin) - M4_wiper_zero_position);
          break;
      }
    case encoder_sensor:
      switch (motor_number)
      {
        case 3:
          current_motor_angle = M3_encoder_home_position + M3_encoder_degrees_per_count * (M3_encoder_count);
          break;
        case 4:
          current_motor_angle = M4_encoder_home_position + M4_encoder_degrees_per_count * (M4_encoder_count);
          break;
      }

  }

  return current_motor_angle;
}

void M3_encoder_interrupt() {
  switch (M3_direction)
  {
    case FORWARD:
      M3_encoder_count++;
      break;
    case BACKWARD:
      M3_encoder_count--;
      break;
  }
}

void M4_encoder_interrupt() {
  switch (M4_direction)
  {
    case FORWARD:
      M4_encoder_count++;
      break;
    case BACKWARD:
      M4_encoder_count--;
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
