/*
KOD till kursen Mj1104 - intro till energiteknik
Vidar Höjer
*/

/* HELPFULL ABOUT ARDUINO
    digitalRead() works on all pins. It will just round the analog value received
    and present it to you. If analogRead(A0) is greater than or equal to 512,
    digitalRead(A0) will be 1, else 0.
    digitalWrite() works on all pins, with allowed parameter 0 or 1.
    digitalWrite(A0,0) is the same as analogWrite(A0,0), and digitalWrite(A0,1)
    is the same as analogWrite(A0,255)
    analogRead() works only on analog pins. It can take any value between 0 and 1023.
    analogWrite() works on all analog pins and all digital PWM pins. You can
    supply it any value between 0 and 255.
*/
#include <Servo.h>

Servo servomotor;
// 0 / 1 EMPTY PINS
const int IR_left_pin   = 2;
const int motor_L_pin_2 = 3;
const int motor_L_pin_1 = 4;
const int EN_L_pin      = 5;
const int EN_R_pin      = 6;
const int motor_R_pin_2 = 7;
const int IR_right_pin  = 8;
// PIN 9 EMPTY
const int servo_pin     = 10;
const int motor_R_pin_1 = 11;
const int US_echo_pin   = 12;
const int US_trig_pin   = 13;

/// GLOBALS
int turning_radius = 30;
int Local_forward = 90;
int Local_left = 180;
int Local_right = 0;
int parking_distance = 100; // obstacle to parking square
int finish_line = 100; // parking spqce to black wall
/////////// Defined functions /////////
//
// void irTest();
// void DCmotorTest();
// void ServoMotorTest();
// int servoTest(int step = 15);

void stopMotors();
bool runMotors(int time_to_run = 100, int speed_motor_A = 255, int speed_motor_L = 255, bool forward = true);
int distanceUS(int pos, int num_vals = 2);
int closestObject(int min_sweep = 0, int max_sweep = 180, int increment = 15);
void faceObject(int min_sweep = 0, int max_sweep = 180, int increment = 15);
bool checkIRsensor(int sensorPin);
int turnXdegrees(bool turn_right, bool use_sensor, int sensor_direction, int target_distance, int max_iterations = 100);
int mainButWithADifferentNameBecauseFArduino();

// #include "testfunktioner.h"
void setup() {
  Serial.begin(9600);
  // DC_MOTORER
  pinMode(motor_R_pin_1, OUTPUT);
  pinMode(motor_R_pin_2, OUTPUT);
  pinMode(motor_L_pin_1, OUTPUT);
  pinMode(motor_L_pin_2, OUTPUT);
  pinMode(EN_R_pin, OUTPUT);
  pinMode(EN_L_pin, OUTPUT);
  // IR
  pinMode(IR_left_pin, INPUT);
  //digitalWrite(IR_left_pin,LOW); // test to disable IR to save power
  pinMode(IR_right_pin, INPUT);
  //digitalWrite(IR_right_pin,LOW);
  // SERVO
  servomotor.attach(servo_pin);
  servomotor.write(Local_forward);
  // ULTRASONIC
  pinMode(US_trig_pin, OUTPUT);
  digitalWrite(US_trig_pin, LOW); // default off
  pinMode(US_echo_pin, INPUT);

}

void loop(){
  /*
  MAIN Loop
  */
  bool runMAIN = true;
  // turn to false when code has been tested
  // TESTFUNCTIONS
  if (runMAIN) {
    mainButWithADifferentNameBecauseFArduino();
    //clear parking reaktion!
    for (int i = 0;i < 3000; i+=500){
      servomotor.write(80);
      delay(250);
      servomotor.write(100);
      delay(250);
    }
    delay(30000); // long pause after each run
  }
  // Serial.println("TESTING PROGRAM!!");
  // write tests here
}

/////////// FUNCTIONS //////////////
void stopMotors(){
  // FAST STOP to motors incase of danger!
  analogWrite(EN_R_pin, 0);
  analogWrite(EN_L_pin, 0);
  digitalWrite(motor_R_pin_1, LOW);
  digitalWrite(motor_R_pin_2, LOW);
  digitalWrite(motor_L_pin_1, LOW);
  digitalWrite(motor_L_pin_2, LOW);
}

bool runMotors(int time_to_run, int speed_motor_R, int speed_motor_L, bool forward){
  /*
  runs motors for a prescirbed time
  if no speed incicated -> both engines max speed
  In  -> Time in mS (default 1000); motorspeeds (0-255)
  Out -> None
  */
  int stepsize = 10;
  // MAKE SURE DIRECTIONS ARE CORRECT
  if (forward){
    digitalWrite(motor_R_pin_1, LOW);
    digitalWrite(motor_R_pin_2, HIGH);
    digitalWrite(motor_L_pin_1, LOW);
    digitalWrite(motor_L_pin_2, HIGH);
  }
  else {
    digitalWrite(motor_R_pin_1, HIGH);
    digitalWrite(motor_R_pin_2, LOW);
    digitalWrite(motor_L_pin_1, HIGH);
    digitalWrite(motor_L_pin_2, LOW);
  }
  analogWrite(EN_R_pin, speed_motor_R);
  analogWrite(EN_L_pin, speed_motor_L);
  for (time_to_run; time_to_run > 0; time_to_run-=stepsize){

    delay(stepsize);
    if (!checkIRsensor(IR_left_pin)) {
      Serial.println("ERROR, Unexpected IR-trigger! Left");
      return false;
    }
    if (!checkIRsensor(IR_right_pin)){
      Serial.println("ERROR, Unexpected IR-trigger! Right");
      return false;
    }
  }
  return true;
}

bool checkIRsensor(int sensorPin){
  /*
  !!! FIRST CALIBRATE SENSOR WITH ONBOARD VARIABLE RESISTOR !!!
  function tests the sensor until it detects different surfaces at
  correct height
  IN  -> pin to read IR value from
  OUT -> true = good(white surface) LOW Signal
         false = bad(black surface / no reflection) HIGH signal
  */
  return (digitalRead(sensorPin) == LOW);
}

int distanceUS(int pos, int num_vals){
  /*
  reads from the US sensors, average of num_vals values,
  default average of 2 readings
  depends on quality of measurements
  IN  -> angle of servo motor to read from
  OUT -> value in cm
  */
  int max_iter = 2;
  int distance = 0;
  //delay(100);
  servomotor.write(pos);
  delay(100);
  if (abs(pos-Local_forward)>75){
    delay(150);
  }
  for(int i=0;i<= num_vals; i++){
    digitalWrite(US_trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_trig_pin, LOW);
    int readVal = pulseIn(US_echo_pin, HIGH)/58;
    int iter = 0;
    while( (readVal > 2000 )||( iter< max_iter)){
      // above 10m bad reading, retry
      iter++;
      digitalWrite(US_trig_pin, HIGH);
      delayMicroseconds(10);
      digitalWrite(US_trig_pin, LOW);
      readVal = pulseIn(US_echo_pin,HIGH)/58;
    }
    distance += readVal;
  }
  return distance / num_vals;

}

void faceObject(int min_sweep, int max_sweep, int increment){
  /*
  Turn so closest object is straight infront of car
  IN -> max step iterations
  OUT -> None
  */
  int object_direction = closestObject(min_sweep, max_sweep, increment);
  int object_distance = distanceUS(object_direction);
  int tol = 2;
  int prop = 2;
   // 10 iterations to try and fix turn
  Serial.print("> Direction: ");
  Serial.print(object_direction);
  Serial.print(" | Distance (cm): ");
  Serial.println(object_distance);
  //Serial.println("(3 seconds pause)");
  //delay(3000);
  int temp = abs(object_direction - Local_forward);
  int range = 45;
  if (temp < 15){
    range = 15;
  }
  else if ((temp < 30 || temp > 15)){
    range = 30;
  }
  else if (temp > 45){
    range = 90;
  }
  while ( temp > tol){
    if (object_direction < Local_forward-tol) {
      // Target is to the left
      turnXdegrees(true, false, Local_forward, object_distance,temp*prop);
      delay(temp);
      object_direction = closestObject(Local_forward-range,Local_forward+range,increment);
    }
    else if (object_direction > Local_forward+tol){
      // Target is to the right

      turnXdegrees(false, false, Local_forward, object_distance,temp*prop);
      delay(temp);
      object_direction = closestObject(Local_forward-range,Local_forward+range,15);
    }
    temp = abs(object_direction - Local_forward);
    delay(50);
  }
}

int closestObject(int min_sweep , int max_sweep , int increment){
  /*
   Sweeps servo to find closest obstacle with US sensor
  Default SWEEPs FULL SPECTRUM,
  Decrease stepsize to increase resulution of sweep
  In  -> default argument for sweep resulution
  Out -> Direction relative to car
  */
    int num_vals = (max_sweep - min_sweep)/increment + 1;
    int tol = 3; // 3cm difference doesn't count as new object
    int distArr[num_vals];
    int closest;
    int index = 0;
    for (int i=0;i<num_vals;i++){
      if (i==0){
        closest = distanceUS(i*increment+min_sweep);
        distArr[i] = closest;
      }
      else{
        distArr[i] = distanceUS(i*increment+min_sweep);
        if (distArr[i]< closest){
          closest = distArr[i];
          index = i;
        }
      }
    }
    int index2 = index;
    int closest2 = closest;
    int ii;
    if (index > 0){
      ii = index-1;
    }
    else {
      ii = 0;
    }
    for(ii; ii<num_vals;ii++){
      // Serial.print(distArr[ii]);
      // Serial.print(",");
      if (distArr[ii] < closest +tol){
        // same target
        closest2 = distArr[ii];
        index2 = ii;
      }
    }
    Serial.print("\nClosest target at ");
    Serial.print(closest);
    Serial.print("cm, direction ");
    int best_direction = (index2+index)*increment/2+ min_sweep;
    Serial.println(best_direction);
    return best_direction;
}

int turnXdegrees(bool turn_right, bool use_sensor, int sensor_pos, int target_distance, int max_iterations){
  /*
  turns vehicle by spinning both wheels in different direction untill US sensor
  detects target at specified direction,
  IN  -> bool turn_right (determines turn-direction), set to false for left turns
  OUT -> returns number of iterations to complete turn
  */
  int motorspeed = 175;
  int time_step  = 3;  // Size of time-stepping
  int tol = 3;
  analogWrite(EN_R_pin, motorspeed);
  analogWrite(EN_L_pin, motorspeed);
  if (turn_right){
    //
    Serial.println("right");
    digitalWrite(motor_R_pin_1, HIGH);
    digitalWrite(motor_R_pin_2, LOW);
    digitalWrite(motor_L_pin_1, LOW);
    digitalWrite(motor_L_pin_2, HIGH);
  }
  else {
    //
    Serial.println("left");
    digitalWrite(motor_R_pin_1, LOW);
    digitalWrite(motor_R_pin_2, HIGH);
    digitalWrite(motor_L_pin_1, HIGH);
    digitalWrite(motor_L_pin_2, LOW);
  }
  int i = 0;
  if (use_sensor){
    Serial.println("ping");
    int object_distance = distanceUS(sensor_pos);
    while ( (object_distance > target_distance+tol)  && (i < max_iterations) ){
      // uses US to see when turn is finished
      i++;
      delay(time_step);
      object_distance = distanceUS(sensor_pos);
    }
    stopMotors();
    if ((i == max_iterations) && (object_distance > target_distance+tol)){
      Serial.println("ERROR! increase max_iterations or possible hardware bug");
      return 0;
    }
    else {
      return i;
    }
  }
  else {
    Serial.println("pong");
    for (i = 0;i< max_iterations;i++){
      // turns blindly
       delay(time_step);
    }
    stopMotors();
    return i;
  }
}

int mainButWithADifferentNameBecauseFArduino(){
  /* code */
  int slow_speed = 170;
  int fast_speed = 255;
  int tol = 8; // approx half car width
  int timestep = 20;
  //
  faceObject(45,135,15);
  Serial.print("1");
  int object_direction;
  int object_distance = distanceUS(Local_forward);
  Serial.print("2");
  bool IRtrig = false;
  while (object_distance > turning_radius + tol || !IRtrig){
    Serial.print("3");
    IRtrig = runMotors(timestep, slow_speed, slow_speed, true);
    object_distance = distanceUS(Local_forward); // recheck direction
  }
  Serial.print("4");
  stopMotors();
  // if (IRtrig){
  //   // DO something
  //   if (checkIRsensor(IR_left_pin) || !checkIRsensor(IR_right_pin)){
  //     // left trigger
  //     Serial.println("IRping");
  //   }
  //   else if (!checkIRsensor(IR_left_pin) || checkIRsensor(IR_right_pin)){
  //     // right trigger
  //     Serial.println("IRpong");
  //   }
  //   else {
  //     Serial.println("IRping Pong");
  //   }
  // }
  // turn right and look left,
  int turn_steps = 370;
  turnXdegrees(true,false,Local_left,0,turn_steps-15);
  Serial.print("5");
  while (distanceUS(Local_left) > turning_radius + tol*3){
    // fixing turn angle
    turnXdegrees(true,false,Local_left,0,10);
  }
  Serial.print("6");
  while (object_distance < turning_radius+tol*2){
    object_distance = distanceUS(Local_left);
    IRtrig = runMotors(20,slow_speed,slow_speed, true);
  }
  delay(100);
  while (distanceUS(Local_left) < turning_radius + tol*2){
    delay(10);
  }
  stopMotors();
  int turns = 0;
  // TURNING AROUND THE OBSTACLE
  while (turns < 2){
    turnXdegrees(false,false,Local_left,0,turn_steps-45);
    while (object_distance  > turning_radius + tol){
      IRtrig = runMotors(20,slow_speed,slow_speed, true);
      object_distance = distanceUS(Local_left);
    }
    while (object_distance < turning_radius){
      // drive until passing obstacle
      if (turns == 1){
        // single iteration
        runMotors(40,slow_speed,slow_speed,true);
        stopMotors();
        break;
      }
      else {
        IRtrig = runMotors(10,slow_speed,slow_speed, true);
        object_distance = distanceUS(Local_left);
        runMotors(20,slow_speed,slow_speed, true);
        stopMotors();
      }
    }
    turns++;
  }
  // stopMotors();
  // turn  obstacle and reverse to parking space
  turnXdegrees(false,false,Local_forward,0,turn_steps-35);
  faceObject(45,135,15);
  object_distance = distanceUS(Local_forward);
  while (object_distance < parking_distance){
    //reverse
    runMotors(20,slow_speed+5,slow_speed+5, false);
    object_distance = distanceUS(Local_forward);
  }
  stopMotors();
  // 180 Noscope
  turnXdegrees(true,false,Local_forward,0,turn_steps*2 - 100);
  //parking
  for (int i = 0;i < 3500; i+=500){
    servomotor.write(80);
    delay(250);
    servomotor.write(100);
    delay(250);
  }
  //faceObject(45,135,15);
  object_distance = distanceUS(Local_forward);
  while (object_distance > finish_line){
    IRtrig = runMotors(20,fast_speed,fast_speed, true);
    object_distance = distanceUS(Local_forward);
  }
  while (object_distance > turning_radius){
    IRtrig = runMotors(10,slow_speed,slow_speed, true);
    object_distance = distanceUS(Local_forward);
  }
  if (!IRtrig || object_distance < turning_radius-tol*2){
    stopMotors();
    return 1;
  }
  stopMotors();
  return 0;
}
