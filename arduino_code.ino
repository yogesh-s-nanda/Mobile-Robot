////////////////////////////////////////////////////////////////////////////////////////////
// Autonomous Mobile Robot
// Mini Project for Mobile_Robotics
//
// Author : Jogesh S Nanda
//
// Description : Controlles a 2 wheel differential drive robot to find a target space
//      also uses obstracle avoidance to find a collision free path
//
// Co-Author : Sreelekshmi P
////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables Declaration
////////////////////////////////////////////////////////////////////////////////////////////

// declaring the global variable for converting incremental encoder to absolute encoder
volatile long encoder_A_pos = 0;    // incremental encoder value for Left wheel
volatile long encoder_B_pos = 0;    // incremental encoder value for Right wheel

// distance travelled by the motor wheel 
  float distance_motor_A = 0;     // distance travelled by left wheel
  float distance_motor_B = 0;     // distance travelled by right wheel
  
// Bluetooth 
char bluetooth ;

// Current Position of Robot
int count_x = 0;
int count_y = 0;

// Flag
// job = 0 ===> not completed
// job = 1 ===> moved forward     
// job = 2 ===> taken a turn
int job = 0;

// stage = 1 ===> has to move in Y co-odinate
// stage = 2 ===> has to take left or right 90 degree turn
// stage = 3 ===> has to move in X co-odinate
int stage = 1;

////////////////////////////////////////////////////////////////////////////////////////////
// PORT initilizaton
////////////////////////////////////////////////////////////////////////////////////////////

// Ultrasonic Sensor
int trig = 13;      // Trigger switch to send the echo sound. 
int echo = 12;      // For the detection of return echo. 

// motor LEFT wheel
int enA = 5;   // pwm pin to control the motor speed
int in1 = 6;    // direction control pin
int in2 = 7;    // direction control pin

// motor RIGHT wheel
int in3 = 8;    // direction control pin 
int in4 = 9;    // direction control pin
int enB = 10;    // pwm pin to control the motor speed

// encoder pulse from motor
int encoder_A = 2;    // encoder from left wheel
int encoder_B = 3;    // encoder from right wheel

//////////////////////////////////////////////////////////////////////////////////////////////
// Motor control
/////////////////////////////////////////////////////////////////////////////////////////////

// Motor A //////////////////////
void motor_A(int speed, word direction) {

  if (direction == "forward") {
    // to drive the motor in forward direction
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    
  }
  else if ( direction == "backward") {
    // to drive the motor in reverse direction
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if (direction == "stop") {
    // to STOP the motor
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
  }
  
  // to drive the motor at speed from value 0 to 255
  analogWrite(enA, speed);
  
}

// Motor B //////////////////////////
void motor_B(int speed, word direction) {

  if (direction == "forward") {
    // to drive the motor in forward direction
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if (direction == "backward") {
    // to drive the motor in reverse direction
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if (direction == "stop") {
    // to STOP the motor
    digitalWrite(in3, HIGH);
    digitalWrite(in4, HIGH);
  }
  
  // to drive the motor at speed from value 0 to 255
  analogWrite(enB, speed);
  
}

/////////////////////////////////////////////////////////////////////////////////////////////
// encoder 
////////////////////////////////////////////////////////////////////////////////////////////

// motor_A_encoder ---> from the interupt function increment Variable " encoder_A_pos " each time it's called
// motor_B_encoder ---> from the interupt function increment Variable " encoder_B_pos " each time it's called

// distance_motor_A ---> Global Variable which gives the distance travelled by the Left Wheel
// distance_motor_B ---> Global Variable which gives the distance travelled by the Right Wheel

// distance_reset ---> reset the encoder counter

// robot_distance ---> gives the distance travelled by the robot during forward or reverse direction

// Motor A encoder reading ///////////////////////////////
// works by interrupt funtion.
void motor_A_encoder() {

  // encoder reading
  encoder_A_pos = encoder_A_pos + 1;

  // Calculate the distance travelled by both the motors.   --> unit : cm
  // radius of wheel ===> 2.5 cm
  // number of teeth ===> 32 nos
  // distancce for 1 wheel rotation = 2 x pi x radius of wheel ---> in in this case is 16
  // distance trvelled by wheel = distance travelled by 1 wheel rotation x total number of rotation
  // total number of rotation = encoder_pos / number of teeth (here 32)
  // distance_of_motor = (encoder_pos / number of teeth ) x 2.pi x radius of wheel
  distance_motor_A = encoder_A_pos / 2 ;
  
}

// Motor B encoder reading  //////////////////////////////
// works by interrupt function.
void motor_B_encoder() {
  encoder_B_pos = encoder_B_pos + 1;

  // Calculate the distance travelled by both the motors.   --> unit : cm
  // radius of wheel ===> 2.5 cm
  // number of teeth ===> 32 nos
  // distancce for 1 wheel rotation = 2 x pi x radius of wheel ---> in in this case is 16
  // distance trvelled by wheel = distance travelled by 1 wheel rotation x total number of rotation
  // total number of rotation = encoder_pos / number of teeth (here 32)
  // distance_of_motor = (encoder_pos / number of teeth ) x 2.pi x radius of wheel
  distance_motor_B = encoder_B_pos / 2 ;
}

// encoder to distance conversion
void distance_reset(){
  // reset the distance travelled by both the motors for new function.
  encoder_A_pos = 0;
  encoder_B_pos = 0;

  delay(2000);
  
}

// Function gives the distance travelled by the robot 
// note : it give false reading when taking a turn
int robot_distance() {

  // Variable declaration 
  int distance ;

  // Distance the robot move is calculated by averging.
  distance = ( distance_motor_A + distance_motor_B )/2 ;

  return (distance);
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Ultrasonic Sensor
/////////////////////////////////////////////////////////////////////////////////////////////

// gives the distance from Obstracle in (cm)
float ultrasonic_sensor() {
  
  // declaration of local Variables
  unsigned long duration;    // time taken by the echo pulse
  float distance = 0;           // distance from Obstracle/ Output
  
  // Creating a Pulse for TRIG. ( min duration needed is 10 us).
  digitalWrite(trig, LOW);
  delay (10);
  digitalWrite(trig, HIGH);
  delay(10);
  digitalWrite(trig,LOW);

  duration = pulseIn(echo, HIGH);
 
  // Calculating Distance from Time 
  distance = 0.034 * duration/2;

  return(distance);
  
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Robot Dynamics
/////////////////////////////////////////////////////////////////////////////////////////////

// robot_move ---> move the robot at distance X [in cm]
// robot_turn_left ---> turn the robot at angle (anti-clockwise)
// robot_turn_right ---> turn the robot at an anlge (clock-wise)
// robot_stop ---> emergency STOP

// Function : Robot_move
// input ==> distance to move (in cm) at particular speed (0 to 255)
void robot_move(int target) {
  float travelled, collision_distance;
  
  // averaging the distance or Filtering 
  travelled = (distance_motor_A + distance_motor_B) /2 ;

  // measuring collision distance from ultrasonic sensor.
  collision_distance = ultrasonic_sensor();
  
  // Condition for moving the robot forward.
  if ( travelled < target) {

    // emergency stop if obstacle is found.
    if (collision_distance <20) {
      robot_stop();     // min. obstacle avoidance distance ---> 20 cm
    }

    // robot move forward if no obstacle is found.
    else {
      motor_A(100, "forward");   // earlier - 100
      motor_B(78, "forward");   // earlier - 76
    }
  }
  
  else {
    motor_A(255, "stop");
    motor_B(255, "stop");

    // setting the Flag as done
    job = 1;
    delay(100);
  }
  
}

// Function : robot_turn_left
// input ===> angle to move (in degree)
void robot_turn_left(int angle) {
  
  float distance, target;
  
  // distance to turn is radius x Angle (in radian)
  // length/radius ===> 18.5 cm
  // angle(radian) ===> (angle/180 ) x 3.14
  // distance ===> angle(radian) x radius = angle x 0.314 = angle / 3

  // Calculating distance to rotate the wheel
  target = angle/9 - 0;

  distance = (distance_motor_A + distance_motor_B)/2;
  
  // Left wheel Rotation
  if (distance < target) {
    motor_A(255, "backward");    // 64
  }
  else {
    motor_A(255,"stop");

    // setting the Flag as done.
    job = 2;
  }

  // Right Wheel Rotation
  if (distance < target) {
    motor_B(255, "forward");     // 50
  }
  else {
    motor_B(255,"stop");

    // setting the Flag as done.
    job = 2;
  }
  
}

void robot_turn_right(int angle) {
  
  float distance, target;
  
  // distance to turn is radius x Angle (in radian)
  // length/radius ===> 18.5 cm
  // angle(radian) ===> (angle/180 ) x 3.14
  // distance ===> angle(radian) x radius = angle x 0.314 = angle / 3

  // Calculating distance to rotate the wheel
  target = angle/9 - 0;

  distance = (distance_motor_A + distance_motor_B)/2;
   
  // Left wheel Rotation
  if (distance  < target) {
    motor_A(255, "forward");     // 64
  }
  else {
    motor_A(255,"stop");

    // setting the Flag as done.
    job = 2;
  }

  // Right Wheel Rotation
  if (distance < target) {
    motor_B(255, "backward");      // 50
  }
  else {
    motor_B(255,"stop");

    // setting the Flag as done.
    job = 2;
  }
  
}

// robot_stop ---> Emergeny Stop
void robot_stop () {

    // Stopping both the motor.
    motor_A(255, "stop");
    motor_B(255, "stop");

}

/////////////////////////////////////////////////////////////////////////////////////////////
// Robot Control Autonomous
/////////////////////////////////////////////////////////////////////////////////////////////

void robot_controller (int x, int y) {

  // Stage : 1 
  // Move the robot in Y Co-odinate
  if (stage == 1) {
    
    if ( count_y <= y ) {
      robot_move(y);
      Serial.println("stage 1");
      count_y = robot_distance();
    }
    
    // checking if stage 1 is complete
    if (job == 1) {
      distance_reset();
      Serial.println("moving to stage 2");
      stage = 2;
      delay(100);
    }  
  }


  // stage : 2
  // turn the robot towards appropriate Direction.
  if (stage == 2) {

    // Turnt the robot towards +ve X Co-odinate
    if (x > 0) {
      Serial.println("Turn right");
      robot_turn_right(90);
      
    }
    // Turn the robot towards -ve X Co-odinate
    else if (x < 0) {
      Serial.println("Turn left");
      robot_turn_left(90);
    }
    
    if (job == 2) {
      distance_reset();
      stage = 3;
      Serial.println("moving to stage 3");
      //x = x * -1;
      delay(100);
    }
  }
  
  
  // Stage : 3
  // Move the robot in X Co-odinate.
  if (stage == 3) {
    //if (count_x <x) {

      // Move robot forward
      robot_move(abs(x));
      Serial.println("stage 3");
      count_x = robot_distance();
      
  }
  
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Bluetooth Controller
/////////////////////////////////////////////////////////////////////////////////////////////

void bluetooth_controller() {

  
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:

  // setting the control signals of Motor A OUTPUT
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // setting the control signals of Motor B as OUTPUT
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // setting the encoder pins an INPUT
  pinMode(encoder_A, INPUT);
  pinMode(encoder_B, INPUT);

  // setting Ultrasonic Sensor
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // Start the baud rate of serial plotting in arduino
  Serial.begin(9600);
  
  // attaching the intrupt function for reading encoder values. 
  // interupt 0 == pin 2 ---> set for rising edge detection ---> Motor A
  // interupt 1 == pin 3 ---> set for rising edge detection ---> Motor B 
  attachInterrupt(0, motor_A_encoder, RISING);
  attachInterrupt(1, motor_B_encoder, RISING);
  
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Main Loop
////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  
  // Checking the Availabity of Bluetooth Communication
  //if (Serial.available() > 0) {
  //  bluetooth = Serial.read();
  // }
  
  robot_controller(60,0);
  
  //Serial.println(encoder_A_pos);
  //Serial.println(encoder_B_pos);

}
