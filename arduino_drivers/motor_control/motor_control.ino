/*
  This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
  It won't work with v1.x motor shields! Only for the v2's with built in PWM
  control

  For use with the Adafruit Motor Shield v2
  ---->  http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *Motor3 = AFMS.getMotor(3); //TODO: SWITCH BACK TO SINGLE MOTOR CONTROL 
Adafruit_DCMotor *Motor4 = AFMS.getMotor(4);

// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);
int angle = 0;
int analogPin = A1;
int read_byte = 0;
int torque = 0;
uint8_t i;
const int numMotors = 2;
const int bytesPerLine = 5; //ex: max val is 255. 1 extra for sign. One more for endline character. 
const int numChars = int(bytesPerLine*numMotors);
char receivedChars[numChars];
boolean newData = false;
char command[numMotors][bytesPerLine]; //command is formatted version of serial input. 
const String curShield = "thumb3_index";

int M3Open = 62;
int M3Closed = 196;
int M4Open = 371;
int M4Closed = 69;

enum State {
  IDLE_STATE,
  WAITING_FOR_TORQUE
};

int torque_time = millis();

State curState = IDLE_STATE;

void setup() {
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  digitalWrite(6, LOW);
  digitalWrite(7, HIGH);
 if (curShield=="thumb_index")
  {
    M3Open = 16;
    M3Closed = 1017;
    M4Open = 28;
    M4Closed = 1007;
  }
  else{
    M3Open = 95; 
    M3Closed = 1005;
    M4Open = 31; //m4 is wrist. "open" is pronated.
    M4Closed = 969;
  }
  
  Serial.begin(115200);           // set up Serial library at 9600 bps


  AFMS.begin(1600);
  Motor3->setSpeed(100);
  Motor3->run(FORWARD);
  Motor3->run(RELEASE);
  Motor4->setSpeed(100);
  Motor4->run(FORWARD);
  Motor4->run(RELEASE);


}

void recvWithEndMarker() {
 static byte ndx = 0;
 char endMarker = '\n';
 char rc;
 
 while (Serial.available() > 0 && newData == false) {
   rc = Serial.read();
  
   if (rc != endMarker) {
   receivedChars[ndx] = rc;
   ndx++;
     if (ndx >= numChars) {
      ndx = numChars - 1;
     }
   }
   else {
     receivedChars[ndx] = '\0'; // terminate the string
     ndx = 0;
     newData = true;
   }
 }
}

void format_packet(char command[numMotors][bytesPerLine]) {
  /*
   * Input: Packet that was read from serial
   * Output: Array of arrays
   */
   for(int i = 0; i < numMotors; i++){
    for(int j = 0; j < bytesPerLine-1; j++){
      
      command[i][j] = receivedChars[i*(bytesPerLine-1)+j];
    }
    command[i][bytesPerLine-1] = '\0';
   }
}

void setMotorSpeeds(char command[numMotors][bytesPerLine]) {
  torque = atoi(command[0]);

  if(torque > 1){
    Motor3 -> run(FORWARD);
  }
  else{
    Motor3->run(BACKWARD);
    torque = torque*-1;
  }
  torque = constrain(torque, 0, 150);
  Motor3->setSpeed(torque);

  //Motor 4
  torque = atoi(command[1]);

  if(torque > 1){
    Motor4->run(FORWARD);
  }
  else{
    Motor4->run(BACKWARD);
    torque = torque*-1;
  }
  torque = constrain(torque, 0, 150);
  Motor4->setSpeed(torque);
}

void stopMotors(){
  Motor3->run(RELEASE);
  Motor4->run(RELEASE);
}

void loop() {
  recvWithEndMarker();
  
  if (newData == true){
    if(curState == 0 && atoi(receivedChars) == 1111){
      voltageToAngle(1);
      curState = WAITING_FOR_TORQUE;
    }
    else if(curState == WAITING_FOR_TORQUE){
      if(atoi(receivedChars) != 1111){
      torque_time = millis();
      curState = IDLE_STATE;
      format_packet(command);
      setMotorSpeeds(command);
      }
    }
    
    newData = false;

  }
}
