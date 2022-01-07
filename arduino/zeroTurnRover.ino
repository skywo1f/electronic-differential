#include <Wire.h>
#include <Arduino_NineAxesMotion.h>
#include <IBusBM.h>
#include <VescUart.h>
#include <Firmata.h>


//for connecting to the flysky radio
// Create iBus Object
IBusBM ibus;

//for talking to the motors
 /** Initiate vesc1Uart class */
VescUart vesc1;
VescUart vesc2;

// Create Nine Axes Motion Object
//for talking to the imu
NineAxesMotion mySensor;

// Channel Values
 
int rcCH1 = 0; // Left - Right
int rcCH2 = 0; // Forward - Reverse
int rcCH3 = 0; // Acceleration
int rcCH5 = 0; // Spin Control
bool rcCH6 = 0; // Mode Control
float rpm1 = 0; // Value to be sent to Vesc 1
float rpm2 = 0; // Value to be sent to Vesc 2
float oldrpm1 = 0;// Hold previous value of rpm1 for smoothing
float oldrpm2 = 0; //"  " rpm2
float smooth = 0;
//buzz related constants
int n = 0;
int buzzOn = 0;
float angleLimit = PI/10;
float thrustLimit = 5000;
float accz;
float accx;
float accy;

const int streamPeriod = 40;
unsigned long lastStreamTime = 0;
bool updateSensorData = true;
double angle = 0;
double sensorAngle = 0;//assumung 0 degree tilt on bno055 imu
int maxTorque = 10000;       //should be between 0 and 28k
float sideAccK = 3.0;       //between 0 and 9.8
// TB6612FNG Standby Pin
#define stby 6
 
// Motor Speed Values - Start at zero
int MotorSpeedA = 0;
int MotorSpeedB = 0;
 
// Motor Direction Values - -1 = backward, 1 = forward
int MotorDirA = 1;
int MotorDirB = 1;
double runningDeltaThrust = 0; 
double runningWeight = 0.5;
//convert transceiver value to wheel value
// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
 //read the radio switch and convert to boolean
// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}
 double getAngle(double accz,double accy){
  return atan(accy/accz) + sensorAngle;
 }

int checkAccDec(double angle,double thisThrust){
  int buzzerOn = 0;
  //check uphill
  if(angle < -angleLimit){
    if ( thisThrust < -thrustLimit){
      buzzerOn = 1;
    }
  }
  //check downhill
  if (angle > angleLimit) {
    if(thisThrust > thrustLimit) {
      buzzerOn = 1;
    }
  }
  return buzzerOn;
}

 float calcThrust(float A,float B){
   float thrust = 0;
  
   if (A*B > 0){            //if they are thrusting in the same direction
      thrust = A*abs(B);    //return their product but keep one of their signs   
   }
    
   return thrust;
 }
int checkPivot(int rpm1, int rpm2){
  int buzzerOn = 0;
  if(abs(rpm1) < 300 && abs(rpm2) > 500 || abs(rpm2) < 300 && abs(rpm1) > 500) {
    buzzerOn = 1;
  }
  return buzzerOn;
}

int checkPendulum(float accx,float accy,float accz,int rpm1,int rpm2){
  int buzzerOn = 0;
  //first find out how much slope there is here
  //float localSlope = acos(accz/9.8);
  //next I need to know if I am rotating uphill or downhill and by how much
  float rotation = rpm1 - rpm2;
  Serial.println(rotation);
  Serial.println(maxTorque);
  Serial.println(accx);
  Serial.println(sideAccK);
  Serial.println(" ");
  if (accx > sideAccK && rotation < -maxTorque || accx < -sideAccK && rotation > maxTorque){
    buzzerOn = 1;
  }
  return buzzerOn;
}
 
void setup()
 
{
  // Set pin for buzzer
  pinMode(2, OUTPUT);
 
  // Start serial monitor for debugging
  Serial.begin(9600);
 
  // Attach iBus object to serial port
  ibus.begin(Serial3);

  //Initialize UART ports for Vescs
  Serial1.begin(115200);
  Serial2.begin(115200);
  
  
 /** Define which ports to use as UART */
  vesc1.setSerialPort(&Serial1);
  vesc2.setSerialPort(&Serial2);

// Initialize Nine Axis Motion Sensor

  Wire.begin();
  mySensor.initSensor();
  mySensor.setOperationMode(OPERATION_MODE_NDOF);
  mySensor.setUpdateMode(MANUAL);
  mySensor.updateAccelConfig();
  updateSensorData = true;
  
  
}

void loop() {
 
  // Get RC channel values
  rcCH1 = readChannel(0, -100, 100, 0);
  rcCH2 = readChannel(1, -100, 100, 0);
  rcCH3 = readChannel(2, -100, 100, 0);
  rcCH5 = readChannel(4, -100, 100, 0);
  rcCH6 = readSwitch(5, false);

  buzzOn = 0;
/*
  Serial.print("n =");
  Serial.println(n);

  Serial.print("Az =");
  Serial.print(mySensor.readAccelerometer(Z_AXIS));

  Serial.print("Ax = ");
  Serial.print(mySensor.readAccelerometer(X_AXIS));
  */
  // Set speeds with channel 2 & 3 values
  MotorSpeedA = rcCH2;
  MotorSpeedB = rcCH3;
 
 
//convert wheel speed from computer readable to human readable
  //Convert to RPM
  rpm1 = 141.176*MotorSpeedA;
  rpm2 = 141.176*MotorSpeedB;

//The Gauntlet: Speed manipulation
//probably dont have to read this twice
 mySensor.updateAccel();
 accz = mySensor.readAccelerometer(Z_AXIS);
 accx = mySensor.readAccelerometer(X_AXIS);
 accy = mySensor.readAccelerometer(Y_AXIS);
 //if z isnt getting the full force of gravity and x is seeing some gravity
 //if z isnt seeing the full force of gravity and x is below a certain amount (sloped the other way)
 //if one wheel is moving but the other one isnt
 int maxAcc = 75;
 int maxDec = 25;
 int thisThrust = 0;
 angle = getAngle(accz,accy);
 thisThrust = calcThrust(MotorSpeedA,MotorSpeedB);
// buzzOn += checkAccDec(angle,thisThrust);   //accelerating/deccelerating on a hill
// buzzOn += checkPivot(rpm1,rpm2);           //pivoting on one wheel

 buzzOn += checkPendulum(accx,accy,accz,rpm1,rpm2);                 //attempting to rotate uphill 
 Serial.println(buzzOn);


//buzzmixing may give false positives
if (buzzOn) {
//make sure this isnt a one-off (imu noise)
    n = n+1;
  
  if (n > 4) {
    Serial.print("Danger! Scuffing Likely!");
    digitalWrite(2,HIGH);
  }
  }
   else {
    digitalWrite(2,LOW);
    n = 0;
   }
  //Drive Motors
  vesc1.setRPM(rpm2);
  vesc2.setRPM(rpm1);

 oldrpm1 = rpm1;
 oldrpm2 = rpm2;
  // Slight delay
  delay(100);
 
}
