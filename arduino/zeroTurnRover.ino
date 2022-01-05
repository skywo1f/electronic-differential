#include <Wire.h>
#include <Arduino_NineAxesMotion.h>
#include <IBusBM.h>
#include <VescUart.h>
#include <Firmata.h>



// Create iBus Object
IBusBM ibus;

 /** Initiate vesc1Uart class */
VescUart vesc1;
VescUart vesc2;

// Create Nine Axes Motion Object

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
int n = 0;
float accz;
float accx;
const int streamPeriod = 40;
unsigned long lastStreamTime = 0;
bool updateSensorData = true;

// TB6612FNG Standby Pin
#define stby 6
 
// Motor Speed Values - Start at zero
int MotorSpeedA = 0;
int MotorSpeedB = 0;
 
// Motor Direction Values - -1 = backward, 1 = forward
int MotorDirA = 1;
int MotorDirB = 1;
 


// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
 
// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
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


 
  // Print values to serial monitor for debugging
 // Serial.print("Ch1 = ");
 // Serial.print(rcCH1);
 
 // Serial.print(" Ch2 = ");
 // Serial.print(rcCH2);
 
  //Serial.print(" Ch3 = ");
 // Serial.print(rcCH3);
 
 // Serial.print(" Ch5 = ");
 // Serial.print(rcCH5);
 
 // Serial.print(" Ch6 = ");
 // Serial.println(rcCH6);

 // Serial.print("M1 RPM = ");
  //Serial.print(vesc1.data.rpm);

  //Serial.print("M2 RPM = ");
 // Serial.print(vesc2.data.rpm);

  Serial.print("n =");
  Serial.println(n);

  Serial.print("Az =");
  Serial.print(mySensor.readAccelerometer(Z_AXIS));

  Serial.print("Ax = ");
  Serial.print(mySensor.readAccelerometer(X_AXIS));
  
  // Set speeds with channel 2 & 3 values
  MotorSpeedA = rcCH2;
  MotorSpeedB = rcCH3;
 
 

  //Convert to RPM
  rpm1 = 141.176*MotorSpeedA;
  rpm2 = 141.176*MotorSpeedB;

//The Gauntlet: Speed manipulation

 mySensor.updateAccel();
 accz = mySensor.readAccelerometer(Z_AXIS);
 accx = mySensor.readAccelerometer(X_AXIS);
  if (accz < 9.4 && accx > 1 || accz < 9.4 && accx < .3 || abs(rpm1) < 300 && abs(rpm2) > 500 || abs(rpm2) < 300 && abs(rpm1) > 500){
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
