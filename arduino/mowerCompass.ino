#include <Wire.h>
#include <LSM303.h>

LSM303 compass;
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

char report[80];

  float XaSmooth = 0;
  float YaSmooth = 0;
  float ZaSmooth = 0;
  float azimuth;
void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();

}

double mod(float input){
  float modded;
  modded = input;
  if (input < 0){
    modded = input + 360;
  }
  if (input > 360){
    modded = input - 360;
  }
  return modded;
}

void loop() {  
  compass.read();
 

/*
  Serial.print(compass.m.x);
    Serial.print(" ");
  Serial.print(compass.m.y);
    Serial.print(" ");
  Serial.print(compass.m.z);
  Serial.println("");
*/

/*
  Serial.print(compass.a.x);
    Serial.print(" ");
  Serial.print(compass.a.y);
    Serial.print(" ");
  Serial.print(compass.a.z);
  Serial.println("");
 */
float xOff = -103.5;
float yOff = 72.9;
float zOff = 360;
float stretchY = 1.05;

float CmagX = compass.m.x + xOff;
float CmagY = stretchY*(compass.m.y + yOff);
float CmagZ = compass.m.z + zOff;

azimuth = atan2(CmagX,CmagY)*(180/PI);
float compFrame = 0;
if (azimuth < 0) {
  compFrame = mod(-azimuth - 90);
} else {
  compFrame = mod(360-azimuth -90);
}
/*
  Serial.print(CmagX);
    Serial.print(" ");
  Serial.print(CmagY);
    Serial.print(" ");
  Serial.print(CmagZ);
  Serial.print(" ");
*/


  Serial.print(compFrame );
  Serial.println("");
/*
float Ax = compass.a.x;
float Ay = compass.a.y;
float Az = compass.a.z;

float magnitudeA = sqrt(abs(Ax*Ax + Ay*Ay + Az*Az));

XaSmooth = XaSmooth*0.9 + compass.a.x*0.1/magnitudeA;
YaSmooth = YaSmooth*0.9 + compass.a.y*0.1/magnitudeA;
ZaSmooth = ZaSmooth*0.9 + compass.a.z*0.1/magnitudeA;

float LmagX = CmagX / XaSmooth;
float LmagY = CmagY / YaSmooth;
float LmagZ = CmagZ / ZaSmooth;
/*
   Serial.print(LmagX,7);
    Serial.print(" ");
  Serial.print(LmagY,7);
    Serial.print(" ");
  Serial.print(LmagZ,7);
  Serial.println("");

  Serial.print(compass.m.x);
    Serial.print(" ");
  Serial.print(compass.m.y);
    Serial.print(" ");
  Serial.print(compass.m.z);
  Serial.println("");
*/

/*
     Serial.print(XaSmooth,7);
    Serial.print(" ");
  Serial.print(YaSmooth,7);
    Serial.print(" ");
  Serial.print(ZaSmooth,7);
  Serial.println("");
*/
  delay(100);
}
