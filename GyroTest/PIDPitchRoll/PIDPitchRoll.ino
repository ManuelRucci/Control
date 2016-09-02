// http://www.giuseppecaccavale.it/
// Giuseppe Caccavale
 
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#define MPU 0x68  // I2C address of the MPU-6050
 
Servo ServoX, ServoY;
double AcX,AcY,AcZ;
double VelocityX, VelocityY, VelocityZ = {0};
double PositionX, PositionY, PositionZ = {0};
float Ts = 0.001;

float Pitch_Measured, Roll_Measured;
 
void setup(){
  Serial.begin(9600);
  ServoX.attach(8);
  ServoY.attach(9);
  init_MPU(); // Inizializzazione MPU6050
}
 
void loop()
{
  FunctionsMPU(); // Acquisisco assi AcX, AcY, AcZ.
    
  Roll_Measured = FunctionsPitchRoll(AcX, AcY, AcZ);   //Calcolo angolo Roll
  Pitch_Measured = FunctionsPitchRoll(AcY, AcX, AcZ);  //Calcolo angolo Pitch

  // Get Velocity from acceleration
  VelocityX += Ts*AcX;
  VelocityY += Ts*AcY;
  VelocityZ += Ts*nfc(AcZ,1);
  
  // Get Position from velocity
  PositionX +=  Ts*VelocityX;
  PositionY +=  Ts*VelocityY;
  PositionZ +=  Ts*VelocityZ;

  Serial.print("AccZ: "); Serial.print(AcZ);
  Serial.print("\t");
  Serial.print("VelocityZ: "); Serial.print(VelocityZ);
  Serial.print("\t");
  Serial.print("PositionZ: "); Serial.print(PositionZ);
  Serial.print("\t");
  Serial.print("\n");
  

  // ROLL PID Data
  float TsRoll = 0.01;
  float RefRoll = 0;
  static float p1ErrorRoll = 0;
  static float p2ErrorRoll = 0;
  static float prev_outRoll = 0;
  float KpRoll = 1;
  float KdRoll = 0;
  float KiRoll = 0;

  
  
  double PIDRoll = PID_Control( KpRoll,KdRoll,KiRoll, TsRoll,RefRoll, Roll_Measured, &p1ErrorRoll, &p2ErrorRoll,&prev_outRoll);

  // Pitch PID Data
  float TsPitch = 0.01;
  float RefPitch = 0;
  static float p1ErrorPitch = 0;
  static float p2ErrorPitch = 0;
  static float prev_outPitch = 0;
  float KpPitch = 1;
  float KdPitch = 0;
  float KiPitch = 0;
  
  double PIDPitch = PID_Control( KpPitch,KdPitch,KiPitch, TsPitch, RefPitch, Pitch_Measured, &p1ErrorPitch, &p2ErrorPitch,&prev_outPitch);
 
  int ServoRoll = map((int)PIDRoll, -90, 90, 0, 179);
  int ServoPitch = map((int)PIDPitch, -90, 90, 179, 0);
 
  //ServoX.write(ServoRoll);
  //ServoY.write(ServoPitch);
 
 
  //Serial.print("Pitch: "); Serial.print((int)PIDRoll);
  //Serial.print("\t");
  //Serial.print("Roll: "); Serial.print((int)PIDPitch);
  //Serial.print("\n");

  
//  Serial.print("ErrPitch: "); Serial.print(p1ErrorPitch);
//  Serial.print("\t");
//  Serial.print("ErrRoll: "); Serial.print(p1ErrorRoll);
//  Serial.print("\n");
 
}
 
void init_MPU(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(1000);
}
 
//Funzione per il calcolo degli angoli Pitch e Roll
double FunctionsPitchRoll(double A, double B, double C){
  double DatoA, DatoB, Value;
  DatoA = A;
  DatoB = (B*B) + (C*C);
  DatoB = sqrt(DatoB);
  
  Value = atan2(DatoA, DatoB);
  Value = Value * 180/3.14;
  
  return (int)Value;
}
 
//Funzione per l'acquisizione degli assi X,Y,Z del MPU6050
void FunctionsMPU(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
}


double PID_Control( float Kp, float Kd, float Ki, float Ts, float ref, float measured_value, float *p1Error, float *p2Error,float *prev_out)
{
 // Standard PID control
    
            // Initialize Error and PID output
            float p0Error = 0;  // Initialize error at current sample (k)
            float out = 0;      // Initialize output of the function
                   
            // Previous error assignment, solve pointer issue
            float p1E = *p1Error;           // Error at sample k-1
            float p2E = *p2Error;           // Error at sample k-2
            float p_out = *prev_out;        // Output at sample k-1
            
            // Compute Error
            p0Error = (ref - measured_value);
           //Serial.print("Error");
            //Serial.println(p0Error);

            // Calculate the PID control output
            out = p_out+Kp*(p0Error-p1E)+Ki*Ts*(p0Error)+(Kd/Ts)*(p0Error-2*p1E+p2E);

            // Current output becames previous output
            *prev_out = out;

            // Current error at sample k becomes error at sample k-1
            *p1Error = p0Error;  

            // Error at sample k-1 becomes error at sample k-2
            *p2Error = p1E;

            return out;
}
