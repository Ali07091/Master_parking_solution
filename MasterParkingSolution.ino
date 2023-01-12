#include <Wire.h>
#include <Encoder.h>
#include <MPU9250_WE.h>

#define TRIG 26
#define ECHO 27
#define SOUND_SPEED 0.034 // In cm/microsecond
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
#define MPU9250_ADDR 0x68 // 104 in hexadecimal

uint8_t enc1_count = 0;
uint8_t enc2_count = 0;
float duration = 0;
float distance = 0;
int leftMotor_speed = 255;
int rightMotor_speed =255;
int servoAngle = 90;

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

void setup()
{
  Serial.begin(9600);
  Wire.begin();   // join i2c bus          (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)
  Wire.onReceive(receiveEvent);
  
  // UltraSonic sensor initialization
  pinMode(TRIG,OUTPUT); // Set the trig pin 
  pinMode(ECHO,INPUT); // Set the echo pin 
 
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  if(!myMPU9250.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
    Serial.println("Magnetometer is connected");
  }
 //calibrating the MPU-9250
  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(200);
  
}



void loop() {
//Calling the functions
  ultrasonic();
  mpu();
  SendingToSlave();
  delay(1000);
  // Changing the values to turn the car
  servoAngle = 130;
  leftMotor_speed = 50;
  rightMotor_speed = 160;
  SendingToSlave();
  delay(1000);
// Changing the values to make car go backwards
  servoAngle = 90;
  leftMotor_speed = -255;
  rightMotor_speed = -255;
  SendingToSlave();
// to turn the car 90 degrees anti-clockwise 
      if (distance <=10) {
        servoAngle = 50;
        leftMotor_speed = 50;
        rightMotor_speed = 255;
        SendingToSlave();
        delay(1000);
         leftMotor_speed = -255;
         rightMotor_speed = -255;
         servoAngle = 90;
         SendingToSlave();
        
        }
   
     
  
        leftMotor_speed = -255;
        rightMotor_speed = -255;
        servoAngle = 90;
        SendingToSlave();
        delay(1000);
   //to stop the car if the obstacle is 10cm far
    if(distance <= 10){
      leftMotor_speed = 0;
      rightMotor_speed = 0;
      SendingToSlave();
    
    } 
   
    
exit(0);
  
}
// recieiving encoder values
void receiveEvent(int enc) {

   enc1_count =  Wire.read();
   enc2_count = Wire.read();
 
  Serial.print("encoder 1 = ");
  Serial.print(enc1_count);
  Serial.print("\t");
  Serial.print("encoder 2 = ");
  Serial.print(enc2_count);
  Serial.print("\n");

}
// sending the value to the slave function
void SendingToSlave(){
  
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  
  // sending Left Motor Speed to the Slave
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));        // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));                // second byte of x, containing the 8 LSB - bits 8 to 1
  
  // sending Right Motor Speed to the Slave
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));         // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF ));                // second byte of y, containing the 8 LSB - bits 8 to 1

  // sending Servo Angle to the Slave
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));               // first byte of y, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF ));                     // second byte of y, containing the 8 LSB - bits 8 to 1
  
  Wire.endTransmission();   // stop transmitting
  delay(10);

  
}
// the ultrasonic sensor function to measure distance
void ultrasonic(){
  // Clear the trigPin
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
 
 
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIG, LOW);
  
  // Read the duration of the echo signal in microseconds
  duration = pulseIn(ECHO,HIGH);
  
  // Calculate the distance 
  distance = duration*SOUND_SPEED/2;
  
  // Print the distance in the Serial Monitor
  Serial.print("Distance: ");
  Serial.print (distance);
  Serial.println(" cm");
}
// the MPU-9250 function to measure motion
void mpu(){
  //getting the values of the motion and tempreature
   xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();
  float temp = myMPU9250.getTemperature();
  float resultantG = myMPU9250.getResultantG(gValue);
// outputing the value in the serial monitor
  Serial.println("Acceleration in g (x,y,z):");
  Serial.print(gValue.x);
  Serial.print("   ");
  Serial.print(gValue.y);
  Serial.print("   ");
  Serial.println(gValue.z);
  Serial.print("Resultant g: ");
  Serial.println(resultantG);

  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);

  Serial.println("Magnetometer Data in µTesla: ");
  Serial.print(magValue.x);
  Serial.print("   ");
  Serial.print(magValue.y);
  Serial.print("   ");
  Serial.println(magValue.z);

  Serial.print("Temperature in °C: ");
  Serial.println(temp);

  Serial.println("********************************************");

  delay(1000);
}
