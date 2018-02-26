/*  Code written by Marcelo Maroñas @ Minerva Rockets (Federal University of Rio de Janeiro Rocketry Team) - February 20, 2018
 *  This is an adaptation necessary for using the GY80 IMU module with the microcontroller Teensy version 3.6.
 *  This the very basic code using the library.The library was written using the best libraries I could find at the moment for each GY80 sub-module (Gyro, Accel, Magne, BMP085) and
 *  putting them together in an lightweight and easy to understand code.Dont use it with Arduino, there's a lighter version of GY80 library that doesnt need so much memory, check in my GitHub.
 *  The libraries for each sub-modules are, in majority, adapted adafruit libraries, and because of it, 
 *  they are very heav.But in the counterpart, they also are very robust and have methods for everything that you need to do with the sensor.
 *  You can choose to print values to debug and test in the serial monitor.
 *  The data is printed in a CSV way, so you can copy and paste the serial monitor info into a notepad file and save as a CSV that can be opened in Excel or other CSV softwares.
 *  The structure IMU_s is given by :
 *      IMU_s->double acelerometro[3]; Where positions 0, 1 and 2 in the array are acelerometer x, y and z values respectively, in m/s².
 *      IMU_s->int magnetometro[3]; Where positions 0, 1 and 2 in the array are magnetic field x, y and z values respectively, in vector form.
 *      IMU_s->int giroscopio[3]; Where positions 0, 1 and 2 in the array are gyroscope x, y and z values respectively, in angular acceleration.
 *      IMU_s->double barometro[3]; Where positions 0, 1 and 2 in the array are pressure(in Pa), altitude(in Meters) and temperature(in Celsius) respectively.    
 *  Contact : marcelomaronas at poli.ufrj.br
 *  For more codes : github.com/engmaronas
 */

/* GY-80 Pins
 *  Vcc_In <----------------------> Teensy 3.3V
 *  Gnd    <----------------------> Teensy Gnd
 *  SDA    <----------------------> 18
 *  SCL    <----------------------> 19
 */

#include <GY80TEENSY.h> //Include the library GY80TEENSY
#include <SD.h>
#include <SPI.h>

//Variable definition is within the library, those are the sensors used.The library needs to be updated for multiples GY80 use.
//Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
//Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
//Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12346);
//L3G gyro;

//SD module variables
File myFile; 
const int chipSelect = BUILTIN_SDCARD;

//Structure declaration
IMU_s struct_imu;
IMU_s *pstruct_imu = &struct_imu;

//Use this variable to enable debugging via Serial Monitor
bool DebugSerial = 1; //Prints the values stored in the structure IMU_s
bool sdLog = 1; //Prints the values of IMU_s in SD card

//Modify the Delay_Time variable to control how much info is printed on the serial monitor
float Delay_Time = 500;

//Notification leds
int SdLed = 3;
int RecoveryLed = 4;
int SdRecording = 13;

//Recovery variables
float LastAltitude;

void setup() {
  Serial.begin(9600); //Initialize Serial Port at 9600 baudrate.
   
  pinMode(RecoveryLed, OUTPUT); 

  //SD card setup
  pinMode(SdRecording, OUTPUT); 
  pinMode(SdLed, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("SD module initialization failed!");
    sdLog = 0;
  }
  Serial.println("SD module initialization done.");
  if (sdLog) {
    myFile = SD.open("DADOSMVP.txt", FILE_WRITE);
       if (myFile) {
              myFile.println("sep =, "); //This line handles Excel CSV configuration.
              myFile.println("Time, Pressure, Altitude, Temperature, AcelX, AcelY, AcelZ, GyroX, GyroY, GyroZ, MagnetoX, MagnetoY, MagnetoZ"); 
       }
       else {
          digitalWrite(RecoveryLed, HIGH);
        }    
    myFile.close();
  }
  //End of SD card setup
  
  Serial.println("BasicLibraryCode v1.0 - Teensyduino GY80 library by Marcelo Maronas @ Minerva Rockets");

  Serial.println("sep =, "); //This line handles Excel CSV configuration.
  Serial.println("Time, Pressure, Altitude, Temperature, AcelX, AcelY, AcelZ, GyroX, GyroY, GyroZ, MagnetoX, MagnetoY, MagnetoZ"); 
 
  InitBMP(); //Initialize BMP module
  InitAcel(); //Initialize Accelerometer module
  InitGyro(); //Initialize Gyroscope module
  InitMag(); //Initialize Magnetometer module
  digitalWrite(SdLed, HIGH);
}

void loop() {
  GetBMP(pstruct_imu); //Fills the BMP085 module information into the IMU structure
  GetAcel(pstruct_imu); //Fills the Accelerometer module information into the IMU structure
  GetGyro(pstruct_imu); //Fills the Gyroscope module information into the IMU structure
  GetMag(pstruct_imu); //Fills the Magnetometer module information into the IMU structure
  
   //Delay time
  //delay(Delay_Time);

  if (((abs(struct_imu.acelerometro[0]) < 1) && (abs(struct_imu.acelerometro[1]) < 1) && (abs(struct_imu.acelerometro[2]) < 1)))
{
     digitalWrite(RecoveryLed, HIGH);
     if (sdLog) {
       myFile = SD.open("DADOSMVP.txt", FILE_WRITE);
       if (myFile) {
              myFile.print(millis());myFile.print(" ,");
              myFile.print(struct_imu.acelerometro[2]);myFile.print(" ,");
              myFile.println("Recovery opened");    
       }
       myFile.close();
     }
  }

  //SD logging code
  if (sdLog) {
     myFile = SD.open("DADOSMVP.txt", FILE_WRITE);
     digitalWrite(SdRecording, LOW);
     if (myFile) {
          myFile.print(millis());myFile.print(" ,");
          myFile.print(struct_imu.barometro[0]);myFile.print(" ,");
          myFile.print(struct_imu.barometro[1]);myFile.print(" ,");
          myFile.print(struct_imu.barometro[2]);myFile.print(" ,");
          myFile.print(struct_imu.acelerometro[0]);myFile.print(" ,");
          myFile.print(struct_imu.acelerometro[1]);myFile.print(" ,");
          myFile.print(struct_imu.acelerometro[2]);myFile.print(" ,");
          myFile.print(struct_imu.giroscopio[0]);myFile.print(" ,");
          myFile.print(struct_imu.giroscopio[1]);myFile.print(" ,");
          myFile.print(struct_imu.giroscopio[2]);myFile.print(" ,");
          myFile.print(struct_imu.magnetometro[0]);myFile.print(" ,");
          myFile.print(struct_imu.magnetometro[1]);myFile.print(" ,");
          myFile.println(struct_imu.magnetometro[2]);
     }
     digitalWrite(SdRecording, HIGH);
     myFile.close();
  }
   
  //Code for serial debugging
  if (DebugSerial) {
      Serial.print(millis());Serial.print(" ,");
      Serial.print(struct_imu.barometro[0]);Serial.print(" ,");
      Serial.print(struct_imu.barometro[1]);Serial.print(" ,");
      Serial.print(struct_imu.barometro[2]);Serial.print(" ,");
      Serial.print(struct_imu.acelerometro[0]);Serial.print(" ,");
      Serial.print(struct_imu.acelerometro[1]);Serial.print(" ,");
      Serial.print(struct_imu.acelerometro[2]);Serial.print(" ,");
      Serial.print(struct_imu.giroscopio[0]);Serial.print(" ,");
      Serial.print(struct_imu.giroscopio[1]);Serial.print(" ,");
      Serial.print(struct_imu.giroscopio[2]);Serial.print(" ,");
      Serial.print(struct_imu.magnetometro[0]);Serial.print(" ,");
      Serial.print(struct_imu.magnetometro[1]);Serial.print(" ,");
      Serial.println(struct_imu.magnetometro[2]);
  } 
  LastAltitude = struct_imu.barometro[2];
}
