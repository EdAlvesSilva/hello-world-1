/*  Code written by Marcelo Maroñas @ Minerva Rockets (Federal University of Rio de Janeiro Rocketry Team) - September 6, 2017
 *  This is a basic example that uses the function Get_IMU to store the GY-80 variables in a structure.
 *  You can choose to print values to debug and test in the serial monitor.
 *  The data is printed in a CSV way, so you can copy and paste the serial monitor info into a notepad file and save as a CSV that can be opened in Excel or other CSV softwares.
 *  The structure IMU_s is given by :
 *      IMU_s->double acelerometro[2]; Where positions 0, 1 and 2 in the array are acelerometer x, y and z values respectively, in m/s².
 *      IMU_s->int magnetometro[2]; Where positions 0, 1 and 2 in the array are magnetic field x, y and z values respectively, in vector form.
 *      IMU_s->int giroscopio[2]; Where positions 0, 1 and 2 in the array are gyroscope x, y and z values respectively, in angular acceleration.
 *      IMU_s->double barometro[2]; Where positions 0, 1 and 2 in the array are pressure(in Pa), altitude(in Meters) and temperature(in Celsius) respectively.    
 *  Contact : marcelomaronas at poli.ufrj.br
 *  For more codes : github.com/engmaronas
 */

/* GY-80 Pins
 *  Vcc_In <----------------------> Arduino 5v
 *  Gnd    <----------------------> Arduino Gnd
 *  SDA    <----------------------> A4
 *  SCL    <----------------------> A5
 *  
 * GPS Pins
 *  Vcc    <----------------------> Arduino 5v  
 *  Gnd    <----------------------> Arduino Gnd
 *  TX     <----------------------> Arduino D3
 *  RX     <----------------------> Arduino D7
 */

#include <GY80IMU.h> //Include the library GY80IMU
#include <SPI.h>
#include <SD.h>
#include <TinyGPS_Struct.h>
#include <SoftwareSerial.h>


File myFile;
String filename = "DADOS0.CSV";
int numArquivo = 0;

//Structure declaration
  IMU_s IMUs1; 
  IMU_s *pIMUs1 = &IMUs1;

  GPS_s GPSs1;
  GPS_s *pGPSs1 = &GPSs1;

SoftwareSerial ss(3, 7);

//Use this variables to enable debugging via Serial Monitor
bool DebugSerial = 0;  //Prints the GY80 values from inside the "Get_IMU()" function
bool DebugSDSerial = 1; //Prints the values stored in the structure IMU_s

//Modify the Delay_Time variable to control how much info is printed on the serial monitor
float Delay_Time = 0;

//Free ram function Init
int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
//Free ram function End



void setup() {
  Serial.begin(115200); //Starts the serial port at 115200 baud rate
  ss.begin(9600);
  Wire.begin();

  //
  Serial.println(F("GY-80 IMU library "));
  Serial.println(F("by Marcelo Maronas"));
  Serial.println();
  //

  Serial.println(F("sep =, ")); //This line handles Excel CSV configuration.
  Serial.println(F("Time, Pressure, Altitude, Temperature, AcelX, AcelY, AcelZ, GyroX, GyroY, GyroZ, MagnetoX, MagnetoY, MagnetoZ, GPS Lat, GPS Lon, Satellites, GPS Altitude, GPS Speed, Free Ram"));

  if (!SD.begin(10)) {
    Serial.println(F("initialization failed!"));
    return;
  }
  Serial.println(F("initialization done."));

  for (int ff = 0; ff < 100; ff++){
      numArquivo++;
      filename = "DADOS" + String(numArquivo, DEC) + ".CSV";
      if (!SD.exists(filename)){
        break;
      }
   }

   myFile = SD.open(filename, FILE_WRITE);
  
  if (myFile) {
  myFile.println(F("sep =, ")); //This line handles Excel CSV configuration.
  myFile.println(F("Time, Pressure, Altitude, Temperature, AcelX, AcelY, AcelZ, GyroX, GyroY, GyroZ, MagnetoX, MagnetoY, MagnetoZ, GPS Lat, GPS Lon, Satellites, GPS Altitude, GPS Speed, Free Ram"));
  myFile.close();
   } else {
    // if the file didn't open, print an error:
    Serial.println(F("error opening test.txt"));
  }
}

void loop() {  
  //Stores the GY80 values into the pIMUs1 pointer
   Get_IMU(pIMUs1, DebugSerial);
   Get_GPS(pGPSs1, ss);

   myFile = SD.open(filename, FILE_WRITE);
  //Delay time
   delay(Delay_Time);

     // if the file opened okay, write to it:
  if (myFile) {
    myFile.print(millis());myFile.print(F(" ,"));
    myFile.print(pIMUs1->barometro[0]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->barometro[1]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->barometro[2]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->acelerometro[0]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->acelerometro[1]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->acelerometro[2]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->giroscopio[0]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->giroscopio[1]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->giroscopio[2]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->magnetometro[0]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->magnetometro[1]);myFile.print(F(" ,"));
    myFile.print(pIMUs1->magnetometro[2]);myFile.print(F(" ,"));
    myFile.print(pGPSs1->flat, 6);myFile.print(F(" ,"));
    myFile.print(pGPSs1->flon, 6);myFile.print(F(" ,"));
    myFile.print(pGPSs1->sat);myFile.print(F(" ,"));
    myFile.print(pGPSs1->height);myFile.print(F(" ,"));
    myFile.print(pGPSs1->fspeed);myFile.print(F(" ,"));
    myFile.println(freeRam());
    myFile.close();
    Serial.println(F("Dados gravados"));
  }
  else {
    // if the file didn't open, print an error:
    Serial.println(F("error opening test.txt"));
  }
   
  //if DebugSDSerial = 1, write into the Serial Monitor
  if (DebugSDSerial) {
    Serial.print(millis());Serial.print(F(" ,"));
    Serial.print(pIMUs1->barometro[0]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->barometro[1]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->barometro[2]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->acelerometro[0]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->acelerometro[1]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->acelerometro[2]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->giroscopio[0]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->giroscopio[1]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->giroscopio[2]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->magnetometro[0]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->magnetometro[1]);Serial.print(F(" ,"));
    Serial.print(pIMUs1->magnetometro[2]);Serial.print(F(" ,"));
    Serial.print(pGPSs1->flat, 6);Serial.print(F(" ,"));
    Serial.print(pGPSs1->flon, 6);Serial.print(F(" ,"));
    Serial.print(pGPSs1->sat);Serial.print(F(" ,"));
    Serial.print(pGPSs1->height);Serial.print(F(" ,"));
    Serial.print(pGPSs1->fspeed);Serial.print(F(" ,"));
    Serial.println(freeRam());
}
}
