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
 *  GPS Pins
 *  Vcc    <----------------------> Arduino 5v  
 *  Gnd    <----------------------> Arduino Gnd
 *  TX     <----------------------> Arduino RX0
 *  RX     <----------------------> Arduino TX1
 *  
 *  
 *  PLEASE, YOU GOT TO REMOVE THE GPS TX PIN WHILE UPLOADING A CODE, THE GPS RECEIVED DATA CAN
 *  CAUSE ERRORS IN UPLOADING CODES.
 *  
 *  AS THIS CODE USES HARDWARESERIAL FOR GPS, YOU CANT DEBUG GPS DATA INTO SERIAL PORT, IF YOU ENABLE DebugSDSerial YOU WILL LOSE GPS DATA
 *  
 *  THE GPS DATA INTO SD FILE WILL BE CORRECT IF YOU DONT DEBUG VIA SERIAL
 *  
 *  SO USE ANOTHER METHOD FOR DEBUGGING, I.E. TRANSMITTING THE DATA OR VIEWING IN SD CARD, OR USE THE MVPCobruf_0_1 THAT USES SOFTWARESERIAL AND CAN DEBUG VIA SERIAL PORT
 *  
 *
 */

#include <GY80IMU.h> //Include the library GY80IMU
#include <SPI.h>
#include <SD.h>
#include <NMEAGPS.h>
#include <GPSport.h>


File myFile;
String filename = "DADOS0.CSV";
int numArquivo = 0;

//Structure declaration
  IMU_s IMUs1; 
  IMU_s *pIMUs1 = &IMUs1;

  NMEAGPS  gps; // This parses the GPS characters
  gps_fix  fix; // This holds on to the latest values

//Use this variables to enable debugging via Serial Monitor
bool DebugSerial = 0;  //Prints the GY80 values from inside the "Get_IMU()" function
bool DebugSDSerial = 1; //Prints the values stored in the structure IMU_s.Can only debug IMU_s vairables, GPS cant receive data when debugging, because it uses the hardware serial too for receive data.

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
  Serial.begin(9600); //Starts the serial port at 115200 baud rate
  gpsPort.begin(9600);
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
   while (gps.available( gpsPort )) {
    fix = gps.read();

    DEBUG_PORT.print( F("Location: ") );
    if (fix.valid.location) {
      DEBUG_PORT.print( fix.latitude(), 6 );
      DEBUG_PORT.print( ',' );
      DEBUG_PORT.print( fix.longitude(), 6 );
    }

    DEBUG_PORT.print( F(", Altitude: ") );
    if (fix.valid.altitude)
      DEBUG_PORT.print( fix.altitude() );

    DEBUG_PORT.println();
  }

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
    myFile.print(fix.latitude(), 6);myFile.print(F(" ,"));
    myFile.print(fix.longitude(), 6);myFile.print(F(" ,"));
    myFile.print(F("Not available"));myFile.print(F(" ,"));
    myFile.print(fix.altitude(), 3);myFile.print(F(" ,"));
    myFile.print(F("Not available"));myFile.print(F(" ,"));
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
    Serial.print(fix.latitude(), 6);Serial.print(F(" ,"));
    Serial.print(fix.longitude(), 6);Serial.print(F(" ,"));
    Serial.print(F("Not available"));Serial.print(F(" ,"));
    Serial.print(fix.altitude(), 3);Serial.print(F(" ,"));
    Serial.print(F("Not available"));Serial.print(F(" ,"));
    Serial.println(freeRam());
    delay(500);
}
}
