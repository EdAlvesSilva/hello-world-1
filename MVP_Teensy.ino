/*  Code written by Marcelo Maroñas, Eduardo Alves and Lucas Ribeiro @ Minerva Rockets (Federal University of Rio de Janeiro Rocketry Team) - February 20, 2018
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
#include <SoftwareSerial.h>
#include <RH_RF95.h>


#define RFM95_CS 15
#define RFM95_RST 17
#define RFM95_INT 16

#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

const int sentenceSize = 80;
bool gpsRead;
char sentence[sentenceSize];
char * field;
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
  Serial2.begin(9600); //Initialize GPS port at 9600 baudrate.
  pinMode(RecoveryLed, OUTPUT); 

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  delay(100);
  // Reinicialização Manual
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa nao inicializou");
    Serial.println("Realizando nova tentativa...");
  }
  Serial.println("LoRa inicializado!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

  rf95.setTxPower(23, false);

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


  static int i = 0;
  if (Serial2.available())
  {
    gpsRead = true;
    while (gpsRead) 
    {
      char ch = Serial2.read();
      if (ch != '\n' && i < sentenceSize)
      {
        sentence[i] = ch;
        i++;
      }
      else
      {
        sentence[i] = '\0';
        i = 0;
        gpsRead = false;
      }
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
          displayGPS();
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


  float lat, lon;
  getField(field,3);
  lat = String(field).toFloat();

  getField(field,5);
  lon = String(field).toFloat();

  char c_lat, c_long;

  getField(field,4);
  c_lat = field[0];
  
  getField(field,6);
  c_long = field[0];
  
  
  uint8_t radiopacket[130];
  radiopacket[0] = (uint8_t)'M';
  radiopacket[1] = (uint8_t)'R';
  radiopacket[2] = ((uint32_t)lat & 0x000000ff);
  radiopacket[3] = ((uint32_t)lat & 0x0000ff00) >> 8;
  radiopacket[4] = ((uint32_t)lat & 0x00ff0000) >> 16;
  radiopacket[5] = ((uint32_t)lat & 0xff000000) >> 24;
  radiopacket[6] = ((uint32_t)lon & 0x000000ff);
  radiopacket[7] = ((uint32_t)lon & 0x0000ff00) >> 8;
  radiopacket[8] = ((uint32_t)lon & 0x00ff0000) >> 16;
  radiopacket[9] = ((uint32_t)lon & 0xff000000) >> 24;
  radiopacket[10] = (uint8_t)c_lat;
  radiopacket[11] = (uint8_t)c_long;
  radiopacket[12] = (uint8_t)'O';
  radiopacket[13] = (uint8_t)'I';
  radiopacket[14] = ((uint32_t)struct_imu.barometro[0] & 0x000000ff);
  radiopacket[15] = ((uint32_t)struct_imu.barometro[0] & 0x0000ff00) >> 8;
  radiopacket[16] = ((uint32_t)struct_imu.barometro[0] & 0x00ff0000) >> 16;
  radiopacket[17] = ((uint32_t)struct_imu.barometro[0] & 0xff000000) >> 24;
  radiopacket[18] = ((uint32_t)struct_imu.barometro[1] & 0x000000ff);
  radiopacket[19] = ((uint32_t)struct_imu.barometro[1] & 0x0000ff00) >> 8;
  radiopacket[20] = ((uint32_t)struct_imu.barometro[1] & 0x00ff0000) >> 16;
  radiopacket[21] = ((uint32_t)struct_imu.barometro[1] & 0xff000000) >> 24;
  radiopacket[22] = ((uint32_t)struct_imu.barometro[2] & 0x000000ff);
  radiopacket[23] = ((uint32_t)struct_imu.barometro[2] & 0x0000ff00) >> 8;
  radiopacket[24] = ((uint32_t)struct_imu.barometro[2] & 0x00ff0000) >> 16;
  radiopacket[25] = ((uint32_t)struct_imu.barometro[2] & 0xff000000) >> 24;
  radiopacket[26] = ((uint32_t)struct_imu.barometro[3] & 0x000000ff);
  radiopacket[27] = ((uint32_t)struct_imu.barometro[3] & 0x0000ff00) >> 8;
  radiopacket[28] = ((uint32_t)struct_imu.barometro[3] & 0x00ff0000) >> 16;
  radiopacket[29] = ((uint32_t)struct_imu.barometro[3] & 0xff000000) >> 24;
  radiopacket[30] = ((uint32_t)struct_imu.acelerometro[0] & 0x000000ff);
  radiopacket[31] = ((uint32_t)struct_imu.acelerometro[0] & 0x0000ff00) >> 8;
  radiopacket[32] = ((uint32_t)struct_imu.acelerometro[0] & 0x00ff0000) >> 16;
  radiopacket[33] = ((uint32_t)struct_imu.acelerometro[0] & 0xff000000) >> 24;  
  radiopacket[34] = ((uint32_t)struct_imu.acelerometro[1] & 0x000000ff);
  radiopacket[35] = ((uint32_t)struct_imu.acelerometro[1] & 0x0000ff00) >> 8;
  radiopacket[36] = ((uint32_t)struct_imu.acelerometro[1] & 0x00ff0000) >> 16;
  radiopacket[37] = ((uint32_t)struct_imu.acelerometro[1] & 0xff000000) >> 24;  
  radiopacket[38] = ((uint32_t)struct_imu.acelerometro[2] & 0x000000ff);
  radiopacket[39] = ((uint32_t)struct_imu.acelerometro[2] & 0x0000ff00) >> 8;
  radiopacket[40] = ((uint32_t)struct_imu.acelerometro[2] & 0x00ff0000) >> 16;
  radiopacket[41] = ((uint32_t)struct_imu.acelerometro[2] & 0xff000000) >> 24; 
  radiopacket[42] = ((uint32_t)struct_imu.giroscopio[0] & 0x000000ff);
  radiopacket[43] = ((uint32_t)struct_imu.giroscopio[0] & 0x0000ff00) >> 8;
  radiopacket[44] = ((uint32_t)struct_imu.giroscopio[0] & 0x00ff0000) >> 16;
  radiopacket[45] = ((uint32_t)struct_imu.giroscopio[0] & 0xff000000) >> 24;  
  radiopacket[46] = ((uint32_t)struct_imu.giroscopio[1] & 0x000000ff);
  radiopacket[47] = ((uint32_t)struct_imu.giroscopio[1] & 0x0000ff00) >> 8;
  radiopacket[48] = ((uint32_t)struct_imu.giroscopio[1] & 0x00ff0000) >> 16;
  radiopacket[49] = ((uint32_t)struct_imu.giroscopio[1] & 0xff000000) >> 24;  
  radiopacket[50] = ((uint32_t)struct_imu.giroscopio[2] & 0x000000ff);
  radiopacket[51] = ((uint32_t)struct_imu.giroscopio[2] & 0x0000ff00) >> 8;
  radiopacket[52] = ((uint32_t)struct_imu.giroscopio[2] & 0x00ff0000) >> 16;
  radiopacket[53] = ((uint32_t)struct_imu.giroscopio[2] & 0xff000000) >> 24;  
  radiopacket[54] = ((uint32_t)struct_imu.magnetometro[0] & 0x000000ff);
  radiopacket[55] = ((uint32_t)struct_imu.magnetometro[0] & 0x0000ff00) >> 8;
  radiopacket[56] = ((uint32_t)struct_imu.magnetometro[0] & 0x00ff0000) >> 16;
  radiopacket[57] = ((uint32_t)struct_imu.magnetometro[0] & 0xff000000) >> 24;  
  radiopacket[58] = ((uint32_t)struct_imu.magnetometro[1] & 0x000000ff);
  radiopacket[59] = ((uint32_t)struct_imu.magnetometro[1] & 0x0000ff00) >> 8;
  radiopacket[60] = ((uint32_t)struct_imu.magnetometro[1] & 0x00ff0000) >> 16;
  radiopacket[61] = ((uint32_t)struct_imu.magnetometro[1] & 0xff000000) >> 24;  
  radiopacket[62] = ((uint32_t)struct_imu.magnetometro[2] & 0x000000ff);
  radiopacket[63] = ((uint32_t)struct_imu.magnetometro[2] & 0x0000ff00) >> 8;
  radiopacket[64] = ((uint32_t)struct_imu.magnetometro[2] & 0x00ff0000) >> 16;
  radiopacket[65] = ((uint32_t)struct_imu.magnetometro[2] & 0xff000000) >> 24;  
 //Dados seguintes devem ser inseridos da mesma forma
  
  rf95.send(radiopacket, sizeof(radiopacket));


  rf95.waitPacketSent();
  // Espera o pacote ser enviando
  
} 


void displayGPS()
{
  char field[20];
  getField(field, 0);
  
  if (strcmp(field, "$GPRMC") == 0)
  {
    myFile.print("Lat: ");
    getField(field, 3);  // number
    myFile.print(field);
    getField(field, 4); // N/S
    myFile.println(field);
    
    myFile.print("Long: ");
    getField(field, 5);  // number
    myFile.print(field);
    getField(field, 6);  // E/W
    myFile.println(field);
  }
}

void getField(char* buffer, int index)
{
  int sentencePos = 0;
  int fieldPos = 0;
  int commaCount = 0;
  while (sentencePos < sentenceSize)
  {
    if (sentence[sentencePos] == ',')
    {
      commaCount ++;
      sentencePos ++;
    }
    if (commaCount == index)
    {
      buffer[fieldPos] = sentence[sentencePos];
      fieldPos ++;
    }
    sentencePos ++;
  }
  buffer[fieldPos] = '\0';
} 
