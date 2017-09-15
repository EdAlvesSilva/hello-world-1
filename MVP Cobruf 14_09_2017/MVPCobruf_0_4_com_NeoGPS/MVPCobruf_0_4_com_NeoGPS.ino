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
 */

#include <GY80IMU.h> //Include the library GY80IMU
#include <SPI.h>
#include <SD.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <RH_RF95.h>
#include <EEPROM.h>


#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 5
#define EEPROMADDRESS 0
// Frequência do LoRa! [COUBRUF 2017: USAR 915.0]
#define RF95_FREQ 915.0

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
bool DebugSDSerial = 0; //Prints the values stored in the structure IMU_s

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

// Criacao do objeto da classe RH_RF95
RH_RF95 rf95(RFM95_CS, RFM95_INT);




void setup() {
  DEBUG_PORT.begin(9600);
  gpsPort.begin(9600);
  while (!Serial)
    ;
  DEBUG_PORT.print( F("NMEAsimple.INO: started\n") );
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

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  delay(100);
  
  // Reinicialização Manual
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  //while (!rf95.init()) { 
    //loop eterno enquanto LoRa nao inicializa.[ COBRUF 2017] : NAO UTILIZAR ISSO! 
   // }
  // Apos reinicializacao, eh necessesario setar o valor da frequencia
  rf95.setFrequency(RF95_FREQ);
  
  //Definindo a potência da transmissao para 23dBm
  rf95.setTxPower(23, false);
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
}

  //Verifica o numero do pacote que sera enviado 
  //Assim, eh possivel garantir que o numero do pacote
  //enviado sera sempre o numero certo
  long packetnum = EEPROMReadlong();

  
  unsigned char radiopacket[20] = "ROCKETS!!! #      ";
  itoa(packetnum++, radiopacket+13, 10);
  radiopacket[19] = 0;
  
  //rf95.send((uint8_t *)radiopacket, 20);


  //rf95.waitPacketSent();
  // Espera o pacote ser enviando
  
  //Escreve na EEPROM o ultimo pacote enviado
  EEPROMWritelong((packetnum));
  delay(100);
   
}

void EEPROMWritelong(long value)
      {

      //Escreve os 4 bytes no endereco de referencia + os 3 consecutivos
      EEPROM.write(EEPROMADDRESS, value & 0xFF);//four
      EEPROM.write(EEPROMADDRESS + 1, (value >> 8) & 0xFF); //three
      EEPROM.write(EEPROMADDRESS + 2, (value >> 16) & 0xFF); //two
      EEPROM.write(EEPROMADDRESS + 3, (value >> 24) & 0xFF); //one
      }

long EEPROMReadlong()
      {
      //Le o endereco + os 3 consecutivos
      return ((EEPROM.read(EEPROMADDRESS) << 0) & 0xFF) + (EEPROM.read(EEPROMADDRESS + 1 << 8) & 0xFFFF) + ((long(EEPROM.read(EEPROMADDRESS + 2)) << 16) & 0xFFFFFF) + ((long(EEPROM.read(EEPROMADDRESS + 3)) << 24) & 0xFFFFFFFF);
      }

