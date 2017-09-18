This ReadMe file explains what each code does :

MVPCobruf_0_1 : This code is the MVP Curitiba :
		GPS -> Works, via SoftwareSerial using TinyGPS library;
		GPS Variables -> Latitude, Longitude, Altitude, Satellites, Speed;
		IMU -> Works, via GY80IMU library;
		SD -> Works, via SD.h arduino library;
		SRAM use : 69%
		Can debug via SerialMonitor

MVPCobruf_0_1_com_NeoGPS : This code is the MVP Curitiba with NeoGPS in the place of TinyGPS :
		GPS -> Works, via HardwareSerial, using NeoGPS library;
		GPS Variables -> Latitude, Longitude, Altitude;
		IMU -> Works, via GY80IMU library;
		SD -> Works, via SD.h arduino library;
		SRAM use : 60%
		CANNOT debug via SerialMonitor, only GY80IMU variables are debugable and when debugging the GPS data is corrupted

MVPCobruf_0_2 : This code is the MVPCobruf_0_1 code with LoRa code:
		Nothing works, the arduino crashes with the lack of memory.
		SRAM Use : 89%
		Can debug via SerialMonitor

MVPCobruf_0_4_com_NeoGPS : This code is the MVPCobruf_0_2 code with NeoGPS in the place of TinyGPS:
		Arduino does not crash but cant get any data stored in SD or GPS signal.
		SRAM Use : 80%
		CANNOT debug via SerialMonitor, only GY80IMU variables are debugable and when debugging the GPS data is corrupted

MVPCobruf_0_6_GPS_FAT16_LoRa : This code is the MVPCobruf_0_5 (not yet in this repo) with LoRa code
		HAS TO BE VALIDATED YET
		SRAM Use : 67%
		CANNOT debug via SerialMonitor, only GY80IMU variables are debugable and when debugging the GPS data is corrupted
			

