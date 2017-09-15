Esse arquivo ReadMe explica o que cada código faz :

MVPCobruf_0_1 : Esse é o código do MVP Curitiba:
		GPS -> Funciona, via SoftwareSerial usando a biblioteca TinyGPS;
		Variáveis do GPS -> Latitude, Longitude, Altitude, Satellites, Speed;
		IMU -> Funciona, via biblioteca GY80IMU;
		SD -> Funciona, via biblioteca SD.h do arduino;
		Consumo de SRAM : 69%
		Pode debugar via SerialMonitor

MVPCobruf_0_1_com_NeoGPS : Esse é o código do MVP Curitiba com a NeoGPS no lugar da TinyGPS :
		GPS -> Funciona, via HardwareSerial, usando a biblioteca NeoGPS;
		Variáveis do GPS -> Latitude, Longitude, Altitude;
		IMU -> Funciona, via biblioteca GY80IMU;
		SD -> Funciona, via biblioteca SD.h do arduino;
		Consumo de SRAM : 60%
		NAO pode debugar via SerialMonitor, apenas as variáveis do GY80IMU são debugáveis and quando está debugando os dados do GPS são corrompidas

MVPCobruf_0_2 : Esse código é o MVPCobruf_0_1 com o código do LoRa:
		Nada funciona, o arduino dá crash com a falta de memória.
		Consumo de SRAM : 89%
	        Pode debugar via SerialMonitor

MVPCobruf_0_4_com_NeoGPS : Esse código é o MVPCobruf_0_2 com o NeoGPS no lugar da TinyGPS:
		Arduino não dá crash mas nao consegue guardar nenhum dado no SD e nem receber sinal do GPS
		Consumo de SRAM : 80%
		NAO pode debugar via SerialMonitor, apenas as variáveis do GY80IMU são debugáveis and quando está debugando os dados do GPS são corrompidas

