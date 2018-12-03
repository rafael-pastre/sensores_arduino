// Software para leitura dos sensores BMP280 (temperatura e pressão) e NEO-6M (GPS)
// Para versão final é necessário alterar as funções de escrita para a comunição com a rasp
#include <Wire.h>                         // Biblioteca Padrão
#include <Adafruit_Sensor.h>              // Biblioteca Padrão com pesquisa
#include <Adafruit_BMP280.h>              // Biblioteca Padrão com pesquisa
#include <SoftwareSerial.h>               // Biblioteca Padrão
#include <TinyGPS.h>                      // Biblioteca Externa

#define PC_Serial_Baud 115200
#define pressao_nivel_mar 1013.25         // Pressao ao nível do mar, é passado como parâmetro na leitura de altitude, pode ser definido como variável a ser atribuida no início do programa se for mais conveiente
#define GPS_RX 4                          // O TX do GPS deve ser ligado neste pino
#define GPS_TX 3                          // O RX do GPS deve ser ligado neste pino
#define GPS_Serial_Baud 9600

// Ligacoes do BMP
// VCC e SDO devem ser conectados em 3V3
// GND deve ser conectado com GND
// SDA deve ser conectado na entrada analogica 4
// SCL deve ser conectado na entrada analogica 5
// VCC do GPS deve ser conectado em 5V

Adafruit_BMP280 sensor_bmp;               // Definicao do Sensor
SoftwareSerial gpsSerial(GPS_RX, GPS_TX); // Definicao da conexao Serial entre o GPS
TinyGPS gps;                              // Definicao do GPS

float temperatura_BMP;                    // Temperatura medida pelo BMP, em °C e com precisão 0.01
float pressao_BMP;                        // Pressao medida pelo BMP, em Pa e com precisão 0.01
float altitude_BMP;                       // Altitude medida pelo BMP, em m e com precisão 0.01

float latitude_GPS;                       // Latitude medida pelo GPS, em graus
float longitude_GPS;                      // Longitude medida pelo GPS, em graus
unsigned long idadeInfo_GPS;              // Idade da informacao do GPS, em ms
float altitude_GPS;                       // Altitude medida pelo GPS, em metros
float velocidade_GPS;                     // velocidade medida pelo GPS, em Km/h
unsigned long sentido_GPS;                // sentido medido pelo GPS, verificar unidade
unsigned short satelites_GPS;             // Número de satélites que enviaram informações
unsigned long precisao_GPS;               // Precisão das informações

int incomingByte = 0;                     // Leitura da transmissao serial com o computador

// Função de leitura das informações do BMP
void readBMP(){
  temperatura_BMP = sensor_bmp.readTemperature();
  pressao_BMP = sensor_bmp.readPressure();
  altitude_BMP = sensor_bmp.readAltitude(pressao_nivel_mar);
}

// Função de leitura das informações do GPS
void readGPS(){
  bool newData = false;
  unsigned long chars;
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;){
    while (gpsSerial.available()){
      char c = gpsSerial.read();
      if (gps.encode(c)) // Atribui true para newData caso novos dados sejam recebidos
        newData = true;
      }
    }
    if(newData){
      gps.f_get_position(&latitude_GPS, &longitude_GPS, &idadeInfo_GPS);
      altitude_GPS = gps.f_altitude();
      velocidade_GPS = gps.f_speed_kmph();
      sentido_GPS = gps.course();
      satelites_GPS = gps.satellites();
      precisao_GPS = gps.hdop();
    }
}

// Função de escrita para o computador das informações do BMP
void printBMP(){
  Serial.print("Temperatura = ");
  Serial.print(temperatura_BMP);
  Serial.println(" *C");
  
  Serial.print("Pressao = ");
  Serial.print(pressao_BMP);
  Serial.println(" Pa");
  
  Serial.print("Altitude aprox BMP = ");
  Serial.print(altitude_BMP);
  Serial.println(" m");
  
  Serial.println();
}

// Função de escrita para o computador das informações do GPS
void printGPS(){
 if(latitude_GPS != TinyGPS::GPS_INVALID_F_ANGLE){
  Serial.print("Latitude = ");
  Serial.print(latitude_GPS, 6);
  Serial.println(" graus");
 }
 else Serial.println("Latitude = Invalida");

 if(longitude_GPS != TinyGPS::GPS_INVALID_F_ANGLE){
  Serial.print("Longitude = ");
  Serial.print(longitude_GPS, 6);
  Serial.println(" graus");
 }
 else Serial.println("Longitude = Invalida");

 if(idadeInfo_GPS != TinyGPS::GPS_INVALID_AGE){
  Serial.print("Idade da Informacao = ");
  Serial.print(idadeInfo_GPS);
  Serial.println(" ms");
 }
 else Serial.println("Idade da Informacao = Invalida");
 
 if(altitude_GPS != TinyGPS::GPS_INVALID_ALTITUDE){
  Serial.print("Altitude GPS = ");
  Serial.print(altitude_GPS);
  Serial.println(" m");
 }
 else Serial.println("Altitude = Invalida");

 Serial.print("Velocidade = ");
 Serial.print(velocidade_GPS, 2);
 Serial.println(" Km/h");

 Serial.print("Sentido = ");
 Serial.print(sentido_GPS);
 Serial.println(" graus");

 if(satelites_GPS != TinyGPS::GPS_INVALID_SATELLITES){
  Serial.print("Satelites = ");
  Serial.println(satelites_GPS);
 }
 else Serial.println("Satelites = Invalido");

 if(satelites_GPS != TinyGPS::GPS_INVALID_SATELLITES){
  Serial.print("Precisao = ");
  Serial.println(precisao_GPS);
 }
 else Serial.println("Precisao = Invalida");
 
 Serial.println();

}

// Inicialização
void setup() {
  Serial.begin(PC_Serial_Baud);

  gpsSerial.begin(GPS_Serial_Baud);
  Serial.println("Sensor GPS encontrado! Digite 'G' para realizar uma leitura");
  
  //Verifica a conexão do sensor BMP280
  if (!sensor_bmp.begin())
    Serial.println("Sensor BMP não encontrado. Verifique as conexoes!");
  else
    Serial.println("Sensor BMP encontrado! Digite 'B' para realizar uma leitura");
}

// Loop do programa
void loop() {
  readBMP(); 
  readGPS();
  if(Serial.available() > 0){
    incomingByte = Serial.read();
    if(incomingByte == 'B')
      printBMP();
    if(incomingByte == 'G')
      printGPS();
   }
}
