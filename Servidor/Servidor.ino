#include <Wire.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>
#include "DHT.h"


#define DHTPIN 7
#define DHTTYPE DHT11      
DHT dht(DHTPIN, DHTTYPE);

#define pressure_sensor A3

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
uint32_t LoopTimer;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
float Kalman1DOutput[] = { 0, 0 };


const float KalmanGain = 0.000256; // 0.004 * 0.004 * 4 * 4

//Versão Simplificado do Filtro de Kalmann Unidimensional 
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  
  KalmanUncertainty = KalmanUncertainty + KalmanGain; //Esses ganhos podem ser adaptados dependendo do fabricante
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 9); //Esses ganhos podem ser adaptados dependendo do fabricante
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  //Serial.println(GyroX);
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096-0.05;//Calibração manual -> Valores ajustados por teste    |
  AccY=(float)AccYLSB/4096-0.03;//Calibração manual -> Valores ajustados por teste    |-> aqui da pra fazer uma manipulacao bit a bit maluca
  AccZ=(float)AccZLSB/4096-0.41;//Calibração manual -> Valores ajustados por teste    |
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(0.01746);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(0.01746);
}

class RS485 : public SoftwareSerial {
private:
  int dePin;
  char header;
  String receivedMessage;
  String lastSentMessage;
  const unsigned long timeout = 1000;
  unsigned long timer = 0;

public:
  RS485(char defaultHeader = 'I')
    : SoftwareSerial(2, 3), dePin(4), header(defaultHeader), receivedMessage(""), lastSentMessage("") {
    ////////////////////////////////////////////////
    //
    // Construtor da classe RS485
    //
    // params:
    // defaultHeader : char  Cabeçalho padrão das mensagens
    //
    pinMode(dePin, OUTPUT);
    digitalWrite(dePin, LOW);
    begin(57600);
  }

  void setHeader(char newHeader) {
    ////////////////////////////////////////////////
    //
    // Define um novo cabeçalho para as mensagens
    //
    // params:
    // newHeader : char  Novo cabeçalho a ser utilizado
    //
    header = newHeader;
  }

  bool sendMessage(const String &message) {
    ////////////////////////////////////////////////
    //
    // Envia uma mensagem via RS485
    //
    // params:
    // message : String  Mensagem a ser enviada
    //
    // return  : bool  Retorna verdadeiro se a confirmação ACK for recebida
    //
    lastSentMessage = message;
    digitalWrite(dePin, HIGH);
    write(header);
    write(message.c_str(), message.length());
    write('\n');
    flush();
    digitalWrite(dePin, LOW);
    return receiveACK();
  }

  String receiveMessage() {
    ////////////////////////////////////////////////
    //
    // Recebe uma mensagem via RS485
    //
    // return  : String  Retorna a mensagem recebida ou uma string vazia se nenhuma mensagem for válida
    //
    if (available()) {
      char receivedHeader = read();
      if (receivedHeader == header) {
        receivedMessage = "";
        while (available()) {
          char c = read();
          if (c == '\n') break;
          receivedMessage += c;
        }
        unsigned long dummy = sendAck();
        return receivedMessage;
      }
    }
    return "";
  }

  String getLastMessage() {
    ////////////////////////////////////////////////
    //
    // Retorna a última mensagem recebida
    //
    // return  : String  Última mensagem recebida
    //
    return receivedMessage;
  }

  bool confirmTransmission() {
    ////////////////////////////////////////////////
    //
    // Confirma se há transmissão disponível
    //
    // return  : bool  Retorna verdadeiro se houver dados disponíveis
    //
    delay(10);
    return available();
  }

  unsigned long getTimer() {
    ////////////////////////////////////////////////
    //
    // Obtém o tempo atual em milissegundos
    //
    // return  : unsigned long  Tempo atual em milissegundos
    //
    timer = millis();
    return timer;
  }

private:
  unsigned long sendAck() {
    ////////////////////////////////////////////////
    //
    // Envia uma mensagem de confirmação "ACK"
    //
    // return  : unsigned long  Tempo de envio do ACK
    //
    digitalWrite(dePin, HIGH);
    write("ACK\n");
    flush();
    digitalWrite(dePin, LOW);
    return millis();
  }

    bool receiveACK() {
        unsigned long startTime = millis();
        while (millis() - startTime < timeout) {
            if (available()) {
                String confirmation = readStringUntil('\n');
                if (confirmation == "ACK") {
                    Serial.println("Confirmação ACK recebida.");
                    return true;
                } else {
                    Serial.println("Erro: Confirmação incorreta.");
                    return false;
                }
            }
        }
        Serial.println("Erro: Tempo limite excedido. Confirmação não recebida.");
        return false;
    }
};

RS485 rs485;
bool handshakeCompleted = false;

void setup() {
  Serial.begin(115200);
  Serial.println("RS485 Slave started.");
  
  dht.begin();  
  // IMU
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;    //
  RateCalibrationPitch/=2000;   // -> tem problema dividir por 2048? Ai da pra fazer manipulação bit a bit aqui tbm
  RateCalibrationYaw/=2000;     //
  LoopTimer=micros();
  while (!handshakeCompleted) {
    String msg = rs485.receiveMessage();
    if (msg == "HANDSHAKE" || msg == "IHANDSHAKE") {
      Serial.println("Handshake recebido.");
      handshakeCompleted = true;
      Serial.println("Handshake concluído. Iniciando operações normais.");
    } else {
      Serial.println("Aguardando handshake...");
      delay(1000);
    }
  }
}


void loop() {
  ////////////////////////////////
  //
  // HANDSHAKE
  //
  unsigned long lastCommunicationTime = millis();
  while (millis() - lastCommunicationTime <= 5000) {
    String msg = rs485.receiveMessage();
    if (msg != "") {
      Serial.print("Mensagem recebida do Master: ");
      Serial.println(msg);
      lastCommunicationTime = millis();
      if (msg == "HANDSHAKE" || msg == "IHANDSHAKE") {
        Serial.println("Erro de sincronização.");
        //rs485.sendMessage("Erro de sincronizacao.");
      }
    }

    gyro_signals();
    String RateGyro = "RateRoll: " + (String)RateRoll + " RatePitch: " + (String)RatePitch + " RateYaw: " + (String)RateYaw;
    Serial.println(RateGyro);
    if(rs485.sendMessage(RateGyro)){
      lastCommunicationTime = millis();
    }
    
    String AccGyro = "AccX: " + (String)AccX + " AccY: " + (String)AccY + " AccZ: " + (String)AccZ; 
    if(rs485.sendMessage(AccGyro)){
      lastCommunicationTime = millis();
    }

    String Roll_Pitch = "AngleRoll: " + (String)AngleRoll + " AnglePitch: " + (String)AnglePitch; 
    if(rs485.sendMessage(Roll_Pitch)){
      lastCommunicationTime = millis();
    }

    int h = dht.readHumidity();
    String Humidity = "Humidity: " + (String) h;
    if(rs485.sendMessage(Humidity)){
      lastCommunicationTime = millis();
    }

    float p = analogRead(pressure_sensor);
    String pressure = "Pressure: " + (String)p;
    if(rs485.sendMessage(pressure)){
      lastCommunicationTime = millis();
    }
  }
  Serial.println("Timeout de recepção. Reiniciando handshake.");
  handshakeCompleted = false;
}