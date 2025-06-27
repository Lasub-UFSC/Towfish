#include <SoftwareSerial.h>

class RS485 : public SoftwareSerial {
private:
    int dePin;
    char header;
    String receivedMessage;
    String lastSentMessage;
    const unsigned long timeout = 1000;

public:
    RS485(char defaultHeader = 'I') : SoftwareSerial(2, 3), dePin(4), header(defaultHeader), receivedMessage(""), lastSentMessage("") {
        pinMode(dePin, OUTPUT);
        digitalWrite(dePin, LOW);
        begin(57600);
    }

    void setHeader(char newHeader) {
        header = newHeader;
    }

    bool sendMessage(const String &message) {
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
        if (available()) {
            char receivedHeader = read();
            if (receivedHeader == header) {
                receivedMessage = "";
                while (available()) {
                    char c = read();
                    if (c == '\n') break;
                    receivedMessage += c;
                }
                sendAck();
                return receivedMessage;
            }
        }
        return "";
    }

    String getLastMessage() {
        return receivedMessage;
    }

    bool confirmTransmission() {
        delay(10);
        return available();
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
                    Serial.println(confirmation);
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
    Serial.begin(57600);
    Serial.println("RS485 Master started.");
}

void loop() { 
    // Handshake loop com timeout
    unsigned long handshakeStartTime = millis();
    handshakeCompleted = false;
    while (!handshakeCompleted) {
        if (rs485.sendMessage("HANDSHAKE")) {
            Serial.println("Handshake enviado e confirmado.");
            handshakeCompleted = true;
        } else {
            Serial.println("Falha no handshake. Tentando novamente...");
            delay(1000);
        }
        if (millis() - handshakeStartTime > 10000) {
            Serial.println("Timeout no handshake. Reiniciando...");
            return;
        }
    }

    // Loop principal para enviar mensagens
    unsigned long timer2 = millis();
    while (handshakeCompleted && (millis() - timer2 <= 10000)) {
        if (Serial.available()) {
            String message = Serial.readStringUntil('\n');
            if (rs485.sendMessage(message)) {
                Serial.println("Mensagem enviada com sucesso.");
                timer2 = millis();
            } else {
                Serial.println("Falha ao enviar mensagem.");
            }
        }

        String received = rs485.receiveMessage();
        if (received != "") {
            Serial.print("Mensagem recebida: ");
            Serial.println(received);
            timer2 = millis();
            if(received == "Erro de sincronizacao.\n"){
              break;
            }
        }
        delay(100); 
    }
}