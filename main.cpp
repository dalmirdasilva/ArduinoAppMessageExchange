#include <Arduino.h>
#include <SerialInputStream.h>
#include <SerialOutputStream.h>
#include <Message.h>
#include <MessageParser.h>
#include <RemoteDevice.h>
#include <Notifier.h>
#include <SerialRadioFrequencyBluetooth.h>
#include <SoftwareSerialInputStream.h>
#include <SoftwareSerialOutputStream.h>

#define CONNECTION_TIMEOUT      1000
#define CONNECTION_CHECK_INTERVAL   10000
#define BUFF_SIZE               64
#define TRANSMITING_LED_PIN     13
#define WAITING_LED_PIN         11

unsigned char outgoingMessagePayload[BUFF_SIZE];
unsigned char incomingMessagePayload[BUFF_SIZE];
unsigned char parserBuffer[BUFF_SIZE];

unsigned long lastConnCheck = 0;

SerialRadioFrequencyBluetooth serial(2, 3);

SoftwareSerialInputStream is = SoftwareSerialInputStream(&serial);
SoftwareSerialOutputStream os = SoftwareSerialOutputStream(&serial);
Message outgoingMessage = Message(outgoingMessagePayload);
Message incomingMessage = Message(incomingMessagePayload);
MessageParser parser = MessageParser(parserBuffer, BUFF_SIZE);
RemoteDevice device = RemoteDevice(&parser, &incomingMessage, &os, &is);

void processReceivedMessage() {
    unsigned char raw[BUFF_SIZE];
    unsigned int length = incomingMessage.toRaw(raw);
    for (unsigned int i = 0; i < length; i++) {
        Serial.println(raw[i], HEX);
    }
    if (incomingMessage.getType() == Message::PING) {
        Serial.println("received ping, ponging...");
        outgoingMessage.setType(Message::PONG);
        outgoingMessage.setPayloadSize(0);
        device.transmitMessage(&outgoingMessage);
    }
}

bool receiveMessage() {
    if (device.receiveMessage(&incomingMessage)) {
        processReceivedMessage();
        return true;
    }
    return false;
}

void checkConnection() {
    unsigned long now = millis();
    if (lastConnCheck + CONNECTION_CHECK_INTERVAL < now || now < lastConnCheck) {
        if (parser.isReceivingMessage()) {
            Serial.println("Waiting while a message is fully received...");
            while (!receiveMessage())
                ;
        }
        if (!device.isConnected()) {
            Serial.println("Connecting...");
            while (!device.connect())
                ;
        }
        lastConnCheck = millis();
    }
}

void setup() {
    Serial.begin(9600);
    Serial.println("Initializing...");
    Notifier::setup(TRANSMITING_LED_PIN, WAITING_LED_PIN);
    Serial.println("Done");
    serial.begin(9600);
}

void loop() {
    checkConnection();
    receiveMessage();
}
