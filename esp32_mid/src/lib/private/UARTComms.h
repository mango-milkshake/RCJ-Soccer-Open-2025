#ifndef UARTCOMMS_H
#define UARTCOMMS_H

#include <Arduino.h>

class UARTComms {
    public:
        UARTComms(uint8_t tx, uint8_t rx, byte* buffer, int length, HardwareSerial &uart) : 
            _tx(tx), _rx(rx), _buffer(buffer), _length(length), _uart(uart){
        }

        void init(){
            _uart.begin(115200, SERIAL_8N1, _rx, _tx);
        }

        bool uartRead(byte firstbyte){
            if(_uart.available()>=_length){
                while(_uart.available()>=_length && _uart.peek()!=firstbyte){
                    Serial.printf("First byte not %d\n", firstbyte);
                    _uart.read();
                }
                int readLength = _uart.readBytes(_buffer, _length);
                if(readLength!=_length || _buffer[0]!=firstbyte){
                    Serial.print("Received bad data: length: ");
                    Serial.print(readLength);
                    Serial.print(", data: ");
                    for (int i=0; i<_length; i++) Serial.printf("%d ", _buffer[i]);
                    Serial.println();
                    return false;
                }
                else return true; // read data successfully
            }
            else return false;
        }

        void uartWrite(){
            for (int i=0; i<_length; i++) _uart.write(_buffer[i]);
        }

    private:
        const uint8_t _tx, _rx;
        HardwareSerial &_uart;
        byte* _buffer;
        int _length;
};

#endif