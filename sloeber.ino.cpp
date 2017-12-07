#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2017-12-07 16:18:47

#include "Arduino.h"
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <NewRemoteTransmitter.h>
#include <printf.h>
#include <RF24.h>
#include <stdint.h>
#include <WString.h>

void setup(void) ;
void loop(void) ;
void manageRestarts(int deviceID) ;
void updateDisplay(void) ;
void processMessage(void) ;
void resetDevice(int deviceID) ;
void goodLED(void) ;
void badLED(void) ;
void LEDsOff(void) ;
int equalID(char *receive_payload, const char *targetID) ;
void setPipes(uint8_t *writingPipe, uint8_t *readingPipe) ;
void printD(const char *message) ;
void printDWithVal(const char *message, int value) ;
void printD2Str(const char *str1, char *str2) ;
void powerCycle(int deviceID) ;
void beep(int numBeeps, int onDuration, int offDuration) ;

#include "WirelessWatchdog.ino"


#endif
