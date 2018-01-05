//#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <NewRemoteTransmitter.h>
#include <printf.h>
#include <RF24.h>
#include <stdint.h>
#include <WString.h>

#include <LedFader.h>

#include <U8g2lib.h>
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/
SCL, /* data=*/SDA);   // pin remapping with ESP8266 HW I2C

//#define OLD_HARDWARE
//#define NEW_HARDWARE

NewRemoteTransmitter transmitter(282830, 5, 254, 4);

// Set up nRF24L01 radio on SPI bus plus pins 7 & 8
RF24 radio(7, 8);
// sets the role of this unit in hardware.  Connect to GND to be the 'pong' receiver
// Leave open to be the 'ping' transmitter
//const int Tx433Mhz_pin = 5;
const int redLEDPin = 3;
const int greenLEDPin = 10;
const int blueLEDPin = 9;
//const int buzzer = 6;
const int switchPin = A7;     // the number of the pushbutton pin
const int analogue_switches = A0; // ip port A0

LedFader heartBeat(greenLEDPin, 0, 20, 900);
LedFader rebootAlert(redLEDPin, 0, 20, 456);
LedFader blueAlert(blueLEDPin, 0, 0, 1321);

uint8_t writePipeLocS[] = "NodeS";
uint8_t readPipeLocS[] = "Node0";

uint8_t writePipeLocC[] = "NodeC";
uint8_t readPipeLocC[] = "Node0";

uint8_t writePipeLocG[] = "NodeG";
uint8_t readPipeLocG[] = "Node0";

//
// Payload
const int max_payload_size = 32;
char receive_payload[max_payload_size + 1]; // +1 to allow room for a terminating NULL char

#ifdef OLD_HARDWARE
	static unsigned int goodSecsMax = 10; //20
	static unsigned long maxMillisNoAckFromPi = 1000UL * 30UL; //300 max millisces to wait if no ack from pi before power cycling pi
	static unsigned long waitForPiPowerUpMillis = 1000UL * 20UL; //120
#else
	static unsigned int goodSecsMax = 10; //20
	static unsigned long maxMillisNoAckFromPi = 1000UL * 300UL; //300 max millisces to wait if no ack from pi before power cycling pi
	static unsigned long waitForPiPowerUpMillis = 1000UL * 120UL; //120
#endif
//const unsigned long maxMillisNoAckFromPi = 1000UL * 30UL; // max millisces to wait if no ack from pi before power cycling pi
//const unsigned long waitForPiPowerUpMillis = 1000UL * 30UL;

//const int RxUnit = 15;
const int transmitEnable = 1;
//#define OLED_RESET 4
//Adafruit_SSD1306 display(OLED_RESET);

int switchState = 0;
int x = 0;

// define a struct for each controller instance with related vars
struct controller {
	uint8_t id_number;
	uint8_t zone;
	uint8_t socketID;
	unsigned long lastGoodAckMillis;
	char name[4];
	char heartBeatText[4]; // allow enough space for text plus 1 extra for null terminator
	char badStatusMess[10]; // enugh for two lines on display
	char goodStatusMess[10]; // enugh for two lines on display
	bool isRebooting; //indicate if booting up
	bool isPowerCycling; //indicate if power cycling
	unsigned long rebootMillisLeft;
	unsigned long lastRebootMillisLeftUpdate;
};

struct controller devices[4]; //create an array of 4 controller structures

void setup(void) {

	//populate the array of controller structs
	devices[0].id_number = 0;
	devices[0].zone = 1;
	devices[0].socketID = 14;
	devices[0].lastGoodAckMillis = millis();
	strcpy(devices[0].name, "GRG");
	strcpy(devices[0].heartBeatText, "GGG");
	strcpy(devices[0].badStatusMess, "Away: ");
	strcpy(devices[0].goodStatusMess, "OK");
	devices[0].isRebooting = 0;
	devices[0].isPowerCycling = 0;
	devices[0].rebootMillisLeft = 0;
	devices[0].lastRebootMillisLeftUpdate = millis();

	devices[1].id_number = 1;
	devices[1].zone = 2;
	devices[1].socketID = 4;
	devices[1].lastGoodAckMillis = millis();
	strcpy(devices[1].name, "CNV");
	strcpy(devices[1].heartBeatText, "CCC");
	strcpy(devices[1].badStatusMess, "Away: ");
	strcpy(devices[1].goodStatusMess, "OK");
	devices[1].isRebooting = 0;
	devices[1].isPowerCycling = 0;
	devices[1].rebootMillisLeft = 0;
	devices[1].lastRebootMillisLeftUpdate = millis();

	devices[2].id_number = 2;
	devices[2].zone = 3;
	devices[2].socketID = 15;
	devices[2].lastGoodAckMillis = millis();
	strcpy(devices[2].name, "SHD");
	strcpy(devices[2].heartBeatText, "SSS");
	strcpy(devices[2].badStatusMess, "Away: ");
	strcpy(devices[2].goodStatusMess, "OK");
	devices[2].isRebooting = 0;
	devices[2].isPowerCycling = 0;
	devices[2].rebootMillisLeft = 0;
	devices[2].lastRebootMillisLeftUpdate = millis();

	//setup leds
	pinMode(redLEDPin, OUTPUT);
	pinMode(greenLEDPin, OUTPUT);
	pinMode(blueLEDPin, OUTPUT);
	badLED();
	delay(100);
	goodLED();
	//digitalWrite(redLEDPin, LOW);
	// initialize the pushbutton pin as an input:

	//pinMode(switchPin, INPUT);
	//digitalWrite(switchPin, INPUT_PULLUP);

	heartBeat.begin();
	rebootAlert.begin();
	rebootAlert.off();

	blueAlert.begin();
	//beep(1, 2, 1);

	Serial.begin(115200);
	printf_begin();

	radio.begin();

	// enable dynamic payloads
	radio.enableDynamicPayloads();

	// optionally, increase the delay between retries & # of retries
	//radio.setRetries(15, 15);
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);
	radio.setChannel(124);
	radio.startListening();
	radio.printDetails();
	// autoACK enabled by default

	u8g2.begin();

	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_t0_15_tf);
	//u8g2.setFont(u8g2_font_7x14_tf);
	//u8g2.setFont(u8g2_font_7x14_tf);
	//u8g2.setFont(u8g2_font_8x13B_tf);
	//u8g2.setFont(u8g2_font_7x14_mr);

	// 17 chars by 3 lines at this font size
	u8g2.drawStr(30, 10, "Wireless");
	u8g2.drawStr(30, 21, "Watchdog");
	u8g2.drawStr(45, 32, "V1.1");

	//u8g2.setFont(u8g2_font_6x13_tf);
	//u8g2.setCursor(0, 21);
	//u8g2.print("m");

	//u8g2.setCursor(0, 32);
	//u8g2.print("A");

	u8g2.sendBuffer();

	//display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
	// init done
//	display.clearDisplay();
//	display.setTextSize(2);
//	display.setTextColor(WHITE);
//	display.setCursor(0, 0);
//	display.println("Wireless  Watchdog 1");
//	display.display();
	delay(5000);
//	display.clearDisplay();
//	display.display();
//
	setPipes(writePipeLocC, readPipeLocC); // SHOULD NEVER NEED TO CHANGE PIPES
	radio.startListening();
}

void loop(void) {

	while (radio.available()) {            // Read all available payloads
		//Serial.println(F("Processing next available message.. "));
		processMessage();
	}

	x = analogRead(analogue_switches);
	//Serial.println((x));
	displayKeys(x);

	updateDisplay(); //rotate messages etc if time to
	heartBeat.update();
	rebootAlert.update();
	blueAlert.update();
	// check each device if restart reqd
	manageRestarts(0);

	//pinMode(switchPin, INPUT);
	switchState = analogRead(switchPin);
	//switchState = 1;

	// check if the pushbutton is pressed. If it is, the buttonState is HIGH:
#ifdef OLD_HARDWARE
	switchState = 1024;
#endif
	if (switchState > 512) {
		strcpy(devices[1].goodStatusMess, "OK");
		manageRestarts(1);
	} else {
		strcpy(devices[1].goodStatusMess, "Disabled");
		resetDevice(1);
	}
	manageRestarts(2);
}

int freeRam() {
	extern int __heap_start, *__brkval;
	int v;

	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void printFreeRam(void) {
	Serial.print(F("FR: "));
	Serial.println(freeRam());
}

void displayKeys(int x) {
	/*	if (x < 60) {
	 printDWithVal("L", x);
	 } else if (x < 200) {
	 printDWithVal("U", x);
	 } else if (x < 400) {
	 printDWithVal("D", x);
	 } else if (x < 600) {
	 printDWithVal("R", x);
	 } else if (x < 800) {
	 printDWithVal("K", x);
	 }*/
}

//check a unit and see if restart reqd
void manageRestarts(int deviceID) {
	// now check if need to reboot a device
	if ((millis() - devices[deviceID].lastGoodAckMillis)
			> maxMillisNoAckFromPi) { // over time limit so reboot first time in then just upadte time each other time

		//printFreeRam();
		if (devices[deviceID].isRebooting == 0) { // all this done first time triggered
			//printD("Reboot : ");

			devices[deviceID].isRebooting = 1; //signal device is rebooting

			devices[deviceID].isPowerCycling = 1; // signal in power cycle

			powerCycle(deviceID);

			devices[deviceID].isPowerCycling = 0; // signal in power cycle

			devices[deviceID].rebootMillisLeft = waitForPiPowerUpMillis;
			devices[deviceID].lastRebootMillisLeftUpdate = millis();

			Serial.println("triggered reboot in manage reboots");

			Serial.println(devices[deviceID].rebootMillisLeft);
			//delay(1000);
		} else { // this executes till end of reboot timer
				 //device is rebooting now - do some stuff to update countdown timers
				 //wait for pi to come back up - do nothing
				 //millis since last update
			unsigned long millisLapsed = millis()
					- devices[deviceID].lastRebootMillisLeftUpdate;

			// next subtraction will take us to/over limit
			if (millisLapsed >= devices[deviceID].rebootMillisLeft) {
				//zero or neg reached
				devices[deviceID].rebootMillisLeft = 0;
				//timerDone = 1;
			} else { // ok to do timer subtraction

				devices[deviceID].rebootMillisLeft =
						devices[deviceID].rebootMillisLeft - millisLapsed;

			}
			devices[deviceID].lastRebootMillisLeftUpdate = millis();

			// calc if next time subtraction takes it below zero
			//cant get a neg number from unsigned numbers used
			if (devices[deviceID].rebootMillisLeft == 0) { // reboot stuff completed here
				//if (timerDone == 1) { // reboot stuff completed here
				devices[deviceID].lastGoodAckMillis = millis();
				Serial.print(F("Assume Pi back up:"));
				Serial.println(deviceID);
				//printD("Assume pi back up");
				//printD2Str("Assume up:", devices[deviceID].name);
				devices[deviceID].isRebooting = 0; //signal device has stopped rebooting
			}
		}
	}
}

void updateDisplay(void) {
	//all three lines can be displayed at once
	//so no timer needed
	//check if time to display a new message updates

	int stateCounter; // only initialised once at start
	static unsigned long lastDispUpdateTimeMillis = 0;
	int dispUpdateFreq = 2; // how many updates per sec
	static unsigned long dispUpdateInterval = 1000/dispUpdateFreq;

	unsigned long secsSinceAck = 0;
	// max secs out considered good

	char str_output[20] = { 0 };
	unsigned long secsLeft;
	//Serial.println(stateCounter);
	// this loop
	//create info string for each zone then display it
	//setup disp
	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_7x14_tf);

	if ((millis() - lastDispUpdateTimeMillis) >= dispUpdateInterval) { //ready to update?
		printFreeRam();
		for (stateCounter = 0; stateCounter < 3; stateCounter++) {
			//Serial.println(stateCounter);
			secsSinceAck = (millis() - devices[stateCounter].lastGoodAckMillis)
					/ 1000;

			u8g2.setCursor(0, ((stateCounter + 1) * 10) + (1 * stateCounter));
			// make sure check for restarting device
			//if so display current secs in wait for reboot cycle
			if (devices[stateCounter].isRebooting) {
				secsLeft = (devices[stateCounter].rebootMillisLeft) / 1000UL;

				Serial.print(F("--rebootMillisLeft: "));
				Serial.println((devices[stateCounter].rebootMillisLeft));

				Serial.print(F("--secsLeft var: "));
				Serial.println(secsLeft);

				//build string to show if cycling or coming back up
				//char str_output[20] = { 0 }; //, str_two[]="two";
				//start with device name
				strcpy(str_output, devices[stateCounter].name);
				strcat(str_output,": ");
				//char message[] = " Reboot: ";
				if (devices[stateCounter].isPowerCycling) {
					strcat(str_output, "Power Cycling");
					printD(str_output);
					//secsLeft = '';
				} else {
					strcat(str_output, "Reboot: ");
					printDWithVal(str_output, secsLeft);

				}
				//strcat(str_output, message);

				//strcat(str_output, "Left: ");

				//u8g2.print("r");

				//u8g2.clearBuffer();

			} else if ((secsSinceAck > goodSecsMax)) {

				//u8g2.print("w");
				strcpy(str_output, devices[stateCounter].name);
				strcat(str_output,": ");
				strcat(str_output,devices[stateCounter].badStatusMess);
				printDWithVal(str_output, secsSinceAck);
				//badLED();
				LEDsOff();
			} else {
				//u8g2.print("u");
				strcpy(str_output, devices[stateCounter].name);
				strcat(str_output,": ");
				strcat(str_output,devices[stateCounter].goodStatusMess);
				printD(str_output);
				goodLED();
			}

		}
		//now print out message line at correct cursor loc
		//move cursor tonew loc
		lastDispUpdateTimeMillis = millis();
		u8g2.sendBuffer();
	}
	//now send buffer to display

}

void processMessage(void) {
	// radio is available so read in payload and see who it's from
	//and process accordingly

	//Serial.println(F(" - "));
//	Serial.println((millis() - lastGoodAckMillis[unit]));

	// Grab the message and process
	uint8_t len = radio.getDynamicPayloadSize();

	// If a corrupt dynamic payload is received, it will be flushed
	if (!len) {
		return;
	}

	radio.read(receive_payload, len);

	// Put a zero at the end for easy printing
	receive_payload[len] = 0;

	// Spew it
	//Serial.print(F("Got message: "));
	//Serial.print(len);
	//Serial.print(F(" pre use value="));
	//Serial.println(receive_payload);

	//who was it from?
	//reset that timer
	if (equalID(receive_payload, devices[0].heartBeatText)) {
		resetDevice(0);
		Serial.println(F("RESET GGG"));
	} else if (equalID(receive_payload, devices[1].heartBeatText)) {
		resetDevice(1);
		//devices[1].lastGoodAckMillis = millis();
		Serial.println(F("RESET CCC"));
	} else if (equalID(receive_payload, devices[2].heartBeatText)) {
		resetDevice(2);
		//devices[2].lastGoodAckMillis = millis();
		Serial.println(F("RESET SSS"));
	} else {
		Serial.println(F("NO MATCH"));
		//badLED();
		LEDsOff();
	}

}

void resetDevice(int deviceID) {
	devices[deviceID].lastGoodAckMillis = millis();
	devices[deviceID].isRebooting = 0;
	devices[deviceID].rebootMillisLeft = 0;
}
void goodLED(void) {
	//switch off red led
	heartBeat.update();
//	digitalWrite(redLEDPin, LOW);
//	digitalWrite(greenLEDPin, HIGH);
}
void badLED(void) {
	//switch off red led
	rebootAlert.on();
	heartBeat.update();
//	digitalWrite(redLEDPin, HIGH);
//	digitalWrite(greenLEDPin, LOW);
}

void LEDsOff(void) {
	//switch off red led
	heartBeat.update();
//	digitalWrite(redLEDPin, LOW);
	//digitalWrite(greenLEDPin, LOW);
}

int equalID(char *receive_payload, const char *targetID) {
	//check if same 1st 3 chars
	if ((receive_payload[0] == targetID[0])
			&& (receive_payload[1] == targetID[1])
			&& (receive_payload[2] == targetID[2])) {
		return 1;
	} else {
		return 0;
	}
}

void setPipes(uint8_t *writingPipe, uint8_t *readingPipe) {
	//config radio to comm with a node
	radio.stopListening();
	radio.openWritingPipe(writingPipe);
	radio.openReadingPipe(1, readingPipe);
}

void printD(const char *message) {
//	display.clearDisplay();
//	display.setCursor(0, 0);
	u8g2.print(message);
//	display.display();
}

void printDWithVal(const char *message, int value) {
//	display.clearDisplay();
//	display.setCursor(0, 0);
	u8g2.print(message);
	u8g2.print(value);
//	display.display();
}
void printD2Str(const char *str1, const char *str2) {
//	display.clearDisplay();
//	display.setCursor(0, 0);
	u8g2.print(str1);
	u8g2.print(str2);
//	display.display();
}

void powerCycle(int deviceID) {
	//this is a blocking routine so need to keep checking messages and
	//updating vars etc
	// but do NOT do manage restarts as could be recursive and call this routine again.

//	pinMode(redLEDPin, OUTPUT);
//	digitalWrite(redLEDPin, HIGH);
//	delay(100);
//	digitalWrite(redLEDPin, LOW);

	if (transmitEnable == 1) {
		Serial.println(F("sending off"));
		//printDWithVal("Power off:", devices[deviceID].socketID);
		//printD2Str("Power off:", devices[deviceID].name);
		badLED();
		//beep(1, 2, 1);
		for (int i = 0; i < 3; i++) { // turn socket off
			processMessage();
			updateDisplay();
			transmitter.sendUnit(devices[deviceID].socketID, false);
		}
		processMessage();
		updateDisplay();
		delay(1000);
		processMessage();
		updateDisplay();
		delay(1000);
		processMessage();
		updateDisplay();
		delay(1000);
		processMessage();
		updateDisplay();
		// Switch Rxunit on
		Serial.println(F("sending on"));
		//printD2Str("Power on :", devices[deviceID].name);

		for (int i = 0; i < 3; i++) {  // turn socket back on
			processMessage();
			updateDisplay();
			transmitter.sendUnit(devices[deviceID].socketID, true);
		}
		processMessage();
		updateDisplay();
		LEDsOff();
		//beep(1, 2, 1);
		Serial.println(F("complete"));
		//printD("PowerCyle:cycle done");
		//resetDevice(deviceID);
		//devices[deviceID].lastGoodAckMillis = millis();
	} else {
		Serial.println(F("not transmitting"));

	}
}

void beep(int numBeeps, int onDuration, int offDuration) {
	for (int i = 0; i < numBeeps; i++) {
		//pinMode(buzzer, OUTPUT);
		//digitalWrite(buzzer, HIGH);
		delay(onDuration);
		//digitalWrite(buzzer, LOW);
		delay(offDuration);
	}
}

// vim:cin:ai:sts=2 sw=2 ft=cpp
