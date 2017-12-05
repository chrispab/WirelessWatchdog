#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <NewRemoteTransmitter.h>
#include <printf.h>
#include <RF24.h>
#include <stdint.h>
#include <WString.h>

NewRemoteTransmitter transmitter(282830, 5, 254, 4);

// Set up nRF24L01 radio on SPI bus plus pins 7 & 8
RF24 radio(7, 8);

// sets the role of this unit in hardware.  Connect to GND to be the 'pong' receiver
// Leave open to be the 'ping' transmitter
const int Tx433Mhz_pin = 5;
const int redLEDPin = 3;
const int greenLEDPin = 4;
const int buzzer = 6;

uint8_t writePipeLocS[] = "NodeS";
uint8_t readPipeLocS[] = "Node0";

uint8_t writePipeLocC[] = "NodeC";
uint8_t readPipeLocC[] = "Node0";

uint8_t writePipeLocG[] = "NodeG";
uint8_t readPipeLocG[] = "Node0";

static unsigned int goodSecsMax = 20;
//
// Payload
const int max_payload_size = 32;
char receive_payload[max_payload_size + 1]; // +1 to allow room for a terminating NULL char

unsigned long maxMillisNoAckFromPi = 1000UL * 300UL; // max millisces to wait if no ack from pi before power cycling pi
unsigned long waitForPiPowerUpMillis = 1000UL * 120UL;
//const unsigned long maxMillisNoAckFromPi = 1000UL * 30UL; // max millisces to wait if no ack from pi before power cycling pi
//const unsigned long waitForPiPowerUpMillis = 1000UL * 30UL;

const int RxUnit = 15;
const int transmitEnable = 1;
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

// define a struct for each controller instance with related vars
struct controller {
	int id_number;
	int zone;
	int socketID;
	unsigned long lastGoodAckMillis;
	char name[4];
	char heartBeatText[4]; // allow enough space for text plus 1 extra for null terminator
	char badStatusMess[21]; // enugh for two lines on display
	char goodStatusMess[21]; // enugh for two lines on display
	bool isRebooting; //indicate state
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
	strcpy(devices[0].badStatusMess, "GRG Status-");
	strcpy(devices[0].goodStatusMess, "GRG StatusOK");
	devices[0].isRebooting = 0;
	devices[0].rebootMillisLeft = 0;
	devices[0].lastRebootMillisLeftUpdate = millis();

	devices[1].id_number = 1;
	devices[1].zone = 2;
	devices[1].socketID = 4;
	devices[1].lastGoodAckMillis = millis();
	strcpy(devices[1].name, "CNV");
	strcpy(devices[1].heartBeatText, "CCC");
	strcpy(devices[1].badStatusMess, "CNV Status-");
	strcpy(devices[1].goodStatusMess, "CNV StatusOK");
	devices[1].isRebooting = 0;
	devices[1].rebootMillisLeft = 0;
	devices[1].lastRebootMillisLeftUpdate = millis();

	devices[2].id_number = 2;
	devices[2].zone = 3;
	devices[2].socketID = 15;
	devices[2].lastGoodAckMillis = millis();
	strcpy(devices[2].name, "SHD");
	strcpy(devices[2].heartBeatText, "SSS");
	strcpy(devices[2].badStatusMess, "SHD Status-");
	strcpy(devices[2].goodStatusMess, "SHD StatusOK");
	devices[2].isRebooting = 0;
	devices[2].rebootMillisLeft = 0;
	devices[2].lastRebootMillisLeftUpdate = millis();

	//setup leds
	pinMode(redLEDPin, OUTPUT);
	pinMode(greenLEDPin, OUTPUT);
	badLED();
	delay(100);
	goodLED();
	//digitalWrite(redLEDPin, LOW);

	beep(1, 2, 1);

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

	display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
	// init done
	display.clearDisplay();
	display.setTextSize(2);
	display.setTextColor(WHITE);
	display.setCursor(0, 0);
	display.println("Listening Watchdog 9");
	display.display();
	delay(2000);
	display.clearDisplay();
	display.display();
	setPipes(writePipeLocC, readPipeLocC); // SHOULD NEVER NEED TO CHANGE PIPES
	radio.startListening();
}

void loop(void) {

	while (radio.available()) {            // Read all available payloads
		//Serial.println(F("Processing next available message.. "));
		processMessage();
	}

	updateDisplay(); //rotate messages etc if time to
	// check each device if restart reqd
	manageRestarts(0);
	manageRestarts(1);
	manageRestarts(2);
}

//check a unit and see if restart reqd
void manageRestarts(int deviceID) {
	//int timerDone = 0;

// now check if need to reboot a device
	if ((millis() - devices[deviceID].lastGoodAckMillis)
			> maxMillisNoAckFromPi) { // over time limit so reboot first time in then just upadte time each other time

		if (devices[deviceID].isRebooting == 0) { // all this done first time triggered
			printD("Reboot : ");
			powerCycle(deviceID);
			devices[deviceID].isRebooting = 1; //signal device is rebooting
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

			// next subtraction will take us to/over linit
			if (millisLapsed >= devices[deviceID].rebootMillisLeft) {
				//zero or neg reached
				devices[deviceID].rebootMillisLeft = 0;
				//timerDone = 1;
			} else { // ok to do timer subtraction
				//timerDone = 0;

				devices[deviceID].rebootMillisLeft =
						devices[deviceID].rebootMillisLeft - millisLapsed;

			}
			devices[deviceID].lastRebootMillisLeftUpdate = millis();

			// calc if next time subtraction takes it below zero
			//cant get a neg number from unsigned numbers used
			if (devices[deviceID].rebootMillisLeft == 0) { // reboot stuff completed here
				//if (timerDone == 1) { // reboot stuff completed here
				devices[deviceID].lastGoodAckMillis = millis();
				Serial.println(F("Assume Pi is back up"));
				//printD("Assume pi back up");
				printD2Str("Assume up:", devices[deviceID].name);
				devices[deviceID].isRebooting = 0; //signal device has stopped rebooting
			}
		}
	}
}

void updateDisplay(void) {
	//check if time to display a new message
	//rotate every n secs standard info messages
	// overide if hit reset procedure
	static int stateCounter = 0; // only initialised once at start
	static unsigned long lastDispUpdateTimeMillis = 0;
	int dispUpdateFreq = 2; // delay between updates in secs
	static unsigned long dispUpdateInterval = dispUpdateFreq * 1000;

	unsigned long secsSinceAck = 0;
	// max secs out considered good

	//Serial.println(stateCounter);
	// this loop
	if ((millis() - lastDispUpdateTimeMillis) >= dispUpdateInterval) { //ready to update?
		//Serial.println(stateCounter);

		secsSinceAck = (millis() - devices[stateCounter].lastGoodAckMillis)
				/ 1000;
		// make sure check for restarting device
		//if so display current secs in wait for reboot cycle
		if (devices[stateCounter].isRebooting) {
			unsigned long secsLeft = (devices[stateCounter].rebootMillisLeft)
					/ 1000UL;

			Serial.print("--rebootMillisLeft: ");
			Serial.println((devices[stateCounter].rebootMillisLeft));

			Serial.print("--secsLeft var: ");
			Serial.println(secsLeft);

			char message[] = "Reboot ";
			char str_output[30] = { 0 }; //, str_two[]="two";
			strcpy(str_output, message);
			strcat(str_output, devices[stateCounter].name);
			strcat(str_output, "Left: ");

			printDWithVal(str_output, secsLeft);

		} else if ((secsSinceAck > goodSecsMax)) {

			printDWithVal(devices[stateCounter].badStatusMess, secsSinceAck);
			badLED();
		} else {
			printD(devices[stateCounter].goodStatusMess);
			goodLED();
		}

		stateCounter++;
		if (stateCounter == 3)
			stateCounter = 0;

		lastDispUpdateTimeMillis = millis();
	}
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
	Serial.print(F("Got message: "));
	//Serial.print(len);
	//Serial.print(F(" pre use value="));
	Serial.println(receive_payload);

	//who was it from?
	//reset that timer
	if (equalID(receive_payload, devices[0].heartBeatText)) {
		devices[0].lastGoodAckMillis = millis();
		//Serial.println(F("=RESET GGG="));
	} else if (equalID(receive_payload, devices[1].heartBeatText)) {
		devices[1].lastGoodAckMillis = millis();
		//Serial.println(F("=RESET CCC="));
	} else if (equalID(receive_payload, devices[2].heartBeatText)) {
		devices[2].lastGoodAckMillis = millis();
		//Serial.println(F("=RESET SSS="));
	} else {
		//Serial.println(F("=NO MATCH="));
		badLED();
	}

}

void goodLED(void) {
	//switch off red led
	digitalWrite(redLEDPin, LOW);
	digitalWrite(greenLEDPin, HIGH);
}
void badLED(void) {
	//switch off red led
	digitalWrite(redLEDPin, HIGH);
	digitalWrite(greenLEDPin, LOW);
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
	display.clearDisplay();
	display.setCursor(0, 0);
	display.print(message);
	display.display();
}

void printDWithVal(const char *message, int value) {
	display.clearDisplay();
	display.setCursor(0, 0);
	display.print(message);
	display.print(value);
	display.display();
}
void printD2Str(const char *str1, char *str2) {
	display.clearDisplay();
	display.setCursor(0, 0);
	display.print(str1);
	display.print(str2);
	display.display();
}

void powerCycle(int deviceID) {
	pinMode(redLEDPin, OUTPUT);
	digitalWrite(redLEDPin, HIGH);
	delay(100);
	digitalWrite(redLEDPin, LOW);

	beep(1, 2, 1);

	if (transmitEnable == 1) {
		// Switch unit 15 off
		Serial.println("sending off");
		//printDWithVal("Power off:", devices[deviceID].socketID);
		printD2Str("Power off:", devices[deviceID].name);

		for (int i = 0; i < 5; i++) { // turn socket off
			transmitter.sendUnit(devices[deviceID].socketID, false);
		}

		delay(300);

		// Switch Rxunit on
		Serial.println("sending on");
		printD2Str("Power on :", devices[deviceID].name);
		beep(1, 2, 1);

		for (int i = 0; i < 5; i++) {  // turn socket back on
			transmitter.sendUnit(devices[deviceID].socketID, true);
		}
		Serial.println("complete");
		printD("PowerCyle:cycle done");
	} else {
		Serial.println("not transmitting");

	}
}

void beep(int numBeeps, int onDuration, int offDuration) {
	for (int i = 0; i < numBeeps; i++) {
		pinMode(buzzer, OUTPUT);
		digitalWrite(buzzer, HIGH);
		delay(onDuration);
		digitalWrite(buzzer, LOW);
		delay(offDuration);
	}
}

// vim:cin:ai:sts=2 sw=2 ft=cpp
