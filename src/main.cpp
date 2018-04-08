#include <Arduino.h>
#include <HardwareSerial.h>
#include <NewRemoteTransmitter.h>
#include <NewRemoteReceiver.h>

#include <printf.h>
#include <RF24.h>
#include <stdint.h>
#include <WString.h>
#include <LedFader.h>
#include <U8g2lib.h>
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/
											SCL, /* data=*/SDA);			   // pin remapping with ESP8266 HW I2C

//#define OLD_HARDWARE
////#define NEW_HARDWARE

// forward declarations
void setPipes(uint8_t *writingPipe, uint8_t *readingPipe);
void showCode(NewRemoteCode receivedCode);
void processMessage(void);
void displayKeys(int x);
void updateDisplay(void);
void manageRestarts(int deviceID);
void resetDevice(int deviceID);
void powerCycle(int deviceID);
void printD(const char *message);
void printDWithVal(const char *message, int value);
void LEDsOff(void);
void printD(const char *message);
void goodLED(void);
void badLED(void);
int equalID(char *receive_payload, const char *targetID);

#define TX433PIN 5
#define RX433PIN 2
NewRemoteTransmitter transmitter(282830, TX433PIN, 254, 4);
//NewRemoteReceiver receiver()

// Set up nRF24L01 radio on SPI bus plus pins 7 & 8
RF24 radio(7, 8);

//==================================================
//all tweakable params etc
#define SW_VERSION "2.3"

//const int buzzer = 6;
const int switchPin = A7;		  // the number of the pushbutton pin
const int analogue_switches = A0; // ip port A0
int dispUpdateFreq = 1.5;		  // how many updates per sec
const int redLEDPin = 3;
const int greenLEDPin = 10;
const int blueLEDPin = 9;
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
static unsigned int goodSecsMax = 10;						 //20
static unsigned long maxMillisNoAckFromPi = 1000UL * 30UL;   //300 max millisces to wait if no ack from pi before power cycling pi
static unsigned long waitForPiPowerUpMillis = 1000UL * 20UL; //120
#else														 // new hardware
static unsigned int goodSecsMax = 15;						  //20
static unsigned long maxMillisNoAckFromPi = 1000UL * 300UL;   //300 max millisces to wait if no ack from pi before power cycling pi
static unsigned long waitForPiPowerUpMillis = 1000UL * 120UL; //120
#endif

const uint8_t transmitEnable = 1;

int switchState = 0;
int x = 0;

// define a struct for each controller instance with related vars
struct controller
{
	uint8_t id_number;
	uint8_t zone;
	uint8_t socketID;
	unsigned long lastGoodAckMillis;
	char name[4];
	char heartBeatText[4];   // allow enough space for text plus 1 extra for null terminator
	char badStatusMess[10];  // enough for 1 line on display
	char goodStatusMess[10]; // enough for 1 line on display
	bool isRebooting;		 //indicate if booting up
	bool isPowerCycling;	 //indicate if power cycling
	unsigned long rebootMillisLeft;
	unsigned long lastRebootMillisLeftUpdate;
	uint8_t powerCyclesSincePowerOn;
};

struct controller devices[3]; //create an array of 3 controller structures

void setup(void)
{

	//populate the array of controller structs
	for (int i = 0; i < 3; i++)
	{
		devices[i].id_number = i; //0 to 2
		devices[i].zone = i + 1;  //1 to 3
		devices[i].lastGoodAckMillis = millis();
		devices[i].isRebooting = 0;
		devices[i].isPowerCycling = 0;
		devices[i].rebootMillisLeft = 0;
		devices[i].lastRebootMillisLeftUpdate = millis();
		devices[i].powerCyclesSincePowerOn = 0;
		strcpy(devices[i].badStatusMess, "Away");
		strcpy(devices[i].goodStatusMess, "OK");
	}
	// individual stuff
	devices[0].socketID = 14;
	strcpy(devices[0].name, "GRG");
	strcpy(devices[0].heartBeatText, "GGG");

	devices[1].socketID = 4;
	strcpy(devices[1].name, "CNV");
	strcpy(devices[1].heartBeatText, "CCC");
	// strcpy(devices[1].badStatusMess, "Away");
	// strcpy(devices[1].goodStatusMess, "OK");

	devices[2].socketID = 15;
	strcpy(devices[2].name, "SHD");
	strcpy(devices[2].heartBeatText, "SSS");
	// strcpy(devices[2].badStatusMess, "Away");
	// strcpy(devices[2].goodStatusMess, "OK");

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
	//	u8g2.setFont(u8g2_font_t0_15_tf);
	//	u8g2.setFont(u8g2_font_7x14_tf);
	//u8g2.setFont(u8g2_font_profont15_tf);
	//u8g2.setFont(u8g2_font_8x13B_tf);
	u8g2.setFont(u8g2_font_8x13_tf);

	// 17 chars by 3 lines at this font size
	u8g2.drawStr(30, 10, "Wireless");
	u8g2.drawStr(30, 21, "Watchdog");
	u8g2.drawStr(45, 32, SW_VERSION);

	u8g2.sendBuffer();

	delay(3000);

	setPipes(writePipeLocC, readPipeLocC); // SHOULD NEVER NEED TO CHANGE PIPES
	radio.startListening();

	// Initialize receiver on interrupt 0 (= digital pin 2), calls the callback "showCode"
	// after 2 identical codes have been received in a row. (thus, keep the button pressed
	// for a moment)
	//
	// See the interrupt-parameter of attachInterrupt for possible values (and pins)
	// to connect the receiver.
	NewRemoteReceiver::init(0, 2, showCode);
}

void loop(void)
{

	//while (radio.available()) {            // Read all available payloads
	//Serial.println(F("Processing next available message.. "));
	processMessage();
	//}

	x = analogRead(analogue_switches);
	//Serial.println((x));
	displayKeys(x);
	//Serial.println(x);

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
	if (switchState > 512)
	{
		strcpy(devices[1].goodStatusMess, "OK");
		manageRestarts(1);
	}
	else
	{
		strcpy(devices[1].goodStatusMess, "OFF");
		resetDevice(1);
	}
	manageRestarts(2);
}

int freeRam()
{
	extern int __heap_start, *__brkval;
	int v;

	return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void printFreeRam(void)
{
	Serial.print(F("FR: "));
	Serial.println(freeRam());
}

void displayKeys(int x)
{
	//	char key = ' ';
	//	if (x < 1000) {
	//		do {
	//			u8g2.clearBuffer();
	//			u8g2.setCursor(0,20);
	//			if (x < 60) {
	//				printDWithVal("L", x);
	//				key = 'L';
	//			} else if (x < 200) {
	//				printDWithVal("U", x);
	//				key = 'U';
	//			} else if (x < 400) {
	//				printDWithVal("D", x);
	//				key = 'D';
	//			} else if (x < 600) {
	//				printDWithVal("R", x);
	//				key = 'R';
	//			} else if (x < 800) {
	//				printDWithVal("K", x);
	//				key = 'K';
	//			}
	//			u8g2.sendBuffer();
	//			delay(1000);
	//			x = analogRead(analogue_switches);
	//		} while (key != 'K');
	//		//Serial.println("OUT OF WHILE");
	//	}
}
//Callback function is called only when a valid code is received.
void showCode(NewRemoteCode receivedCode)
{
	// Note: interrupts are disabled. You can re-enable them if needed.

	// Print the received code.
	Serial.print("Addr ");
	Serial.print(receivedCode.address);

	if (receivedCode.groupBit)
	{
		Serial.print(" group");
	}
	else
	{
		Serial.print(" unit ");
		Serial.print(receivedCode.unit);
	}

	switch (receivedCode.switchType)
	{
	case NewRemoteCode::off:
		Serial.print(" off");
		break;
	case NewRemoteCode::on:
		Serial.print(" on");
		break;
	case NewRemoteCode::dim:
		Serial.print(" dim");
		break;
	}

	if (receivedCode.dimLevelPresent)
	{
		Serial.print(", dim level: ");
		Serial.print(receivedCode.dimLevel);
	}

	Serial.print(", period: ");
	Serial.print(receivedCode.period);
	Serial.println("us.");
}

//check a unit and see if restart reqd
void manageRestarts(int deviceID)
{
	// now check if need to reboot a device
	if ((millis() - devices[deviceID].lastGoodAckMillis) > maxMillisNoAckFromPi)
	{ // over time limit so reboot first time in then just upadte time each other time

		//printFreeRam();
		if (devices[deviceID].isRebooting == 0)
		{ // all this done first time triggered
			//printD("Reboot : ");

			devices[deviceID].isRebooting = 1; //signal device is rebooting

			devices[deviceID].isPowerCycling = 1; // signal in power cycle

			powerCycle(deviceID);

			devices[deviceID].isPowerCycling = 0; // signal in power cycle
			devices[deviceID].powerCyclesSincePowerOn++;
			devices[deviceID].rebootMillisLeft = waitForPiPowerUpMillis;
			devices[deviceID].lastRebootMillisLeftUpdate = millis();

			Serial.println("triggered reboot in manage reboots");

			Serial.println(devices[deviceID].rebootMillisLeft);
			//delay(1000);
		}
		else
		{ // this executes till end of reboot timer
			//device is rebooting now - do some stuff to update countdown timers
			//wait for pi to come back up - do nothing
			//millis since last update
			unsigned long millisLapsed = millis() - devices[deviceID].lastRebootMillisLeftUpdate;

			// next subtraction will take us to/over limit
			if (millisLapsed >= devices[deviceID].rebootMillisLeft)
			{
				//zero or neg reached
				devices[deviceID].rebootMillisLeft = 0;
				//timerDone = 1;
			}
			else
			{ // ok to do timer subtraction

				devices[deviceID].rebootMillisLeft =
					devices[deviceID].rebootMillisLeft - millisLapsed;
			}
			devices[deviceID].lastRebootMillisLeftUpdate = millis();

			// calc if next time subtraction takes it below zero
			//cant get a neg number from unsigned numbers used
			if (devices[deviceID].rebootMillisLeft == 0)
			{ // reboot stuff completed here
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

void updateDisplay(void)
{
	//all three lines can be displayed at once

	int stateCounter; // only initialised once at start
	static unsigned long lastDispUpdateTimeMillis = 0;

	static unsigned long dispUpdateInterval = 1000 / dispUpdateFreq;

	unsigned int secsSinceAck = 0;
	// max secs out considered good

	char str_output[20] = {0};
	unsigned int secsLeft;
	char buf[4];
	//Serial.println(stateCounter);
	// this loop
	//create info string for each zone then display it
	//setup disp
	u8g2.clearBuffer();
	//u8g2.setFont(u8g2_font_7x14_tf);

	//check if time to display a new message updates

	if ((millis() - lastDispUpdateTimeMillis) >= dispUpdateInterval)
	{ //ready to update?
		printFreeRam();
		for (stateCounter = 0; stateCounter < 3; stateCounter++)
		{
			//Serial.println(stateCounter);
			secsSinceAck = (millis() - devices[stateCounter].lastGoodAckMillis) / 1000;

			u8g2.setCursor(0, ((stateCounter + 1) * 10) + (1 * stateCounter));
			// make sure check for restarting device
			//if so display current secs in wait for reboot cycle
			if (devices[stateCounter].isRebooting)
			{
				secsLeft = (devices[stateCounter].rebootMillisLeft) / 1000UL;

				Serial.print(F("--rebootMillisLeft: "));
				Serial.println((devices[stateCounter].rebootMillisLeft));

				Serial.print(F("--secsLeft var: "));
				Serial.println(secsLeft);

				//build string to show if cycling or coming back up
				//char str_output[20] = { 0 }; //, str_two[]="two";
				//start with device name
				strcpy(str_output, devices[stateCounter].name);
				strcat(str_output, ": ");
				//char message[] = " Reboot: ";
				if (devices[stateCounter].isPowerCycling)
				{
					strcat(str_output, "Power Cycle");
					printD(str_output);
					//secsLeft = '';
				}
				else
				{
					strcat(str_output, "Reboot: ");
					printDWithVal(str_output, secsLeft);
				}
			}
			else if ((secsSinceAck > goodSecsMax))
			{
				strcpy(str_output, devices[stateCounter].name);
				strcat(str_output, ": ");
				strcat(str_output, devices[stateCounter].badStatusMess);
				strcat(str_output, ": ");
				printDWithVal(str_output, secsSinceAck);
				//badLED();
				LEDsOff();
			}
			else
			{
				//u8g2.print("u");
				strcpy(str_output, devices[stateCounter].name);
				strcat(str_output, ": ");
				strcat(str_output, devices[stateCounter].goodStatusMess);
				//add restarts soince power on
				strcat(str_output, " (");

				sprintf(buf, "%i",
						devices[stateCounter].powerCyclesSincePowerOn);

				strcat(str_output, buf);
				strcat(str_output, ")");
				printD(str_output);
				goodLED();
			}
		}
		lastDispUpdateTimeMillis = millis();
		u8g2.sendBuffer();
	}
}

void processMessage(void)
{
	while (radio.available())
	{ // Read all available payloads

		// Grab the message and process
		uint8_t len = radio.getDynamicPayloadSize();

		// If a corrupt dynamic payload is received, it will be flushed
		if (!len)
		{
			return;
		}

		radio.read(receive_payload, len);

		// Put a zero at the end for easy printing
		receive_payload[len] = 0;

		//who was it from?
		//reset that timer
		if (equalID(receive_payload, devices[0].heartBeatText))
		{
			resetDevice(0);
			Serial.println(F("RESET GGG"));
		}
		else if (equalID(receive_payload, devices[1].heartBeatText))
		{
			resetDevice(1);
			//devices[1].lastGoodAckMillis = millis();
			Serial.println(F("RESET CCC"));
		}
		else if (equalID(receive_payload, devices[2].heartBeatText))
		{
			resetDevice(2);
			//devices[2].lastGoodAckMillis = millis();
			Serial.println(F("RESET SSS"));
		}
		else
		{
			Serial.println(F("NO MATCH"));
			//badLED();
			LEDsOff();
		}
	}
}

void resetDevice(int deviceID)
{
	devices[deviceID].lastGoodAckMillis = millis();
	devices[deviceID].isRebooting = 0;
	devices[deviceID].rebootMillisLeft = 0;
}
void goodLED(void)
{
	heartBeat.update();
}
void badLED(void)
{
	rebootAlert.on();
	heartBeat.update();
}

void LEDsOff(void)
{
	heartBeat.update();
}

int equalID(char *receive_payload, const char *targetID)
{
	//check if same 1st 3 chars
	if ((receive_payload[0] == targetID[0]) && (receive_payload[1] == targetID[1]) && (receive_payload[2] == targetID[2]))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void setPipes(uint8_t *writingPipe, uint8_t *readingPipe)
{
	//config radio to comm with a node
	radio.stopListening();
	radio.openWritingPipe(writingPipe);
	radio.openReadingPipe(1, readingPipe);
}

void printD(const char *message)
{
	u8g2.print(message);
}

void printDWithVal(const char *message, int value)
{
	u8g2.print(message);
	u8g2.print(value);
}

void printD2Str(const char *str1, const char *str2)
{
	u8g2.print(str1);
	u8g2.print(str2);
}

void powerCycle(int deviceID)
{
	//this is a blocking routine so need to keep checking messages and
	//updating vars etc
	// but do NOT do manage restarts as could be recursive and call this routine again.

	if (transmitEnable == 1)
	{
		Serial.println(F("sending off"));
		badLED();
		//beep(1, 2, 1);
		for (int i = 0; i < 3; i++)
		{ // turn socket off
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

		for (int i = 0; i < 3; i++)
		{ // turn socket back on
			processMessage();
			updateDisplay();
			transmitter.sendUnit(devices[deviceID].socketID, true);
		}
		processMessage();
		updateDisplay();
		LEDsOff();
		//beep(1, 2, 1);
		Serial.println(F("complete"));
	}
	else
	{
		Serial.println(F("not transmitting"));
	}
}

void beep(int numBeeps, int onDuration, int offDuration)
{
	for (int i = 0; i < numBeeps; i++)
	{
		//pinMode(buzzer, OUTPUT);
		//digitalWrite(buzzer, HIGH);
		delay(onDuration);
		//digitalWrite(buzzer, LOW);
		delay(offDuration);
	}
}

// vim:cin:ai:sts=2 sw=2 ft=cpp
