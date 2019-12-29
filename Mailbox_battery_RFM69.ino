/**
* The MySensors Arduino library handles the wireless radio link and protocol
* between your home built sensors/actuators and HA controller of choice.
* The sensors forms a self healing radio network with optional repeaters. Each
* repeater and gateway builds a routing tables in EEPROM which keeps track of the
* network topology allowing messages to be routed to nodes.
*
* Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
* Copyright (C) 2013-2015 Sensnology AB
* Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
*
* Documentation: http://www.mysensors.org
* Support Forum: http://forum.mysensors.org
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
*******************************
*
* DESCRIPTION
*
* Simple deepsleeping mailbox example
*
* Schematics:
* sense pin ----- normally closed lid switch ----- probe pin ------ normally closed door switch ---- ground
*    pin 3                                           pin 4
* This creates a closed loop between sense and ground when in rest.
*
* Alternative if you are not interested in the mailbox door status
* sense pin ----- normally closed lid switch ----- ground
*
*/
#include "DS18B20.h"
#include <Bounce2.h>

// Enable debug Serial.prints to serial monitor
//#define MY_DEBUG 

#if defined MY_DEBUG
#define Sprintln(a) (Serial.println(a))
#define Sprint(a) (Serial.print(a))
#else 
#define Sprintln(a)
#define Sprint(a)
#endif

// Enable and select radio type attached
#define MY_RADIO_RFM69
//#define MY_RFM69_NEW_DRIVER
#define MY_RFM69_FREQUENCY RFM69_868MHZ
//#define MY_IS_RFM69HW

#define MY_NODE_ID 1
//#define MY_TRANSPORT_UPLINK_CHECK_DISABLED  // Option not yet generally available; is in development trunk
#define MY_SMART_SLEEP_WAIT_DURATION_MS 50

#include <MySensors.h>
#include <SPI.h>


#define ACK 0        // = false
#define CHILD_ID_TEMPERATURE 2
#define CHILD_ID 5
#define SENSE_PIN 3 // The sense pin for the LID and DOOR switch; MUST be an interruptable PIN (2 (in use for RFM69) or 3)
#define PROBE_PIN 4    // Arduino Digital I/O pin for button/reed switch

#define SLEEP_IN_MS 3600000 // wake up every hour

int8_t interruptedBy = -1;
int oldBatLevel;
float oldTemperature;
int countInterrupts = -1;

// Change to V_BINARY if you use S_STATUS in presentation below
MyMessage msg(CHILD_ID, V_TRIPPED);
MyMessage msgChipTemp(CHILD_ID_TEMPERATURE, V_TEMP);

unsigned long startMillis;
bool awake = false;

// Instantiate a Bounce object
Bounce debouncer = Bounce();
bool pinInterruptHandled = false;

DS18B20 Sensor;  // Insert the three legs of the sensor into GND, Pin 5 (Vcc), and Pin 6(Data). 

void before()
{
	// All buttons as input-pullup as per ATMEGA recommendation to use less power (and more safety) 
	// (http://electronics.stackexchange.com/questions/43460/how-should-unused-i-o-pins-be-configured-on-atmega328p-for-lowest-power-consumpt)
	for (int i = 1; i <= 8; i++)
	{
		pinMode(i, INPUT_PULLUP);
	}

	// Now explicity set pins as needed
	// Set SENSE pin as input, activate internal pull-up
	pinMode(SENSE_PIN, INPUT_PULLUP);
	debouncer.attach(SENSE_PIN, INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode
	debouncer.interval(25); // Use a debounce interval of 25 milliseconds

	// Setup PROBE pin as an open loop
	pinMode(PROBE_PIN, INPUT);

	// Setup Temp Sensor
	// Timer 2 initialization from wiring.c for an ATmega 328P (Arduino Uno rev 3) + 20 bytes to sketch size
	TCCR2A |= _BV(COM2A1) | _BV(WGM20);    // Enable timer 2 to _delay_ms() works properly
	TCCR2B |= CS22;                        // set clkT2S/64 (From prescaler)

	pinMode(5, INPUT);    // Supply no power to DS18B20 at this moment
}

void setup()
{
}


// Battery measure
int getBatteryLevel()
{
	int results = (readVcc() - 2000) / 10;

	if (results > 100)
		results = 100;
	if (results < 0)
		results = 0;
	return results;
} // end of getBandgap

  // when ADC completed, take an interrupt 
EMPTY_INTERRUPT(ADC_vect);

long readVcc() {
	long result;
	// Read 1.1V reference against AVcc
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	delay(2); // Wait for Vref to settle
	noInterrupts();
	// start the conversion
	ADCSRA |= _BV(ADSC) | _BV(ADIE);
	set_sleep_mode(SLEEP_MODE_ADC);    // sleep during sample
	interrupts();
	sleep_mode();
	// reading should be done, but better make sure
	// maybe the timer interrupt fired 
	while (bit_is_set(ADCSRA, ADSC));
	result = ADCL;
	result |= ADCH << 8;
	result = 1126400L / result; // Back-calculate AVcc in mV

	return result;
}

void presentation() {
	sendSketchInfo("Mailbox", "2.0", ACK);

	// Register binary input sensor to gw (they will be created as child devices)
	// You can use S_DOOR, S_MOTION or S_STATUS here depending on your usage. 
	// If S_STATUS is used, remember to update variable type you send in. See "msg" above.
	present(CHILD_ID, S_DOOR, "", ACK);
	present(CHILD_ID_TEMPERATURE, S_TEMP);

}

//  Check if digital input has changed and send in new value
void loop()
{
	if (!awake)
	{
		startMillis = millis();
		awake = true;
		Sprintln(F("Awoken by interrupt (pin or timer)"));
	}

	if (interruptedBy == digitalPinToInterrupt(SENSE_PIN))
	{
		// Update the Bounce instance :
		debouncer.update();
		
		if (!pinInterruptHandled && debouncer.rose()) {
			// Interrupted; find out by which switch 
			// Make PROBE pin an LOW output
			digitalWriteFast(PROBE_PIN, LOW);
			pinMode(PROBE_PIN, OUTPUT);

			// Test if SENSE_PIN goes LOW now
			if (digitalReadFast(SENSE_PIN) == LOW) {
				// Then this must be the DOOR switch
				Sprintln(F("It's the DOOR switch"));
				send(msg.set(false), ACK);
			}
			else
			{
				// Then this must be the LID switch
				Sprintln(F("It's the LID Switch"));
				send(msg.set(true), ACK);
			}
			// Make PROBE pin an open loop
			pinMode(PROBE_PIN, INPUT);
			pinInterruptHandled = true;
		}
	}


	// Stay awake for 3 seconds; or longer if SENSE pin has not gone LOW yet
	if ((millis() - startMillis) > 3000 && (!(interruptedBy == digitalPinToInterrupt(SENSE_PIN)) || debouncer.read() == LOW)) {
		awake = false;
		pinInterruptHandled = false;

		Sprintln(F("Initializing sleep..."));

		if (interruptedBy != MY_SLEEP_NOT_POSSIBLE) {
			// Before sleeping...
			// Send temperature
			digitalWriteFast(5, HIGH);
			pinMode(5, OUTPUT);
			wait(20);

			// Set the sensor's resolution to 11 bits
			Sensor.SetResolution(11);

			Sensor.StartConversion();

			wait(800);
			uint16_t temp = Sensor.GetTemperature();

			digitalWriteFast(5, LOW);
			pinMode(5, INPUT);

			float newTemp = temp;
			newTemp = newTemp / 100;   // Strange 'trick' to get the decimals right
			if (oldTemperature != newTemp && temp != 8500)
			{
				Sprintln(F("Send temperature first"));
				send(msgChipTemp.set(newTemp, 1), ACK);
				oldTemperature = newTemp;
			}

			countInterrupts++;
			// Send bat level AT LEAST every 24 hrs (or more often when mailbox has been triggered)
			if (countInterrupts % 24 == 0) {
				Sprintln(F("Send battery level first"));
				int batLevel = getBatteryLevel();
				if (oldBatLevel != batLevel)
				{
					sendBatteryLevel(batLevel, ACK);
					oldBatLevel = batLevel;
				}

			}
		}
		wait(1000); // Because I'm not using smartSleep; some extra time
		Sprintln(F("Entering sleep now"));

		// Do NOT use smartSleep; this send an awake message to the controller. This takes valuable time, during which
		// the sensing of the Pins cannot be done.
		// Also, since this is a send-only node it has no benefits.
		interruptedBy = sleep(digitalPinToInterrupt(SENSE_PIN), RISING, SLEEP_IN_MS);
	}
}

void receive(const MyMessage& message) {
	// We only expect one type of message from controller. But we better check anyway.
	if (message.isAck()) {
		Sprintln(F("This is an ack from gateway"));
	}

	if (message.type == V_STATUS && message.getCommand() == C_SET) {

		// Write some debug info
		Sprint(F("Incoming change for sensor:"));
		Sprint(message.sensor);
		Sprint(F(", New status: "));
		Sprintln(message.getBool());
	}
}
