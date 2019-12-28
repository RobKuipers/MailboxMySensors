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
* Connect lid-button or (reed) switch between
* digital I/O pin LID_PIN (see below) and COMMON_PIN.
*
* Connect door-button or (reed) switch between
* digital I/O pin DOOR_PIN (see below) and COMMON_PIN.
*/

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
#define COMMON_PIN 3 // The common pin for the LID and DOOR switch; MUST be an interruptable PIN (2 (in use for RFM69) or 3)
#define LID_PIN 4    // Arduino Digital I/O pin for button/reed switch
#define DOOR_PIN 5   // Arduino Digital I/O pin for button/reed switch

//#define SLEEP_IN_MS 21600000 // wake up 4 times a day
#define SLEEP_IN_MS 3600000 // wake up every hour (for initial reliability testing)

int8_t interruptedBy = -1;
int oldBatLevel;
float oldTemperature;

// Change to V_BINARY if you use S_STATUS in presentation below
MyMessage msg(CHILD_ID, V_TRIPPED);
MyMessage msgChipTemp(CHILD_ID_TEMPERATURE, V_TEMP);

void before()
{
	// All buttons as input-pullup as per ATMEGA recommendation to use less power (and more safety) 
	// (http://electronics.stackexchange.com/questions/43460/how-should-unused-i-o-pins-be-configured-on-atmega328p-for-lowest-power-consumpt)
	for (int i = 1; i <= 8; i++)
	{
		pinMode(i, INPUT_PULLUP);
	}

	// Now explicity set pins as needed
	// Set COMMON pin as input, activate internal pull-up
	pinMode(COMMON_PIN, INPUT_PULLUP);

	// Setup lid switch as output with state LOW
	pinMode(LID_PIN, OUTPUT);
	digitalWrite(LID_PIN, LOW);

	// Setup door switch as output with state LOW
	pinMode(DOOR_PIN, OUTPUT);
	digitalWrite(DOOR_PIN, LOW);
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
	sendSketchInfo("Mailbox", "1.6", ACK);

	// Register binary input sensor to gw (they will be created as child devices)
	// You can use S_DOOR, S_MOTION or S_STATUS here depending on your usage. 
	// If S_STATUS is used, remember to update variable type you send in. See "msg" above.
	present(CHILD_ID, S_DOOR, ACK);
	present(CHILD_ID_TEMPERATURE, S_TEMP);

}

//  Check if digital input has changed and send in new value
void loop()
{
	if (interruptedBy == digitalPinToInterrupt(COMMON_PIN))
	{
		bool gotState = false;
		int numLoops = 10;  // prevent eternal loop if state could not be determined (unlikely, but still)

		do
		{
			// Interrupted; find out by which switch
			digitalWrite(LID_PIN, HIGH);
			wait(5);
			// Test if COMMON_PIN stays LOW now
			if (digitalReadFast(COMMON_PIN) == LOW) {
				// Then this must be the DOOR switch
				Sprintln(F("Mailbox has been emptied"));
				send(msg.set(false), ACK);

				gotState = true;
				digitalWrite(LID_PIN, LOW);
			}
			else
			{
				// Interrupted; find out by which switch
				digitalWrite(DOOR_PIN, HIGH);
				wait(5);
				// Test if COMMON_PIN stays LOW now
				if (digitalReadFast(COMMON_PIN) == LOW) {
					// Then this must be the LID switch
					Sprintln(F("New mail received"));
					send(msg.set(true), ACK);

					gotState = true;
					digitalWrite(DOOR_PIN, LOW);
				}
			}
			numLoops--;
		} while (!gotState && numLoops > 0);

		wait(500);

	}
	else
	{
		float newTemp = readTemp();
		if (oldTemperature != newTemp)
		{
			send(msgChipTemp.set(newTemp, 1), ACK);
			oldTemperature = newTemp;
		}

	}

	// Changed to: ALWAYS SEND BAT LEVEL
	// if (interruptedBy==-1) // Timeout on sleep (once a day)
	// {
	int batLevel = getBatteryLevel();
	if (oldBatLevel != batLevel)
	{
		sendBatteryLevel(batLevel, ACK);
		oldBatLevel = batLevel;
	}
	// }

	interruptedBy = smartSleep(digitalPinToInterrupt(LID_PIN), FALLING, SLEEP_IN_MS);
}

void receive(const MyMessage &message) {
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

float readTemp()
{
#if defined (xxMY_RADIO_RFM69) && !defined(MY_RFM69_NEW_DRIVER)
	return _radio.readTemperature(-3);
#else
	// Read 1.1V reference against MUX3  
	return (readMUX(_BV(REFS1) | _BV(REFS0) | _BV(MUX3)) - 125) * 0.1075f;
#endif
}

long readMUX(uint8_t aControl)
{
	long result;

	ADMUX = aControl;
	delay(20); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Convert
	while (bit_is_set(ADCSRA, ADSC));
	result = ADCL;
	result |= ADCH << 8;
	return result;
}


// Utter nonsense, but needed for attaching an interrupt to...
void debounce() {
}
