#include <QueueArray.h>
#include <XBee.h>

#include <Xbee.h>
#include "XbeeApiStream.h"

const int button_pin = 5;

unsigned long debounce_timestamp = 0, debounce_delay = 50;
int button_state = LOW, last_button_state = LOW;
bool state_paused = true;

XbeeApiStream xbeeStream = XbeeApiStream();

void setup() {
	Serial.begin(9600);
	xbeeStream.begin(Serial);
	pinMode(button_pin, INPUT);
	digitalWrite(button_pin, HIGH);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, !state_paused);
}

void loop() {
	int reading = digitalRead(button_pin);
	if(reading != last_button_state)	debounce_timestamp = millis();
	if(millis() - debounce_timestamp > debounce_delay){
		if(reading != button_state){
			button_state = reading;
			if(button_state == LOW){
				digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
				xbeeStream.write((uint8_t)0x00);
			}
		}
	}
	last_button_state = reading;
}
