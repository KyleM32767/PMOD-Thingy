

void serialEventRun() {
	if (Serial.available())
		serialEvent();
}


// keys to press to flip JA bits
const char KEYS_JA[9] = "qwertyui";

// keys to press to flip JB bits
const char KEYS_JB[9] = "asdfghjk";

// bitfield with status of JA bits
uint8_t out_ja = 0b00000000;

// bitfield with status of JB bits
uint8_t out_jb = 0b00000000;

// PMOD JA pin numbers
const int PINS_JA[8] = {0,  1,  2,  3,  4,  5,  6,  7 };

// PMOD JB pin numbers
const int PINS_JB[8] = {8,  9,  10, 11, 12, 13, 14, 15};

void serialEvent() {
	char rx = Serial.read();

	
	// set relevant bit
	for (int i=0; i<8; i++) {

		// check for JA toggle key
		if (rx == KEYS_JA[i]) {

			// toggle relevant bitfield bit
			out_ja ^= (1<<i); // toggle relevant bitfield bit
			
			// update relevant bit
			if (out_ja & (1<<i))
				digitalWrite(PINS_JA[i], HIGH);
			else
				digitalWrite(PINS_JA[i], LOW);
		}

	
		// check for JB toggle key
		if (rx == KEYS_JB[i]) {

			// toggle relevant bitfield bit
			out_jb ^= (1<<i); // toggle relevant bitfield bit
			
			// update relevant bit
			if (out_jb & (1<<i))
				digitalWrite(PINS_JB[i], HIGH);
			else
				digitalWrite(PINS_JB[i], LOW);
		}
	}


	// print status
	for (int i=7; i>=0; i--) {
		Serial.print((out_ja >> i) & 0b00000001);
	}
	Serial.print('\t');
	for (int i=7; i>=0; i--) {
		Serial.print((out_jb >> i) & 0b00000001);
	}

	Serial.print('\n');

}

void setup() {
	Serial.begin(9600);

	for (int i=0; i<8; i++) {
		pinMode(PINS_JA[i], OUTPUT);
		pinMode(PINS_JB[i], OUTPUT);
	}
}

void loop() {
	// put your main code here, to run repeatedly:

}
