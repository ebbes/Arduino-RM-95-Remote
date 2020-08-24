/*
 * Arduino RM95 emulator (LANC interface)
 * Allows to modify EEPROM as well as start recording on DCR TRV camcorders
 *
 * Based on code from
 * - https://create.arduino.cc/projecthub/L-Rosen/serial-to-lanc-control-l-70f735
 *    by L. Rosén, licensed as GPLv3+
 *    Relevant portions: most of the Arduino-specific code, although they are mostly rewritten by now.
 *    In addition, we use a way simpler circuit for interfacing with the LANC bus.
 * - RM95 Adjusting Remote Simulator for Linux
 *    Copyright (C) 1999 by Johannes Overmann <overmann@iname.com>, licensed as GPLv2+
 *    Relevant portions: parts of transmit/receive routine
 * This code is thus licensed as GPLv3+.
 *
 * Works on Arduino Nano, but should also work on Arduino Pro Mini (5V variant)
 * as they share the same µC.
 * 
 * Required pinout:
 * Arduino pin D4  <-> LANC signal (tip)
 * Arduino GND     <-> LANC GND (sleeve)
 * Currently not required since Arduino is USB-powered:
 * Arduino VIN     <-> LANC VCC (ring)
 *
 * Using an additional external pull-up will probably improve
 * the edges' quality, improving compatibility for longer wires.
 * In this case, connect D4 to +5V using a 4.7k resistor and
 * #define EXTERNAL_PULL_UP.
 * This even seems to work when there is no external pull-up present...
 *
 * NOTE: Pin D4 can be changed by (probably) any other digital pin,
 * but configuration below needs to be adjusted in this case. */

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

/////////////////////////////////////////////////////////////////////
// GENERAL CONFIGURATION
/////////////////////////////////////////////////////////////////////

// define PRINT_SYNC_REPEATS to show number of synchronization attempts and command repetitions
#define PRINT_SYNC_REPEATS

// When using an external pull-up resistor (instead of the internal one),
// bus operations are performed differently.
#define EXTERNAL_PULL_UP

// baud rate for controlling Arduino
#define BAUD_RATE 115200

// number of pulses to wait for synchronization:
// we wait until we get a LOW -> HIGH -> LOW pulse that lasts for at least 5 ms on
// HIGH level: The level Is HIGH for 20ms (PAL) resp. 16.6ms (NTSC) between two telegrams.
// Level is also HIGH between two bytes of the same telegram, but this pulse is considerably
// shorter than 5ms (1.2 to 1.4 ms).
// Thus, 5ms seems a good compromise.
// (cf. http://www.boehmel.de/lanc)
//
// We will wait at most 50 pulses until we consider synchronization failed:
// In the worst case, each byte has value 0b01010101, resulting in the largest
// possible values of pulses per byte and telegram, namely 4 per byte and
// a total of 7 intra-byte separator pulses. This makes 39 HIGH-pulses,
// the 40th is the actual inter-frame synchronization gap.
// We increase this number to 50 in order to compensate some possible glitches etc.
#define SYNC_TRIES 50

// number of times a command is repeated while waiting for the acknowledge.
// In either case, repeating is canceled once acknowledge has been received,
// but if no acknowledge is received after the given number of repeats,
// it's considered a timeout.
#define NUM_REPEATS_EEPROM_READ  4
#define NUM_REPEATS_EEPROM_WRITE 4

// repeat "normal" commands four times.
// Camera seems to respect them after a single transmission, but only
// seems to acknowledge them after the fourth transmission...
#define NUM_REPEATS_NORMAL		 4

// input pin configuration
// Arduino digital pin number (required for pulseIn function)
#define BUS_PIN_ARDUINO 4
// number within register
#define BUS_PIN_NUMBER  PD4
// registers for direct I/O access
#define BUS_DDR  DDRD
#define BUS_PIN  PIND
#define BUS_PORT PORTD

/////////////////////////////////////////////////////////////////////
// MACROS FOR ACCESSING PORTS
/////////////////////////////////////////////////////////////////////

// since the bit-banging code is timing-sensitive, we use bare-metal accesses via port registers:
// It takes ~3.2us to set a pin state via the digitalWrite command
// It takes ~80ns  to set a pin state when directly accessing the registers

// LED pin is PB5
#define turnLedOn()  do { PORTB |=  (1 << PB5); } while (0)
#define turnLedOff() do { PORTB &= ~(1 << PB5); } while (0)

#ifdef EXTERNAL_PULL_UP // using external pull-up
	// initialize bus: configure pin as input and disable internal pull up
	#define busInitialize() do { BUS_DDR &= ~(1 << BUS_PIN_NUMBER); BUS_PORT &= ~(1 << BUS_PIN_NUMBER); } while (0)

	// switch to input (which is pulled up externally).
	// thus, the input will be high unless it is pulled down externally (by camera)
	#define busPullUp()     do { BUS_DDR &= ~(1 << BUS_PIN_NUMBER); } while (0) // DDR = 0 => input (since we set PORT = 0 in busInitialize(), internal pull-up is disabled)

	// switch to output driven low, forcing bus LOW.
	#define busForceLow()   do { BUS_DDR |= (1 << BUS_PIN_NUMBER); } while (0)  // DDR = 1 => output (since we set PORT = 0 in busInitialize(), output is LOW)
#else // using internal pull-up
	#define busInitialize() do { BUS_DDR &= ~(1 << BUS_PIN_NUMBER); BUS_PORT |= (1 << BUS_PIN_NUMBER); } while (0) // DDR = 0, PORT = 1 => input, pulled up

	// switch to input with pull-up.
	// this mode is also used if we don't want to send a logical one (LOW level) as it doesn't force the bus
	// to a specific level; other bus participants (i.e. camera) can force it LOW at their discretion
	#define busPullUp()     do { BUS_DDR &= ~(1 << BUS_PIN_NUMBER); BUS_PORT |= (1 << BUS_PIN_NUMBER); } while (0) // DDR = 0, PORT = 1 => input, pulled up

	// switch to output driven low, forcing bus LOW.
	// this is done in reverse to above: first disable pull-up (BUS_PORT = 0), shortly making input tri-state.
	// This is preferable to shortly driving bus HIGH when configuring DDR to output first.
	#define busForceLow()   do { BUS_PORT &= ~(1 << BUS_PIN_NUMBER); BUS_DDR |= (1 << BUS_PIN_NUMBER); } while (0) // DDR = 1, PORT = 0 => output, driven low
#endif // EXTERNAL_PULL_UP

// reading current bus value (only works while bus is configured as input, see above)
#define busValue   (BUS_PIN & (1 << BUS_PIN_NUMBER))

/////////////////////////////////////////////////////////////////////
// TIMING CONSTANTS
/////////////////////////////////////////////////////////////////////

// duration of one LANC bit in microseconds:
// corresponds to 9600 baud RS-232:
// 1,000,000 us divided by 9600 bit/second.
#define BIT_DURATION (1000000 / 9600)
// half of BIT_DURATION
#define HALF_BIT_DURATION (BIT_DURATION / 2)

/////////////////////////////////////////////////////////////////////
// COMMANDS
/////////////////////////////////////////////////////////////////////

// first byte of EEPROM-accessing commands
#define CMD_BYTE0_EEPROM 0xFF
// first byte of normal operation commands (VTR or video camera)
#define CMD_BYTE0_NORMAL 0x18

// second byte of commands we send,
// all sent via CMD_BYTE0_EEPROM
#define CMD_READ 0x00
#define CMD_PAGE_INC 0x67
#define CMD_PAGE_DEC 0x65
#define CMD_ADDR_INC 0x38
#define CMD_ADDR_DEC 0x36
#define CMD_DATA_INC 0x34
#define CMD_DATA_DEC 0x30
#define CMD_STORE 0x32

// CMD_READ is acked by 0xF0 in frame[5], writes are acked by 0xF1
#define ACK_READ 0xF0
#define ACK_WRITE 0xF1

// sent via CMD_BYTE0_NORMAL
#define CMD_NORMAL_REC 0x3A
#define CMD_NORMAL_RECPAUSE	0x3C

// EEPROM commands are acknowledged in byte 5
#define ACK_INDEX_EEPROM 5
// normal commands are acknowledged in byte 4
#define ACK_INDEX_NORMAL 4

/////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES
/////////////////////////////////////////////////////////////////////

// frame data: 0-3 are transmit buffer, 4-7 receive buffer.
// (frame[2] and frame[3] are always sent as 0x00)
uint8_t frame[8];

bool do_send(uint8_t byte0, uint8_t command, uint8_t num_repeats, bool needs_acknowledge, uint8_t acknowledge_index, uint8_t acknowledge)
{
	uint8_t sync_tries = 1;
	uint8_t buffer;
	uint8_t repeats;

	// initialize all bytes in write buffer
	frame[0] = byte0;
	frame[1] = command;
	frame[2] = 0x00;
	frame[3] = 0x00;

	// Sync to next LANC message:
	// wait for LOW -> HIGH -> LOW transition that rests at least 5ms on HIGH level.
	// The inter-telegram gap should be 20ms (PAL) resp. 16.6ms (NTSC)
	// while the inter-byte gap should be no longer than 1.4ms.
	// Thus, 5ms should be fine for detecting a new telegram's start bit.

	// configure bus pin as (possibly internally pulled-up) input.
	// This shouldn't be required as it is already initialized correctly in main()
	// as well as after every transmitted bit.
	// but it doesn't hurt to make sure.
	busPullUp();
	while (pulseIn(BUS_PIN_ARDUINO, HIGH) < 5000)
	{
		if (sync_tries++ > SYNC_TRIES)
		{
			Serial.println(F("Syncing to LANC message failed"));
			return false;
		}
	}

	for (repeats = 0; repeats < num_repeats; repeats++)
	{
		turnLedOn();

		for (uint8_t curr_byte = 0; curr_byte < 8; curr_byte++)
		{
			// wait for middle of start bit
			delayMicroseconds(HALF_BIT_DURATION);

			// only the first 4 bytes are transmitted, the last 4 bytes are received!
			bool transmit = curr_byte < 4;

			// if transmitting, use input data. if receiving, use empty buffer.
			buffer = transmit ? frame[curr_byte] : 0;

			for (uint8_t curr_bit = 0; curr_bit < 8; curr_bit++)
			{
				// wait for start of next bit
				delayMicroseconds(HALF_BIT_DURATION);

				if (transmit)
				{
					if (buffer & 0x1)
					{
						// force bus pin DOWN
						busForceLow();
					}
					else
					{
						// don't force bus pin DOWN: switch to input with pull-up
						// (other bus participants may pull bus down by connecting to GND anyway,
						//  but we won't do so: we effectively output a weak HIGH)
						busPullUp();
					}
				}

				// wait for middle of bit
				delayMicroseconds(HALF_BIT_DURATION);

				// if receiving, read level at middle of bit
				if (!transmit)
				{
					if (!busValue)
					{
						// bus is LOW, thus logical 1. Set LSB
						buffer |= 0x01;
					}
					// else: bus is HIGH, thus logical 0.
					// Nothing to do in this case as we started out with
					// an empty buffer value.
				}

				// rotate buffer right
				buffer = (buffer >> 1) | (buffer << 7);
			}

			// wait for end of last bit
			delayMicroseconds(HALF_BIT_DURATION);

			if (transmit)
			{
				// We were transmitting.
				// Since this means we might still be forcing the bus DOWN (last written bit was 1),
				// we have to switch back to input with pull-up
				busPullUp();
			}
			else
			{
				// we were receiving. Write received data back to buffer.
				// In fact, we could even write transmitted bytes back since
				// we rotated it _eight_ times.
				frame[curr_byte] = buffer;
			}

			if (repeats == (num_repeats - 1) && curr_byte == 7)
			{
				// if this is the last repeat we're sending _and_ the last byte
				// of the current telegram, there is no need to wait for the start
				// of the next byte:
				// this would wait for the start of the next telegram, which is
				// about 20ms wasted time (for PAL).
				break;
			}
			// wait for start of next byte.
			// first, ensure we're in the middle of the stop bit (which has HIGH level)
			delayMicroseconds(HALF_BIT_DURATION);
			while (busValue)
			{
				// second, loop while bus is HIGH (stop bit)
			}
			// third, bus is LOW again: this is the start bit of the next byte
		}

		turnLedOff();

		if (needs_acknowledge && frame[acknowledge_index] == acknowledge)
		{
#ifdef PRINT_SYNC_REPEATS
			Serial.print(sync_tries, DEC);
			Serial.print(F(" sync tries, "));
			Serial.print(repeats, DEC);
			Serial.println(F(" repeats"));
#endif

			return true;
		}
	}

#ifdef PRINT_SYNC_REPEATS
	Serial.print(sync_tries, DEC);
	Serial.print(F(" sync tries, "));
	Serial.print(repeats, DEC);
	Serial.println(F(" repeats"));
#endif

	if (!needs_acknowledge) {
		return true;
	}

	Serial.print(F("Timeout waiting for acknowledge of command 0x"));
	Serial.print(command, HEX);
	Serial.println("");
	return false;
}

bool send_eeprom_command(uint8_t command)
{
	if (command != CMD_READ && command != CMD_PAGE_INC && command != CMD_PAGE_DEC && command != CMD_ADDR_INC && command != CMD_ADDR_DEC && command != CMD_DATA_INC && command != CMD_DATA_DEC && command != CMD_STORE)
	{
		Serial.println(F("Invalid command in send_eeprom_command"));
		return false;
	}

	if (command != CMD_READ)
	{
		// perform actual command requested

		if (!do_send(CMD_BYTE0_EEPROM, command, NUM_REPEATS_EEPROM_WRITE, true, ACK_INDEX_EEPROM, ACK_WRITE))
		{
			Serial.println(F("Write command failed"));
			return false;
		}
	}

	// perform a read command.
	// this is either the requested command (command == CMD_READ) or an automatic follow-up read command after
	// some other (write) command.
	// The reason for doing a follow-up read command is that every write command also returns all the information
	// about current page/address/data we require UNLESS write access has not been enabled yet:
	// In this case, reading certain pages fails (command is not ACKed), but write commands (e.g. incrementing
	// the address or even moving to the locked page) return data = 0x00, thus an invalid value.
	// Therefore, we gather the information about current page/address/data from a follow-up read command.
	//
	// In fact, RM95EMUL does the same: After each write, a read is performed.
	if (!do_send(CMD_BYTE0_EEPROM, CMD_READ, NUM_REPEATS_EEPROM_READ, true, ACK_INDEX_EEPROM, ACK_READ))
	{
		Serial.println(F("Read failed. Current page might be locked."));
		return false;
	}

	return true;
}

bool send_normal_command(uint8_t command, bool needs_acknowledge, uint8_t acknowledge)
{
	return do_send(CMD_BYTE0_NORMAL, command, NUM_REPEATS_NORMAL, needs_acknowledge, ACK_INDEX_NORMAL, acknowledge);
}

void loop()
{
	// make sure input buffer is empty so that no spurious command is received
	// during next loop() invocation
	while (Serial.available() > 0)
	{
		Serial.read();
	}

	Serial.println(F(
		"\r\n"
		"Normal commands:\r\n"
		"0 - Start recording\r\n"
		"EEPROM commands:\r\n"
		"e - Increment Page       r - Decrement Page\r\n"
		"d - Increment Address    f - Decrement Address\r\n"
		"c - Increment Data       v - Decrement Data\r\n"
		"s - Store              any - Read current Page/Address\r\n"));

	// wait for command
	while (Serial.available() == 0)
	{
		continue;
	}

	char input = (char)Serial.read();

	// make sure input buffer is empty so that no spurious command is received next loop iteration.
	while (Serial.available() > 0)
	{
		Serial.read();
	}

	uint8_t command;
	bool eeprom_command = true;
	bool normal_needs_acknowledge = false;
	uint8_t normal_acknowledge = 0xFF; // dummy value

	Serial.print(F("Sending "));
	switch (input)
	{
	case 'E':
	case 'e': // increment page
		command = CMD_PAGE_INC;
		Serial.print(F("increment page"));
		break;
	case 'R':
	case 'r': // decrement page
		command = CMD_PAGE_DEC;
		Serial.print(F("decrement page"));
		break;
	case 'D':
	case 'd': // increment address
		command = CMD_ADDR_INC;
		Serial.print(F("increment address"));
		break;
	case 'F':
	case 'f': // decrement address
		command = CMD_ADDR_DEC;
		Serial.print(F("decrement address"));
		break;
	case 'C':
	case 'c': // increment data
		command = CMD_DATA_INC;
		Serial.print(F("increment data"));
		break;
	case 'V':
	case 'v': // decrement data
		command = CMD_DATA_DEC;
		Serial.print(F("decrement data"));
		break;
	case 'S':
	case 's': // store
		command = CMD_STORE;
		Serial.print(F("store"));
		break;
	case '0': // record
		command = CMD_NORMAL_REC;
		eeprom_command = false;
		normal_needs_acknowledge = true;
		normal_acknowledge = 0x04;
		Serial.print(F("record"));
		break;
	default: // anything else, perform a read
		command = CMD_READ;
		Serial.print(F("read"));
		break;
	}
	Serial.println(F(" command..."));

	if (eeprom_command)
	{
		if (!send_eeprom_command(command))
		{
			Serial.println(F("\r\nCommand failed!"));
		}
		else
		{
			// print current data
			
			uint8_t page = frame[4] >> 4;
			Serial.print(F("\r\nPage: "));
			if (page < 16)
			{
				Serial.print(F("0")); // leading zero if required
			}
			Serial.print(page, HEX);

			Serial.print(F("  Addr: "));
			if (frame[6] < 16)
			{
				Serial.print(F("0")); // leading zero if required
			}
			Serial.print(frame[6], HEX);

			Serial.print(F("  Data: "));
			if (frame[7] < 16)
			{
				Serial.print(F("0")); // leading zero if required
			}
			Serial.print(frame[7], HEX);

			Serial.println(F(""));
		}
	}
	else
	{
		if (!send_normal_command(command, normal_needs_acknowledge, normal_acknowledge))
		{
			Serial.print(F("\r\nCommand failed with status 0x"));
			Serial.print(frame[4], HEX);
			Serial.println(F("!"));
		}
	}
}

__attribute__ ((OS_main)) int main(void)
{
	init();

	// Initially configuration of bus pin
	busInitialize();

	// Configure LED pin as output
	DDRB |= (1 << PB5);

	Serial.begin(BAUD_RATE);

	Serial.println(F("Arduino RM95 Remote Emulator v1.0"));
	
	// According to http://www.boehmel.de/lanc,
	// "Connect LANC Signal to GND for more than 140ms to power on"
	// this doesn't seem to be needed, works fine without it.

	while (true)
	{
		loop();
	}
}
