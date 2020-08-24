# Arduino RM-95 Emulator

## Introduction

I recently got a Digital8 camcorder (DCR-TRV 120E) to digitize some Hi8 recordings.
In order to also digitize some VHS tapes, I wanted to use its Analog-in to DV-out feature as I wouldn't need to buy any additional analog to digital converter. But, being a PAL model, this feature was disabled.

Luckily, it is possible to enable it by patching the camcorder's EEPROM.
This is possible via the LANC bus (an open-collector bus for remotely controlling the camcorder using a 2.5mm headphone jack).
The process is usually called "DV-In enable" (since the primary use case is to enable DV input on these camcorders; analog-in is merely a side effect to enabling DV input)

In the past, different solutions have been developed to interface with the LANC bus.
Usually, some circuitry was used to connect a PC's parallel port to LANC along with some software to emulate a RM-95 remote control.

Unfortunately, in 2020, i was a bit late to the party:
Few people use still use Digital8 camcorders.
Most web pages dealing with DV-In don't exist any longer (although the Internet Archive comes to the rescue) and most RM-95 emulator software is increasingly hard to find.
But worst of all, parallel ports are quite uncommon on today's computers and the emulator tools only work on ancient versions of Windows.

Long story short, I wanted to save money by using my camcorder as an analog to digital converter.

Luckily, somebody on the internet had [the idea to use an Arduino for interfacing with the LANC bus](https://create.arduino.cc/projecthub/L-Rosen/serial-to-lanc-control-l-70f735), basically bridging UART to LANC.
However, there were two problems I had with this solution:

1. The supplied schematics required too many components: A transistor, a zener diode and two resistors. This is probably due to being adapted from the parallel port adapters that required different pins for reading and writing the bus. Since the Arduino's microcontroller supports tri state pins, this can be greatly simplified.
1. The text-based user interface was very rudimentary, requiring to input raw LANC commands instead of simply pressing some keys that are translated to appropriate commands.
1. I didn't like most of the code.

Thus, I modified the code and schematics.
The send/receive routines are heavily inspired by a GPL-licensed RM-95 emulator for Linux, see the source code for credits.

## Usage

You will need an Arduino Nano, an Arduino Pro Mini (5 V variant) should also work, although this hasn't been tested.

Choose from one of the following two pin-outs:

### "Complicated" pin-out

* Connect Arduino `D4` to the LANC signal line (headphone jack tip) and pull it up to `+5V` using a 4.7k resistor.
* Connect Arduino `GND` to LANC GND (headphone jack sleeve)
* Leave LANC VCC (headphone jack ring) unconnected (we power the Arduino using USB)

Make sure to `#define EXTERNAL_PULL_UP` in the code.

(Actually, `D4` can be adjusted to your needs, see the code)

### Simple pin-out

Same as above, but no pull-up resistor is needed. Instead, the microcontroller's internal pull-ups are used.
However, this might not work as reliably as using an external pull-up (but in my case, it worked).

Do **not** `#define EXTERNAL_PULL_UP` in the code.

### Software usage
In either case, compile the code and upload it to your Arduino.
Use the serial monitor (configured to 115200 baud, although this can be changed as needed in the code) to interface with the Arduino LANC remote:

````
Normal commands:
0 - Start recording
EEPROM commands:
e - Increment Page       r - Decrement Page
d - Increment Address    f - Decrement Address
c - Increment Data       v - Decrement Data
s - Store              any - Read current Page/Address
````

Just press the mentioned keys (and return) and the remote will execute the appropriate command.
Successful EEPROM commands will always return the current page/address/data values.

If the interface seems to be locked up after pressing a key, the Arduino can't sync to the LANC start bit.
In this case, double-check your wiring and your camcorder.

## Closing remarks

I successfully used above pinouts with this code to enable DV-In on both a DCR-TRV 120E and a DCR-TRV 210E (the latter one doesn't support Analog to DV pass-through but only recording digitized analog signals to Digital8 tape).

However, your mileage may vary and I neither take any responsibility nor give warranty of any kind.
The schematics and code worked for me and my setup.
Yet, you might fry your Arduino (if your camcorder somehow uses different voltage levels than mine) or even brick your camcorder:
Corrupting its EEPROM will likely render it unusable.

As said before, it worked for me.
It might work for you, too; it might cause your equipment to halt and catch fire. You have been warned.
