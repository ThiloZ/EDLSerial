# EDLSerial

Arduino library for decoding ECUMaster EMU Serial Logger stream (EDL-1 mode).

## Benefit over traditional serial protocol
- more channels available
- faster due to higher baud rate
- let you use EDL-1 and custom dash/gauges in parallel

## Features
- Parses 260-byte EDL frames
- Exposes engine data like RPM, Lambda, IAT, EGT, etc.
- Works on any Arduino-compatible board with UART

## Disclamer
- tested with EMU Classic FW version 1.226
- tested with ESP32
- not all channels are tested, please contact me if you notice something strange
- CAN channels for EGT and wheelspeed not implemented

## Hardware
- ECU Master ECUs (both Classic and Black) use RS232 (15V), so for an Arduino or ESP32 you will need an converter board to TTL (5V/3.3V)! MAX3232CPE works just fine.
 
## Usage
Default all available data streamed by the EMU is captured and parsed. For performance and memory optimisation it's strongly advised to comment or delete unused channels. Therefore you have to modify the library on your own. If you do so make shure to modify both files below the same way!

EDLTypes.h
 -> from line 7 to 201 all available channels are definded

EDLSerial.cpp
 -> from line 29 to 223 channels are parsed

```cpp
#include <EDLSerial.h>

EDLSerial edl;

void setup() {
  Serial.begin(115200);  // default serial to print to console
  Serial1.begin(115200); // RS232 from EMU
  edl.begin(Serial1);
}

void loop() {
  if (edl.update()) {                   //updates the frame, returns true if frame is valid
    Serial.println(edl.getFrame().RPM); //returns value from last captured frame
	Serial.println(edl.getFrame().MAP);
	Serial.println(edl.getFrame().TPS);
	Serial.println(edl.getFrame().wboLambda);
  }
}
```
