#include <EDLSerial.h>

EDLSerial edl;

void setup() {
  Serial.begin(115200);  // default serial to print to console
  Serial1.begin(115200); // RS232 from EMU
  edl.begin(Serial1);
}

void loop() {
  if (edl.update()) {                   //updates the frame, returns true if frame is valid
    const auto &frame = edl.getFrame();
	Serial.println(frame.rpm);
	Serial.println(frame.map);
	Serial.println(frame.tps);
	Serial.println(frame.wboLambda);
  }
}
