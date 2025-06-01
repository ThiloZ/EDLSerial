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