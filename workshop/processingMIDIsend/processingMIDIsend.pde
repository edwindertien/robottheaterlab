import themidibus.*; //Import the library
MidiBus myBus; // The MidiBus


void setup() {
  size(400, 400);
  background(0);
  //frameRate(20);
  MidiBus.list(); // List all available Midi devices on STDOUT. This will show each device's index and name.
  //myBus = new MidiBus(this, 0, 1); // Create a new MidiBus using the device index to select the Midi input and output devices respectively.                  |            |                     |
  //myBus = new MidiBus(this, "IncomingDeviceName", "OutgoingDeviceName"); // Create a new MidiBus using the device names to select the Midi input and output devices respectively.
  myBus = new MidiBus(this, 1, 2); // Create a new MidiBus with no input device and the default Java Sound Synthesizer as the output device.

}
int n;
void draw() {
n++;
if(n>127) n = 0;
myBus.sendControllerChange(1, 1, n);
}

void noteOn(int channel, int pitch, int velocity) {
  // Receive a noteOn
  println();
  println("Note On:");
  println("--------");
  println("Channel:"+channel);
  println("Pitch:"+pitch);
  println("Velocity:"+velocity);
  myBus.sendNoteOn(0,pitch,velocity);
}

void noteOff(int channel, int pitch, int velocity) {
  // Receive a noteOff
  println();
  println("Note Off:");
  println("--------");
  println("Channel:"+channel);
  println("Pitch:"+pitch);
  println("Velocity:"+velocity);
  myBus.sendNoteOff(0,pitch,velocity);
}

void controllerChange(int channel, int number, int value) {
  // Receive a controllerChange
  println();
  println("Controller Change:");
  println("--------");
  println("Channel:"+channel);
  println("Number:"+number);
  println("Value:"+value);
  myBus.sendControllerChange(channel, number, value);
}
