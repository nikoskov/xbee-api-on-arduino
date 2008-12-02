#include <SoftwareSerial.h>
#include <XBeeReader.h>


// Associate the xbee with the standard serial port.
XBeeReader xbee(Serial);
SoftwareSerial swserial = SoftwareSerial(11, 10);

void setup()
{
  int r;
  digitalWrite(13, HIGH);
  pinMode(13, OUTPUT);
  pinMode(10, OUTPUT);

  // Open the xbee at 57600 baud (optional second parameter, "force API mode", defaults to true,
  //  causing an ATAP1 to be sent.  This takes a few seconds to accomplish.
  // Default baud rate if none given is 9600.
  
  r = xbee.begin(57600);
  
  if (r < 0) {
      // If there was an error on startup, flash the LED forever.
      while(1) {
        digitalWrite(13, LOW);
        delay(100);
        digitalWrite(13, HIGH);
        delay(100);
      }
  }
  
  digitalWrite(13, LOW);
  swserial.begin(9600);
  swserial.println("SWSERIAL started");
}


void loop()
{
  byte addr[8];
  
  
  // Should be called every time through loop() to operate the xbee data pump
  xbee.poll();
  
  // Default address is 0xffff (broadcast).
  xbee.send("hi", 2);
  
  // If there's a packet available coming in, process it...
  if (xbee.available()) {
    XBeeDataFrame f;
    int digitalVals[9];
    int analogVals[6];
    int r;
    
    // Get packet reading into f
    xbee.getXBeeReading(f);
    
    swserial.print("pkt type: ");
    swserial.println(f.getApiID(), HEX);

    if (f.getAddress16(addr) >= 0) {
      swserial.print(" addr16: ");
      swserial.print(addr[0], HEX);
      swserial.println(addr[1], HEX);
    }
    
    r = f.getDigital(digitalVals);
    if (r >= 0) {
      swserial.print(" Digital lines (");
      swserial.print(r);
      swserial.print("): ");
      for (int i = 0; i < 9; i++) {
        if (digitalVals[i] == -1) {
          swserial.print("?");
        } else {
          swserial.print(digitalVals[i], HEX);
        }
      }
    }
    
    if (f.getAnalog(analogVals) >= 0) {
      swserial.print(" Analog lines (");
      swserial.print(r);
      swserial.print("): ");
      for (int i = 0; i < 6; i++) {
        swserial.print(analogVals[i]);
        swserial.print(" ");
      }
      swserial.println();
    }
    
    swserial.println();
  }
}
