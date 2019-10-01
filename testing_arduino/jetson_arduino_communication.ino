#include <Servo.h>

Servo servo;  // create servo object to control a servo

void setup() {
  // start serial port at 9600 bps:
  Serial.begin(9600);
  // initialize digital pin LED_BUILTIN as an output.
  //pinMode(LED_BUILTIN, OUTPUT);
  
  servo.attach(4);  // attaches the servo on pin 9 to the servo object

  while (!Serial) {
    ; // wait for serial port to connect.
  }

}

void loop() {
  char buffer[16];
  // if we get a command, turn the LED on or off:
  if (Serial.available() > 0) {
    for( int i = 0; i < sizeof(buffer);  i++ )
      buffer[i] = (char)0;
      
    int size = Serial.readBytesUntil('\n', buffer, 12);
    int angle = atoi(buffer);
    Serial.println(angle);
    
    servo.write(angle);
    
    // if (buffer[0] == 'Y') {
    //   digitalWrite(LED_BUILTIN, HIGH);
    // }
    // if (buffer[0] == 'N') {
    //   digitalWrite(LED_BUILTIN, LOW);
    // }
  }
}
