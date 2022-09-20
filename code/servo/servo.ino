// rc_servo_sample1

#include <Servo.h>
Servo myservo1;
Servo myservo2;

const int RservoPIN = 7;
const int LservoPIN = 3;

//int pos = 0;

void setup() {
  // put your setup code here, to run once:
  myservo1.attach( RservoPIN );
  myservo2.attach( LservoPIN );
}

void loop() {
  // put your main code here, to run repeatedly:
  myservo1.write( 0 );
  delay(500);
  myservo2.write( 180 );
  delay(500);
  myservo1.write(180);
  delay(500);
  myservo2.write(0);
  delay(500);
  //for ( pos = 180; pos > 0; pos -= 1 ) {
//  myservo.write( 180 );
//  delay( 15 );
//  }

}
