const int MOTOR_PIN=5; //mosfetに接続した端子を指定

#define trigPin 8
#define echoPin 9

float Duration=0;
float Distance=0;


// サーボ設定
#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int potpin = A0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

void setup() {
  //pinMode(MOTOR_PIN,OUTPUT);
  //接続した端子を出力モードにする:

  Serial.begin(9600); //シリアルモニタの開始

  pinMode(echoPin,INPUT); //エコーピンを入力に
  pinMode(trigPin,OUTPUT); //トリガーピンを出力に

  myservo.attach(A0);

}

void loop() {

  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);

  Duration = pulseIn(echoPin,HIGH);
  Duration = Duration / 2;
  Distance = Duration * 340 * 100 / 1000000;

  if(Distance >= 400 || Distance <= 2){
    Serial.println("距離 = 測定範囲外");
  }

  else{
    Serial.print("距離 ");
    Serial.print(Distance);
    Serial.println(" cm");
    
  }
      delay(100);

  val = map(Distance, 2, 50, 0, 180);     // scale it for use with the servo (value between 0 and 180)
  myservo.write(val);                  // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there  


  
  //digitalWrite(MOTOR_PIN,HIGH);
  //delay(1000);

  //digitalWrite(MOTOR_PIN,LOW);
  //delay(1000);

}
