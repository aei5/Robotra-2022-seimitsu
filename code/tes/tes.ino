
#include <QTRSensors.h>

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {3, 4, 5, 6, 7, 8, 9, 10}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];


#include<Wire.h>
#include <PCA9685.h> //PCA9685用ヘッダーファイル

PCA9685 pwm = PCA9685(0x40);    //PCA9685のアドレス指定

#define SERVOMIN 150            //最小パルス幅 
#define SERVOMAX 600            //最大パルス幅


void setup()
{
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // シリアルモニタの最初に, 各センサの, キャリブレーション時に観測した最大最小値を表示する
  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println(); //ただの改行

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  pwm.begin();                   //初期設定 (アドレス0x40用)
  pwm.setPWMFreq(50);            //PWM周期を50Hzに設定 (アドレス0x40用)
}


void loop()
{
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  unsigned int position = qtrrc.readLine(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.println(position); // comment this line out if you are using raw values

  delay(250);

  int rightSpeed = 0;
  int leftSpeed = 180;
  servo_write(0, rightSpeed);
  servo_write(1, leftSpeed);


  int error = position - 3500;

  if(error > 500)
  {
    rightSpeed = 90;
    servo_write(0, leftSpeed);
    servo_write(1, rightSpeed);

  }
  else if(error < -500)
  {
    leftSpeed = 90;
    servo_write(0, leftSpeed);
    servo_write(1, rightSpeed);
  }



}

void servo_write(int ch, int ang){ //動かすサーボチャンネルと角度を指定
  ang = map(ang, 0, 180, SERVOMIN, SERVOMAX); //角度（0～180）をPWMのパルス幅（150～500）に変換
  pwm.setPWM(ch, 0, ang);
}
