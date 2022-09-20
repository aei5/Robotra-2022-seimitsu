#include <QTRSensors.h>
 
// create an object for your type of sensor (RC or Analog)
// in this example we have three sensors on analog inputs 0 - 2 (digital pins 14 - 16)
QTRSensorsRC qtr((char[]) {2, 3, 4, 5, 6, 7, 8, 9}, 8);
// QTRSensorsA qtr((char[]) {0, 1, 2}, 3);

#include<Wire.h>
#include <PCA9685.h> //PCA9685用ヘッダーファイル

PCA9685 pwm = PCA9685(0x40);    //PCA9685のアドレス指定

#define SERVOMIN 150            //最小パルス幅 
#define SERVOMAX 600            //最大パルス幅


void setup()
{
  // optional: wait for some input from the user, such as  a button press
 
  // then start calibration phase and move the sensors over both
  // reflectance extremes they will encounter in your application:
  int i;
  for (i = 0; i < 250; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
    Serial.begin(9600);
  }

  //servo setting:
  pwm.begin();                   //初期設定 (アドレス0x40用)
  pwm.setPWMFreq(50);            //PWM周期を50Hzに設定 (アドレス0x40用)
  }


int leftMotorSpeed = 100;
int rightMotorSpeed = 100;

void loop()
{
  unsigned int sensors[8];
  // get calibrated sensor values returned in the sensors array, along with the line
  // position, which will range from 0 to 2000, with 1000 corresponding to the line
  // over the middle sensor.
  int position = qtr.readLine(sensors);
  Serial.println(position);
 
  // if all three sensors see very low reflectance, take some appropriate action for this 
  // situation.
  if (sensors[0] > 750 && sensors[3] > 750 && sensors[7] > 750)
  {
    // do something.  Maybe this means we're at the edge of a course or about to fall off 
    // a table, in which case, we might want to stop moving, back up, and turn around.
    return;
  }
 
  // compute our "error" from the line position.  We will make it so that the error is
  // zero when the middle sensor is over the line, because this is our goal.  Error
  // will range from -1000 to +1000.  If we have sensor 0 on the left and sensor 2 on
  // the right,  a reading of -1000 means that we see the line on the left and a reading
  // of +1000 means we see the line on the right.
  int error = position - 3000;
 
  servo_write(1,100);
  servo_write(0,100);
  
  if (error < -500)  // the line is on the left
    //leftMotorSpeed = 0;  // turn right
    servo_write(1,90);
  if (error > 500)  // the line is on the right
    //rightMotorSpeed = 0;  // turn left
    servo_write(0,90);
 
  // set motor speeds using the two motor speed variables above
}

void servo_write(int ch, int ang){ //動かすサーボチャンネルと角度を指定
  ang = map(ang, 0, 180, SERVOMIN, SERVOMAX); //角度（0～180）をPWMのパルス幅（150～500）に変換
  pwm.setPWM(ch, 0, ang);
}
 
