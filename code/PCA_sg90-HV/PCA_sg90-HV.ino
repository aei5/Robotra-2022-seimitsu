#include<Wire.h>
#include <PCA9685.h> //PCA9685用ヘッダーファイル

PCA9685 pwm = PCA9685(0x40);    //PCA9685のアドレス指定

#define SERVOMIN 150            //最小パルス幅 
#define SERVOMAX 500            //最大パルス幅500→2400に改変

void setup() {
 pwm.begin();                   //初期設定 (アドレス0x40用)
 pwm.setPWMFreq(50);            //PWM周期を50Hzに設定 (アドレス0x40用)
}

void loop(){
  int i = 0;
  while(i<=10)
  {
    servo_write(1,174);//1が右
    servo_write(0,180);
    delay(500);
    
    servo_stop();
    delay(500);
    
    servo_write(1,6);
    servo_write(0,0);
    delay(500);

    servo_stop();
    delay(500);
    
    i++;
  }
}
  

void servo_write(int ch, int ang){ //動かすサーボチャンネルと角度を指定
  ang = map(ang, 0, 180, SERVOMIN, SERVOMAX); //角度（0～180）をPWMのパルス幅（150～500）に変換
  pwm.setPWM(ch, 0, ang);
}

void servo_stop(){ //ロボットの移動を停止する
    servo_write(1,89);
    servo_write(0,87);
}
