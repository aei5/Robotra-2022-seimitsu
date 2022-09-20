// PID制御でのライントレースを行なうプログラム
// ひとまずこれを基に動作させる
// 長くて読みにくいので一部関数化, ライブラリ化を図る


//  サーボモータードライバ設定
#include<Wire.h>
#include <PCA9685.h> //PCA9685用ヘッダーファイル

PCA9685 pwm = PCA9685(0x40);    //PCA9685のアドレス指定

#define SERVOMIN 150            //最小パルス幅 
#define SERVOMAX 500            //最大パルス幅500


//  センサ設定
#include <QTRSensors.h>

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {3, 4, 5, 6, 7, 8, 9, 10},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];


//// ユーザ設定 /////

//  キャリブレーションの設定(ラインセンサ)
#define LED_PIN       13    // キャリブレーション時に使用するデジタルピン(LEDの点灯)


//  PIDの定数. 限界感度法を参照
#define DELTA_T       0.02
#define KP            2.5
#define KD            3.3
#define KI            0.1


// 各種モーターパラメータ
#define baseSpeedL    170    //  ロボットがなんらの影響も受けない場合の直進速度
#define baseSpeedR    10
#define target        3500   //  なんか端っこに行く。センサの高さか
#define refL          0.7    //  制御量の適応割合. 倍率
#define refR          0.68   //  制御量の適応割合




///// main program /////

void setup(){

  pwm.begin();                   //初期設定 (アドレス0x40用)
  pwm.setPWMFreq(50);            //PWM周期を50Hzに設定 (アドレス0x40用)

  calibration();        //calibration動作を行なう

  Serial.begin(9600);            //センサ値観測始め. 以降持続して行なわれる

  }


void loop(){
  
  LineTrace();
  slarom();
  
  
}

////////////////////////



////////関数////////

void calibration(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  delay(500);
  int i = 0;
  
    servo_write(1,5);//1が右で前向き
    servo_write(0,5);
    for (int i = 0; i < 10; i++)  // make the calibration take about 1 seconds
    {
      qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    }
    
    servo_stop();
    for (int i = 0; i < 20; i++)  // make the calibration take about 1 seconds
    {
      qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    }
    
  while(i<=3){
    servo_write(1,174);//1が右で前向き
    servo_write(0,174);
    for (int i = 0; i < 20; i++)  // make the calibration take about 1 seconds
    {
      qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    }
    
    servo_stop();
    for (int i = 0; i < 20; i++)  // make the calibration take about 1 seconds
    {
      qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    }
    
    servo_write(1,6);
    servo_write(0,0);
    for (int i = 0; i < 20; i++)  // make the calibration take about 1 seconds
    {
      qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    }

    servo_stop();
    for (int i = 0; i < 20; i++)  // make the calibration take about 1 seconds
    {
      qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    }
    
    i++;
  }

    servo_write(1,175);//1が右で前向き
    servo_write(0,175);
    for (int i = 0; i < 20; i++)  // make the calibration take about 1 seconds
    {
      qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    }
    
    servo_stop();
    for (int i = 0; i < 20; i++)  // make the calibration take about 1 seconds
    {
      qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    }

  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
}

void servo_write(int ch, int ang){ //動かすサーボチャンネルと角度を指定
  ang = map(ang, 0, 180, SERVOMIN, SERVOMAX); //角度（0～180）をPWMのパルス幅（150～500）に変換
  pwm.setPWM(ch, 0, ang);
}

void servo_stop(){ //ロボットの移動を停止する
  servo_write(1,87);
  servo_write(0,88);
}

float pidLeft(signed short sensor_val, signed short target_val){ //左車輪のpid
  static signed long diff_L[2];
  static float integral_L;

  float p, i, d;

  diff_L[0] = diff_L[1];
    if((round(abs((sensor_val - target_val) - diff_L[0])))>2000){  //  外れ値の除外, 前回観測した偏差と今回観測した偏差の差が2000以上で棄却
    diff_L[1] = diff_L[0];
  }
  else{
    diff_L[1] = sensor_val - target_val; //偏差を取得
  }
  integral_L += (diff_L[1] + diff_L[0]) / 2 * DELTA_T;

  p = KP * diff_L[1];
  i = KI * integral_L;
  d = KD * (diff_L[1] - diff_L[0]) / DELTA_T;

  return math_limit(p+i+d);
  }


float pidRight(signed short sensor_val, signed short target_val){ //右車輪のpid
  static signed long diff_R[2];
  static float integral_R;

  float p, i, d;

  diff_R[0] = diff_R[1];
  if(round(abs((sensor_val - target_val) - diff_R[0]))>2000){  //外れ値の除外
    diff_R[1] = diff_R[0];
  }
  else{
    diff_R[1] = sensor_val - target_val; //偏差を取得
  }
  integral_R += (diff_R[1] + diff_R[0]) / 2 * DELTA_T;
  p = KP * diff_R[1];
  i = KI * integral_R;
  d = KD * (diff_R[1] - diff_R[0]) / DELTA_T;

  return math_limit(p+i+d);
  }


float math_limit(float pid) {
  pid = constrain(pid, -90 , 90); //定義域-90 <= pid <= 90

  return pid;
}


void LineTrace(){                  //float manipulate(baseSpeedL,refL, pidLeft, baseSpeedR,refR, pidRight ){
  while(1){                        //whileは無理か. 操作量のみを返す変数になりそう?
    //モーター設定
    //左モーターの動き
    float morterSpeedL = baseSpeedL - refL*pidLeft(target, position); //-
    servo_write(0,morterSpeedL);   //return morterSpeedL;

    //右モーターの動き
    float morterSpeedR = baseSpeedR + refR*pidRight(target, position); //+
    servo_write(1,morterSpeedR); 
  }
