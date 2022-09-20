#include<math.h>
#include <QTRSensors.h>

#define Encoder_pin_R 21           //右エンコーダ
#define Encoder_pin_L 20           //左エンコーダ

volatile int count_R=0;            //右エンコーダ
volatile int count_L=0;            //左エンコーダ

#define LED_G    38                //緑LED
#define LED_R    40                //赤LED
int SW_pin[5]={36,A13,42,A12,34};  //スイッチ
#define WRP       7                //右タイヤPWM
#define WR1       6                //右タイヤ1
#define WR2       5                //右タイヤ2
#define WLP       2                //左タイヤPWM
#define WL1       4                //左タイヤ1
#define WL2       3                //左タイヤ2
#define IN_pin   11                //受信ピン
#define OUT_pin  12                //送信ピン
#define PSD_pin  A8                //PSDセンサ

//ラインセンサー
QTRSensors qtr;
const uint8_t SensorCount = 9;
uint16_t sensorValues[SensorCount];

//エンコーダカウント--------------------------
void R_enc_changed(){count_R++;}
void L_enc_changed(){count_L++;}

float Dis_R(){return (count_R*57.8*M_PI)/12;}
float Dis_L(){return (count_L*57.8*M_PI)/12;}

//スイッチ用関数------------------------------
bool SW(int i){//0:Start 1:1 2:2 3:3 4:4
  if(digitalRead(SW_pin[i])){return false;}
  else                      {return  true;}
}

//PSDセンサ関数-----------------------------------
bool PSD() {
  if (analogRead(PSD_pin) > 400) {return true;}
  else                           {return false;}
}

//ラインセンサ---------------------------------
bool Line(int i){//コーンは8
  qtr.read(sensorValues);
  if(sensorValues[i]>=1800){ return true; }
  else                     { return false; }
}

//受信用-------------------------------------
bool IN(){
  if(digitalRead(IN_pin)==LOW){return true;}
  else                        {return false;}
}
void OUT(bool x){
  if(x){digitalWrite(OUT_pin,HIGH);}
  else {digitalWrite(OUT_pin,LOW);}
}

//走行Wheel(右速度、左速度)-100～100----------------
void Wheel(int x,int y) {//-10～10
  int ST_R=55,ST_L=55;
  int Rightspeed,Leftspeed;
  if(x>0){
    Rightspeed = x+ST_R;
    digitalWrite(WR1,HIGH);
    digitalWrite(WR2,LOW);
    analogWrite(WRP,Rightspeed);
  }
  else if(x==0){
    digitalWrite(WR1,HIGH);
    digitalWrite(WR2,HIGH);
  }
  else if(x<0){
    Rightspeed = -x+ST_R;
    digitalWrite(WR1,LOW);
    digitalWrite(WR2,HIGH);
    analogWrite(WRP,Rightspeed);
  }
  if(y>0){
    Leftspeed = y+ST_L;
    digitalWrite(WL1,HIGH);
    digitalWrite(WL2,LOW);
    analogWrite(WLP,Leftspeed);
  }
  else if(y==0){
    digitalWrite(WL1,HIGH);
    digitalWrite(WL2,HIGH);
  }
  else if(y<0){
    Leftspeed = -y+ST_L;
    digitalWrite(WL1,LOW);
    digitalWrite(WL2,HIGH);
    analogWrite(WLP,Leftspeed);
  }
}

//距離移動関数
void Move(int R,int L,int S){
  int R_S,L_S;           //左右スピード
  int D_count_R,D_count_L; //目標カウント数

  //スピード定義:マイナスならマイナス,ゼロならゼロ
  if     (R< 0){R_S=-S;R=-R;}
  else if(R==0){R_S=0;}
  else         {R_S=S;}
  
  if      (L<0){L_S=-S;L=-L;}
  else if(L==0){L_S=0;}
  else         {L_S=S;}
  
  D_count_R=round(((float)R*12)/(57.8*M_PI));
  D_count_L=round(((float)L*12)/(57.8*M_PI));
  
  count_R=0;         //エンコーダ初期化
  count_L=0;

  while(count_R < D_count_R ||count_L < D_count_L){
    Wheel(R_S,L_S);
  }
  Wheel(0,0);
  delay(500);
}

void Turn(byte x){//true:R,false:L
  if(x==1){
    Move(100,100,100);
    Wheel(0,0);
    Move(140,-140,140);
    Wheel(0,0);
    while(Line(3)==false){
      Wheel(140,-140);
    }
  }
  
  else if(x==2){
    Move(100,100,100);
    Wheel(0,0);
    Move(-100,100,140);
    Wheel(0,0);
    while(Line(4)==false){
      Wheel(-140,140);
    }
  }
  else if(x==3){
    Move(-300,300,150);
    Wheel(0,0);
    while(PSD()==false){
      Wheel(-150,0);
      delay(10);
    }
  }
  Wheel(0,0);
}

void Linetrace(int x){//1:HIGH,2:Middle,3:LOW
  byte HI,LO;
  if     (x==1){HI=200;LO=100;}
  else if(x==2){HI=150;LO=50;}
  else if(x==3){HI=150;LO=0;}
  else if(x==4){HI=100;LO=0;}
  
  if(Line(3) || Line(4)){
      Wheel(HI,HI);
    }
    else if(Line(2)||Line(1)){
      Wheel(LO,HI);
    }
    else if(Line(5) || Line(6)){
     Wheel(HI,LO);
    }
    else{
      Wheel(0,0);
    }
    delay(10);
}

void Linetrace_D(int x,int D){
  byte HI,LO;
  int D_count=round(((float)D*12)*2/(57.8*M_PI));//目標カウント数左右合計
  
  if     (x==1){HI=200;LO=100;}
  else if(x==2){HI=150;LO=50;}
  else if(x==3){HI=150;LO=0;}
  else if(x==4){HI=100;LO=0;}
  
  count_R=0;         //エンコーダ初期化
  count_L=0;
  
  while(count_R+count_L<D_count){
    if(Line(3) || Line(4)){
      Wheel(HI,HI);
    }
    else if(Line(0)||Line(1)||Line(2)){
      Wheel(LO,HI);
    }
    else if(Line(5) || Line(6)||Line(7)){
     Wheel(HI,LO);
    }
    else{
      Wheel(0,0);
    }
  }
  
  Wheel(0,0);
}

void setup() {//-------------------------------------------------------
  //シリアル通信
  Serial.begin(9600);
  
  //スイッチ定義
  pinMode(LED_G,OUTPUT);
  pinMode(LED_R,OUTPUT);
  int i;
  for(i=0;i<5;i++){pinMode(SW_pin[i],INPUT_PULLUP);}

  //通信用ピン
  pinMode(IN_pin,INPUT_PULLUP);
  pinMode(OUT_pin,OUTPUT);
  
  //エンコーダ用
  pinMode(Encoder_pin_R,INPUT_PULLUP);
  pinMode(Encoder_pin_L,INPUT_PULLUP);
  attachInterrupt(2, R_enc_changed, CHANGE); //pinAの信号変化に合わせて割り込み処理
  attachInterrupt(3, L_enc_changed, CHANGE); //pinAの信号変化に合わせて割り込み処理

  //PSDセンサ
  pinMode(PSD_pin,INPUT);
  
  //QTRセンサー
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7,A9}, SensorCount);

  //走行モータ用
  pinMode(WRP,OUTPUT);
  pinMode(WR1,OUTPUT);
  pinMode(WR2,OUTPUT);
  pinMode(WLP,OUTPUT);
  pinMode(WL1,OUTPUT);
  pinMode(WL2,OUTPUT);

  //startボタン
  while(SW(0)==false){
    delay(100);
  }
  
  digitalWrite(LED_G,HIGH); //緑LEDオン
  OUT(false);               //送信オフ
}

void loop(){
  //スタートから玉まで----------------------------------------------------
  while(Line(7)==false){
    Linetrace(4);
  }
  Wheel(0,0);
  delay(500);
  
  digitalWrite(LED_G,LOW);
  digitalWrite(LED_R,HIGH);
  
  Turn(1);
  while(Line(7)==false){
    Linetrace(4);
  }
  Wheel(0,0);
  delay(500);
  Move(0,500,100);
  Wheel(0,0);
  delay(500);
  
  for(int i=0;i<300;i++){
    Wheel(-100,0);
    delay(10);
    if(PSD()){break;}
  }
  Wheel(0,0);
  Move(-150,-150,100);
  Wheel(0,0);
  while(1);
  
  //玉から迷いの森-------------------------------------------------
  Move(140,0,120);                        //右向きに
  Wheel(0,0);
  Move(200,200,120);                      //少し直進
  Wheel(0,0);
  while(Line(3)==false||Line(4)==false){  //線に当たるまで直進
    Wheel(150,150);
    delay(10);
  }
  Move(100,100,150);                      //直進
  Wheel(0,0);
  while(Line(3)==false||Line(4)==false){  //ラインに復帰
    Wheel(150,-150);
    delay(10);
  }
  Linetrace_D(3,700);                     //低速ライントレース
  
  while(Line(0)==false||Line(7)==false){  //迷いの森までライントレース
    Linetrace(3);
  }
  Move(2200,2200,200);                    //迷いの森通過

  while(Line(0)==false && Line(7)==false){//線まで前進
    Wheel(100,100);
    delay(10);
  }
  
}
/*

  //迷いの森からゴール------------------------------------
  Turn(2);
  Linetrace_D(1,3400);       //一番長いところ
  digitalWrite(LED_R,HIGH);
  digitalWrite(LED_G,LOW);
  Linetrace_D(3,1600);       //最初のカーブ
  digitalWrite(LED_R,LOW);   
  digitalWrite(LED_G,HIGH);
  
  while(Line(0)==false){
    Linetrace(2);
  }
  
  Turn(2);                     //最初のL字
  Linetrace_D(3,1700);         //L字からクランクゾーン
  digitalWrite(LED_R,HIGH);
  digitalWrite(LED_G,LOW);
  while(Line(7)==false){       //ラストの直線へ
    Linetrace(2);
  }
  
  Turn(1);
  Linetrace_D(1,1200);         //ラスト直線
  digitalWrite(LED_R,LOW);
  digitalWrite(LED_G,HIGH);
  while(Line(7)==false){
    Linetrace(2);
  }
  
  //------------------------------

  //玉入れゾーン
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_R, LOW);
  //ゴールからコーンへ--------------
  Linetrace_D(4,280);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_R,HIGH);
  while(Line(0)==false){
    Linetrace(4);
  }
  Wheel(0,0);
  delay(500);
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_R, LOW);
  Turn(2);
  Linetrace_D(4,200);
  while(Line(8)==true){
    Linetrace(4);
  }
  Wheel(0,0);
  delay(500);

  //コーンへ玉入れ-------------------
  OUT(true);
  delay(1000);
  OUT(false);
  while(IN()==false){
    digitalWrite(LED_G,HIGH);
    digitalWrite(LED_R, LOW);
    delay(200);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_R,HIGH);
    delay(200);
  }
  //コーンから右タワーへ-------------
  Move(-130,-130,100);
  Move(180,-180,140);
  Move(300,300,100);
  Wheel(0,0);
  delay(500);

  //タワーへ玉入れ-------------------
  OUT(true);
  delay(1000);
  OUT(false);
  while(IN()==false){
    digitalWrite(LED_G,HIGH);
    digitalWrite(LED_R, LOW);
    delay(200);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_R,HIGH);
    delay(200);
  }
  //右タワーから玉へ
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_R,LOW);
  
  Move(-300,-300,100);
  while(Line(4)==false){
    Wheel(140,-140);
    delay(10);
  }
  Linetrace_D(4,200);
  digitalWrite(LED_G,LOW);
  digitalWrite(LED_R,HIGH);
  while(Line(0)==false && Line(7)==false){
    Linetrace(4);
  }
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_R,LOW);
  Turn(1);
  Linetrace_D(4,100);
  Wheel(0,0);
  delay(500);
  Move(-300,-300,100);
  //終了処理------------------------
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_R,HIGH);
  Wheel(0,0);
  while(1); 
}

/*
//メインプログラム

//ロボットアーム調整用4
while(1){
  if(IN()){
    digitalWrite(LED_R,HIGH);
  }
  else{
    digitalWrite(LED_R,LOW);
  }
}

------------------------------------
    Serial.print(R_S);
    Serial.print(" ");
    Serial.print(L_S);
    Serial.print(" ");
    Serial.print(D_count_R);
    Serial.print(" ");
    Serial.print(D_count_L);
    Serial.print(" ");
    Serial.print(count_R);
    Serial.print(" ");
    Serial.println(count_L);
*/
