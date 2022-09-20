
// スラローム用のprg

// センサ設定
#define trigPin    11
#define echoPin    12
// 反応距離[cm]
#define thre       50

// モーターの各種パラメータ設定
#define motorSig1L  0
#define motorSig1R  1
#define motorSig1L  2
#define motorSig1R  3
// 障害物が無いときのスピード
#define baseSpeed   170
#define baseSpeed   10
// 右旋回量
// 左旋回量

void setup() {
  // put your setup code here, to run once:

}

void loop() {

  //ライントレースする
  LineTrace();

  int i = 0;
  if( dist <= thre ){   // 測定距離が設定値以下になると
    while( i < 4 ){
      j = (-1) ** i;
      if(j == 1){       // j == 1の時, 以下の処理を行なう
        while(1){
          //右モーターと左モーターに差を付け駆動
          if(2000 < position || position < 4000){
            moter_stop();
            break;
          }
        }
      }
        if(j == -1){       // j == -1の時, 以下の処理を行なう
        while(1){
          //右モーターと左モーターに差を付け駆動
          if(2000 < position || position < 4000){
            moter_stop();
            break;
          }
        }
      }
    }
  }

  

}
