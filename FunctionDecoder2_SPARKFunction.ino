// DCC Function Decoder Rev 2 for DS-DCC decode
// By yaasan
// Based on Nicolas's sketch http://blog.nicolas.cx
// Inspired by Geoff Bunza and his 17 Function DCC Decoder & updated library
//
// pwm light Effect function
// http://1st.geocities.jp/dcc_digital/
// O1:F1 でON/OFF
// O2:F2 でON/OFF
// O3:F3 でON/OFF
// O4:ヘッドライト F0でON/OFF F4でFX効果切り替え

//第１０章　続・ポインタ天国
//http://www7b.biglobe.ne.jp/~robe/cpphtml/html03/cpp03010.html

#include <NmraDcc.h>
#include <avr/eeprom.h>	 //required by notifyCVRead() function if enabled below

//FX効果
//panta spark
unsigned char ptn1[10][4]={{'I',0,5,1},{'O',1,5,1},  {'O',1,0,1},{'O',1,5,1},  {'O',1,0,1},{'O',1,5,1},  {'O',1,0,1},{'O',3,255,1},{'O',1,0,1},{'E',0,0,1}};
unsigned char ptn2[10][4]={{'I',0,5,1},{'O',1,255,1},{'O',1,0,1},{'O',1,255,1},{'O',1,0,1},{'O',1,255,1},{'O',1,0,1},{'O',3,255,1},{'O',1,0,1},{'E',0,0,1}};
unsigned char ptn3[8][4]= {{'I',0,5,1},{'O',1,5,1},  {'O',1,0,1},{'O',1,5,1},  {'O',1,0,1},{'O',1,5,1},  {'O',1,0,1},{'E',0,0,1}};
unsigned char ptn4[6][4]= {{'I',0,5,1},{'O',1,5,1},  {'O',1,0,1},{'O',1,5,1},  {'O',1,0,1},{'E',0,0,1}};
unsigned char ptn5[4][4]= {{'I',0,5,1},{'O',3,255,1},{'O',1,0,1},{'E',0,0,1}};

//Cmd,Time,Val,Frq
//I:初期状態,O:出力,S:スイープ,L:ループ,E:終了
//unsigned char ptn1[2][4]={{'I',0,255,1},{'E',0,255,1}}; //10ms すぐ点灯
//unsigned char ptn2[3][4]={{'I',0,5,1},{'S',100,255,1},{'E',0,255,1}}; //10ms もやっと点灯
//unsigned char ptn3[4][4]={{'I',0,5,1},{'S',50,255,1},{'S',50,5,1},{'L',0,5,1}}; //10ms 三角波
//unsigned char ptn4[6][4]={{'I',0,0,1},{'O',10,255,1},{'O',10,125,1},{'O',10,255,1},{'O',10,80,1},{'L',0,0,1}}; // ランダム
//unsigned char ptn5[6][4]={{'I',0,0,1},{'S',50,255,1},{'S',50,0,1},{'S',50,128,1},{'S',50,0,1},{'L',0,0,1}};//マーズライト
unsigned char ptn6[4][4]={{'I',0,0,1},{'O',30,255,1},{'S',20,128,1},{'L',0,0,1}};//フラッシュライト
unsigned char ptn7[4][4]={{'I',0,0,1},{'O',20,255,1},{'O',80,0,1},{'L',0,0,1}}; //シングルパルスストロボ
unsigned char ptn8[6][4]={{'I',0,0,1},{'O',10,255,1},{'O',20,0,1},{'O',10,255,1},{'O',60,0,1},{'L',0,0,1}}; //ダブルパルスストロボ
unsigned char ptn9[6][4]={{'I',0,0,1},{'O',25,128,1},{'O',40,255,1},{'O',25,128,1},{'O',40,0,1},{'L',0,0,1}}; //ミディアムパルスストロボ
unsigned char ptn10[8][4]={{'I',0,0,1},{'O',1,255,1},{'O',1,0,1},{'O',1,255,1},{'O',30,0,1},{'O',10,50,1},{'O',100,255,1},{'E',0,0,1}}; //グロー管蛍光灯
unsigned char ptn11[9][4]={{'I',0,0,1},{'O',1,255,1},{'O',1,0,1},{'O',1,255,1},{'O',30,0,1},{'O',30,40,1},{'O',50,255,1},{'O',70,0,1},{'L',0,0,1}}; //グロー管蛍光灯切れそう
unsigned char (*ptn)[4];

//モールス効果
//0:単音 1:長音
//[space]:文字区切り [_]:単語区切り [*]:終了
unsigned char mptn1[]="1010 1101_1010 1101_1010 1101_100 0_01 1011 01*";// CQ CQ CQ DE AYA
unsigned char mptn2[]="000 111 000_000 111 000_000 111 000*";// SOS SOS SOS
unsigned char mptn3[]="1000 00 0100_1000 00 0100_1000 00 0100_1000 00 0100*";// ばか ばか ばか ばか (ぽにょより)
unsigned char (*mptn);

//各種設定、宣言

#define DECODER_ADDRESS 3
#define DCC_ACK_PIN 0   // Atiny85 PB0(5pin) if defined enables the ACK pin functionality. Comment out to disable.
//                      // Atiny85 DCCin(7pin)
#define O1 0            // Atiny85 PB0(5pin)
#define O2 1            // Atiny85 PB1(6pin) analogwrite
#define O3 3            // Atint85 PB3(2pin)
#define O4 4            // Atiny85 PB4(3pin) analogwrite

#define MAX_PWMDUTY 255
#define MID_PWMDUTY 10

#define CV_VSTART		2
#define CV_ACCRATIO		3
#define CV_DECCRATIO	4
#define CV_F0_FORWARD 33
#define CV_F0_BACK 34
#define CV_F1 35
#define CV_F2 36
#define CV_F3 37
#define CV_F4 38
#define CV_F5 39
#define CV_F6 40
#define CV_F7 41
#define CV_F8 42
#define CV_F9 43
#define CV_F10 44
#define CV_F11 45
#define CV_F12 46
#define CV_49_F0_FORWARD_LIGHT 49

//ファンクションの変数
uint8_t fn_bit_f0 = 0;
uint8_t fn_bit_f1 = 0;
uint8_t fn_bit_f2 = 0;
uint8_t fn_bit_f3 = 0;
uint8_t fn_bit_f4 = 0;
uint8_t fn_bit_f5 = 0;
uint8_t fn_bit_f6 = 0;
uint8_t fn_bit_f7 = 0;
uint8_t fn_bit_f8 = 0;
uint8_t fn_bit_f9 = 0;
uint8_t fn_bit_f10 = 0;
uint8_t fn_bit_f11 = 0;
uint8_t fn_bit_f12 = 0;
uint8_t fn_bit_f13 = 0;
uint8_t fn_bit_f14 = 0;
uint8_t fn_bit_f15 = 0;
uint8_t fn_bit_f16 = 0;
uint8_t fn_bit_f17 = 0;
uint8_t fn_bit_f18 = 0;
uint8_t fn_bit_f19 = 0;
uint8_t fn_bit_f20 = 0;
uint8_t fn_bit_f21 = 0;
uint8_t fn_bit_f22 = 0;
uint8_t fn_bit_f23 = 0;
uint8_t fn_bit_f24 = 0;
uint8_t fn_bit_f25 = 0;
uint8_t fn_bit_f26 = 0;
uint8_t fn_bit_f27 = 0;
uint8_t fn_bit_f28 = 0;

//使用クラスの宣言
NmraDcc	 Dcc;
DCC_MSG	 Packet;

//Task Schedule
unsigned long gPreviousL5 = 0;

//進行方向
uint8_t gDirection = 128;

//Function State
uint8_t gState_F0 = 0;
uint8_t gState_F1 = 0;
uint8_t gState_F2 = 0;
uint8_t gState_F3 = 0;
uint8_t gState_F4 = 0;

//モータ制御関連の変数
uint32_t gSpeedRef = 1;

//CV related
uint8_t gCV1_SAddr = 3;
uint8_t gCVx_LAddr = 3;
uint8_t gCV49_fx = 1;

//Internal variables and other.
#if defined(DCC_ACK_PIN)
const int DccAckPin = DCC_ACK_PIN ;
#endif

struct CVPair {
  uint16_t	CV;
  uint8_t	Value;
};
CVPair FactoryDefaultCVs [] = {
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, DECODER_ADDRESS}, // CV01
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},               // CV09 The LSB is set CV 1 in the libraries .h file, which is the regular address location, so by setting the MSB to 0 we tell the library to use the same address as the primary address. 0 DECODER_ADDRESS
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},          // CV17 XX in the XXYY address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},          // CV18 YY in the XXYY address
  {CV_29_CONFIG, 128 },                                // CV29 Make sure this is 0 or else it will be random based on what is in the eeprom which could caue headaches
  {CV_49_F0_FORWARD_LIGHT, 1},                         // CV49 F0 Forward Light
  //  {CV_VSTART, 16},
  //  {CV_ACCRATIO, 64},
  //  {CV_DECCRATIO, 64},
};

void(* resetFunc) (void) = 0;  //declare reset function at address 0
void LightMes( char,char );
void pulse(void);
uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);

//uint16_t limitSpeed(uint16_t inSpeed);

void notifyCVResetFactoryDefault()
{
  //When anything is writen to CV8 reset to defaults.

  resetCVToDefault();
  //Serial.println("Resetting...");
  delay(1000);  //typical CV programming sends the same command multiple times - specially since we dont ACK. so ignore them by delaying

  resetFunc();
};

//------------------------------------------------------------------
// CVをデフォルトにリセット
// Serial.println("CVs being reset to factory defaults");
//------------------------------------------------------------------
void resetCVToDefault()
{
  for (int j = 0; j < FactoryDefaultCVIndex; j++ ) {
    Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
  }
};



extern void	   notifyCVChange( uint16_t CV, uint8_t Value) {
  //CVが変更されたときのメッセージ
  //Serial.print("CV ");
  //Serial.print(CV);
  //Serial.print(" Changed to ");
  //Serial.println(Value, DEC);
};

//------------------------------------------------------------------
// CV Ack
// Smile Function Decoder は未対応
//------------------------------------------------------------------
void notifyCVAck(void)
{
  //Serial.println("notifyCVAck");
  digitalWrite(O4,HIGH);
 // analogWrite(O1, 0);
//  analogWrite(O4, 64);

  delay( 6 );
  digitalWrite(O4,LOW);
//  analogWrite(O4, 0);
}

//------------------------------------------------------------------
// Arduino固有の関数 setup() :初期設定
//------------------------------------------------------------------
void setup()
{
  uint8_t cv_value;

//ptn = ptn1;
//mptn = mptn1;

  //シリアル通信開始
  //Serial.begin(115200);

  //D9,D10 PWM キャリア周期:31kHz
//  TCCR1 &= B11110000;
//  TCCR1 |= B00000001;

  //PWMを10bit化
  //TCCR1A |= B00000011;

//    TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00;
 //   TCCR0B = 0<<WGM02 | 1<<CS00;    
 //   TCCR1 = 0<<PWM1A | 0<<COM1A0 | 1<<CS10;
 //   GTCCR = 1<<PWM1B | 2<<COM1B0;
#if 0
  TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00; // B1111 0011
  TCCR0B = 0<<WGM02 | 1<<CS00;               // B0000 0001 分周無し
  TCCR1 = 0<<PWM1A | 0<<COM1A0 | 1<<CS10;
  GTCCR = 1<<PWM1B | 2<<COM1B0;  
#endif

// TCCR1 =0x03;最初に0x03が設定されている。
// D9,D10 PWM キャリア周期:31kHz
// TCCR1 &= B11110000;
// TCCR1 |= B00000001;

//TCCR1 = 1<<CTC1 | 1<<PWM1A | 3<<COM1A0 | 1<<CS10;
  TCCR1 = 0<<CTC1 | 0<<PWM1A | 0<<COM1A0 | 1<<CS10;
  
  pinMode(O1, OUTPUT);
  pinMode(O2, OUTPUT);
  pinMode(O3, OUTPUT);
  pinMode(O4, OUTPUT);

  //DCCの応答用負荷ピン
#if defined(DCCACKPIN)
  //Setup ACK Pin
  pinMode(DccAckPin, OUTPUT);
  digitalWrite(DccAckPin, 0);
#endif

#if !defined(DECODER_DONT_DEFAULT_CV_ON_POWERUP)
  if ( Dcc.getCV(CV_MULTIFUNCTION_PRIMARY_ADDRESS) == 0xFF ) {	 //if eeprom has 0xFF then assume it needs to be programmed
    //Serial.println("CV Defaulting due to blank eeprom");
    notifyCVResetFactoryDefault();

  } else {
    //Serial.println("CV Not Defaulting");
  }
#else
  //Serial.println("CV Defaulting Always On Powerup");
  notifyCVResetFactoryDefault();
#endif

  // Setup which External Interrupt, the Pin it's associated with that we're using, disable pullup.
  Dcc.pin(0, 2, 0); // Atiny85 7pin(PB2)をDCC_PULSE端子に設定

  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 100,   FLAGS_MY_ADDRESS_ONLY , 0 );

  //Reset task
  gPreviousL5 = millis();

  //Init CVs
  gCV1_SAddr = Dcc.getCV( CV_MULTIFUNCTION_PRIMARY_ADDRESS ) ;

//#if 0
  gCV49_fx = Dcc.getCV( CV_49_F0_FORWARD_LIGHT );
  switch(gCV49_fx){
    case 0:
    break;
    case 1: ptn = ptn1;
    break;
    case 2: ptn = ptn2;
    break;
    case 3: ptn = ptn3;    
    break;
    case 4: ptn = ptn4;
    break;
    case 5: ptn = ptn5;
    break;
    case 6: ptn = ptn6;
    break;
    case 7: ptn = ptn7;
    break;
    case 8: ptn = ptn8;
    break;
    case 9: ptn = ptn9;
    break;
    case 10: ptn = ptn10;
    break;
    case 11: ptn = ptn11;
    break;
  }
//#endif

//デバック信号：起動 　
//注意：2016/4/9 このコメントを外し起動時にアドレスを点灯させるようにすると、
//      loopにたどり着く時間が長くなり、CV書き込みが失敗します。
//LightMes(0);
//LightMes(Dcc.getCV(CV_MULTIFUNCTION_PRIMARY_ADDRESS));
}

void loop() {
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  if ( (millis() - gPreviousL5) >= 10) // 100:100msec  10:10msec  Function decoder は 10msecにしてみる。
  {
    //Headlight control
    HeadLight_Control();

    //Motor drive control
    //Motor_Control();

    //Reset task
    gPreviousL5 = millis();
  }

}


//---------------------------------------------------------------------
// PantographSpark効果ステートマシン
// 10ms周期で起動
// unsigned chart ptn[4][5]{{'I',0,0,1},{'S',20,255,1},{'S',40,0,1},{'E',0,0,1}};
//---------------------------------------------------------------------
void PantaSparkEffect_Control(){
  static char state = 0;    // ステート
  static char adr = 0;      // アドレス
  static int timeUp = 0;    // 時間
  static float delt_v = 0;  // 100msあたりの増加量 
  static float pwmRef =0;
  static int nextSparkWait =0;  // 点滅間隔 10ms
  long randNumber = 0;
  long sparkSel = 0;
    
  if(gState_F0 == 0){ // F0 OFF
    state = 0; 
    adr = 0;
    timeUp = 0;
    pwmRef = 0;
    TCCR1 = 0<<CS10;  //分周比をセット
    //       OCR1B有効   high出力　
    GTCCR = 0 << PWM1B | 0 << COM1B0;
    analogWrite(O4, 0);
    digitalWrite(O4, LOW);               // 消灯
  }

  S00:  
  switch(state){
    case 0: // S00:idel
      if(gState_F0 >0){ // F1 ON
        adr = 0;
        timeUp = 0;
        pwmRef = 0;
        TCCR1 = 1<<CS10;  //分周比をセット
        //       OCR1B有効   high出力　
        GTCCR = 1 << PWM1B | 2 << COM1B0;
        analogWrite(O4, 0);
        state = 1;
        goto S00;     // 100ms待たずに再度ステートマシーンに掛ける
      }
      break;

    case 1: // スピードによる点灯間隔算出 max 128 step?
        if(gSpeedRef <= 10){
            state = 0;
        } else if(gSpeedRef >= 11 && gSpeedRef <=30 ){
            nextSparkWait = random(300, 500);   // 3000ms - 5000ms
            state = 2;
        } else if(gSpeedRef >= 31 && gSpeedRef <=50 ){
            nextSparkWait = random(100, 300);   // 1000ms - 3000ms          
            state = 2;
        } else if(gSpeedRef >= 51 && gSpeedRef <=70 ){
            nextSparkWait = random(50, 100);   // 500ms - 1000ms          
            state = 2;
        } else if(gSpeedRef >= 71 && gSpeedRef <=100 ){
            nextSparkWait = random(1, 50);   // 10ms - 500ms            
            state = 2;
        }else if(gSpeedRef >= 101 && gSpeedRef <=128 ){
            nextSparkWait = random(1, 5);   // 10ms - 50ms            
            state = 2;
        }

        break;
        
    case 2: // 点灯トリガ

      sparkSel = random(1,6);
      switch(sparkSel){
        case 1:
          ptn = ptn1;
          break;
        case 2:
          ptn = ptn2;
          break;
        case 3:
          ptn = ptn3;
          break;        
        case 4:
          ptn = ptn4;
          break;
        case 5:
          ptn = ptn5;
          break;
        case 6:
          ptn = ptn6;
          break;
        default:
          ptn = ptn1;
          break;
      }
      adr = 0; 
      state = 4;
      goto S00;   // 10ms待たずに再度ステートマシーンに掛ける
      break;

    case 3: // 次のスパークまでのウエイト処理

      nextSparkWait--;
      if(nextSparkWait <= 0){
        state = 1;
      }
      break;
      
    case 4: // S01:コマンド処理
        if( ptn[adr][0]=='I'){ // I:初期化
          timeUp = ptn[adr][1];
          pwmRef = ptn[adr][2];
          delt_v = 0; // 変化量0
          TCCR1 = ptn[adr][3]<<CS10;  //分周比をセット
          analogWrite(O4, (unsigned char)pwmRef); // 0〜255            
          adr++;
          state = 4;
          goto S00;   // 10ms待たずに再度ステートマシーンに掛ける
        } else if( ptn[adr][0]=='E'){ // E:end
          state = 3;
        } else if( ptn[adr][0]=='O' ){ // O:出力
          timeUp = ptn[adr][1];
          pwmRef = ptn[adr][2];
          TCCR1 = ptn[adr][3]<<CS10;  //分周比をセット
          delt_v = 0;
          state = 5;          
        } else if( ptn[adr][0]=='S' ){ // S:sweep
          timeUp = ptn[adr][1];
          TCCR1 = ptn[adr][3]<<CS10;  //分周比をセット      
          delt_v = (ptn[adr][2]-pwmRef)/timeUp;  // 変化量を算出
          state = 5;
        }
      break;
      
    case 5: // S02:時間カウント
      timeUp--;
      pwmRef = pwmRef + delt_v;
      if(pwmRef<=0){            // 下限、上限リミッタ
          pwmRef = 0;
      } else if(pwmRef>=255){
          pwmRef = 255;
      }
      analogWrite(O4, (unsigned char)pwmRef); // 0〜255         
 
      if( timeUp <= 0 ){
        adr ++;
        state = 4;  //次のコマンドへ
      }
      break;
      
      default:
      break;
  }
}




//---------------------------------------------------------------------
// FX効果ステートマシン
// 10ms周期で起動
// unsigned chart ptn[4][5]{{'I',0,0,1},{'S',20,255,1},{'S',40,0,1},{'E',0,0,1}};
//---------------------------------------------------------------------
void FXeffect_Control(){
  static char state = 0;    // ステート
  static char adr = 0;      // アドレス
  static int timeUp = 0;    // 時間
  static float delt_v = 0;  // 100msあたりの増加量 
  static float pwmRef =0;

  if(gState_F0 == 0){ // F0 OFF
    state = 0; 
    adr = 0;
    timeUp = 0;
    pwmRef = 0;
    TCCR1 = 0<<CS10;  //分周比をセット
    //       OCR1B有効   high出力　
    GTCCR = 0 << PWM1B | 0 << COM1B0;
    analogWrite(O4, 0);
    digitalWrite(O4, LOW);               // 消灯
  }

  S00:  
  switch(state){
    case 0: // S00:idel
      if(gState_F0 >0){ // F0 ON
        adr = 0;
        timeUp = 0;
        pwmRef = 0;
        TCCR1 = 1<<CS10;  //分周比をセット
        //       OCR1B有効   high出力　
        GTCCR = 1 << PWM1B | 2 << COM1B0;
        analogWrite(O4, 0);
        state = 1;
        goto S00;     // 100ms待たずに再度ステートマシーンに掛ける
      }
      break;
      
    case 1: // S01:コマンド処理
        if( ptn[adr][0]=='I'){ // I:初期化
          timeUp = ptn[adr][1];
          pwmRef = ptn[adr][2];
          delt_v = 0; // 変化量0
          TCCR1 = ptn[adr][3]<<CS10;  //分周比をセット
          analogWrite(O4, (unsigned char)pwmRef); // 0〜255            
          adr++;
          state = 1;
          goto S00;   // 100ms待たずに再度ステートマシーンに掛ける
        } else if( ptn[adr][0]=='E'){ // E:end
          state = 3;
        } else if( ptn[adr][0]=='L' ){  // L:Loop
          adr = 0;
          state =1;
          goto S00;   // 100ms待たずに再度ステートマシーンに掛ける
        } else if( ptn[adr][0]=='O' ){ // O:出力
          timeUp = ptn[adr][1];
          pwmRef = ptn[adr][2];
          TCCR1 = ptn[adr][3]<<CS10;  //分周比をセット
          delt_v = 0;
          state = 2;          
        } else if( ptn[adr][0]=='S' ){ // S:sweep
          timeUp = ptn[adr][1];
          TCCR1 = ptn[adr][3]<<CS10;  //分周比をセット      
          delt_v = (ptn[adr][2]-pwmRef)/timeUp;  // 変化量を算出
          state = 2;
        }
      break;
      
    case 2: // S02:時間カウント
      timeUp--;
      pwmRef = pwmRef + delt_v;
      if(pwmRef<=0){            // 下限、上限リミッタ
          pwmRef = 0;
      } else if(pwmRef>=255){
          pwmRef = 255;
      }
      analogWrite(O4, (unsigned char)pwmRef); // 0〜255         
 
      if( timeUp <= 0 ){
        adr ++;
        state = 1;  //次のコマンドへ
      }
      break;

      case 3: // stay
      break;

      default:
      break;
  }
}


//---------------------------------------------------------------------
// モールス効果ステートマシン
// 100ms周期で起動
// 単音1,長音3,文字間3,単語間7
//---------------------------------------------------------------------
void MorseCode_Control(void)
{
  static char state = 0;    // ステート
  static char adr = 0;      // アドレス
  static char cnt = 0;      // 単音、長音のカウント

  if(gState_F3 == 0){ // F3 OFF
    state = 0; 
  }

  S10:  
  switch(state){
    case 0:
      adr = 0;
      if(gState_F3 == 1){ // F3 ON
        state = 1;
        digitalWrite(O2, LOW);               // 消灯 
        goto S10;            
        break;
      }
    case 1:
      switch(mptn[adr]){
        case '0':  // 単音?
          cnt = 1;
          digitalWrite(O2, HIGH);           // 点灯 
          state = 2;
        break;
        
        case '1':  // 長音？
          cnt = 3;
          digitalWrite(O2, HIGH);           // 点灯 
          state = 2;
        break;
        
        case ' ':  // 文字区切り
          cnt = 3;
          state = 2;
        break;

        case '_':  // 単語区切り
          cnt = 7;
          state = 2;
        break;

        case '*':  // 終わり
          state = 0;
        break;

        default:
         break;
      }
    break;
    
    case 2:   // 時間が来たら消灯
      cnt --;
      if(cnt == 0){
        digitalWrite(O2, LOW);               // 消灯 
        state = 3;
      }
    break;
    
    case 3:   // 短点と長点の間隔処理とアドレスを進める
        adr++;
        state = 1;
    break;
    
    default:
    break;
  }
}


//---------------------------------------------------------------------
// HeadLight control Task (10Hz:100ms)
//---------------------------------------------------------------------
void HeadLight_Control()
{
  static char sw = 1;
  static char prev = 0;
//  uint16_t aPwmRef = 0;

#if 0
  if ( gState_F0 > 0)
  {

    if ( gState_F2 > 0)
    {
      if ( gSpeedRef > 1)
      {
        aPwmRef = MAX_PWMDUTY;
      }
      else
      {
        aPwmRef = MID_PWMDUTY;
      }
    }
    else if ( gState_F1 > 0)
    {
      aPwmRef = MID_PWMDUTY;
    }
    else
    {
      aPwmRef = MAX_PWMDUTY;
    }
  }
  else
  {
    aPwmRef = 0;
  }
#endif


//MorseCode_Control();
//FXeffect_Control(); 
PantaSparkEffect_Control();

//---------------------------------------------------------------------
// PWMによるLED調光
//--------------------------------------------------------------------- 
#if 0
  if(gState_F0 > 0)
  {
    if( gDirection == 0){                // Reverse 後進(DCS50Kで確認)
      // PB1:O2:6pin PWM 出力 , PB4:O4:3pin LOW出力

      //      OCR1B無効   OCR1B未使用　
      GTCCR = 0; 
      digitalWrite(O4, LOW); // PB4:O4:3pin LOW出力

      //               OCR1A有効   high出力          分周無し　
      TCCR1 = 1<<PWM1A | 2<<COM1A0 | 1<<CS10; 

      // analogWrite(F3, gSpeedRef); // analogWriteは使えない 
      OCR1A = gSpeedRef;               // 0〜255 
      
    } else {                             // Forward 前進(DCS50Kで確認)
      // PB1:O2:6pin LOW出力  , PB4:O4:3pin PWM 出力

      //               OCR1A無効   OCR1A未使用　 分周無し　
      TCCR1 = 1<<CS10; 
      digitalWrite(O2, LOW); // PB1:O2:6pin LOW出力
      //                OCR1B有効     high出力　
      GTCCR = 1 << PWM1B | 2 << COM1B0; 
      analogWrite(O4, gSpeedRef); // 0〜255 
      // OCR1B = gSpeedRef;         // この設定もできる 
    }
  }
  
  if(gState_F0 == 0){                    // DCC F0 コマンドの処理
    TCCR1 = 1<<CS10; 
    digitalWrite(O2, LOW);               // 消灯
    digitalWrite(O4, LOW);               // 消灯
  }
#endif

//---------------------------------------------------------------------
// ヘッド・テールライトコントロール  
//--------------------------------------------------------------------- 
#if 0
  if(gState_F0 == 0){                    // DCC F0 コマンドの処理
    digitalWrite(O2, LOW);               // アナログ出力:ヘッドライト
    digitalWrite(O4, LOW);   
  } else {
    if( gDirection == 0){                // Reverse 後進(DCS50Kで確認)
      digitalWrite(O2, HIGH); 
      digitalWrite(O4, LOW);
    } else {                             // Forward 前進(DCS50Kで確認)
      digitalWrite(O2, LOW);

      if ( gSpeedRef > 1){               // 調光機能
        aPwmRef = MAX_PWMDUTY;           // 速度が1以上だったらMAX
      } else {
        aPwmRef = MID_PWMDUTY;           // 速度が0だったらMID
      }
      analogWrite(O4, aPwmRef);
    }
  }
#endif

// F1 受信時の処理
#if 0
  if(gState_F1 == 0){
    digitalWrite(O1, LOW);
  } else {
    digitalWrite(O1, HIGH);
    ptn=ptn1;
  }
#endif

// F2 受信時の処理
#if 0
  if(gState_F2 == 0){                   // DCS50KのF2は1shotしか光らないので、コメントアウト
    digitalWrite(O2, LOW);  
  } else {
    digitalWrite(O2, HIGH);
    ptn=ptn7;
  }
#endif

// F3 受信時の処理
#if 0
  if(gState_F3 == 0){
    digitalWrite(O3, LOW);
  } else {
    digitalWrite(O3, HIGH);
    ptn=ptn7;
  }
#endif

//F3 受信時の処理　変数を覗く
#if 0
  if(gState_F3 != 0){
    LightMes(gCV49_fx,4);for(;;);
  }
#endif

// F4 受信時の処理
#if 0
  if(gState_F4 == 0){
    digitalWrite(O4, LOW);
  } else {
    digitalWrite(O4, HIGH);
  }
#endif

// F4 を押下でFX効果を次々に変える（デモ用）
#if 0
  if(gState_F4 == 0){
    if(prev==1){
        prev = 0;
        sw ++;
    }
  } else {
    if(prev==0){
        prev = 1;
        sw ++;
    }
  }

  if(sw >= 12)
    sw = 1;
  
  switch(sw){
    case 1: ptn=ptn1;
    break;
    case 2: ptn=ptn2;
    break;
    case 3: ptn=ptn3;
    break;
    case 4: ptn=ptn4;
    break;
    case 5: ptn=ptn5;
    break;
    case 6: ptn=ptn6;
    break;
    case 7: ptn=ptn7;
    break;
    case 8: ptn=ptn8;
    break;
    case 9: ptn=ptn9;
    break;
    case 10: ptn=ptn10;
    break;  
    case 11: ptn=ptn11;
    break;  
    default:
    break;
  }
#endif

}

//DCC速度信号の受信によるイベント
extern void notifyDccSpeed( uint16_t Addr, uint8_t Speed, uint8_t ForwardDir, uint8_t MaxSpeed )
{
  if ( gDirection != ForwardDir)
  {
    gDirection = ForwardDir;
  }
  gSpeedRef = Speed;
}

//---------------------------------------------------------------------------
//ファンクション信号受信のイベント
//FN_0_4とFN_5_8は常時イベント発生（DCS50KはF8まで）
//FN_9_12以降はFUNCTIONボタンが押されたときにイベント発生
//前値と比較して変化あったら処理するような作り。
//---------------------------------------------------------------------------
extern void notifyDccFunc( uint16_t Addr, FN_GROUP FuncGrp, uint8_t FuncState)
{

  
  if( FuncGrp == FN_0_4)
  {
    if( gState_F0 != (FuncState & FN_BIT_00))
    {
      //Get Function 0 (FL) state
      gState_F0 = (FuncState & FN_BIT_00);
    }
    if( gState_F1 != (FuncState & FN_BIT_01))
    {
      //Get Function 1 state
      gState_F1 = (FuncState & FN_BIT_01);
    }
    if( gState_F2 != (FuncState & FN_BIT_02))
    {
      gState_F2 = (FuncState & FN_BIT_02);
    }
    if( gState_F3 != (FuncState & FN_BIT_03))
    {
      gState_F3 = (FuncState & FN_BIT_03);
    }
    if( gState_F4 != (FuncState & FN_BIT_04))
    {
      gState_F4 = (FuncState & FN_BIT_04);
    }
  }



#if 0  
  static char state = 0;    // ステート
  static char fn04PrevState = 0; // FN_0_4 の1個前のデータ
  uint8_t mask;

  S20:
  switch(state){
    case 0:   //S00 初回線路電源ON時にビットがONになってしまうので、受け流し処理。DC50Kだけ？
      if( FuncGrp == FN_0_4 ){
        fn04PrevState = FuncState;
        state = 1;
      }
    break;

    case 1:   //S01
      if( FuncGrp == FN_0_4 ){
        fn04PrevState = FuncState;
        state = 2;
        goto S20;   // 強制的にS20へ（時間短縮の為）
      }
    break;

    case 2:   //S02 FN_0_4 処理ステート
      mask = fn04PrevState xor FuncState;   // 前値との変化ビットを算出
      fn04PrevState = FuncState;                  // 現在値を保存

    //Get Function 0 (FL) state
    if( gState_F0 != (mask & FN_BIT_00)) {        // F0ビットが変化した？
      gState_F0 = (mask & FN_BIT_00);             // この処理でON/OFFがトグルする
    }

    //Get Function 1 state
    if( gState_F1 != (mask & FN_BIT_01)){
      gState_F1 = (mask & FN_BIT_01);
    }

    //Get Function 2 state DCS50KではF2ボタンが押されている間、ONになる仕様。
    if( gState_F2 != (mask & FN_BIT_02)){
      gState_F2 = (mask & FN_BIT_02);
    }

    //Get Function 3 state
    if( gState_F3 != (mask & FN_BIT_03)){
      gState_F3 = (mask & FN_BIT_03);
    }

    //Get Function 4 state
    if( gState_F4 != (mask & FN_BIT_04)) {
      gState_F4 = (mask & FN_BIT_04);
    }
    state = 1;
    break;
  }

#endif
   
}

//-----------------------------------------------------
// F4 を使って、メッセージを表示させる。
//上位ビットから吐き出す
// ex 5 -> 0101 -> ー・ー・
//-----------------------------------------------------
void LightMes( char sig ,char set)
{
  char cnt;
  for( cnt = 0 ; cnt<set ; cnt++ ){
    if( sig & 0x80){
      digitalWrite(O1, HIGH); // 短光
      delay(200);
      digitalWrite(O1, LOW);
      delay(200);
    } else {
      digitalWrite(O1, HIGH); // 長光
      delay(1000);
      digitalWrite(O1, LOW);            
      delay(200);
    }
    sig = sig << 1;
  }
      delay(400);
}

//-----------------------------------------------------
// Debug用Lﾁｶ
//-----------------------------------------------------
void pulse()
{
  digitalWrite(O1, HIGH); // 短光
  delay(100);
  digitalWrite(O1, LOW);
}


