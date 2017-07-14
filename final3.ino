///////////////////////////////////////
//                                   //
//        FFT#2 -                    //
//                                   //
///////////////////////////////////////


#include <SoftwareSerial.h>
#include "arduinoFFT.h"
arduinoFFT FFT = arduinoFFT();

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

#define Theta 6.2831 //2*Pi

#define LED 13
#define BAUDRATE 57600
#define DEBUGOUTPUT 0


#define powercontrol 10

SoftwareSerial BTSerial(10, 11); // RX | TX

const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
double signalFrequency = 10;
double samplingFrequency = 40;
uint8_t amplitude = 1;

double vReal[samples]; 
double vImag[samples];

// checksum variables
byte generatedChecksum = 0;
byte checksum = 0; 
int payloadLength = 0;
byte payloadData[64] = {0};
byte poorQuality = 0;
byte attention = 0;
byte meditation = 0;

long hob = 0;
long mob = 0;
long lob = 0;
long power[8] = {0};

int rawdata = 0;
int value0 = 0;
int value1 = 0;
int raw_cnt=0;
int temp_cnt=0;
int cnt=0;
//int raw_temp[4]={0};
int raw_arr[512]={0};
int start=0;
int z,tog;
int realByte[80];

//Method2 ARR
int raw_arr2[128];

//method3 additional ARR
int raw_arr3[4][128];

// system variables
long lastReceivedPacket = 0;
boolean bigPacket = false;


//double Blink
double Blink;
int blink_state;

//delay
unsigned int currentMillis,previousMillis;
int stop_state;

///norm////
int temp_Realfactor;
int Realfactor[4];
int avg_factor;
int fac_Value;
int Min_factor=20;
int Max_factor=140;
int norm_factor;


int Min_fac_arr[10];
int Max_fac_arr[10];


double A_vReal;
double B_vReal;
double Re_vReal;





double norm_arr[10];
double Max_vReal;
double Min_vReal;
double arr_sum;
double arr_avg;
int FFT_cnt;


///////////////////////////////////vehicle////////////////////////


const int AIA = 2;  // (pwm) pin 9 connected to pin A-IA 
const int AIB = 4;  // (pwm) pin 5 connected to pin A-IB 
const int BIA = 3; // (pwm) pin 10 connected to pin B-IA  
const int BIB = 5;  // (pwm) pin 6 connected to pin B-IB 
//int state;
byte speed = 0;

//////////////////////////////////////////////////////////////////


/////////////State LED ( Red = stop / Yellow = turn / Green = Move)//////////////////

const int Red=6;
const int Yellow=7;
const int Green=8;

const int s_Red1=13;
const int s_Red2=12;
const int s_Red3=9;
int state =0;  
int a;
//state =0 -->stop
//state =1 -->turn
//state =2 -->move




void setup() 

{
  pinMode(LED, OUTPUT);
  Serial.begin(BAUDRATE);           // USB
 // Serial1.begin(BAUDRATE); //TX1=18 , RX1= 19
  BTSerial.begin(BAUDRATE);
  pinMode(AIA, OUTPUT); // set pins to output
  pinMode(AIB, OUTPUT);
  pinMode(BIA, OUTPUT);
  pinMode(BIB, OUTPUT);

  pinMode(Red, OUTPUT);
  pinMode(Green, OUTPUT);
  pinMode(Yellow, OUTPUT);

  currentMillis=previousMillis=millis();
}
void backward()
{
  analogWrite(AIA, 0);
  analogWrite(AIB, speed);
  analogWrite(BIA, 0);
  analogWrite(BIB, speed);
}

void forward()
{
  analogWrite(AIA, speed);
  analogWrite(AIB, 0);
  analogWrite(BIA, speed);
  analogWrite(BIB, 0);
}

void left()
{
  analogWrite(AIA, speed);
  analogWrite(AIB, 0);
  analogWrite(BIA, 0);
  analogWrite(BIB, speed);
}

void right()
{
  analogWrite(AIA, 0);
  analogWrite(AIB, speed);
  analogWrite(BIA, speed);
  analogWrite(BIB, 0);
}
void AVG_Method2()
{
  for(int i=0;  i < samples; i++)
    raw_arr2[i] = (raw_arr[(i*4)]+raw_arr[(i*4)+1]+raw_arr[(i*4)+2]+raw_arr[(i*4)+3])/4;
}


void AVG_Method3()
{
 for(int j=0; j<4;j++){
   for(int i=0;  i < samples; i++)
      raw_arr2[i] = raw_arr[(i*4)+j];
  }
  Mindwave_FFT();

}


byte ReadOneByte() 
{
  int ByteRead;
  while(!BTSerial.available());
  ByteRead = BTSerial.read();

#if DEBUGOUTPUT  
  BTSerial.print((char)ByteRead);   // echo the same byte out the USB serial (for debug purposes)
#endif

  return ByteRead;
}

void Mindwave_FFT() 
{
  for (uint16_t i = 0; i < samples; i++)
  {
    vReal[i] =  raw_arr2[i];
    vImag[i] = 0;
  }
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */

  // for (int i = 0; i < 64; i++)
  // {
  //   Serial.print(vReal[i]);
  //   Serial.print(' ');
  // }

 
  Blink=vReal[0]+vReal[1]+vReal[2];   //Blink check
  A_vReal=vReal[8]+vReal[9]+vReal[10]; //Alpha core Fq
  B_vReal=vReal[18]+vReal[19]+vReal[20]; //Beta core Fq
  
  Re_vReal=B_vReal/A_vReal; //relative Alpha & Beta

  Realfactor[fac_Value] =(int)(Re_vReal*100); // double to int convert

  avg_factor=(Realfactor[0]+Realfactor[1]+Realfactor[2])/3; //factor average


  // Serial.print(Realfactor[fac_Value]);
  // Serial.print(" ");
  // Serial.print(avg_factor);
  // Serial.print(" ");
  // Serial.print(fac_Value);
  // Serial.print(" ");
  // Serial.print(Realfactor[0]);
  //   Serial.print(" ");
  // Serial.print(Realfactor[1]);
  //   Serial.print(" ");
  // Serial.print(Realfactor[2]);
  //   Serial.print(" ");
  // Serial.print(Realfactor[3]);

  if(Realfactor[fac_Value]>=300){   //noise soft filtering
    Realfactor[fac_Value]=temp_Realfactor;
  }

  temp_Realfactor=Realfactor[fac_Value];
  
  norm_factor=((double)(avg_factor-Min_factor)/(double)(Max_factor-Min_factor))*100;

  if(norm_factor<0)
    norm_factor=0;
  else if(norm_factor>100)
    norm_factor=100;

  fac_Value++;
  
  Serial.print(norm_factor);
  Serial.print(" ");

  if(fac_Value>=3){ 
    fac_Value=0;
    //norm_start=1;
  }




}

 

void loop() 

{
  currentMillis=millis();

  if(state==0){
    //speed=0;
    digitalWrite(Red,1);
    digitalWrite(Green,0);
    digitalWrite(Yellow,0);


      
    analogWrite(AIA, 0);
    analogWrite(AIB, 0);
    analogWrite(BIA, 0);
    analogWrite(BIB, 0);

  }
  else if(state==1){
    if(stop_state==1){
      analogWrite(AIA, 0);
      analogWrite(AIB, 0);
      analogWrite(BIA, 0);
      analogWrite(BIB, 0);
      //previousMillis=millis();
      if(currentMillis-previousMillis>=200){
        //previousMillis=currentMillis;

        stop_state=0;
      }

    }
    else{
       speed=190;
      digitalWrite(Red,0);
      digitalWrite(Green,0);
      digitalWrite(Yellow,1);
      forward();
      blink_state=!blink_state;
    }
    

  }
  else{
    if(stop_state==1){
      analogWrite(AIA, 0);
      analogWrite(AIB, 0);
      analogWrite(BIA, 0);
      analogWrite(BIB, 0);
      //previousMillis=millis();
        if(currentMillis-previousMillis>=200){
     // previousMillis=currentMillis;
     
      stop_state=0;
    }

    }
    else{
      if(blink_state==0){
        digitalWrite(Red,0);
        digitalWrite(Green,1);
        digitalWrite(Yellow,0);
        speed=2*a+10;
        right();
      }

    }

  }

  
  // Look for sync bytes
  if(ReadOneByte() == 0xAA) 
  {
    if(ReadOneByte() == 0xAA) 
    {
        payloadLength = ReadOneByte();
      
        if(payloadLength > 169)         //Payload length can not be greater than 169
        return;
        
        generatedChecksum = 0;        
        
        for(int i = 0; i < payloadLength; i++) 
        {  
          payloadData[i] = ReadOneByte();        //Read payload into memory
          generatedChecksum += payloadData[i];
        }   

        checksum = ReadOneByte();                      //Read checksum byte from stream      
        generatedChecksum = 255 - generatedChecksum;   //Take one's compliment of generated checksum

        if(checksum == generatedChecksum) 
        {    
          poorQuality = 200;
          attention = 0;
          meditation = 0;

          for(int i = 0; i < payloadLength; i++) 
          {                                          // Parse the payload
          switch (payloadData[i]) 
          {
          case 2:
            i++;            
            poorQuality = payloadData[i];
            bigPacket = true;            
            break;
          case 4:
            i++;
            attention = payloadData[i];                        
            break;
          case 5:
            i++;
            meditation = payloadData[i];
            break;
          case 0x80:
            value0 = payloadData[i + 2];
            value1 = payloadData[i + 3];
            rawdata = (value0 << 8) | value1;
          
            if(start==1){
              raw_arr[raw_cnt]=rawdata;
              cnt=raw_cnt;
              raw_cnt++;
              
            }

            i = i + 3;
            break;
          case 0x83:
          
          i++; 
          for (int j = 0; j < 8; j++) { 
            power[j] = ((long)payloadData[++i] << 16) | ((long)payloadData[++i] << 8) | (long)payloadData[++i]; 
          }      
          // i++;
          //   for (int j = 0; j < 24; j += 3) {
          //     hob = payloadData[i + j + 1];
          //     mob = payloadData[i + j + 2];
          //     lob = payloadData[i + j + 3];
          //     power[j / 3] = (hob <<16) | (mob << 8) | (lob );
          //   }
            
           //  i = i + 25;      
            break;
          
          default:
            break;
          } // switch
        } // for loop

#if !DEBUGOUTPUT
        // *** Add your code here ***
        if(bigPacket) 
        {
          if(start==1){
            
            AVG_Method2();
            Mindwave_FFT();
            //Serial.print("\n");
            //Serial.println(cnt);
            raw_cnt=0;
            
            /////Norm////




          }
          start=1;
          // if(poorQuality == 0)
          //    digitalWrite(LED, HIGH);
          //else
          //   digitalWrite(LED, LOW);
          // Serial.print("\n");
          // Serial.print("power: ");
          // for(int k=0; k<=7; k++)
          // {
          //   Serial.print(power[k], DEC);
          //   Serial.print(" ");
          // }
          // Serial.print("\n");
          // Serial.print("PoorQuality: ");
          // Serial.print(poorQuality, DEC);
          // Serial.print(" ");
           // Serial.print(" Attention: ");
          Serial.println(attention, DEC);
          //Serial.print(" ");

          a= attention;

          if(poorQuality==200){
            state=0;
          }
          else if(Blink>=25000){
            if(state==1)
              stop_state=0;
            else
              stop_state=1;

            state=1;
            //Serial.print("Blink");
            previousMillis=millis();
          }
          else{
            if(state==1)
              stop_state=0;
            else if(state==2)
              stop_state=0;
            else
              stop_state=1;

            state=2;
            previousMillis=millis();
          }
          // Serial.print(meditation, DEC);
          // Serial.print(" ");
          //  a=(attention*17)+700;
          //  Serial.print(a);
          // Serial.print(" Time since last packet: ");
          // Serial.print(millis() - lastReceivedPacket, DEC);
          // lastReceivedPacket = millis();
          //       Serial.print("\n");
          //       for(int n=0;n<z;n++){
          //         Serial.print(z);
          // Serial.print(1.2);
          //       }
          //      Serial.print(FUCK3);
          //       z=0;
          //       tog=0;
        }
        else {
          // Serial.println(rawdata, DEC);
          // Serial.print(" ");
          // Serial.print(raw_arr[raw_arr1][raw_arr2], DEC);
          // Serial.print(raw_arr2);
          // tog=1;
          //     Serial.print(1.3);
          //Serial.print(FUCK2);
        }
#endif        
        bigPacket = false;
        }
      else {
       //Serial.println("FUCK");  // Checksum Error
      }  // end if else for checksum
    } // end if read 0xAA byte
  } // end if read 0xAA byte  
}
