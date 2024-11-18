//for encoder
#include <ESP32Encoder.h>
ESP32Encoder BaseEnc;
ESP32Encoder ShoulderEnc;
ESP32Encoder ElbowEnc;

//for motors 
const int BaseMotor = 25; // top pin right pin
const int ShoulderMotor = 26; // second top right pin
const int ElbowMotor = 27; //5th top right pin
const int ElvowOpAmpInput = 14; //6th top right pin for elbow
const int BaseOpAmpInput = 13; //6th top right pin for shoulder
const int ShoulderOpAmpInput = 12; //6th top right pin for base
const int PwmFrequency = 50000;

void setup() {
  Serial.begin(921600);
  
  // encoder setup
  // Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors = puType::up;
  
	// use pin 19 and 18 for the first encoder
	BaseEnc.attachFullQuad(33, 32);
  ShoulderEnc.attachFullQuad(35, 34);
  ElbowEnc.attachFullQuad(39, 36);
  //sets values for zeroing 
  BaseEnc.setCount(100000);
  ShoulderEnc.setCount(-60000);
  ElbowEnc.setCount(44500); 
  
  // motor output setup
  ledcAttach(BaseMotor, PwmFrequency, 8);
  //pinMode(BaseMotor, OUTPUT);
  
  delay(100);
  ledcAttach(ShoulderMotor, PwmFrequency, 8);
  //pinMode(ShoulderMotor, OUTPUT);
  delay(100);
  ledcAttach(ElbowMotor, PwmFrequency, 8);
  //pinMode(ElbowMotor, OUTPUT);
  delay(100);
  // opamp output setup
  ledcAttach(ElvowOpAmpInput, PwmFrequency, 8);
  //pinMode(ElvowOpAmpInput, OUTPUT);
  delay(100);
  ledcAttach(BaseOpAmpInput, PwmFrequency, 8);
  //pinMode(BaseOpAmpInput, OUTPUT);
  delay(100);
  ledcAttach(ShoulderOpAmpInput, PwmFrequency, 8);
  //pinMode(ShoulderOpAmpInput, OUTPUT);
  delay(100);
  
  //sets all the outputs so robot isnt moving
  analogWrite(ElvowOpAmpInput, 120);//elbow. lower values start the elbow going up
  analogWrite(BaseOpAmpInput, 116);//shoulder. lower values start shoulder going forward
  analogWrite(ShoulderOpAmpInput, 120);//base. lower values start the base going clock wise
  analogWrite(BaseMotor, 120);
  analogWrite(ShoulderMotor, 120);
  analogWrite(ElbowMotor, 120);
  
  delay(2000);
}

long int time_before_write;
long int time_after_write;

String EncoderValProcess = " ";
char EncoderValOutput[25] = {};
int *EncoderVals[2]={};
int MaxBytesForWrite = 25;
//max is 16, lowest is 12
int MinBytesForRead = 14;
char TerminatingChar = ';';
int PWMOutput[3] = {};
int PointInArray = 0;
char testarray[12] ={'1','1','1',' ','2','2','2',' ','3','3','3'};
const int PWMReadValuesLength = 12;
char PWMReadValues[PWMReadValuesLength] = {' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};

// Input array struct: {'#', '#', '#', ' ', '#', '#', '#', ' ', '#', '#', '#', ';'}
// Output array struct: {'±', '#', '#', '#', '#', ' ', '±', '#', '#', '#', '#', ' ', '±', '#', '#', '#', ';'}

String num;
char c;
int vals[3] = {120, 120, 120};
String *Test = &num;
size_t bytes_written=0;
void loop() {
  
  while(1){
 
  //records values for writing to computer for encoder values
  
  EncoderValProcess = String(BaseEnc.getCount()) + ":" + String(ShoulderEnc.getCount()) + ":" + String(ElbowEnc.getCount()) + ";"; 
  //EncoderValProcess = String(vals[0]) + ":" + String(vals[1]) + ":" + String(vals[2]) + ";"; 
  //EncoderValProcess = String(Test) + ":" + String(1) + ":" + String(1) + ";" + '\n'; 
  time_before_write = micros();
  
  for(int i = EncoderValProcess.length(); i < MaxBytesForWrite-1; i++){
    EncoderValProcess += ' ';
  }
  EncoderValProcess += '\n';

  EncoderValProcess.toCharArray(EncoderValOutput, MaxBytesForWrite+1);
  
 
  
  //if reading slows down function enough can remove
  while (Serial.availableForWrite() < MaxBytesForWrite+1){}
  bytes_written = Serial.write(EncoderValOutput, MaxBytesForWrite);
 
  //Serial.write("\n",1);
  //Serial.print(EncoderValProcess);
  


  //for reading 
  /*
  if(Serial.available()){
    c = char(Serial.read());
      if (c == '[') {//checks for end of values
        PointInArray = int(Serial.read())-'0';
        vals[PointInArray] = num.toInt(); // sets values to the assinged joint in vals
        num = "";// reset 
        analogWrite(BaseMotor,vals[0]);
        analogWrite(ShoulderMotor,vals[1]);
        analogWrite(ElbowMotor,vals[2]);
        Serial.read();
        
      } 
      /* 
      else if (num[num.length()-1] == '\n' || num[num.length()-1] == ' ') {
        num ="";
      }*//*
      else{
        num += c;
      }
  }*/
  
  while(Serial.available()){// checks that there is a char to read
    
    //Serial.println("test");
    c = Serial.read();//stores in value from buffer
    if (c == ':') {//checks for end of values
      vals[PointInArray] = num.toInt(); // sets values to the assinged joint in vals
      num = "";// reset 
      PointInArray++;
    } else if (c == ';') {
      vals[PointInArray] = num.toInt();
      num = "";
      PointInArray = 0;
      //Serial.write('1');
      analogWrite(BaseMotor,vals[0]);
      analogWrite(ShoulderMotor,vals[1]);
      analogWrite(ElbowMotor,vals[2]);
      //clear buffer
      while(Serial.read()!='\n'){}
      break;
      
    } else if (isDigit(c)) {
      num += c;
    }

  }


  
  //match with ros2 code
  while(time_after_write - time_before_write < 500){
    time_after_write =micros();
  }

  //Serial.print("time: " + String(time_after_write - time_before_write) + ". ");
 
  
  //Serial.println(time_before_write - time_after_write);
  }

}