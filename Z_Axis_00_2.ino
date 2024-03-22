#include <SPI.h>                                         
//#include "AVR_PWM.h"



#define SPEED_1      10
#define IN_1        4
#define IN_2        5

//#define IN_3        6
//#define IN_4        7
#define SPEED_2      9

#define ENC_1        2
#define ENC_12       3

#define STEPP       6
#define DIRR       8

#define STEP1       13
#define DIR1       12





volatile int lastEncoded = 0; // Here updated value of encoder store.

volatile long encoderValue = 0; // Raw encoder value


int VoltMAX = 140;  // Значение, при котором MotorShield выдаёт 12В
int VoltMIN = 40;   // Минимальное значение старта движения моторов

double InVolt = 14.5;
double OutVolt = 6;

int PWM = 100*OutVolt/InVolt;

int enc = 0;

//AVR_PWM* PWM_Instance;

int SP1 = 0;        // Переменная скорости мотора

void setup(){

  Serial.begin(9600);

  for (int i = 8; i < 10; i++) {     
    pinMode(i, OUTPUT);
  }
  pinMode(STEPP, OUTPUT);
  //pinMode(DIRR, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(DIR1, OUTPUT);

 // PWM_Instance = new AVR_PWM(SPEED_1, 500, 15);

  pinMode(ENC_1, INPUT_PULLUP); 
  pinMode(ENC_12, INPUT_PULLUP);

  digitalWrite(ENC_1, HIGH); //turn pullup resistor on
  digitalWrite(ENC_12, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

 // PWM_frequency(SPEED_1, 25000, CORRECT_PWM);

}

void loop(){

    // Если есть соединение с джойстиком, то ...

    for (int i = 9; i < PWM; i++) {

          //Serial.println("VVE PRAVO");  // Движение робота вправо-вверх

          //SP1 = mod*VoltMAX;            // Скорость Левого (1-го) мотора пропорциональна модулю радиус-вектора
          //SP2 = map(y, 0, 1000, VoltMIN, VoltMAX)/koef; // Скорость правого 

        digitalWrite(IN_1, HIGH); 
        digitalWrite(IN_2, LOW);
 //       digitalWrite(IN_3, LOW); 
       // digitalWrite(IN_4, HIGH);   // направление мотора 1 - вперед (Если увидите, что машинка едет не в ту сторону или происходит что-то не то - меняйте направления, пока она не начнёт ехать предсказуемо)
        analogWrite(SPEED_1, SP1); 
        analogWrite(SPEED_2, SP1); 
          //PWM_Instance->setPWM(SPEED_1, 62500, 0);
        enc = digitalRead(ENC_1);

        //PWM_set(SPEED_1, 70);
        //Serial.println(enc);
        Serial.println(encoderValue/8344);
        //Serial.println(lastEncoded);

         // delay(350);
    }

    digitalWrite(DIRR,1); 
    digitalWrite(DIR1,1); 

    for(int x = 0; x < 150; x++) {

        digitalWrite(STEPP,HIGH); 
        digitalWrite(STEP1,HIGH); 
        delay(1);    // by changing this time delay between the steps we can change the rotation speed
        digitalWrite(STEPP,LOW);  
        digitalWrite(STEP1,LOW);  
        //Serial.println("LOOP");
        
         // by changing this time delay between the steps we can change the rotation speed
        
        //Serial.println("LOOP");
    }

    digitalWrite(DIRR,0); 
    digitalWrite(DIR1,0); 

    delay(700);

    for(int x = 0; x < 150; x++) {

        digitalWrite(STEPP,HIGH); 
        digitalWrite(STEP1,HIGH); 
        delay(1);    // by changing this time delay between the steps we can change the rotation speed
        digitalWrite(STEPP,LOW);  
        digitalWrite(STEP1,LOW);  
        //Serial.println("LOOP");
    }
    

    delay(700);

    

        //weapon = data[3];

        // Если значение переменной орудия 1, то включаем реле и запускаем мотор орудия
       // if (weapon == 1) {
       //   digitalWrite(RELAY_PIN, HIGH);
       //   Serial.println("On");
       // }
        //else { // Если значение переменной орудия -1, то выключаем реле и мотор
         // digitalWrite(RELAY_PIN, LOW);
          //Serial.println("Off");
        //}

      delay(25);
        
       
}

void updateEncoder(){
  int MSB = digitalRead(ENC_1); //MSB = most significant bit
  int LSB = digitalRead(ENC_12); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    
    encoderValue --; 
    //Serial.println("Counter ClockWise");
  }
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) { 
    encoderValue ++;
    //Serial.println("ClockWise");
  }

  

  lastEncoded = encoded; //store this value for next time

}