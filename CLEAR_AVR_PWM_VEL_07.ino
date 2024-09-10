//#include "AVR_PWM.h"
//#include "PWM.h"
#include <avr/eeprom.h>

#define _sign(x) ((x) > 0 ? 1 : -1) //Сигнум для смены приращения скорости на уменьшение


#define SPEED_1     8 //8 исходно //6 при одном таймере
#define IN_1        9
#define IN_2        10

#define SPEED_2     11 //13 исходно// 11 после приведения к одной частоте //7 при одном таймере
#define IN_3        12
#define IN_4        13 //исх 11

#define ENC_1        2 //энкодер первый таймер 0
#define ENC_12       3

#define ENC_2        18 //энкодер второй таймер 1
#define ENC_22       19

bool m = 1; //Флаг прекращения вывода

/////////////////////////////////////////////
//////////// ПЕРЕМЕННЫЕ PID ///////////////// 

enum GM_workMode { //Определение направления вращения или режима удержания и торможения;
    FORWARD,
    BACKWARD,
    STOP,
    BRAKE,
    AUTO = 0,
}; 

GM_workMode _mode = FORWARD;

int _accel = 2000; //ускорение в отсчётах энкодера в секунду
int _maxSpeed = 10000; //максимальная скорость в отсчётах энкодера в секунду

int _dt = 10; //Временной шаг вызова функции регулирования
float _dts = (float)_dt/1000; //Временной шаг для подсчёта в единицах в секунду теор. значений скоростей V и позиций pos
uint32_t _tmr2 = 0; //Переменная таймера для подсчёта значений V и pos
long _targetPos = 48000; //Целевое положение в отсчётах энкодера

long controlPos = 0; //Теоретическое положение, идущее в PID в
float controlPosFl = 0;
float controlSpeed = 0;

uint8_t _minDuty = 30; //Минимальное значение страгивания ДПТ: (_minDuty-255)


long _prevInput = 0;
float _dutyF = 0;

float kp = 1.1;//2.0;		// (знач. по умолчанию)
  // интегральный - позволяет нивелировать ошибку со временем, имеет накопительный эффект
float ki = 0.00005;//0.9;		// (знач. по умолчанию)
    
    // дифференциальный. Позволяет чуть сгладить рывки, но при большом значении
    // сам становится причиной рывков и раскачки системы!
float kd = 0.01;//0.1;		// (знач. по умолчанию)

float integral = 0;

int _maxDuty = 255;

int _stopzone = 300; //отсечка по тикам
int _max = 400000, _min = -400000;

bool _cutoff = true; 


int16_t _duty = 0;

bool _level = 1;//Переменная направления для digitalWrite(IN_1) setpins в run
bool _direction = 0; //инверсия направлений при 1 с FORWARD на BACKWARD для отладки

uint8_t _state = 1;

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

//AVR_PWM* PWM_Instance;
//const float frequency = 30000;



///////////////////////////////////////////////////////////
//////////ПЕРЕМЕННЫЕ ЭНКОДЕРОВ И РЕАЛЬНОЙ СКОРОСТИ/////////

//uint8_t freq1 = 10; // частота опроса 
uint8_t per1 = 2;///freq1; //период опроса 1000/60 = 17 мс
uint32_t t1 = 0; //начальный момент времени unsigned long
uint32_t t2 = 0; //unsigned long 4

volatile int lastEncoded1 = 0; // Here updated value of encoder store.
volatile long encoderValue1 = 0; // Raw encoder value
volatile long encREF1 = 0; //сбрасываемое значениеэнкодера для скорости
float Velocity1 = 0.0; //Переменная мгновенной скорости 1


volatile int lastEncoded2 = 0; // Here updated value of encoder store.
volatile long encoderValue2 = 0; // Raw encoder value
volatile long encREF2 = 0; //сбрасываемое значениеэнкодера для скорости
float Velocity2 = 0.0; //Переменная мгновенной скорости 2

float T_Theoretic = 0;

int ratio = 8344; //8433 = 28*298
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

int Voltmax = 6; //Балансировочные значения напряжений для ДПТ
int VoltMIN = 3.3;

float POWER = 100; // скорость в процентах

int napravlenie = 1;


int PWM = map(POWER, 0, 100, 0, 120); //ограничение максимального значения скважности СТАРОЕ

uint32_t timer = 0;
uint32_t timer2 = 0;
uint16_t f = 0;
unsigned long time;

float ta = 0; //конечный момент времени
float tb = 0; //время окончания разгона
float tc = 0; //время окончания равномерного движения

float xb = 0;
float xc = 0;
float curTime = 0;
long controlPos2 = 0;
int N = 0; // Число шагов дискретизации 

int8_t curve = 1;

void setup() {

  setMinDuty(30);
  

  t1 = t2 = millis();

  TCCR4B = (TCCR4B & 0xF8) |3;
  TCCR1B = (TCCR1B & 0xF8) |3;

  //PWM_Instance = new AVR_PWM(SPEED_1, frequency, 50);

  Serial.begin(9600);
  

  for (int i = 0; i < 10; i++) {     

    pinMode(i, OUTPUT);
  
  }
  //pinMode(SPEED_1, HIGH);

  pinMode(ENC_1, INPUT_PULLUP); 
  pinMode(ENC_12, INPUT_PULLUP);

  digitalWrite(ENC_1, HIGH); //turn pullup resistor on
  digitalWrite(ENC_12, HIGH); //turn pullup resistor on

  pinMode(ENC_2, INPUT_PULLUP); 
  pinMode(ENC_22, INPUT_PULLUP);

  digitalWrite(ENC_2, HIGH); //turn pullup resistor on
  digitalWrite(ENC_22, HIGH); //turn pullup resistor on

  attachInterrupt(digitalPinToInterrupt(2), updateEncoder1, CHANGE); //номер таймера для 2 и 3 пинов - это 0 по таблице. Заменено функцией определеня таймера.
  attachInterrupt(digitalPinToInterrupt(3), updateEncoder1, CHANGE);

  attachInterrupt(4, updateEncoder2, CHANGE); 
  attachInterrupt(5, updateEncoder2, CHANGE); 

  analogWrite(SPEED_1, 0);
  analogWrite(SPEED_2, 0);

  //setDeg(360*3); //Выставление значения цели в градусах
  //setTarget(-15000);
  setObor(-35);

  teor(_targetPos, _accel, _maxSpeed);
  Serial.println("DTS: " + String(_dts));
  Calculus(_accel, _maxSpeed, _targetPos, _dts);
  
  delay(1000);
  timer = millis(); 
  timer2 = millis();
  _tmr2 = millis();

}

void loop() {


      //Serial.println(PWM);
      //analogWrite(SPEED_1, PWM);
      //analogWrite(SPEED_2, PWM); 

 // control_dpt(encoderValue1, encoderValue2); //Старый вариант
  //ControlPos2(); //Проверка подсчёта позиции
  Move2();
  

  if (millis()-t1 >= per1) {

    //8344 = 28 * 298 импульсов на оборот
    t1 = millis();

    noInterrupts();
    uint16_t encREF1copy = encREF1;
    uint16_t encREF2copy = encREF2;
    encREF1 = encREF2 = 0;
    
    interrupts();

    Velocity1 = (float)encREF1copy/(float)per1;
    Velocity2 = (float)encREF2copy/(float)per1;
    //controlPosFl = (float)controlPos/ratio;
    //Serial.println(controlPos);
  }
  
  //Serial.println(controlPos);
  //comp_cur_pos();

  //if (millis()-t2 >= 10) {

     //Serial.println(encoderValue1/8344); //8344 = 28 * 298 
     //Serial.println(encoderValue2/8344);
     //Serial.println(per1);
    //t2 = millis();

    
    //Serial.println();

 // }


}

void updateEncoder1(){

    int MSB1 = digitalRead(ENC_1);  //MSB = most significant bit
    int LSB1 = digitalRead(ENC_12); //LSB = least significant bit
    

    int encoded1 = (MSB1 << 1) |LSB1; //converting the 2 pin value to single number
    int sum1  = (lastEncoded1 << 2) | encoded1; //adding it to the previous encoded1 value

    
    if (sum1 == 0b1101 || sum1 == 0b0100 || sum1 == 0b0010 || sum1 == 0b1011) {
    
     encoderValue1 --;
     encREF1 --; 
    //Serial.println("Counter ClockWise");
    }

    if (sum1 == 0b1110 || sum1 == 0b0111 || sum1 == 0b0001 || sum1 == 0b1000) { 

     encoderValue1 ++;
     encREF1 ++;
    //Serial.println("ClockWise");
    }

    lastEncoded1 = encoded1; //store this value for next time

}

void updateEncoder2() {

    int MSB2  = digitalRead(ENC_2); //MSB = most significant bit
    int LSB2  = digitalRead(ENC_22); //LSB = least significant bit

    int encoded2 = (MSB2 << 1) |LSB2; //converting the 2 pin value to single number
    int sum2  = (lastEncoded2 << 2) | encoded2; //adding it to the previous encoded1 value

    if (sum2 == 0b1101 || sum2 == 0b0100 || sum2 == 0b0010 || sum2 == 0b1011) {
    
    encoderValue2 --; 
    encREF2 --;
    //Serial.println("Counter ClockWise");
    }

    if (sum2 == 0b1110 || sum2 == 0b0111 || sum2 == 0b0001 || sum2 == 0b1000) { 

    encoderValue2 ++;
    encREF2 ++;
    //Serial.println("ClockWise");
    }

    lastEncoded2 = encoded2; //store this value for next time

}

void Direction() { //направление вращения моторов напрямую

  switch (napravlenie) { 
    case 1:
    
      digitalWrite(IN_1, LOW); 
      digitalWrite(IN_2, HIGH);
      digitalWrite(IN_3, LOW); 
      digitalWrite(IN_4, HIGH); 
      break;

    case 0:
      digitalWrite(IN_1, HIGH); 
      digitalWrite(IN_2, LOW);
      digitalWrite(IN_3, HIGH); 
      digitalWrite(IN_4, LOW);
      break;

  }
}


void control_dpt(long enc1, long enc2) {

    comp_cur_pos();
    hod_v2(enc1);
    //hod_v2(enc2);
}


void hod_v2(long enc) {

    long _currentPos = enc;
    PIDcontrol(controlPos, _currentPos, true);
    
}


void comp_cur_pos() {

    if (millis() - _tmr2 >= _dt) {
      _dts = (millis() - _tmr2) / 1000.0;
      _tmr2 = millis();       
      long err = _targetPos - controlPos;												// "ошибка" позиции
      if (err != 0) {
            if (_accel != 0) {
                bool thisDir = (controlSpeed * controlSpeed / _accel / 2.0 >= abs(err));  // пора тормозить (false до приближения к параболе торможения обр. от ускор.)
                controlSpeed += _accel * _dts * (thisDir ? -_sign(controlSpeed) : _sign(err));
            } else {
                controlSpeed = err / _dts;	// профиль постоянной скорости
            }
            controlSpeed = constrain(controlSpeed, -_maxSpeed, _maxSpeed);
            controlPos += controlSpeed * _dts;
            controlPos = constrain(controlPos, -_targetPos, _targetPos);
      }
    }
}

void setPins(bool a, bool b, int c) {//Выход сигнала на пины

  digitalWrite(IN_1, a);
  digitalWrite(IN_2, b);
  analogWrite(SPEED_1, c);

}


int GetState() {return _state;}

void run(GM_workMode mode, int16_t duty) {

  if (_direction) {
    if (mode == FORWARD) mode = BACKWARD;
    else if (mode == BACKWARD) mode = FORWARD;
  }

  switch (mode) {
      case FORWARD:
        setPins(_level, !_level, duty);
        _state = 1;
        break;
      case BACKWARD:
        setPins(!_level, _level, duty);
        _state = -1;
        break;
      case BRAKE:
        setPins(!_level, !_level, !_level * 255);
        _state = 0;
        break;  // при 0/255 analogWrite сделает 0/1
      case STOP:
        setPins(_level, _level, _level * 255);
        _duty = 0;
        _state = 0;
        break;
  }

}


float _k = 1.0;
void setMinDuty(int duty) {
    _minDuty = duty;
    _k = 1.0 - (float)_minDuty / _maxDuty;
}

void setSpeed(int8_t duty) {

    if (_mode < 2) {  // FORWARD/BACKWARD/AUTO
        _duty = constrain(duty, -_maxDuty, _maxDuty);

        // фикс стандартного analogWrite(пин, 255) для >8 бит
        if (_maxDuty > 255 && abs(_duty) == 255) _duty++;

        if (duty == 0) {
          
          run(STOP, 0);
          eeprom_update_float(1, encoderValue1);
          
          }
        else {
            if (duty > 0) { //Вращение при регулировании в направлении вперёд по умолчанию 
                if (_minDuty != 0) _duty = _duty * _k + _minDuty;  // сжимаем диапазон
                run(_mode, _duty);
            } else {// Вращение в обратном напра
                if (_minDuty != 0) _duty = _duty * _k - _minDuty;  // сжимаем диапазон
                run(BACKWARD, -_duty);
            }
        }
    }
  
    // инверт
}

void PIDcontrol(long target, long current, bool cutoff) {
    // cutoff нужен только для стабилизации позиции, обнуляет integral и учитывает мёртвую зону мотора
    long err = target - current;				// ошибка регулирования
    long deltaInput = _prevInput - current;		// изменение входного сигнала
    _dutyF = 0;
    if (!cutoff) _dutyF = err * kp;				// P составляющая для режимов скорости	
    _dutyF += (float)deltaInput * kd / _dts;	// D составляющая
    _prevInput = current;						// запомнили текущий
    integral += (float)err * ki * _dts;			// интегральная сумма
    if (cutoff) integral += deltaInput * kp;	// +P по скорости изменения для режимов позиции
    integral = constrain(integral, -_maxDuty, _maxDuty);	// ограничили
    _dutyF += integral;							// I составляющая	
    if (cutoff) {								// отсечка (для режимов позиции)
        if (abs(err) < _stopzone) {integral = 0; _dutyF = 0;}
    } else {									// для скорости
        if (err == 0 && target == 0) integral = 0;
    }
    _dutyF = constrain(_dutyF, -_maxDuty, _maxDuty);	// ограничиваем по разрешению
    if (cutoff && _min != 0 && _max != 0 && (current <= _min || current >= _max)) {
        setSpeed(0);	// вырубаем, если вышли за диапазон
    } else setSpeed(_dutyF);								// и поехали
}

void teor (int position, int accel, int V) {

  float Vmax = (float)V;

  if (accel != 0) {

    T_Theoretic = float( ( (_targetPos - (pow(Vmax, 2)/accel)) / Vmax) + (2*Vmax/accel) );

  } else {

    T_Theoretic = (float)(_targetPos/Vmax);

  }
  
  Serial.println("T_Theoretic: " + String(T_Theoretic));
  //Serial.println("T_Theoretic = tc: Target/Vmax: " + String((float)_targetPos/Vmax));

}

void setTarget(long TargetPos) {
  
  if (TargetPos < 0) {_direction = 1;} //Инверсия направления

  _targetPos = abs(TargetPos);
  Serial.println("_targetPos: " + String(_targetPos));
  
  }

void setObor(float ob) {

  if (ob < 0) {_direction = 1;} //Инверсия направления
  _targetPos = abs(round(ob * ratio));//ratio - число тиков на оборот

  Serial.println("Ob: ");
  Serial.println(ob);
  Serial.println("_targetPos:");
  Serial.println(_targetPos);

}

void setDeg(long Deg) {

  if (Deg < 0) {_direction = 1;}// Инверсия направления

  _targetPos = abs(Deg*8344/360);

  Serial.println("Deg: ");
  Serial.println(Deg);
  Serial.println("_targetPos:");
  Serial.println(_targetPos);

}

void setRatio(uint16_t _ratio) {ratio = _ratio;}

void Calculus(int acceleration, int V, long targetPos, float dts) {//Подсчитывает моменты времени и

    float Xta = (float)targetPos;//Функция координаты от времени для t;
    float dti = dts;
    float Vmax = (float)V;
    float accel = acceleration;

    tb = Vmax/accel; //момент времени завершения разгона
    tc = Xta/(Vmax); //Момент времени завершения равномерного движения

    xb = (float)(Vmax*Vmax)/(2 * accel);//координата завершения разгона

    float taorig = 0.0;

    
    if (_accel != 0) {

      if (xb < _targetPos/2) { //Если возможна кривая ускорения, равномерного движения и замедления

        taorig = (float)(Xta/Vmax + Vmax/accel);//Завершающий момент времени 3 сегмента
        curve = 2; //3 сегмента кривой

      } else if (xb > _targetPos/2 && xb < _targetPos) {//Ускорение и равномерное движение

        taorig = _targetPos/Vmax + tb/2;
        tc = taorig;
        curve = 1;

      } else if (xb == _targetPos/2) {//Если возможно только ускорение и замедление
        
        taorig = (float)(2*(Vmax/accel));
        curve =0;
        
      } else if (xb >= _targetPos) { //Если возможно только ускорение

        taorig = sqrt(2*_targetPos/accel);
        curve = -1;
      } 

    }else {
      
      taorig = _targetPos/Vmax;
      
    }

    ta = (int)(round(taorig*100));
    ta = (float)ta/100;

    N = round(ta/dti);

    Serial.println("ta_Calculus: " + String(ta));
    Serial.println("tb: " + String(tb));
    Serial.println("xb: " + String(xb));
    
    //T = (0:dti:(ta+dti));
    //X = zeros(N+2, 1);
    Serial.println("tc: " + String(tc));
    Serial.println("Curve: " + String(curve));

}

void ControlPos2 () {

  if (millis() - timer >= _dt) {

    f++;
    curTime = f*_dts;
    timer = millis();
    
    if (_accel != 0 && controlPos2 <= _targetPos) {

      if (curTime <= tb) {
      
        controlPos2 = _accel*curTime*curTime/2;
        //Serial.println(controlPos2);
      
      }else if (curTime > tb  && (curTime <= tc || curve == 1) ) {//&& curTime <= tc

        controlPos2 = (_maxSpeed*curTime - xb); //xb - координата прекращения ускорения
        //Serial.println(controlPos2);

      }else if (curTime > tc && curTime <= ta && curve == 2) {

        controlPos2 = _targetPos - (_accel*((curTime - tc - tb)*(curTime-tc-tb))/2);
        //Serial.println(controlPos2);
      //Serial.println(curTime);
      }//else if (curTime > ta && curTime < ta+0.10) {controlPos2 = _targetPos;
    
    }
    else if (controlPos2 < _targetPos) {
      
      controlPos2 = _maxSpeed*curTime;
      
    }

    //Serial.println(millis()-timer2);    
    //}
    //Serial.println(f);
    //Serial.println(controlPos2);
  }

}    

void Move2() {

  ControlPos2();
  PIDcontrol(controlPos2, encoderValue1, true);
  
  if (controlPos2 <= _targetPos-1 && m == 1) {
    
    //Serial.println(controlPos2);

  }
  if (((controlPos2 >= _targetPos-2) || (controlPos >= _targetPos-2)) && m == 1) {
    
    m = 0;
    Serial.println(millis()-timer2);
    
  }
  //Serial.println(controlPos2);
}