/* Garage 39 project
  Pins D:
  2 Датчик скорости (холла)
  3
  4 Индикатор
  5 Направление: подъём/опускание
  6 Старт/стоп подвески
  7 Вибрация в ручках

  Pins A:
  0 Датчик газа
  1 Управление газом
  2 Управление тормозом
  3 Управление скоростью подъёма/спуска


*/

#include "SmartDelay.h"

const byte pinInSpeedSensor = 3;
const byte pinOutIndicator = 4;
const byte pinOutDirecton = 5;
const byte pinOutStartStop = 6;
const byte pinOutVibro = 7;
const byte pinInLeft = 8; // левая ручка
const byte pinInRight = 9; // правая ручка
const byte pinInUp = 10;   // верхний концевик
const byte pinInDown = 11; // нижний концевик

const byte pinInGas = 0;
const byte pinOutGas = 1;
const byte pinOutBrake = 2;
const byte pinOutSuspSpeed = 3;

// Таймауты
//const unsigned long toutDownToUp = 250 * 1000UL; // От стояния внизу до начала подъёма.
//const unsigned long toutTurnOnToReady = 250 * 1000UL; // От готовности к включению до включения.

SmartDelay dDebug(3000 * 1000UL); // вывод отладки

//SmartDelay dDownToUp(toutDownToUp);
//SmartDelay dTurnOnToReady(toutTurnOnToReady);
SmartDelay dDelay14(250 * 1000UL);
SmartDelay dDelay2(2 * 1000UL * 1000UL);
SmartDelay dDelay10(10 * 1000UL * 1000UL);

//enum Event {Idle = 0, ReadyToTurnOffDone, LeftOn, RightOn, LeftOff, RightOff, ReadyToTurnOnDone, ReadyToGoUpDownDone, IsUp, IsDown, SpeedUp, SpeedOff, Reset, Delay14, Delay2, Delay10};
//enum State {StayDown = 0, ReadyToGoUp, GoingUp, StayUp, ReadyGoDown, Moving, ReadyToStop, GoingDown, TurnedOff, ReadyToTurnOn, ReadyToTurnOff, ErrorState};

enum Event {Idle = 0, ReadyToTurnOffDone, HandlesOn, HandlesOff, ReadyToTurnOnDone, ReadyToGoUpDownDone, IsUp, IsDown, SpeedUp, SpeedOff, Reset};
enum State {StayDown = 0, ReadyToGoUp, GoingUp, StayUp, ReadyGoDown, Moving, ReadyToStop, GoingDown, TurnedOff, ReadyToTurnOn, ReadyToTurnOff, ErrorState};


enum State state = TurnedOff;

/*
  byte buttonState = 0x00; // состояние кнопок
  // биты состояния
  #define LEFT_HAND (0x01)
  #define RIGHT_HAND (0x02)
*/

#define isButtonPressed(button,mask) ((button) & (mask))
#define isButtonReleased(button,mask) (!((button) & (mask)))

template<class T> inline Print &operator <<(Print &obj, T arg) {
  obj.print(arg);
  return obj;
}
#define endl ("\n")
#define eol (endl)

unsigned long currentSpeed = 0;

volatile unsigned int veloTick = 0;
volatile unsigned long veloMicros = 0;
unsigned long veloDist = 0;
const unsigned int veloLength = 3.14f * 25 * 2.54f; // 25"

byte bothHandles() {
  return digitalRead(pinInLeft) && digitalRead(pinInRight);
}

void intSpeed() {
  veloTick++;
  veloMicros = micros();
  //Serial.println("intr");
}

SmartDelay veloUpdate(250 * 1000UL);

unsigned long  calcSpeed() {
  unsigned long velo;
  if (veloTick > 0) {
    velo = veloLength / ((float)(veloMicros - veloDist) / veloTick / 3600.f);
    veloTick = 0;
    veloDist = veloMicros;
  } else {
    velo = 0;
  }
  if (velo > 400) velo = 0;
  //Serial << "Speed=" << velo << eol;
  return velo;
}

void doVibro(byte init = 0) { // 1 старт, 0 ждать
  static byte vs = 0;
  static SmartDelay vd(250 * 1000UL);
  if (init) {
    digitalWrite(pinOutVibro, HIGH);
    vd.Wait();
    vs = 1;
  } else {
    if (vs && vd.Now()) {
      digitalWrite(pinOutVibro, LOW);
      vs = 0;
    }
  }
}

void doStartStop(byte init = 0) {
  static byte vs = 0;
  static SmartDelay vd(250 * 1000UL);
  if (init) {
    digitalWrite(pinOutStartStop, HIGH);
    vd.Wait();
    vs = 1;
  } else {
    if (vs && vd.Now()) {
      digitalWrite(pinOutStartStop, LOW);
      vs = 0;
    }
  }
}

const byte DIR_UP = 0;
const byte DIR_DOWN = 1;

void doDirection(byte dir) {
  switch (dir) {
    case DIR_UP:
      digitalWrite(pinOutDirecton, LOW);
      break;
    case DIR_DOWN:
      digitalWrite(pinOutDirecton, HIGH);
      break;
  }
}

void doUpDown(byte init = 0, byte dir = 0) {
  static byte vs = 0;
  static SmartDelay vd(120 * 1000UL * 1000UL); // надо поднять за 2 минуты
  if (init == 1) {
    Serial << "susp start " << init << " " << dir << eol;
    doDirection(dir);
    vd.Wait();
    doStartStop(1);
    vs = 1;
  } else {
    if (vs && (vd.Now() || init == 2)) {
      Serial << "susp stop" << init << eol;
      doStartStop(1);
      vs = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.print("Starting...");
  pinMode(pinInLeft, INPUT_PULLUP);
  pinMode(pinInRight, INPUT_PULLUP);

  //Serial.println("before speed inp");
  //delay(1000);
  digitalWrite(pinInSpeedSensor, LOW);
  pinMode(pinInSpeedSensor, INPUT);
  //Serial.println("speed inp");
  //delay(1000);

  pinMode(pinOutIndicator, OUTPUT);
  pinMode(pinOutDirecton, OUTPUT);
  pinMode(pinOutStartStop, OUTPUT);
  pinMode(pinOutVibro, OUTPUT);

  pinMode(pinInUp, INPUT_PULLUP);
  pinMode(pinInDown, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinInSpeedSensor), intSpeed, RISING);
  Serial.println("READY");
}

SmartDelay dToUpDown(4 * 1000UL * 1000UL); // ожидание до подъёма и спуска (трогание кнопок)
SmartDelay dToTurnOn(2 * 1000UL * 1000UL); // ожидание включения. выключено - стоит внизу
SmartDelay dToTurnOff(10 * 1000UL * 1000UL);

void doEvent(enum Event event) {
  // Обрабатываем события МКА
  switch (event) {
    case ReadyToTurnOffDone:
      Serial.println("Left Off");
      state = TurnedOff;
      digitalWrite(pinOutIndicator, LOW);
      doVibro(1);
      break;
    case HandlesOn:
      //Serial.println("Hands On");
      switch (state) {
        case TurnedOff:
          state = ReadyToTurnOn;
          dToTurnOn.Wait();
          Serial.println("state = ReadyToTurnOn");
          break;
        case StayDown:
          state = ReadyToGoUp;
          dToUpDown.Wait();
          Serial.println("state = ReadyToGoUp");
          break;
      }
      break;
    case HandlesOff:
      //Serial.println("Hands Off");
      switch (state) {
        case StayDown:
          state = ReadyToTurnOff;
          digitalWrite(pinOutIndicator, LOW);
          Serial.println("state = TurnedOff");
          dToTurnOff.Wait();
          break;
        case StayUp:
          Serial.println("stayup -> going down");
          state = GoingDown; ////
          break;
        case ReadyToGoUp:
          Serial.println("to turn off");
          state = ReadyToTurnOff;
          // отключиться через 10 сек
          dToTurnOff.Wait();
          break;
      }
      break;
    case ReadyToTurnOnDone:
      Serial.println("ReadyToTurnOnDone");
      digitalWrite(pinOutIndicator, HIGH);
      state = ReadyToGoUp;
      dToUpDown.Wait();
      Serial.println("state = ReadyToGoUp");
      doVibro(1);
      break;
    case ReadyToGoUpDownDone:
      Serial.println("ReadyToGoUpDownDone");
      switch (state) {
        case ReadyToGoUp:
          state = GoingUp;
          Serial.println("turn suspension up");
          // управление контроллером подъёма
          doVibro(1);
          doUpDown(1, DIR_UP);
      }
      break;
    case IsUp:
      switch (state) {
        case GoingUp:
          Serial.println("IsUp: stop");
          state = StayUp;
          doUpDown(2);
      }
      break;
    case IsDown:
      switch (state) {
        case GoingDown:
          Serial.println("IsDn: stop");
          state = StayDown;
          doUpDown(2);
      }
      break;
  }

}

void checkGas() {
  byte d = 0; //dDebug.Now();
  // Читаем ручку газа/тормоза и отделяем газ от тормоза
  int i = analogRead(pinInGas);
  int gas = map(i, 512, 0 , 0, 1023);
  if (i > 512) gas = 0;
  int brake = map(i, 513, 1023, 0, 1023);
  if (i < 513) brake = 0;
  // Вывод отладки, если пора
  if (d) Serial << "checkGas: Pin=" << i << " Gas=" << gas << " Brake=" << brake << eol;
  // Пишем газ и тормоз в исполнительные механизмы
  if (state == StayUp && state == Moving && state == ReadyToStop) {
    // если можно ехать - едем
    analogWrite(pinOutGas, gas);      // газ в контроллер
    analogWrite(pinOutBrake, brake);  // тормоз в хз куда
    if (d) Serial << "checkGas (moving is possible) Gas=" << gas << " Brake=" << brake << eol;
  } else {
    // нельзя ехать
    analogWrite(pinOutGas, 0);                              // стоять, газ отключен
    if (d) Serial << "checkGas (gas released)" << eol;
    if (state != StayDown) {
      analogWrite(pinOutBrake, brake); // тормоз работает
      if (d) Serial << "checkGas (brake works) Brake=" << brake << eol;
    } else {
      analogWrite(pinOutBrake, 0);     // не тормозить тк стоим
      if (d) Serial << "checkGas (brake released)" << eol;
    }
  }
}

void loop() {
  // считаем скорость
  if (veloUpdate.Now()) currentSpeed = calcSpeed();
  // газ/тормоз читаем
  checkGas();
  // читаем состояние ручек руля
  if (bothHandles()) doEvent(HandlesOn);
  else doEvent(HandlesOff);
  /*
    if (isButtonReleased(buttonState, LEFT_HAND) && digitalRead(pinInLeft)) doEvent(LeftOn);
    if (isButtonPressed(buttonState, LEFT_HAND) && !digitalRead(pinInLeft)) doEvent(LeftOff);
    if (isButtonReleased(buttonState, RIGHT_HAND) && digitalRead(pinInRight)) doEvent(RightOn);
    if (isButtonPressed(buttonState, RIGHT_HAND) && !digitalRead(pinInRight)) doEvent(RightOff);
  */
  // читаем состояние движения
  if (currentSpeed > 0) doEvent(SpeedUp);
  if (currentSpeed == 0) doEvent(SpeedOff);
  if (digitalRead(pinInUp)) doEvent(IsUp);
  if (digitalRead(pinInDown)) doEvent(IsDown);
  // таймеры
  if (state == ReadyToTurnOn && dToTurnOn.Now()) doEvent(ReadyToTurnOnDone);
  if (state == ReadyToGoUp && dToUpDown.Now()) doEvent(ReadyToGoUpDownDone);
  if (state == ReadyToTurnOff && dToTurnOff.Now()) doEvent(ReadyToTurnOffDone);
  // Эффекты
  doVibro();
  doStartStop();
  doUpDown();
}
