/* Garage 39 project
  Pins D:
  2 Датчик скорости (холла)
  3 Управление газом
  4 Индикатор
  5 Направление: подъём/опускание
  6 Старт/стоп подвески
  7 Вибрация в ручках
  8 левая ручка
  9 правая ручка
  10 нижний концкевик
  11 верхний концевик
  12 вход от стопсигнала

  Pins A:
  0 Датчик газа
  1 



*/

// #include "ssd1306.h"
#include <stdio.h>
#include "SmartDelay.h"

#define DIR_SIGNAL_UP (LOW)
#define DIR_SIGNAL_DOWN (HIGH)

const byte pinInSpeedSensor = 2;
const byte pinOutGas = 3;
const byte pinOutIndicator = 4; // свет (зелёный)
const byte pinOutDirecton = 5; // направление подъём/спуск
const byte pinOutStartStop = 6; // подвеска старт/стоп
const byte pinOutVibro = 7; // (beep)
const byte pinInLeft = 8; // левая ручка
const byte pinInRight = 9; // правая ручка
const byte pinInUp = 10;   // верхний концевик
const byte pinInDown = 11; // нижний концевик
const byte pinInBrake = 12; // имитация датчика тормоза (вместе с аналоговым газом) оно же стопсигнал (красный)

const byte pinInGas = 0;

// Таймауты
//const unsigned long toutDownToUp = 250 * 1000UL; // От стояния внизу до начала подъёма.
//const unsigned long toutTurnOnToReady = 250 * 1000UL; // От готовности к включению до включения.

SmartDelay dDebug(3000 * 1000UL); // вывод отладки
SmartDelay disp(250*1000UL); // обновление дисплея

// Что используется?
//SmartDelay dDownToUp(toutDownToUp);
//SmartDelay dTurnOnToReady(toutTurnOnToReady);
SmartDelay dDelay14(250 * 1000UL);
SmartDelay dDelay2(2 * 1000UL * 1000UL);
SmartDelay dDelay10(10 * 1000UL * 1000UL);

enum EventFrame {ErrorEventFrame = 0, HandlesOn, HandlesOff, IsUp, IsDown};
enum StateFrame {ErrorStateFrame = 0, TurnedOff, ReadyToTurnOn, ReadyToTurnOff, StayDown, ReadyToGoUp, GoingUp, StayUp, ReadyToGoDown, GoingDown};
enum StateFrame stateFrame = TurnedOff;

enum EventBike {ErrorEventBike = 0, SpeedUp, SpeedOff, BrakingOn, BrakingOff };
enum StateBike {ErrorStateBike = 0, Moving, Moved, Standing, Stopped };
enum StateBike stateBike = Standing;

byte stateBrake = 0;

/*
  byte buttonState = 0x00; // состояние кнопок
  // биты состояния
  #define LEFT_HAND (0x01)
  #define RIGHT_HAND (0x02)
*/

// #define isButtonPressed(button,mask) ((button) & (mask))
// #define isButtonReleased(button,mask) (!((button) & (mask)))

template<class T> inline Print &operator <<(Print &obj, T arg) {
  obj.print(arg);
  return obj;
}
#define endl ("\n")
#define eol (endl)

/*
  #ifdef DEBUG_LOG
  #define log(obj) (Serial << obj)
  #else
  #define log(obj) (;)
  #endif
*/

unsigned long currentSpeed = 0;
unsigned long currentGotSpeed = 0;

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

void doVibro(byte init = 0, unsigned long ms = 250) { // 1 старт, 0 ждать
  static byte vs = 0;
  static SmartDelay vd(ms * 1000UL);
  if (init) {
    digitalWrite(pinOutVibro, HIGH);
    vd.Wait();
    vs = 1;
    Serial.println("Vibro on");
  } else {
    if (vs && vd.Now()) {
      digitalWrite(pinOutVibro, LOW);
      vs = 0;
      Serial.println("Vibro off");
    }
  }
}

void doStartStop(byte init = 0) {
  static byte vs = 0;
  static SmartDelay vd(250 * 1000UL); // время нажатия кнопки старт/стоп
  if (init) {
    digitalWrite(pinOutStartStop, HIGH);
    vd.Wait();
    vs = 1;
    Serial.println("Start/Stop on");
  } else {
    if (vs && vd.Now()) {
      digitalWrite(pinOutStartStop, LOW);
      vs = 0;
      Serial.println("Start/Stop off");
    }
  }
}

const byte DIR_UP = 0;
const byte DIR_DOWN = 1;

void doDirection(byte dir) {
  switch (dir) {
    case DIR_UP:
      digitalWrite(pinOutDirecton, DIR_SIGNAL_UP);
      Serial << "susp dir UP " << endl;
      break;
    case DIR_DOWN:
      digitalWrite(pinOutDirecton, DIR_SIGNAL_DOWN);
      Serial << "susp dir DOWN " << endl;
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

  digitalWrite(pinInSpeedSensor, LOW);
  pinMode(pinInSpeedSensor, INPUT_PULLUP);

  pinMode(pinOutIndicator, OUTPUT);
  pinMode(pinOutDirecton, OUTPUT);
  pinMode(pinOutStartStop, OUTPUT);
  pinMode(pinOutVibro, OUTPUT);

  digitalWrite(pinOutIndicator, LOW);
  digitalWrite(pinOutDirecton, DIR_SIGNAL_UP);
  digitalWrite(pinOutStartStop, LOW);
  digitalWrite(pinOutVibro, LOW);

  pinMode(pinInUp, INPUT_PULLUP);
  pinMode(pinInDown, INPUT_PULLUP);
  pinMode(pinInBrake, INPUT_PULLUP);

  // ssd1306_128x32_i2c_init();
  //ssd1306_fillScreen( 0x00 );
  //ssd1306_clearScreen();
  ssd1306_charF6x8(0, 0, "Electrobike v0.9");
  ssdprint(0, 1, "%-20s", "TurnedOff");
  ssdprint(0, 2, "%-20s", "Standing");

  attachInterrupt(digitalPinToInterrupt(pinInSpeedSensor), intSpeed, RISING);
  Serial.println("READY");
}

void ssdprint(byte c, byte r, const char *fmt, ...) {
  char buf[80];
  va_list argList;
  va_start(argList, fmt);
  vsprintf(buf, fmt, argList);
  va_end(argList);
  //ssd1306_charF6x8(c, r, buf);
}

// используется?
SmartDelay dToUp(4 * 1000UL * 1000UL); // ожидание до подъёма и спуска (трогание кнопок)
SmartDelay dToDown(30 * 1000UL * 1000UL); // ожидание до подъёма и спуска (трогание кнопок)
SmartDelay dToTurnOn(2 * 1000UL * 1000UL); // ожидание включения. выключено - стоит внизу
SmartDelay dToTurnOff(10 * 1000UL * 1000UL);
SmartDelay dFrameMovement(120 * 1000UL * 1000UL); // время подъёма/спуска подвески

void doEventFrame(enum EventFrame eventFrame) {
  // Обрабатываем события МКА
  switch (eventFrame) {
    case HandlesOn:
      switch (stateFrame) {
        case TurnedOff:
          stateFrame = ReadyToTurnOn;
          dToTurnOn.Wait();
          doVibro(1);
          Serial.println("state = ReadyToTurnOn");
          ssdprint(0, 1, "%-20s", "ReadyToTurnOn");
          break;
        case ReadyToTurnOn:
          if (dToTurnOn.Now()) {
            stateFrame = StayDown;
            digitalWrite(pinOutIndicator, HIGH);
            doVibro(1, 1000);
            Serial.println("state = StayDown");
            ssdprint(0, 1, "%-20s", "StayDown");
          }
          break;
        case ReadyToTurnOff:
          break;
        case StayDown:
          stateFrame = ReadyToGoUp;
          dToUp.Wait();
          Serial.println("state = ReadyToGoUp");
          ssdprint(0, 1, "%-20s", "ReadyToGoUp");
          break;
        case ReadyToGoUp:
          if (dToUp.Now()) {
            stateFrame = GoingUp;
            doVibro(1);
            doUpDown(1, DIR_UP);
            dFrameMovement.Wait();
            Serial.println("state = GoingUp");
            ssdprint(0, 1, "%-20s", "GoingUp");
          }
          break;
        case  GoingUp:
          if (dFrameMovement.Now()) {
            stateFrame = StayUp;
            doVibro(1);
            Serial.println("state = StayUp TIMEOUT");
            ssdprint(0, 1, "%-20s", "StayUp");
          }
          break;
        case StayUp:
          break;
        case ReadyToGoDown:
          stateFrame = StayUp;
          ssdprint(0, 1, "%-20s", "StayUp");
          break;
        case GoingDown:
          // подождать и поехать наверх
          break;
        case ErrorStateFrame:
          break;
      }
      break;
    case HandlesOff:
      switch (stateFrame) {
        case TurnedOff:
          break;
        case ReadyToTurnOn:
          break;
        case ReadyToTurnOff:
          if (dToTurnOff.Now()) {
            stateFrame = TurnedOff;
            digitalWrite(pinOutIndicator, LOW);
            doVibro(1, 1000);
            Serial.println("state = TurnedOff");
            ssdprint(0, 1, "%-20s", "TurnedOff");
          }
          break;
        case StayDown:
          stateFrame = ReadyToTurnOff;
          dToTurnOff.Wait();
          doVibro(1);
          Serial.println("state = ReadyToTurnOff");
          ssdprint(0, 1, "%-20s", "ReadyToTurnOff");
          break;
        case ReadyToGoUp:
          break;
        case  GoingUp:
          break;
        case StayUp:
          if (stateBike == Standing && !stateBrake) { // стоим и правый тормоз не нажат
            stateFrame = ReadyToGoDown;
            dToDown.Wait();
            Serial.println("state = ReadyToGoDown");
            ssdprint(0, 1, "%-20s", "ReadyToGoDown");
          } else {
            //Serial.println("StayUp Hands off -> we're moving");
          }
          break;
        case ReadyToGoDown:
          if (stateBike != Standing || stateBrake) { // не опускаемся если двигаемся или нажат тормоз
            stateFrame = StayUp;
          } else {
            if (dToDown.Now()) {
              stateFrame = GoingDown;
              doVibro(1);
              doUpDown(1, DIR_DOWN);
              dFrameMovement.Wait();
              Serial.println("state = GoingDown");
              ssdprint(0, 1, "%-20s", "GoingDown");
            }
          }
          break;
        case GoingDown:
          if (dFrameMovement.Now()) {
            stateFrame = StayDown;
            doVibro(1);
            Serial.println("state = StayDown TIMEOUT");
            ssdprint(0, 1, "%-20s", "StayDown");
          }
          break;
        case ErrorStateFrame:
          break;
      }
      break;
    case IsUp:
      switch (stateFrame) {
        case GoingUp:
          Serial.println("IsUp: stop");
          stateFrame = StayUp;
          doUpDown(2);
          doVibro(1);
          ssdprint(0, 1, "%-20s", "StayUp");
      }
      break;
    case IsDown:
      switch (stateFrame) {
        case GoingDown:
          Serial.println("IsDn: stop");
          stateFrame = StayDown;
          doUpDown(2);
          doVibro(1);
          ssdprint(0, 1, "%-20s", "StayDown");
      }
      break;
  }

}

void doEventBike(enum EventBike eventBike) {
  // Обрабатываем события МКА
  switch (eventBike) {
    case SpeedUp:
      switch (stateBike) {
        case Moving:
          break;
        case Moved:
          break;
        case Standing:
          stateBike = Moving; // Moved
          ssdprint(0, 2, "%-20s", "Moving");
          break;
        case Stopped:
          break;
        case ErrorStateBike:
          break;
      }
      break;
    case SpeedOff:
      switch (stateBike) {
        case Moving:
          stateBike = Standing; // Moved
          ssdprint(0, 2, "%-20s", "Standing");
          break;
        case Moved:
          break;
        case Standing:
          break;
        case Stopped:
          break;
        case ErrorStateBike:
          break;
      }
      break;
    case BrakingOn:
      stateBrake = 1;
      switch (stateBike) {
        case Moving:
          break;
        case Moved:
          break;
        case Standing:
          break;
        case Stopped:
          break;
        case ErrorStateBike:
          break;
      }
      break;
    case BrakingOff:
      stateBrake = 0;
      switch (stateBike) {
        case Moving:
          break;
        case Moved:
          break;
        case Standing:
          break;
        case Stopped:
          break;
        case ErrorStateBike:
          break;
      }
      break;
  }
}

byte possibleMove() {
  return stateFrame == StayUp;
}

byte possibleBrake() {
  return stateFrame != TurnedOff;
}

byte isBrakeOn() {
  return stateBrake || digitalRead(pinInBrake);
}

void displayState() {
  if (disp.Now()) {
    // рисуем дисплей
  }
}

void checkGas() {
  byte d = dDebug.Now();
  // Читаем ручку газа
  int i = analogRead(pinInGas);
  int gas = map(i, 100, 1023 , 0, 1023);
  if (gas<0) gas = 0;
  // Вывод отладки, если пора
  if (d) Serial << "checkGas: Pin = " << i << " Gas = " << gas << " ";
  // Пишем газ и тормоз в исполнительные механизмы
  if (possibleMove()) {
    // если можно ехать - едем
    analogWrite(pinOutGas, gas);      // газ в контроллер
    if (d) Serial << "(moving is possible) Gas = " << gas << " ";
  } else {
    // нельзя ехать
    analogWrite(pinOutGas, 0);                              // стоять, газ отключен
    if (d) Serial << "(gas is blocked) ";
  }
  if (isBrakeOn() && stateFrame != TurnedOff) {
    doEventBike(BrakingOn);
  } else {
    doEventBike(BrakingOff);
  }
  if (d) Serial << eol;
}

void loop() {
  // Утилиты
  // считаем скорость
  if (veloUpdate.Now()) {
    currentGotSpeed = calcSpeed();
    currentSpeed = (currentGotSpeed + currentSpeed) / 2;
    ssdprint(0, 3, "Speed = %3d", currentSpeed);
  }
  // газ/тормоз читаем
  checkGas();
  // выводим на дисплей
  displayState();

  // Рама
  // читаем состояние ручек руля
  if (bothHandles()) doEventFrame(HandlesOn);
  else doEventFrame(HandlesOff);
  if (digitalRead(pinInUp)) doEventFrame(IsUp);
  if (digitalRead(pinInDown)) doEventFrame(IsDown);
  // Эффекты
  doVibro();
  doStartStop();
  doUpDown();

  // Мотик
  // читаем состояние движения
  if (currentSpeed > 0) doEventBike(SpeedUp);
  if (currentSpeed == 0) doEventBike(SpeedOff);
}
