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


*/

#include "SmartDelay.h"

const byte pinInSpeedSensor = 2;
const byte pinOutIndicator = 4;
const byte pinOutDirecton = 5;
const byte pinOutStartStop = 6;
const byte pinOutVibro = 7;

const byte pinInGas = 0;

// Таймауты
const unsigned long toutDownToUp = 250 * 1000; // От стояния внизу до начала подъёма.
const unsigned long toutTurnOnToReady = 250 * 1000; // От готовности к включению до включения.

SmartDelay dDownToUp(toutDownToUp);
SmartDelay dTurnOnToReady(toutTurnOnToReady);

void setup() {

}

void loop() {
}
