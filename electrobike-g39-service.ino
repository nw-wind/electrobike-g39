#include "SmartDelay.h"
#include "SmartButton.h"

/*

  Расширитель портов?

  Кнопки:
  Вкл/Выкл сервисного режима.
  Вверх/вниз.
  Фара вкл/выкл.
  Фара дальний вкл/выкл.

*/

// Кнопки
const byte pinService=8;
const byte pinUpDown=9;
const byte pinLight=10;
const byte pinBeam=11;

// Управление
const byte pinBeep=4;
const byte pinFrame=5;
const byte pinLightHigh=6;
const byte pinLightLow=7;

enum class ToggleState : byte {ON, OFF};

class SmartToggle public SmartButton {
  private:
    ToggleState state;
  public:
    SmartToggle::SmartToggle(byte port): SmartButton(port) {};
    SmartTogle~::SmartToggle() {};
    onClick() {
      if (state == ToggleState::ON) state = ToggleState::OFF;
      else state = ToggleState::ON;
    }
};



SmartToggle btService(pinService);
SmartToggle btUpDown(pinUpDown);
SmartToggle btLight(pinLight);
SmartToggle btBeam(pinBeam);

SmartDelay upDown(13000*1000UL);

void setup() {
  Serial.begin(115200);
}

void loop() {
  btService.run();
  btUpDown.run();
  btLight.run();
  btBeam.run();
  
}

