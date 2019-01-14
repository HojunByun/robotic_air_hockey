#include "tester.h"
#include "Arduino.h"

void change()
{
  digitalWrite(LED_BUILTIN, 1 - digitalRead(LED_BUILTIN));
}
