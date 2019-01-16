#include "tester.h"

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  change();
  delay(500);
}
