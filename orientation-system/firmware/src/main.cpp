#include <Arduino.h>
#include "Application.h"

namespace { Application app; }

void setup() { app.begin(); }
void loop()  { app.loopOnce(); }
