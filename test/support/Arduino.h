#pragma once

// Minimal host stub so Config.h pin names resolve without an Arduino core.
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW 0
#endif
#ifndef A0
#define A0 14
#endif
#ifndef A1
#define A1 15
#endif
#ifndef A2
#define A2 16
#endif
#ifndef A3
#define A3 17
#endif
#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif
#ifndef F
#define F(x) (x)
#endif
