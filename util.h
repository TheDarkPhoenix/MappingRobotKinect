#ifndef UTIL_H
#define UTIL_H

#include <windows.h>
#include <conio.h>
#include <iostream>

#include "types.h"

int sendread(HANDLE hSerial, __int8 l);
void init(HANDLE& hSerial, DCB& dcbSerialParams, COMMTIMEOUTS& timeouts);

#endif // UTIL_H

