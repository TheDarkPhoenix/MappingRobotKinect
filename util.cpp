#include "util.h"

using namespace std;

int sendread(HANDLE hSerial, __int8 l)
{
    __int8* ch = &l;
    DWORD dwBytesRead = 0;
    __int8 low;
    __int8 high;
    __int16 bateria, dalmierz;
    __int8 enkoder1, enkoder2;
    if(!WriteFile(hSerial, ch, 1, &dwBytesRead, NULL))
    {
        cout << "error writing" << endl;
    }
    dwBytesRead = 0;
    if (l == 3)
    {
        if(!ReadFile(hSerial, &low, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        if(!ReadFile(hSerial, &high, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        dalmierz = low | (high << 8);
        cout << "Dalmierz: " << dalmierz << endl;
    }
    else if (l == 4)
    {
        if(!ReadFile(hSerial, &low, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        if(!ReadFile(hSerial, &high, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        bateria = low | (high << 8);
        cout << "Bateria: " << bateria << endl;
    }
    else if (l == 5)
    {
        /*if(!ReadFile(hSerial, &low, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        if(!ReadFile(hSerial, &high, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        enkoder1 = low | (high << 8);
        cout << "Enkoder1: " << enkoder1 << endl;
        return enkoder1;*/
        if(!ReadFile(hSerial, &enkoder1, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        //cout << "Enkoder1: " << (int)enkoder1 << endl;
        return enkoder1;
    }
    else if (l == 6)
    {
        /*if(!ReadFile(hSerial, &low, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        if(!ReadFile(hSerial, &high, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        enkoder2 = low | (high << 8);
        cout << "Enkoder2: " << enkoder2 << endl;
        return enkoder2;*/
        if(!ReadFile(hSerial, &enkoder2, 1, &dwBytesRead, NULL))
        {
            cout << "error reading" << endl;
        }
        //enkoder2 = -enkoder2;
        //cout << "Enkoder2: " << (int)enkoder2 << endl;
        return enkoder2;
    }
}

void init(HANDLE& hSerial, DCB& dcbSerialParams, COMMTIMEOUTS& timeouts)
{
    hSerial = CreateFile("COM9",
                         GENERIC_READ | GENERIC_WRITE,
                         0,
                         0,
                         OPEN_EXISTING,
                         FILE_ATTRIBUTE_NORMAL,
                         0);
    if(hSerial==INVALID_HANDLE_VALUE)
    {
        if(GetLastError()==ERROR_FILE_NOT_FOUND)
        {
            cout << "serial port doesnt exist" << endl;
        }
        cout << "error" << endl;
    }

    dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        cout << "error getting state" << endl;
    }
    dcbSerialParams.BaudRate=CBR_9600;
    dcbSerialParams.ByteSize=8;
    dcbSerialParams.StopBits=ONESTOPBIT;
    dcbSerialParams.Parity=NOPARITY;
    if(!SetCommState(hSerial, &dcbSerialParams))
    {
        cout << "error setting serial port state" << endl;
    }

    timeouts.ReadIntervalTimeout=50;
    timeouts.ReadTotalTimeoutConstant=50;
    timeouts.ReadTotalTimeoutMultiplier=10;
    timeouts.WriteTotalTimeoutConstant=50;
    timeouts.WriteTotalTimeoutMultiplier=10;
    if(!SetCommTimeouts(hSerial, &timeouts))
    {
        cout << "error occureed" << endl;
    }
}
