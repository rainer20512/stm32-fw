#include "Utilities.h"
#include <stdlib.h>
#include <stdio.h>

uint8_t TestBit(uint8_t var, int8_t bi)
{
    if (((var) & (0b00000001 << (bi))))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

char *my_utoa(char *str, unsigned int value, int radix)
{
    int size;
    const char *format = 0;
    switch (radix)
    {
    case 8:
        format = "%o";
        break;
    case 10:
        format = "%u";
        break;
    case 16:
        format = "%x";
        break;
    }
    if (format == 0)
        return str;
    size = sprintf(str, format, value);
    return &str[size];
}

uint8_t ArrayLength(uint8_t *arr)
{
    return sizeof(arr) / sizeof(arr[0]);
}