#include "freeram.h"

#if defined(ARDUINO_TEENSY32) || defined(ARDUINO_TEENSY35) || defined(ARDUINO_TEENSY36)
    extern char* __brkval;
    unsigned int freeram() {
        char tos;
        return &tos - __brkval;
    }
#elif defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41)
    extern unsigned long _heap_start;
    extern unsigned long _heap_end;
    extern char *__brkval;
    unsigned int freeram() {
        return (char *)&_heap_end - __brkval;
    }
#else
    unsigned int freeram() {
        printf("insert free ram function here!\n");
        return 0;
    }
#endif
