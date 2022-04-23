/*
Copyright (c) 2012-2013 Ben Croston

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <stdint.h>
#pragma once

class bpi_c_gpio {
    public:
        static int setup(void);
        static void setup_gpio(int gpio, int direction, int pud);
        static int gpio_function(int gpio);
        static void output_gpio(int gpio, int value);
        static int input_gpio(int gpio);
        static void set_rising_event(int gpio, int enable);
        static void set_falling_event(int gpio, int enable);
        static void set_high_event(int gpio, int enable);
        static void set_low_event(int gpio, int enable);
        static int eventdetected(int gpio);
        static void cleanup(void);

    private:
        static void set_pullupdn(int gpio, int pud);//void sunxi_pullUpDnControl (int pin, int pud)
        static uint32_t readl(uint32_t addr);
        static void writel(uint32_t val, uint32_t addr);
        static void clear_event_detect(int gpio);
};
#define BPI_SETUP_OK          0
#define BPI_SETUP_DEVMEM_FAIL 1
#define BPI_SETUP_MALLOC_FAIL 2
#define BPI_SETUP_MMAP_FAIL   3

#define BPI_INPUT  1 // is really 0 for control register!
#define BPI_OUTPUT 0 // is really 1 for control register!
#define BPI_ALT0   4

#define BPI_HIGH 1
#define BPI_LOW  0

#define BPI_PUD_OFF  0
#define BPI_PUD_DOWN 2
#define BPI_PUD_UP   1

//#define BAPI_DEBUG   1

#ifdef BAPI_DEBUG
#define	D	printf("BAPI: __%d__(%s:%s)\n",__LINE__,__FUNCTION__,__FILE__);
#else
#define D
#endif

#define lemakerDebug 0
