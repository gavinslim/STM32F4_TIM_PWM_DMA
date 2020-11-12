/*
 * tm1637.c
 *
 *  Created on: Sep 25, 2020
 *      Author: Gavin
 */


#include <tm1637_.h>

void _tm1637Start(void);
void _tm1637Stop(void);
void _tm1637ReadResult(void);
void _tm1637WriteByte(unsigned char b);
void _tm1637DelayUsec(unsigned int i);
void _tm1637ClkHigh(void);
void _tm1637ClkLow(void);
void _tm1637DioHigh(void);
void _tm1637DioLow(void);

// Configuration.

#define CLK_PORT GPIOB	//PB0
#define DIO_PORT GPIOA	//PA4
#define CLK_PIN GPIO_PIN_0
#define DIO_PIN GPIO_PIN_4
#define CLK_PORT_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE
#define DIO_PORT_CLK_ENABLE __HAL_RCC_GPIOA_CLK_ENABLE

#define MICROSEC 2
#define MICROSEC_L 10

const char segmentMap[] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, // 0-7
    0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, // 8-9, A-F
    0x00
};

void tm1637Init(void)
{
    CLK_PORT_CLK_ENABLE();
    DIO_PORT_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pull = GPIO_NOPULL;
    g.Mode = GPIO_MODE_OUTPUT_PP; // OD = open drain
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Pin = CLK_PIN;
    HAL_GPIO_Init(CLK_PORT, &g);
    g.Pin = DIO_PIN;
    HAL_GPIO_Init(DIO_PORT, &g);

    tm1637SetBrightness(8);
}

void tm1637DisplayDecimal(int v, int displaySeparator)
{
    unsigned char digitArr[4];
    for (int i = 0; i < 4; ++i) {
        digitArr[i] = segmentMap[v % 10];
        if (i == 2 && displaySeparator) {
            digitArr[i] |= 1 << 7;
        }
        v /= 10;
    }

    _tm1637Start();
    _tm1637WriteByte(0x40);
    _tm1637ReadResult();
    _tm1637Stop();

    _tm1637Start();
    _tm1637WriteByte(0xc0);
    _tm1637ReadResult();

    for (int i = 0; i < 4; ++i) {
        _tm1637WriteByte(digitArr[3 - i]);
        _tm1637ReadResult();
    }

    _tm1637Stop();
}

void tm1637DisplayTime(int min, int sec, int displaySeparator)
{
    unsigned char digitArr[4];

    // Display minutes
    for (int i = 2; i < 4; ++i) {
        digitArr[i] = segmentMap[min % 10];
        if (i == 2 && displaySeparator) {
            digitArr[i] |= 1 << 7;
        }
        min /= 10;
    }

    // Display seconds
    for (int i = 0; i < 2; ++i) {
        digitArr[i] = segmentMap[sec % 10];
        sec /= 10;
    }

    _tm1637Start();
    _tm1637WriteByte(0x40);
    _tm1637ReadResult();
    _tm1637Stop();

    _tm1637Start();
    _tm1637WriteByte(0xc0);
    _tm1637ReadResult();

    for (int i = 0; i < 4; ++i) {
        _tm1637WriteByte(digitArr[3 - i]);
        _tm1637ReadResult();
    }

    _tm1637Stop();
}

// Valid brightness values: 0 - 8.
// 0 = display off.
void tm1637SetBrightness(char brightness)
{
    // Brightness command:
    // 1000 0XXX = display off
    // 1000 1BBB = display on, brightness 0-7
    // X = don't care
    // B = brightness
    _tm1637Start();
    _tm1637WriteByte(0x87 + brightness);
    _tm1637ReadResult();
    _tm1637Stop();
}

void _tm1637Start(void)
{
    _tm1637ClkHigh();
    _tm1637DioHigh();
    _tm1637DelayUsec(MICROSEC_L);
    _tm1637DioLow();
}

void _tm1637Stop(void)
{
    _tm1637ClkLow();
    _tm1637DelayUsec(MICROSEC);
    _tm1637DioLow();
    _tm1637DelayUsec(MICROSEC);
    _tm1637ClkHigh();
    _tm1637DelayUsec(MICROSEC);
    _tm1637DioHigh();
}

void _tm1637ReadResult(void)
{
    _tm1637ClkLow();
    _tm1637DelayUsec(MICROSEC_L);
    // while (dio); // We're cheating here and not actually reading back the response.
    _tm1637ClkHigh();
    _tm1637DelayUsec(MICROSEC);
    _tm1637ClkLow();
}

void _tm1637WriteByte(unsigned char b)
{
    for (int i = 0; i < 8; ++i) {
        _tm1637ClkLow();
        if (b & 0x01) {
            _tm1637DioHigh();
        }
        else {
            _tm1637DioLow();
        }
        _tm1637DelayUsec(3);
        b >>= 1;
        _tm1637ClkHigh();
        _tm1637DelayUsec(3);
    }
}

/*
void _tm1637DelayUsec(unsigned int i)
{
    for (; i>0; i--) {
        for (int j = 0; j < 10; ++j) {
            __asm__ __volatile__("nop\n\t":::"memory");
        }
    }
}
*/

void _tm1637DelayUsec(unsigned int i)
{
  volatile unsigned int j;
  while(i--)
  {
    for(j=0; j<10; j++)
    {
      __NOP();
    }
  }
}

void _tm1637ClkHigh(void)
{
    HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_SET);
}

void _tm1637ClkLow(void)
{
    HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_RESET);
}

void _tm1637DioHigh(void)
{
    HAL_GPIO_WritePin(DIO_PORT, DIO_PIN, GPIO_PIN_SET);
}

void _tm1637DioLow(void)
{
    HAL_GPIO_WritePin(DIO_PORT, DIO_PIN, GPIO_PIN_RESET);
}
