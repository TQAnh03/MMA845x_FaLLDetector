#ifndef _DELAY_H
#define _DELAY_H

#include "stdio.h"
#include "stdint.h"

void Delay_Config(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
#endif