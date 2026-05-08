/*
 * LowPassFilter.c
 *
 *  Created on: Dec 22, 2024
 *      Author: Szymon
 */
#include "main.h"
#include"LowPassFilter.h"

void LowPassFilter_Init(LowPassFilter_t *LPF, float alpha)
{
	LPF->alpha = alpha;
	LPF->output = 0.0f;
}
float LowPassFilter_Update(LowPassFilter_t *LPF, float input)
{
	LPF->output = LPF->alpha * input + (1.0f - LPF->alpha) * LPF->output;

	return LPF->output;
}
