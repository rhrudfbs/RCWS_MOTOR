#include "Motor_class.h"

uint32_t TC_5_Rate = 1;
uint32_t TC_4_Rate = 1;
uint32_t TC_3_Rate = 1;
uint32_t TCC_2_Rate = 1;

void setup() {

    Serial.begin(115200);

    TC_5_tcConfigure(TC_5_Rate);
    TC_4_tcConfigure(TC_4_Rate);
    TC_3_tcConfigure(TC_3_Rate);
    TCC_2_tcConfigure(TCC_2_Rate);
}

void loop() {

}

void TC5_Handler(void) {



    TC5->COUNT16.INTFLAG.bit.MC0 = 1;
}

void TC4_Handler(void) {



    TC4->COUNT16.INTFLAG.bit.MC0 = 1;
}

void TC3_Handler(void) {



    TC3->COUNT16.INTFLAG.bit.MC0 = 1;
}

void TCC2_Handler(void)
{


    TCC2->INTFLAG.bit.OVF = 1;
}


void TC_5_tcConfigure(int sampleRate)
{
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
    while (GCLK->STATUS.bit.SYNCBUSY);

    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;

    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;

    TC5->COUNT16.CC[0].reg = (uint16_t)(SystemCoreClock * sampleRate);
    while (TC_5_tcIsSyncing());

    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TC_5_tcIsSyncing());

    NVIC_DisableIRQ(TC5_IRQn);
    NVIC_ClearPendingIRQ(TC5_IRQn);
    NVIC_SetPriority(TC5_IRQn, 0);
    NVIC_EnableIRQ(TC5_IRQn);

    TC5->COUNT16.INTENSET.bit.MC0 = 1;
    while (TC_5_tcIsSyncing());
}

bool TC_5_tcIsSyncing()
{
    return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void TC_4_tcConfigure(int sampleRate)
{
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
    while (GCLK->STATUS.bit.SYNCBUSY);

    TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

    TC4->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;

    TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;

    TC4->COUNT16.CC[0].reg = (uint16_t)(SystemCoreClock * sampleRate);
    while (TC_4_tcIsSyncing());

    TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TC_4_tcIsSyncing());

    NVIC_DisableIRQ(TC4_IRQn);
    NVIC_ClearPendingIRQ(TC4_IRQn);
    NVIC_SetPriority(TC4_IRQn, 0);
    NVIC_EnableIRQ(TC4_IRQn);

    TC4->COUNT16.INTENSET.bit.MC0 = 1;
    while (TC_4_tcIsSyncing());
}

bool TC_4_tcIsSyncing()
{
    return TC4->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void TC_3_tcConfigure(int sampleRate)
{
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC2_TC3));
    while (GCLK->STATUS.bit.SYNCBUSY);

    TC3->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
    TC3->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
    TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
    TC3->COUNT16.CC[0].reg = (uint16_t)(SystemCoreClock * sampleRate);
    while (TC_3_tcIsSyncing());

    TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TC_3_tcIsSyncing());

    NVIC_DisableIRQ(TC3_IRQn);
    NVIC_ClearPendingIRQ(TC3_IRQn);
    NVIC_SetPriority(TC3_IRQn, 0);
    NVIC_EnableIRQ(TC3_IRQn);

    TC3->COUNT16.INTENSET.bit.MC0 = 1;
    while (TC_3_tcIsSyncing());
}

bool TC_3_tcIsSyncing()
{
    return TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void TCC_2_tcConfigure(int sampleRate)
{
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3;
    while (GCLK->STATUS.bit.SYNCBUSY);

    TCC2->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
    while (TCC2->SYNCBUSY.bit.WAVE);

    TCC2->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_PRESCSYNC_PRESC;

    TCC2->CC[0].reg = (uint16_t)(SystemCoreClock * sampleRate);

    TCC2->CTRLA.bit.ENABLE = 1;
    while (TCC2->SYNCBUSY.bit.ENABLE);

    NVIC_DisableIRQ(TCC2_IRQn);
    NVIC_ClearPendingIRQ(TCC2_IRQn);
    NVIC_SetPriority(TCC2_IRQn, 0);
    NVIC_EnableIRQ(TCC2_IRQn);

    TCC2->INTENSET.bit.OVF = 1;
}