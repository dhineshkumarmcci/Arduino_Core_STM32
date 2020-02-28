/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include "stm32l0xx_hal.h"
#include "stm32l0xx.h"
#include <stdio.h>

uint32_t MeasureMicrosPerRtcSecond(void);
uint32_t Stm32_CalibrateSystemClock(void);

static void rtccount_begin();
static uint16_t rtccount_read();
uint32_t SystemClock_Calib(void);

uint32_t calibTime;
uint32_t calibValue;
uint16_t lptimCounter;

/**
  * @brief  The application entry point.
  * @retval int
  */

/* RTC overrides for "weak" definitions in core HAL. These definitions 	*/
/* are already defined in library Catena-Arduino-Platform, which will 	*/
/* throw an error. Thus these definitions should be ported here without	*/
/* being used in library. Link for the library file - 					*/
/* https://github.com/mcci-catena/Catena-Arduino-Platform/blob/master/src/lib/stm32/stm32l0/CatenaStm32L0Rtc.cpp */

/*
extern "C" {

static volatile uint32_t *gs_pAlarm;
static RTC_HandleTypeDef *gs_phRtc;

void RTC_IRQHandler(void)
    {
    HAL_RTC_AlarmIRQHandler(gs_phRtc);
    }

void HAL_RTC_AlarmAEventCallback(
    RTC_HandleTypeDef *	hRtc
    )
    {
    if (gs_pAlarm)
        *gs_pAlarm = 1;
    }

void HAL_RTC_MspInit(
    RTC_HandleTypeDef *	hRtc
    )
    {
    if (hRtc->Instance == RTC)
        {
        // USER CODE BEGIN RTC_MspInit 0

        // USER CODE END RTC_MspInit 0
        // Peripheral clock enable
        __HAL_RCC_RTC_ENABLE();
        // USER CODE BEGIN RTC_MspInit 1
        HAL_NVIC_SetPriority(RTC_IRQn, TICK_INT_PRIORITY, 0U);
        HAL_NVIC_EnableIRQ(RTC_IRQn);
        // USER CODE END RTC_MspInit 1
        }
    }

void HAL_RTC_MspDeInit(
    RTC_HandleTypeDef *	hRtc
    )
    {
    if (hRtc->Instance == RTC)
        {
        // USER CODE BEGIN RTC_MspDeInit 0
        HAL_NVIC_DisableIRQ(RTC_IRQn);
        // USER CODE END RTC_MspDeInit 0
        // Peripheral clock disable
        __HAL_RCC_RTC_DISABLE();
        // USER CODE BEGIN RTC_MspDeInit 1

        // USER CODE END RTC_MspDeInit 1
        }
    }

uint32_t HAL_AddTick(
    uint32_t delta
    )
    {
    extern __IO uint32_t uwTick;
    // copy old interrupt-enable state to flags.
    uint32_t const flags = __get_PRIMASK();

    // disable interrupts
    __set_PRIMASK(1);

    // observe uwTick, and advance it.
    uint32_t const tickCount = uwTick + delta;

    // save uwTick
    uwTick = tickCount;

    // restore interrupts (does nothing if ints were disabled on entry)
    __set_PRIMASK(flags);

    // return the new value of uwTick.
    return tickCount;
    }

} /* extern "C" */

// RTC needs to be initialized before we calibrate the clock.
bool rtcbegin() {
    RTC_TimeTypeDef Time;
    RTC_DateTypeDef Date;
    uint32_t RtcClock;
    RTC_HandleTypeDef hRtc;

    memset(&hRtc, 0, sizeof(hRtc));

    hRtc.Instance = RTC;
    hRtc.Init.HourFormat = RTC_HOURFORMAT_24;
    RtcClock = __HAL_RCC_GET_RTC_SOURCE();
    if (RtcClock == RCC_RTCCLKSOURCE_LSI)
        {
        hRtc.Init.AsynchPrediv = 37 - 1; /* 37kHz / 37 = 1000Hz */
        hRtc.Init.SynchPrediv = 1000 - 1; /* 1000Hz / 1000 = 1Hz */
        }
    else if (RtcClock == RCC_RTCCLKSOURCE_LSE)
        {
        hRtc.Init.AsynchPrediv = 128 - 1; /* 32768Hz / 128 = 256Hz */
        hRtc.Init.SynchPrediv = 256 - 1; /* 256Hz / 256 = 1Hz */
        }
    else
        {
        /*
        || use HSE clock --
        || we don't support use of HSE as RTC because it's connected to
        || TCXO_OUT, and that's controlled by the LoRaWAN software.
        */
        Serial.println(
            " HSE can not be used for RTC clock!"
            );
        return false;
        }


    hRtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hRtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
    hRtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hRtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

    if (HAL_RTC_Init(&hRtc) != HAL_OK)
        {
        Serial.println(
            "HAL_RTC_Init() failed"
            );
        return false;
        }

    /* Initialize RTC and set the Time and Date */
    if (HAL_RTCEx_BKUPRead(&hRtc, RTC_BKP_DR0) != 0x32F2)
        {
        Time.Hours = 0x0;
        Time.Minutes = 0x0;
        Time.Seconds = 0x0;
        Time.SubSeconds = 0x0;
        Time.TimeFormat = RTC_HOURFORMAT12_AM;
        Time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        Time.StoreOperation = RTC_STOREOPERATION_RESET;

        if (HAL_RTC_SetTime(
            &hRtc,
            &Time,
            RTC_FORMAT_BIN
            ) != HAL_OK)
            {
            /*Serial.print(
                "HAL_RTC_SetTime() failed"
                );*/
            return false;
            }

        /* Sunday 1st January 2017 */
        Date.WeekDay = RTC_WEEKDAY_SUNDAY;
        Date.Month = RTC_MONTH_JANUARY;
        Date.Date = 0x1;
        Date.Year = 0x0;

        if (HAL_RTC_SetDate(
            &hRtc,
            &Date,
            RTC_FORMAT_BIN
            ) != HAL_OK)
            {
            /*Serial.print(
                "HAL_RTC_SetDate() failed"
                );*/
            return false;
            }

        HAL_RTCEx_BKUPWrite(&hRtc, RTC_BKP_DR0, 0x32F2);
        }

    /* Enable Direct Read of the calendar registers (not through Shadow) */
    HAL_RTCEx_EnableBypassShadow(&hRtc);

    HAL_RTC_DeactivateAlarm(&hRtc, RTC_ALARM_A);
    return true;
}

uint32_t setup_calibrateSystemClock(void) {
    uint32_t calibStartTime;

	calibStartTime = millis();
	calibValue = Stm32_CalibrateSystemClock();
	calibTime = millis() - calibStartTime;

    rtccount_begin();
	lptimCounter = rtccount_read();
}

uint32_t Stm32_CalibrateSystemClock(void)
    {
    uint32_t Calib;
    uint32_t CalibNew;
    uint32_t CalibLow;
    uint32_t CalibHigh;
    uint32_t mSecond;
    uint32_t mSecondNew;
    uint32_t mSecondLow;
    uint32_t mSecondHigh;
    bool fHaveSeenLow;
    bool fHaveSeenHigh;
    const bool fCalibrateMSI =  HAL_RCC_GetHCLKFreq() < 16000000;

    if (! rtcbegin()) {
        return 0;
    }

    if (fCalibrateMSI)
        {
        Calib = (RCC->ICSCR & RCC_ICSCR_MSITRIM) >> 24;
        }
    else
        {
        Calib = (RCC->ICSCR & RCC_ICSCR_HSITRIM) >> 8;
        }

    /* preapre to loop, setting suitable defaults */
    CalibNew = Calib;
    CalibLow = 0;
    CalibHigh = 0;
    mSecondLow = 0;
    mSecondHigh = 2000000;
    fHaveSeenLow = fHaveSeenHigh = false;

    /* loop until we have a new value */
    do  {
        /* meassure the # of millis per RTC second */
        mSecond = MeasureMicrosPerRtcSecond();

        /* invariant: */
        if (Calib == CalibNew)
            mSecondNew = mSecond;

        /* if mSecond is low, this meaans we must increase the system clock */
        if (mSecond <= 1000000)
            {
            // Serial.print('-');
            /*
            || the following condition establishes that we're
            || below the target frequency, but closer than we've been
            || before (mSecondLow is the previous "low" limit). If
            || so, we reduce the limit, and capture the "low" calibration
            || value.
            */
            if (mSecond > mSecondLow)
                {
                mSecondLow = mSecond;
                CalibLow = Calib; /* save previous calibration value */
                fHaveSeenLow = true;
                }

            /*
            || if we are low, and we have never exceeded the high limit,
            || we can  increase the clock.
            */
            if (! fHaveSeenHigh)
                {
                if (fCalibrateMSI)
                    {
                    if (Calib < 0xFF)
                        {
                        ++Calib;
                        __HAL_RCC_MSI_CALIBRATIONVALUE_ADJUST(Calib);
                        }
                    else
                        break;
                    }
                else
                    {
                    if (Calib < 0x1F)
                        {
                        ++Calib;
                        __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(Calib);
                        }
                    else
                        {
                        break;
                        }
                    }

                /* let the clock settle */
                delay(500);
                }
            }

        /* if mSecond is high, we must reduce the system clock */
        else
            {
            // Serial.print('+');
            /*
            || the following condition establishes that we're
            || above the target frequency, but closer than we've been
            || before (mSecondHigh is the previous "high" limit). If
            || so, we reduce the limit, and capture the calibration
            || value.
            */
            if (mSecond < mSecondHigh)
                {
                mSecondHigh = mSecond;
                CalibHigh = Calib;
                fHaveSeenHigh = true;
                }

            /*
            || if we are above the target frequency, and we have
            || never raised the frequence, we can lower the
            || frequency
            */
            if (! fHaveSeenLow)
                {
                if (Calib == 0)
                    break;

                --Calib;
                if (fCalibrateMSI)
                    {
                    __HAL_RCC_MSI_CALIBRATIONVALUE_ADJUST(Calib);
                    }
                else
                    {
                    __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(Calib);
                    }
                delay(500);
                }
            }
        } while ((Calib != CalibNew) &&
                (! fHaveSeenLow || !fHaveSeenHigh));

    //
    // We are going to take higher calibration value first and
    // it allows us not to call LMIC_setClockError().
    //
    if (fHaveSeenHigh)
        {
        mSecondNew = mSecondHigh;
        CalibNew = CalibHigh;
        }
    else if (fHaveSeenLow)
        {
        mSecondNew = mSecondLow;
        CalibNew = CalibLow;
        }
    else
        {
        // Use original value
        Serial.println(
            "?CalibrateSystemClock: can't calibrate"
            );
        }

    if (CalibNew != Calib)
        {
        // Serial.print(CalibNew < Calib ? '+' : '-');
        if (fCalibrateMSI)
            {
            __HAL_RCC_MSI_CALIBRATIONVALUE_ADJUST(CalibNew);
            }
        else
            {
            __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(CalibNew);
            }
        delay(500);
        }

    /* Serial.print(" 0x");
    Serial.println(CalibNew, HEX); */
    return CalibNew;
    }

uint32_t
MeasureMicrosPerRtcSecond(
    void
    )
    {
    uint32_t second;
    uint32_t now;
    uint32_t start;
    uint32_t end;

    /* get the starting time */
    second = RTC->TR & (RTC_TR_ST | RTC_TR_SU);

    /* wait for a new second to start, and capture millis() in start */
    do  {
        now = RTC->TR & (RTC_TR_ST | RTC_TR_SU);
        start = micros();
        } while (second == now);

    /* update our second of interest */
    second = now;

    /* no point in watching the register until we get close */
    delay(500);

    /* wait for the next second to start, and capture millis() */
    do  {
        now = RTC->TR & (RTC_TR_ST | RTC_TR_SU);
        end = micros();
        } while (second == now);

    /* return the delta */
    return end - start;
    }

static void rtccount_begin()
    {
    // enable clock to LPTIM1
    __HAL_RCC_LPTIM1_CLK_ENABLE();
    auto const pLptim = LPTIM1;

    // set LPTIM1 clock to LSE clock.
    __HAL_RCC_LPTIM1_CONFIG(RCC_LPTIM1CLKSOURCE_LSE);

    // disable everything so we can tweak the CFGR
    pLptim->CR = 0;

    // disable interrupts (needs to be done while disabled globally)
    pLptim->IER = 0;

    // upcount from selected internal clock (which is LSE)
    auto rCfg = pLptim->CFGR & ~0x01FEEEDF;
    rCfg |=  0;
    pLptim->CFGR = rCfg;

    // enable the counter but don't start it
    pLptim->CR = LPTIM_CR_ENABLE;
    delayMicroseconds(100);

    // set ARR to max value so we can count from 0 to 0xFFFF.
    // must be done after enabling.
    pLptim->ARR = 0xFFFF;

    // start in continuous mode.
    pLptim->CR = LPTIM_CR_ENABLE | LPTIM_CR_CNTSTRT;
    }

static uint16_t rtccount_read()
    {
    auto const pLptim = LPTIM1;
    uint32_t v1, v2;

    for (v1 = pLptim->CNT & 0xFFFF; (v2 = pLptim->CNT & 0xFFFF) != v1; v1 = v2)
        /* loop */;

    return (uint16_t) v1;
    }