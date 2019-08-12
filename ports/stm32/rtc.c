/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>

#include "py/runtime.h"
#include "extint.h"
#include "rtc.h"
#include "irq.h"

/// \moduleref pyb
/// \class RTC - real time clock
///
/// The RTC is and independent clock that keeps track of the date
/// and time.
///
/// Example usage:
///
///     rtc = pyb.RTC()
///     rtc.datetime((2014, 5, 1, 4, 13, 0, 0, 0))
///     print(rtc.datetime())

RTC_HandleTypeDef RTCHandle;

/*--------------------------------*/
/*----Add to support stm32f1------*/
/*--------------------------------*/
#if defined(STM32F1)
_calendar_obj calendar;//時鐘結構體 

u8 Is_Leap_Year(u16 year)
{			  
	if(year%4==0) //必須能被4整除
	{ 
		if(year%100==0) 
		{ 
			if(year%400==0)return 1;//如果以00結尾,還要能被400整除 	   
			else return 0;   
		}else return 1;   
	}else return 0;	
}
//月份數據表											 
u8 const table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //月修正數據表	  
//平年的月份日期表
const u8 mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};
//獲得現在是星期幾
//功能描述:輸入公歷日期得到星期(只允許1901-2099年)
//year,month,day：公歷年月日 
//返回值：星期號																						 
u8 RTC_Get_Week(u16 year,u8 month,u8 day)
{	
	u16 temp2;
	u8 yearH,yearL;
	
	yearH=year/100;	yearL=year%100; 
	// 如果為21世紀,年份數加100  
	if (yearH>19)yearL+=100;
	// 所過閏年數只算1900年之後的  
	temp2=yearL+yearL/4;
	temp2=temp2%7; 
	temp2=temp2+day+table_week[month-1];
	if (yearL%4==0&&month<3)temp2--;
	return(temp2%7);
}

u8 RTC_Get(void)
{
	static u16 daycnt=0;
	u32 timecount=0; 
	u32 temp=0;
	u16 temp1=0;	  
 	timecount=RTC->CNTH;//得到計數器中的值(秒鐘數)
	timecount<<=16;
	timecount+=RTC->CNTL;			 

 	temp=timecount/86400;   //得到天數(秒鐘數對應的)
	if(daycnt!=temp)//超過一天了
	{	  
		daycnt=temp;
		temp1=1970;	//從1970年開始
		while(temp>=365)
		{				 
			if(Is_Leap_Year(temp1))//是閏年
			{
				if(temp>=366)temp-=366;//閏年的秒鐘數
				else break;  
			}
			else temp-=365;	  //平年 
			temp1++;  
		}   
		calendar.w_year=temp1;//得到年份
		temp1=0;
		while(temp>=28)//超過了一個月
		{
			if(Is_Leap_Year(calendar.w_year)&&temp1==1)//當年是不是閏年/2月份
			{
				if(temp>=29)temp-=29;//閏年的秒鐘數
				else break; 
			}
			else 
			{
				if(temp>=mon_table[temp1])temp-=mon_table[temp1];//平年
				else break;
			}
			temp1++;  
		}
		calendar.w_month=temp1+1;	//得到月份
		calendar.w_date=temp+1;  	//得到日期 
	}
	temp=timecount%86400;     		//得到秒鐘數   	   
	calendar.hour=temp/3600;     	//小時
	calendar.min=(temp%3600)/60; 	//分鐘	
	calendar.sec=(temp%3600)%60; 	//秒鐘
	calendar.week=RTC_Get_Week(calendar.w_year,calendar.w_month,calendar.w_date);//獲取星期   
	return 0;
}	 


u8 RTC_Set(u16 syear,u8 smon,u8 sday,u8 hour,u8 min,u8 sec)
{
	u16 t;
	u32 seccount=0;
	
	if(syear<1970||syear>2099)return 1;	   
	for(t=1970;t<syear;t++)	//把所有年份的秒鐘相加
	{
		if(Is_Leap_Year(t))seccount+=31622400;//閏年的秒鐘數
		else seccount+=31536000;			  //平年的秒鐘數
	}
	smon-=1;
	for(t=0;t<smon;t++)	   //把前面月份的秒鐘數相加
	{
		seccount+=(u32)mon_table[t]*86400;//月份秒鐘數相加
		if(Is_Leap_Year(syear)&&t==1)seccount+=86400;//閏年2月份增加一天的秒鐘數	   
	}
	seccount+=(u32)(sday-1)*86400;//把前面日期的秒鐘數相加 
	seccount+=(u32)hour*3600;//小時秒鐘數
    seccount+=(u32)min*60;	 //分鐘秒鐘數
	seccount+=sec;//最後的秒鐘加上去
	//設置時鐘
    RCC->APB1ENR|=1<<28;//使能電源時鐘
    RCC->APB1ENR|=1<<27;//使能備份時鐘
	PWR->CR|=1<<8;    //取消備份區寫保護
	//上面三步是必須的!
	RTC->CRL|=1<<4;   //允許配置 
	RTC->CNTL=seccount&0xffff;
	RTC->CNTH=seccount>>16;
	RTC->CRL&=~(1<<4);//配置更新
	while(!(RTC->CRL&(1<<5)));//等待RTC寄存器操作完成 
	RTC_Get();
	return 0;	    
}
#endif

// rtc_info indicates various things about RTC startup
// it's a bit of a hack at the moment
static mp_uint_t rtc_info;

// Note: LSI is around (32KHz), these dividers should work either way
// ck_spre(1Hz) = RTCCLK(LSE) /(uwAsynchPrediv + 1)*(uwSynchPrediv + 1)
// modify RTC_ASYNCH_PREDIV & RTC_SYNCH_PREDIV in board/<BN>/mpconfigport.h to change sub-second ticks
// default is 3906.25 us, min is ~30.52 us (will increase Ivbat by ~500nA)
#ifndef RTC_ASYNCH_PREDIV
#define RTC_ASYNCH_PREDIV (0x7f) //32767
#endif
#ifndef RTC_SYNCH_PREDIV
#define RTC_SYNCH_PREDIV  (0x00ff)
#endif

STATIC HAL_StatusTypeDef PYB_RTC_Init(RTC_HandleTypeDef *hrtc);
STATIC void PYB_RTC_MspInit_Kick(RTC_HandleTypeDef *hrtc, bool rtc_use_lse, bool rtc_use_byp);
STATIC HAL_StatusTypeDef PYB_RTC_MspInit_Finalise(RTC_HandleTypeDef *hrtc);
STATIC void RTC_CalendarConfig(void);

#if MICROPY_HW_RTC_USE_LSE || MICROPY_HW_RTC_USE_BYPASS
STATIC bool rtc_use_lse = true;
#else
STATIC bool rtc_use_lse = false;
#endif
STATIC uint32_t rtc_startup_tick;
STATIC bool rtc_need_init_finalise = false;

void rtc_init_start(bool force_init) {
    RTCHandle.Instance = RTC;
	/*--------------------------------*/
	/*----Add to support stm32f1------*/
	/*--------------------------------*/
	#if defined(STM32F1)
	RTCHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV; 	//時鐘週期設置(有待觀察,看是否跑慢了?)理論值：32767	
	RTCHandle.Init.AsynchPrediv = 32767;
	
	rtc_need_init_finalise = false;
	
	if (!force_init) {
        uint32_t bdcr = RCC->BDCR;
        if ((bdcr & (RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL | RCC_BDCR_LSEON | RCC_BDCR_LSERDY))
            == (RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_0 | RCC_BDCR_LSEON | RCC_BDCR_LSERDY)) {
            // LSE is enabled & ready --> no need to (re-)init RTC
            // remove Backup Domain write protection
            HAL_PWR_EnableBkUpAccess();
            // Clear source Reset Flag
            __HAL_RCC_CLEAR_RESET_FLAGS();
            // provide some status information
            rtc_info |= 0x40000 | (RCC->BDCR & 7) | (RCC->CSR & 3) << 8;
            return;
        } else if ((bdcr & (RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL))
            == (RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_1)) {
            // LSI configured as the RTC clock source --> no need to (re-)init RTC
            // remove Backup Domain write protection
            HAL_PWR_EnableBkUpAccess();
            // Clear source Reset Flag
            __HAL_RCC_CLEAR_RESET_FLAGS();
            // Turn the LSI on (it may need this even if the RTC is running)
            RCC->CSR |= RCC_CSR_LSION;
            // provide some status information
            rtc_info |= 0x80000 | (RCC->BDCR & 7) | (RCC->CSR & 3) << 8;
            return;
        }
    }

    rtc_startup_tick = HAL_GetTick();
    rtc_info = 0x3f000000 | (rtc_startup_tick & 0xffffff);
    PYB_RTC_MspInit_Kick(&RTCHandle, rtc_use_lse, MICROPY_HW_RTC_USE_BYPASS);
	
	#else
    /* Configure RTC prescaler and RTC data registers */
    /* RTC configured as follow:
      - Hour Format    = Format 24
      - Asynch Prediv  = Value according to source clock
      - Synch Prediv   = Value according to source clock
      - OutPut         = Output Disable
      - OutPutPolarity = High Polarity
      - OutPutType     = Open Drain */
    RTCHandle.Init.HourFormat = RTC_HOURFORMAT_24;
    RTCHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
    RTCHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
    RTCHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
    RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

    rtc_need_init_finalise = false;

    if (!force_init) {
        uint32_t bdcr = RCC->BDCR;
        if ((bdcr & (RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL | RCC_BDCR_LSEON | RCC_BDCR_LSERDY))
            == (RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_0 | RCC_BDCR_LSEON | RCC_BDCR_LSERDY)) {
            // LSE is enabled & ready --> no need to (re-)init RTC
            // remove Backup Domain write protection
            HAL_PWR_EnableBkUpAccess();
            // Clear source Reset Flag
            __HAL_RCC_CLEAR_RESET_FLAGS();
            // provide some status information
            rtc_info |= 0x40000 | (RCC->BDCR & 7) | (RCC->CSR & 3) << 8;
            return;
        } else if ((bdcr & (RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL))
            == (RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_1)) {
            // LSI configured as the RTC clock source --> no need to (re-)init RTC
            // remove Backup Domain write protection
            HAL_PWR_EnableBkUpAccess();
            // Clear source Reset Flag
            __HAL_RCC_CLEAR_RESET_FLAGS();
            // Turn the LSI on (it may need this even if the RTC is running)
            RCC->CSR |= RCC_CSR_LSION;
            // provide some status information
            rtc_info |= 0x80000 | (RCC->BDCR & 7) | (RCC->CSR & 3) << 8;
            return;
        }
    }

    rtc_startup_tick = HAL_GetTick();
    rtc_info = 0x3f000000 | (rtc_startup_tick & 0xffffff);
    PYB_RTC_MspInit_Kick(&RTCHandle, rtc_use_lse, MICROPY_HW_RTC_USE_BYPASS);
	#endif
}

void rtc_init_finalise() {
    if (!rtc_need_init_finalise) {
        return;
    }

    rtc_info = 0;
    while (PYB_RTC_Init(&RTCHandle) != HAL_OK) {
        if (rtc_use_lse) {
            #if MICROPY_HW_RTC_USE_BYPASS
            if (RCC->BDCR & RCC_BDCR_LSEBYP) {
                // LSEBYP failed, fallback to LSE non-bypass
                rtc_info |= 0x02000000;
            } else
            #endif
            {
                // LSE failed, fallback to LSI
                rtc_use_lse = false;
                rtc_info |= 0x01000000;
            }
            rtc_startup_tick = HAL_GetTick();
            PYB_RTC_MspInit_Kick(&RTCHandle, rtc_use_lse, false);
            HAL_PWR_EnableBkUpAccess();
            RTCHandle.State = HAL_RTC_STATE_RESET;
        } else {
            // init error
            rtc_info |= 0xffff; // indicate error
            return;
        }
    }

    // RTC started successfully
    rtc_info = 0x20000000;

    // record if LSE or LSI is used
    rtc_info |= (rtc_use_lse << 28);

    // record how long it took for the RTC to start up
    rtc_info |= (HAL_GetTick() - rtc_startup_tick) & 0xffff;

    // fresh reset; configure RTC Calendar
    RTC_CalendarConfig();
    #if defined(STM32L4)
    if(__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET) {
    #else
    if(__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET) {
    #endif
        // power on reset occurred
        rtc_info |= 0x10000;
    }
    if(__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET) {
        // external reset occurred
        rtc_info |= 0x20000;
    }
    // Clear source Reset Flag
    __HAL_RCC_CLEAR_RESET_FLAGS();
    rtc_need_init_finalise = false;
}

STATIC HAL_StatusTypeDef PYB_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct) {
    /*------------------------------ LSI Configuration -------------------------*/
    if ((RCC_OscInitStruct->OscillatorType & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI) {
        // Check the LSI State
        if (RCC_OscInitStruct->LSIState != RCC_LSI_OFF) {
            // Enable the Internal Low Speed oscillator (LSI).
            __HAL_RCC_LSI_ENABLE();
        } else {
            // Disable the Internal Low Speed oscillator (LSI).
            __HAL_RCC_LSI_DISABLE();
        }
    }

    /*------------------------------ LSE Configuration -------------------------*/
    if ((RCC_OscInitStruct->OscillatorType & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE) {
        #if !defined(STM32H7)
        // Enable Power Clock
        __HAL_RCC_PWR_CLK_ENABLE();
        #endif

        // Enable access to the backup domain
        HAL_PWR_EnableBkUpAccess();
        uint32_t tickstart = HAL_GetTick();

        #if defined(STM32F7) || defined(STM32L4) || defined(STM32H7)
        //__HAL_RCC_PWR_CLK_ENABLE();
        // Enable write access to Backup domain
        //PWR->CR1 |= PWR_CR1_DBP;
        // Wait for Backup domain Write protection disable
        while ((PWR->CR1 & PWR_CR1_DBP) == RESET) {
            if (HAL_GetTick() - tickstart > RCC_DBP_TIMEOUT_VALUE) {
                return HAL_TIMEOUT;
            }
        }
        #else
        // Enable write access to Backup domain
        //PWR->CR |= PWR_CR_DBP;
        // Wait for Backup domain Write protection disable
		//__HAL_RCC_BKP_CLK_ENABLE();		//使能BSP時鐘
        while ((PWR->CR & PWR_CR_DBP) == RESET) {
            if (HAL_GetTick() - tickstart > RCC_DBP_TIMEOUT_VALUE) {
                return HAL_TIMEOUT;
            }
        }
        #endif

        #if MICROPY_HW_RTC_USE_BYPASS
        // If LSEBYP is enabled and new state is non-bypass then disable LSEBYP
        if (RCC_OscInitStruct->LSEState == RCC_LSE_ON && (RCC->BDCR & RCC_BDCR_LSEBYP)) {
            CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON);
            while (RCC->BDCR & RCC_BDCR_LSERDY) {
            }
            CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP);
        }
        #endif

        // Set the new LSE configuration
        __HAL_RCC_LSE_CONFIG(RCC_OscInitStruct->LSEState);
    }


    return HAL_OK;
}


STATIC HAL_StatusTypeDef PYB_RTC_Init(RTC_HandleTypeDef *hrtc) {
	/*--------------------------------*/
	/*----Add to support stm32f1------*/
	/*--------------------------------*/
	#if defined(STM32F1)
	// Check the RTC peripheral state
    if (hrtc == NULL) {
        return HAL_ERROR;
    }
	
	if (hrtc->State == HAL_RTC_STATE_RESET) {
        // Allocate lock resource and initialize it
        hrtc->Lock = HAL_UNLOCKED;
        // Initialize RTC MSP
        if (PYB_RTC_MspInit_Finalise(hrtc) != HAL_OK) {
            return HAL_ERROR;
        }
    }
	
	if( HAL_RTC_Init(hrtc) != HAL_OK)
			return HAL_ERROR;
		else
			return HAL_OK;

	#else
    // Check the RTC peripheral state
    if (hrtc == NULL) {
        return HAL_ERROR;
    }
    if (hrtc->State == HAL_RTC_STATE_RESET) {
        // Allocate lock resource and initialize it
        hrtc->Lock = HAL_UNLOCKED;
        // Initialize RTC MSP
        if (PYB_RTC_MspInit_Finalise(hrtc) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    // Set RTC state
    hrtc->State = HAL_RTC_STATE_BUSY;

    // Disable the write protection for RTC registers
    __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

    // Set Initialization mode
    if (RTC_EnterInitMode(hrtc) != HAL_OK) {
        // Enable the write protection for RTC registers
        __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

        // Set RTC state
        hrtc->State = HAL_RTC_STATE_ERROR;

        return HAL_ERROR;
    } else {
        // Clear RTC_CR FMT, OSEL and POL Bits
        hrtc->Instance->CR &= ((uint32_t)~(RTC_CR_FMT | RTC_CR_OSEL | RTC_CR_POL));
        // Set RTC_CR register
        hrtc->Instance->CR |= (uint32_t)(hrtc->Init.HourFormat | hrtc->Init.OutPut | hrtc->Init.OutPutPolarity);

        // Configure the RTC PRER
        hrtc->Instance->PRER = (uint32_t)(hrtc->Init.SynchPrediv);
        hrtc->Instance->PRER |= (uint32_t)(hrtc->Init.AsynchPrediv << 16);

        // Exit Initialization mode
        hrtc->Instance->ISR &= (uint32_t)~RTC_ISR_INIT;

        #if defined(STM32L4) || defined(STM32H7)
        hrtc->Instance->OR &= (uint32_t)~RTC_OR_ALARMOUTTYPE;
        hrtc->Instance->OR |= (uint32_t)(hrtc->Init.OutPutType);
        #elif defined(STM32F7)
        hrtc->Instance->OR &= (uint32_t)~RTC_OR_ALARMTYPE;
        hrtc->Instance->OR |= (uint32_t)(hrtc->Init.OutPutType);
        #else
        hrtc->Instance->TAFCR &= (uint32_t)~RTC_TAFCR_ALARMOUTTYPE;
        hrtc->Instance->TAFCR |= (uint32_t)(hrtc->Init.OutPutType);
        #endif

        // Enable the write protection for RTC registers
        __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

        // Set RTC state
        hrtc->State = HAL_RTC_STATE_READY;

        return HAL_OK;
    }
	#endif
}

STATIC void PYB_RTC_MspInit_Kick(RTC_HandleTypeDef *hrtc, bool rtc_use_lse, bool rtc_use_byp) {
    /* To change the source clock of the RTC feature (LSE, LSI), You have to:
       - Enable the power clock using __PWR_CLK_ENABLE()
       - Enable write access using HAL_PWR_EnableBkUpAccess() function before to
         configure the RTC clock source (to be done once after reset).
       - Reset the Back up Domain using __HAL_RCC_BACKUPRESET_FORCE() and
         __HAL_RCC_BACKUPRESET_RELEASE().
       - Configure the needed RTc clock source */

    // RTC clock source uses LSE (external crystal) only if relevant
    // configuration variable is set.  Otherwise it uses LSI (internal osc).

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    #if MICROPY_HW_RTC_USE_BYPASS
    if (rtc_use_byp) {
        RCC_OscInitStruct.LSEState = RCC_LSE_BYPASS;
        RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
    } else
    #endif
    if (rtc_use_lse) {
        RCC_OscInitStruct.LSEState = RCC_LSE_ON;
        RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
    } else {
        RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
        RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    }
    PYB_RCC_OscConfig(&RCC_OscInitStruct);

    // now ramp up osc. in background and flag calendear init needed
    rtc_need_init_finalise = true;
}

#ifndef MICROPY_HW_RTC_LSE_TIMEOUT_MS
#define MICROPY_HW_RTC_LSE_TIMEOUT_MS 1000  // ST docs spec 2000 ms LSE startup, seems to be too pessimistic
#endif
#ifndef MICROPY_HW_RTC_LSI_TIMEOUT_MS
#define MICROPY_HW_RTC_LSI_TIMEOUT_MS 500   // this is way too pessimistic, typ. < 1ms
#endif
#ifndef MICROPY_HW_RTC_BYP_TIMEOUT_MS
#define MICROPY_HW_RTC_BYP_TIMEOUT_MS 150
#endif

STATIC HAL_StatusTypeDef PYB_RTC_MspInit_Finalise(RTC_HandleTypeDef *hrtc) {
    // we already had a kick so now wait for the corresponding ready state...
    if (rtc_use_lse) {
        // we now have to wait for LSE ready or timeout
        uint32_t timeout = MICROPY_HW_RTC_LSE_TIMEOUT_MS;
        #if MICROPY_HW_RTC_USE_BYPASS
        if (RCC->BDCR & RCC_BDCR_LSEBYP) {
            timeout = MICROPY_HW_RTC_BYP_TIMEOUT_MS;
        }
        #endif
        uint32_t tickstart = rtc_startup_tick;
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET) {
            if ((HAL_GetTick() - tickstart ) > timeout) {
                return HAL_TIMEOUT;
            }
        }
    } else {
        // we now have to wait for LSI ready or timeout
        uint32_t tickstart = rtc_startup_tick;
        while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET) {
            if ((HAL_GetTick() - tickstart ) > MICROPY_HW_RTC_LSI_TIMEOUT_MS) {
                return HAL_TIMEOUT;
            }
        }
    }

    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    if (rtc_use_lse) {
        PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    } else {
        PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    }
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        //Error_Handler();
        return HAL_ERROR;
    }

    // enable RTC peripheral clock
    __HAL_RCC_RTC_ENABLE();
    return HAL_OK;
}


STATIC void RTC_CalendarConfig(void) {
	/*--------------------------------*/
	/*----Add to support stm32f1------*/
	/*--------------------------------*/
	#if defined(STM32F1)
	RTC_Set(2019,8,12,5,10,0);
	

	#else
    // set the date to 1st Jan 2015
    RTC_DateTypeDef date;
    date.Year = 15;
    date.Month = 1;
    date.Date = 1;
    date.WeekDay = RTC_WEEKDAY_THURSDAY;

    if(HAL_RTC_SetDate(&RTCHandle, &date, RTC_FORMAT_BIN) != HAL_OK) {
        // init error
        return;
    }

    // set the time to 00:00:00
    RTC_TimeTypeDef time;
    time.Hours = 0;
    time.Minutes = 0;
    time.Seconds = 0;
	#if !defined(STM32F1)
    time.TimeFormat = RTC_HOURFORMAT12_AM;
    time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    time.StoreOperation = RTC_STOREOPERATION_RESET;
	#endif
	
    if (HAL_RTC_SetTime(&RTCHandle, &time, RTC_FORMAT_BIN) != HAL_OK) {
        // init error
        return;
    }
	#endif
}

/******************************************************************************/
// MicroPython bindings

typedef struct _pyb_rtc_obj_t {
    mp_obj_base_t base;
} pyb_rtc_obj_t;

STATIC const pyb_rtc_obj_t pyb_rtc_obj = {{&pyb_rtc_type}};

/// \classmethod \constructor()
/// Create an RTC object.
STATIC mp_obj_t pyb_rtc_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 0, 0, false);

    // return constant object
    return MP_OBJ_FROM_PTR(&pyb_rtc_obj);
}

// force rtc to re-initialise
mp_obj_t pyb_rtc_init(mp_obj_t self_in) {
    rtc_init_start(true);
    rtc_init_finalise();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(pyb_rtc_init_obj, pyb_rtc_init);

/// \method info()
/// Get information about the startup time and reset source.
///
///  - The lower 0xffff are the number of milliseconds the RTC took to
///    start up.
///  - Bit 0x10000 is set if a power-on reset occurred.
///  - Bit 0x20000 is set if an external reset occurred
mp_obj_t pyb_rtc_info(mp_obj_t self_in) {
    return mp_obj_new_int(rtc_info);
}
MP_DEFINE_CONST_FUN_OBJ_1(pyb_rtc_info_obj, pyb_rtc_info);

/// \method datetime([datetimetuple])
/// Get or set the date and time of the RTC.
///
/// With no arguments, this method returns an 8-tuple with the current
/// date and time.  With 1 argument (being an 8-tuple) it sets the date
/// and time.
///
/// The 8-tuple has the following format:
///
///     (year, month, day, weekday, hours, minutes, seconds, subseconds)
///
/// `weekday` is 1-7 for Monday through Sunday.
///
/// `subseconds` counts down from 255 to 0

#define MEG_DIV_64 (1000000 / 64)
#define MEG_DIV_SCALE ((RTC_SYNCH_PREDIV + 1) / 64)

#if defined(MICROPY_HW_RTC_USE_US) && MICROPY_HW_RTC_USE_US
uint32_t rtc_subsec_to_us(uint32_t ss) {
    return ((RTC_SYNCH_PREDIV - ss) * MEG_DIV_64) / MEG_DIV_SCALE;
}

uint32_t rtc_us_to_subsec(uint32_t us) {
    return RTC_SYNCH_PREDIV - (us * MEG_DIV_SCALE / MEG_DIV_64);
}
#else
#define rtc_us_to_subsec
#define rtc_subsec_to_us
#endif


mp_obj_t pyb_rtc_datetime(size_t n_args, const mp_obj_t *args) {
    rtc_init_finalise();
	/*--------------------------------*/
	/*----Add to support stm32f1------*/
	/*--------------------------------*/
	#if defined(STM32F1)
	if (n_args == 1) {
        // get date and time
        // note: need to call get time then get date to correctly access the registers
        RTC_Get();
        mp_obj_t tuple[8] = {
            mp_obj_new_int(calendar.w_year),
            mp_obj_new_int(calendar.w_month),
            mp_obj_new_int(calendar.w_date),
            mp_obj_new_int(calendar.week),
            mp_obj_new_int(calendar.hour),
            mp_obj_new_int(calendar.min),
            mp_obj_new_int(calendar.sec),
			#if !defined(STM32F1)
            mp_obj_new_int(rtc_subsec_to_us(time.SubSeconds)),
			#else
			mp_obj_new_int(rtc_subsec_to_us(0)),
			#endif
        };
        return mp_obj_new_tuple(8, tuple);
    } else {
        // set date and time
        mp_obj_t *items;

        mp_obj_get_array_fixed_n(args[1], 8, &items);
        calendar.w_year		= mp_obj_get_int(items[0]);
        calendar.w_month	= mp_obj_get_int(items[1]);
        calendar.w_date		= mp_obj_get_int(items[2]);
        //date.WeekDay = mp_obj_get_int(items[3]);
        calendar.hour		= mp_obj_get_int(items[4]);
        calendar.min		= mp_obj_get_int(items[5]);
        calendar.sec		= mp_obj_get_int(items[6]);
		
		//printf("%d %d %d %d %d %d\r\n",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
        RTC_Set(	calendar.w_year   ,\
					calendar.w_month  ,\
					calendar.w_date   ,\
					calendar.hour     ,\
					calendar.min      ,\
					calendar.sec);

        return mp_const_none;
    }
	#else
    if (n_args == 1) {
        // get date and time
        // note: need to call get time then get date to correctly access the registers
        RTC_DateTypeDef date;
        RTC_TimeTypeDef time;
        HAL_RTC_GetTime(&RTCHandle, &time, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&RTCHandle, &date, RTC_FORMAT_BIN);
        mp_obj_t tuple[8] = {
            mp_obj_new_int(2000 + date.Year),
            mp_obj_new_int(date.Month),
            mp_obj_new_int(date.Date),
            mp_obj_new_int(date.WeekDay),
            mp_obj_new_int(time.Hours),
            mp_obj_new_int(time.Minutes),
            mp_obj_new_int(time.Seconds),
            mp_obj_new_int(rtc_subsec_to_us(time.SubSeconds)),
        };
        return mp_obj_new_tuple(8, tuple);
    } else {
        // set date and time
        mp_obj_t *items;
        mp_obj_get_array_fixed_n(args[1], 8, &items);

        RTC_DateTypeDef date;
        date.Year = mp_obj_get_int(items[0]) - 2000;
		date.Year = 40;
        date.Month = mp_obj_get_int(items[1]);
        date.Date = mp_obj_get_int(items[2]);
        date.WeekDay = mp_obj_get_int(items[3]);
        HAL_RTC_SetDate(&RTCHandle, &date, RTC_FORMAT_BIN);

        RTC_TimeTypeDef time;
        time.Hours = mp_obj_get_int(items[4]);
        time.Minutes = mp_obj_get_int(items[5]);
        time.Seconds = mp_obj_get_int(items[6]);
		#if !defined(STM32F1)
        time.TimeFormat = RTC_HOURFORMAT12_AM;
        time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        time.StoreOperation = RTC_STOREOPERATION_SET;
		#endif
        HAL_RTC_SetTime(&RTCHandle, &time, RTC_FORMAT_BIN);

        return mp_const_none;
    }
	#endif
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_rtc_datetime_obj, 1, 2, pyb_rtc_datetime);

#if defined(STM32F0)
#define RTC_WKUP_IRQn RTC_IRQn
#endif

// wakeup(None)
// wakeup(ms, callback=None)
// wakeup(wucksel, wut, callback)
mp_obj_t pyb_rtc_wakeup(size_t n_args, const mp_obj_t *args) {
	
	/*--------------------------------*/
	/*----Add to support stm32f1------*/
	/*--------------------------------*/
	#if defined(STM32F1)
	
	// wut is wakeup counter start value, wucksel is clock source
    // counter is decremented at wucksel rate, and wakes the MCU when it gets to 0
    // wucksel=0b000 is RTC/16 (RTC runs at 32768Hz)
    // wucksel=0b001 is RTC/8
    // wucksel=0b010 is RTC/4
    // wucksel=0b011 is RTC/2
    // wucksel=0b100 is 1Hz clock
    // wucksel=0b110 is 1Hz clock with 0x10000 added to wut
    // so a 1 second wakeup could be wut=2047, wucksel=0b000, or wut=4095, wucksel=0b001, etc

    rtc_init_finalise();

    // disable wakeup IRQ while we configure it
    HAL_NVIC_DisableIRQ(RTC_IRQn);

    bool enable = false;
    mp_int_t wucksel;
    mp_int_t wut;
    mp_obj_t callback = mp_const_none;
    if (n_args <= 3) {
        if (args[1] == mp_const_none) {
            // disable wakeup
        } else {
            // time given in ms
            mp_int_t ms = mp_obj_get_int(args[1]);
            mp_int_t div = 2;
            wucksel = 3;
            while (div <= 16 && ms > 2000 * div) {
                div *= 2;
                wucksel -= 1;
            }
            if (div <= 16) {
                wut = 32768 / div * ms / 1000;
            } else {
                // use 1Hz clock
                wucksel = 4;
                wut = ms / 1000;
                if (wut > 0x10000) {
                    // wut too large for 16-bit register, try to offset by 0x10000
                    wucksel = 6;
                    wut -= 0x10000;
                    if (wut > 0x10000) {
                        // wut still too large
                        mp_raise_ValueError("wakeup value too large");
                    }
                }
            }
            // wut register should be 1 less than desired value, but guard against wut=0
            if (wut > 0) {
                wut -= 1;
            }
            enable = true;
        }
        if (n_args == 3) {
            callback = args[2];
        }
    } else {
        // config values given directly
        wucksel = mp_obj_get_int(args[1]);
        wut = mp_obj_get_int(args[2]);
        callback = args[3];
        enable = true;
    }

    // set the callback
	#if defined(STM32F1)
    MP_STATE_PORT(pyb_extint_callback)[RTC_IRQn] = callback;
	#else
	MP_STATE_PORT(pyb_extint_callback)[EXTI_RTC_WAKEUP] = callback;
	#endif

    // wait until WUTWF is set
    while(!(RTC->CRL&(1<<5)));//等待RTC寄存器操作完成 

    if (enable) {
        // program WUT
        //RTC->WUTR = wut;

        // set WUTIE to enable wakeup interrupts
        // set WUTE to enable wakeup
        // program WUCKSEL
        //RTC->CR = (RTC->CR & ~7) | (1 << 14) | (1 << 10) | (wucksel & 7);

        // enable register write protection
        HAL_PWR_DisableBkUpAccess();	//取消備份區域寫保護

        // enable external interrupts on line EXTI_RTC_WAKEUP
        SET_BIT(RTC->CRH, RTC_IT_SEC);
	
		// clear interrupt flags
		RTC->CRL = ~(RTC_FLAG_SEC);


        NVIC_SetPriority(RTC_IRQn, IRQ_PRI_RTC_WKUP);
        HAL_NVIC_EnableIRQ(RTC_IRQn);

        //printf("wut=%d wucksel=%d\n", wut, wucksel);
    } else {
        // clear WUTIE to disable interrupts
        //RTC->CR &= ~RTC_CR_WUTIE;

        // enable register write protection
        HAL_PWR_DisableBkUpAccess();	//取消備份區域寫保護

        // disable external interrupts on line EXTI_RTC_WAKEUP
       CLEAR_BIT(RTC->CRH, RTC_IT_SEC);
    }

    return mp_const_none;
	
	
	
	
	
	#else
	
    // wut is wakeup counter start value, wucksel is clock source
    // counter is decremented at wucksel rate, and wakes the MCU when it gets to 0
    // wucksel=0b000 is RTC/16 (RTC runs at 32768Hz)
    // wucksel=0b001 is RTC/8
    // wucksel=0b010 is RTC/4
    // wucksel=0b011 is RTC/2
    // wucksel=0b100 is 1Hz clock
    // wucksel=0b110 is 1Hz clock with 0x10000 added to wut
    // so a 1 second wakeup could be wut=2047, wucksel=0b000, or wut=4095, wucksel=0b001, etc

    rtc_init_finalise();

    // disable wakeup IRQ while we configure it
    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);

    bool enable = false;
    mp_int_t wucksel;
    mp_int_t wut;
    mp_obj_t callback = mp_const_none;
    if (n_args <= 3) {
        if (args[1] == mp_const_none) {
            // disable wakeup
        } else {
            // time given in ms
            mp_int_t ms = mp_obj_get_int(args[1]);
            mp_int_t div = 2;
            wucksel = 3;
            while (div <= 16 && ms > 2000 * div) {
                div *= 2;
                wucksel -= 1;
            }
            if (div <= 16) {
                wut = 32768 / div * ms / 1000;
            } else {
                // use 1Hz clock
                wucksel = 4;
                wut = ms / 1000;
                if (wut > 0x10000) {
                    // wut too large for 16-bit register, try to offset by 0x10000
                    wucksel = 6;
                    wut -= 0x10000;
                    if (wut > 0x10000) {
                        // wut still too large
                        mp_raise_ValueError("wakeup value too large");
                    }
                }
            }
            // wut register should be 1 less than desired value, but guard against wut=0
            if (wut > 0) {
                wut -= 1;
            }
            enable = true;
        }
        if (n_args == 3) {
            callback = args[2];
        }
    } else {
        // config values given directly
        wucksel = mp_obj_get_int(args[1]);
        wut = mp_obj_get_int(args[2]);
        callback = args[3];
        enable = true;
    }

    // set the callback
    MP_STATE_PORT(pyb_extint_callback)[EXTI_RTC_WAKEUP] = callback;

    // disable register write protection
    RTC->WPR = 0xca;
    RTC->WPR = 0x53;

    // clear WUTE
    RTC->CR &= ~RTC_CR_WUTE;

    // wait until WUTWF is set
    while (!(RTC->ISR & RTC_ISR_WUTWF)) {
    }

    if (enable) {
        // program WUT
        RTC->WUTR = wut;

        // set WUTIE to enable wakeup interrupts
        // set WUTE to enable wakeup
        // program WUCKSEL
        RTC->CR = (RTC->CR & ~7) | (1 << 14) | (1 << 10) | (wucksel & 7);

        // enable register write protection
        RTC->WPR = 0xff;

        // enable external interrupts on line EXTI_RTC_WAKEUP
        #if defined(STM32L4)
        EXTI->IMR1 |= 1 << EXTI_RTC_WAKEUP;
        EXTI->RTSR1 |= 1 << EXTI_RTC_WAKEUP;
        #elif defined(STM32H7)
        EXTI_D1->IMR1 |= 1 << EXTI_RTC_WAKEUP;
        EXTI->RTSR1 |= 1 << EXTI_RTC_WAKEUP;
        #else
        EXTI->IMR |= 1 << EXTI_RTC_WAKEUP;
        EXTI->RTSR |= 1 << EXTI_RTC_WAKEUP;
        #endif

        // clear interrupt flags
        RTC->ISR &= ~RTC_ISR_WUTF;
        #if defined(STM32L4)
        EXTI->PR1 = 1 << EXTI_RTC_WAKEUP;
        #elif defined(STM32H7)
        EXTI_D1->PR1 = 1 << EXTI_RTC_WAKEUP;
        #else
        EXTI->PR = 1 << EXTI_RTC_WAKEUP;
        #endif

        NVIC_SetPriority(RTC_WKUP_IRQn, IRQ_PRI_RTC_WKUP);
        HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

        //printf("wut=%d wucksel=%d\n", wut, wucksel);
    } else {
        // clear WUTIE to disable interrupts
        RTC->CR &= ~RTC_CR_WUTIE;

        // enable register write protection
        RTC->WPR = 0xff;

        // disable external interrupts on line EXTI_RTC_WAKEUP
        #if defined(STM32L4)
        EXTI->IMR1 &= ~(1 << EXTI_RTC_WAKEUP);
        #elif defined(STM32H7)
        EXTI_D1->IMR1 |= 1 << EXTI_RTC_WAKEUP;
        #else
        EXTI->IMR &= ~(1 << EXTI_RTC_WAKEUP);
        #endif
    }

    return mp_const_none;
	#endif
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_rtc_wakeup_obj, 2, 4, pyb_rtc_wakeup);

// calibration(None)
// calibration(cal)
// When an integer argument is provided, check that it falls in the range [-511 to 512]
// and set the calibration value; otherwise return calibration value
mp_obj_t pyb_rtc_calibration(size_t n_args, const mp_obj_t *args) {
	
	/*--------------------------------*/
	/*----Add to support stm32f1------*/
	/*--------------------------------*/
	#if defined(STM32F1)
	return 0;
	#else
	
    rtc_init_finalise();
    mp_int_t cal;
    if (n_args == 2) {
        cal = mp_obj_get_int(args[1]);
        mp_uint_t cal_p, cal_m;
        if (cal < -511 || cal > 512) {
#if defined(MICROPY_HW_RTC_USE_CALOUT) && MICROPY_HW_RTC_USE_CALOUT
            if ((cal & 0xfffe) == 0x0ffe) {
                // turn on/off X18 (PC13) 512Hz output
                // Note:
                //      Output will stay active even in VBAT mode (and inrease current)
                if (cal & 1) {
                    HAL_RTCEx_SetCalibrationOutPut(&RTCHandle, RTC_CALIBOUTPUT_512HZ);
                } else {
                    HAL_RTCEx_DeactivateCalibrationOutPut(&RTCHandle);
                }
                return mp_obj_new_int(cal & 1);
            } else {
                mp_raise_ValueError("calibration value out of range");
            }
#else
            mp_raise_ValueError("calibration value out of range");
#endif
        }
        if (cal > 0) {
            cal_p = RTC_SMOOTHCALIB_PLUSPULSES_SET;
            cal_m = 512 - cal;
        } else {
            cal_p = RTC_SMOOTHCALIB_PLUSPULSES_RESET;
            cal_m = -cal;
        }
        HAL_RTCEx_SetSmoothCalib(&RTCHandle, RTC_SMOOTHCALIB_PERIOD_32SEC, cal_p, cal_m);
        return mp_const_none;
    } else {
        // printf("CALR = 0x%x\n", (mp_uint_t) RTCHandle.Instance->CALR); // DEBUG
        // Test if CALP bit is set in CALR:
        if (RTCHandle.Instance->CALR & 0x8000) {
            cal = 512 - (RTCHandle.Instance->CALR & 0x1ff);
        } else {
            cal = -(RTCHandle.Instance->CALR & 0x1ff);
        }
        return mp_obj_new_int(cal);
    }
	#endif
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_rtc_calibration_obj, 1, 2, pyb_rtc_calibration);

STATIC const mp_rom_map_elem_t pyb_rtc_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&pyb_rtc_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&pyb_rtc_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_datetime), MP_ROM_PTR(&pyb_rtc_datetime_obj) },
    { MP_ROM_QSTR(MP_QSTR_wakeup), MP_ROM_PTR(&pyb_rtc_wakeup_obj) },
    { MP_ROM_QSTR(MP_QSTR_calibration), MP_ROM_PTR(&pyb_rtc_calibration_obj) },
};
STATIC MP_DEFINE_CONST_DICT(pyb_rtc_locals_dict, pyb_rtc_locals_dict_table);

const mp_obj_type_t pyb_rtc_type = {
    { &mp_type_type },
    .name = MP_QSTR_RTC,
    .make_new = pyb_rtc_make_new,
    .locals_dict = (mp_obj_dict_t*)&pyb_rtc_locals_dict,
};
