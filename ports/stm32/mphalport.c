#include <string.h>

#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "extmod/misc.h"
#include "usb.h"
#include "uart.h"

// this table converts from HAL_StatusTypeDef to POSIX errno
const byte mp_hal_status_to_errno_table[4] = {
    [HAL_OK] = 0,
    [HAL_ERROR] = MP_EIO,
    [HAL_BUSY] = MP_EBUSY,
    [HAL_TIMEOUT] = MP_ETIMEDOUT,
};

NORETURN void mp_hal_raise(HAL_StatusTypeDef status) {
    mp_raise_OSError(mp_hal_status_to_errno_table[status]);
}

MP_WEAK int mp_hal_stdin_rx_chr(void) {
    for (;;) {
#if 0
#ifdef USE_HOST_MODE
        pyb_usb_host_process();
        int c = pyb_usb_host_get_keyboard();
        if (c != 0) {
            return c;
        }
#endif
#endif
        if (MP_STATE_PORT(pyb_stdio_uart) != NULL && uart_rx_any(MP_STATE_PORT(pyb_stdio_uart))) {
            return uart_rx_char(MP_STATE_PORT(pyb_stdio_uart));
        }
        int dupterm_c = mp_uos_dupterm_rx_chr();
        if (dupterm_c >= 0) {
            return dupterm_c;
        }
        MICROPY_EVENT_POLL_HOOK
    }
}

void mp_hal_stdout_tx_str(const char *str) {
    mp_hal_stdout_tx_strn(str, strlen(str));
}

MP_WEAK void mp_hal_stdout_tx_strn(const char *str, size_t len) {
    if (MP_STATE_PORT(pyb_stdio_uart) != NULL) {
        uart_tx_strn(MP_STATE_PORT(pyb_stdio_uart), str, len);
    }
#if 0 && defined(USE_HOST_MODE) && MICROPY_HW_HAS_LCD
    lcd_print_strn(str, len);
#endif
    mp_uos_dupterm_tx_strn(str, len);
}

// Efficiently convert "\n" to "\r\n"
void mp_hal_stdout_tx_strn_cooked(const char *str, size_t len) {
    const char *last = str;
    while (len--) {
        if (*str == '\n') {
            if (str > last) {
                mp_hal_stdout_tx_strn(last, str - last);
            }
            mp_hal_stdout_tx_strn("\r\n", 2);
            ++str;
            last = str;
        } else {
            ++str;
        }
    }
    if (str > last) {
        mp_hal_stdout_tx_strn(last, str - last);
    }
}

#if __CORTEX_M >= 0x03
void mp_hal_ticks_cpu_enable(void) {
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        #if defined(__CORTEX_M) && __CORTEX_M == 7
        // on Cortex-M7 we must unlock the DWT before writing to its registers
        DWT->LAR = 0xc5acce55;
        #endif
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}
#endif

void mp_hal_gpio_clock_enable(GPIO_TypeDef *gpio) {
    #if defined(STM32L476xx) || defined(STM32L496xx)
    if (gpio == GPIOG) {
        // Port G pins 2 thru 15 are powered using VddIO2 on these MCUs.
        HAL_PWREx_EnableVddIO2();
    }
    #endif

    // This logic assumes that all the GPIOx_EN bits are adjacent and ordered in one register

    #if defined(STM32F0)
    #define AHBxENR AHBENR
    #define AHBxENR_GPIOAEN_Pos RCC_AHBENR_GPIOAEN_Pos
	/*--------------------------------*/
	/*----Add to support stm32f1------*/
	/*--------------------------------*/
	#elif defined(STM32F1)
    #define AHBxENR APB2ENR  //stm32f1系列gpio的clk在apb2上
    #define AHBxENR_GPIOAEN_Pos RCC_APB2ENR_IOPAEN_Pos
    #elif defined(STM32F4) || defined(STM32F7)
    #define AHBxENR AHB1ENR
    #define AHBxENR_GPIOAEN_Pos RCC_AHB1ENR_GPIOAEN_Pos
    #elif defined(STM32H7)
    #define AHBxENR AHB4ENR
    #define AHBxENR_GPIOAEN_Pos RCC_AHB4ENR_GPIOAEN_Pos
    #elif defined(STM32L4)
    #define AHBxENR AHB2ENR
    #define AHBxENR_GPIOAEN_Pos RCC_AHB2ENR_GPIOAEN_Pos
    #endif

    uint32_t gpio_idx = ((uint32_t)gpio - GPIOA_BASE) / (GPIOB_BASE - GPIOA_BASE);  
	//計算這是gpio多少
    RCC->AHBxENR |= 1 << (AHBxENR_GPIOAEN_Pos + gpio_idx);
    volatile uint32_t tmp = RCC->AHBxENR; // Delay after enabling clock
    (void)tmp;
}

/*--------------------------------*/
/*----Add to support stm32f1------*/
/*--------------------------------*/
#if defined(STM32F1)

#include <stdio.h>

#include "py/obj.h"
#include "py/mphal.h"
#include "pin.h"

void mp_hal_pin_config(mp_hal_pin_obj_t pin_obj, uint32_t mode, uint32_t pull, const pin_af_obj_t *af) {
    GPIO_TypeDef *gpio = pin_obj->gpio;  //gpio的暫存器結構體
    uint32_t pin = pin_obj->pin; //0-15
    mp_hal_gpio_clock_enable(gpio);
	
	//mode 選擇
	//MP_HAL_PIN_MODE_INPUT
	//MP_HAL_PIN_MODE_OUTPUT
	//MP_HAL_PIN_MODE_ALT
	//MP_HAL_PIN_MODE_ANALOG
	//MP_HAL_PIN_MODE_ADC
	//MP_HAL_PIN_MODE_OPEN_DRAIN
	//MP_HAL_PIN_MODE_ALT_OPEN_DRAIN
	//pull選擇
	//MP_HAL_PIN_PULL_NONE
	//MP_HAL_PIN_PULL_UP
	//MP_HAL_PIN_PULL_DOWN
	
	//stm32f1 mode
	//GPIO_MODE_INPUT    
	//GPIO_MODE_OUTPUT_PP
	//GPIO_MODE_OUTPUT_OD
	//GPIO_MODE_AF_PP    
	//GPIO_MODE_AF_OD    
	//GPIO_MODE_AF_INPUT  = GPIO_MODE_INPUT
	//GPIO_MODE_ANALOG
	
	//alt設置 詳看手冊110頁
	
	/*
	
					mode				pull
	// TIM2/3/4/5
	TIM ETR			GPIO_MODE_INPUT		GPIO_NOPULL
	TIM CHx			GPIO_MODE_AF_PP		NO_USE
	TIM CHx_ETR		GPIO_MODE_INPUT		GPIO_NOPULL
	// TIM1/8
	TIM CHxN		GPIO_MODE_AF_PP		NO_USE
	TIM BKIN		GPIO_MODE_INPUT		GPIO_NOPULL
	
	USART RTS		GPIO_MODE_AF_PP		NO_USE
	USART CTS		GPIO_MODE_INPUT		GPIO_NOPULL OR GPIO_PULLUP
	USART CK		GPIO_MODE_AF_PP		NO_USE
	USART TX		GPIO_MODE_AF_PP		NO_USE
	USART RX		GPIO_MODE_INPUT		GPIO_NOPULL OR GPIO_PULLUP
	
	SPI SCK			
	主模式			GPIO_MODE_AF_PP		NO_USE
	從模式			GPIO_MODE_INPUT		GPIO_NOPULL
	SPI MOSI		
	主模式			GPIO_MODE_AF_PP		NO_USE
	從模式			GPIO_MODE_INPUT		GPIO_NOPULL OR GPIO_PULLUP
	SPI MISO		
	主模式			GPIO_MODE_INPUT		GPIO_NOPULL OR GPIO_PULLUP
	從模式			GPIO_MODE_AF_PP		NO_USE
	SPI NSS			
	硬件主/從模式	GPIO_MODE_INPUT		GPIO_NOPULL OR GPIO_PULLUP OR GPIO_PULLDOWM
	硬件主模式/
	NSS輸出使能		GPIO_MODE_AF_PP		NO_USE
	
	I2C SCL			GPIO_MODE_AF_OD		NO_USE
	I2C SDA			GPIO_MODE_AF_OD		NO_USE
	
	CAN TX			GPIO_MODE_AF_PP		NO_USE
	CAN RX			GPIO_MODE_INPUT		GPIO_NOPULL OR GPIO_PULLUP
	*/
	/*
	printf("\nsetting\n");
	printf("\nmode = %ld\n",mode);
	printf("\npull = %ld\n",pull);
	*/
	
	if(		\
		( af->fn == AF_FN_TIM && af->type == AF_PIN_TYPE_TIM_ETR )		|| \
		( af->fn == AF_FN_TIM && af->type == AF_PIN_TYPE_TIM_CH1_ETR )		|| \
		/*( af->fn == AF_FN_TIM && af->type == AF_PIN_TYPE_TIM_CH2_ETR )	|| \*/
		/*( af->fn == AF_FN_TIM && af->type == AF_PIN_TYPE_TIM_CH3_ETR )	|| \*/
		/*( af->fn == AF_FN_TIM && af->type == AF_PIN_TYPE_TIM_CH4_ETR )	|| \*/
		( af->fn == AF_FN_UART  && af->type == AF_PIN_TYPE_UART_CTS )		|| \
		( af->fn == AF_FN_UART  && af->type == AF_PIN_TYPE_UART_RX )		\
		)
	{
		mode = GPIO_MODE_INPUT;
	}
	
	else if(	 \
		( af->fn == AF_FN_TIM && af->type == AF_PIN_TYPE_TIM_CH1 )			|| \
		( af->fn == AF_FN_TIM && af->type == AF_PIN_TYPE_TIM_CH1N )			|| \
		( af->fn == AF_FN_TIM && af->type == AF_PIN_TYPE_TIM_CH2 )			|| \
		( af->fn == AF_FN_TIM && af->type == AF_PIN_TYPE_TIM_CH2N )			|| \
		( af->fn == AF_FN_TIM && af->type == AF_PIN_TYPE_TIM_CH3 )			|| \
		( af->fn == AF_FN_TIM && af->type == AF_PIN_TYPE_TIM_CH3N )			|| \
		( af->fn == AF_FN_TIM && af->type == AF_PIN_TYPE_TIM_CH4 )			|| \
		/*( af->fn == AF_FN_TIM && af->type == AF_PIN_TYPE_TIM_CH4N )		|| \*/
		( af->fn == AF_FN_UART  && af->type == AF_PIN_TYPE_UART_RTS )		|| \
		( af->fn == AF_FN_USART && af->type == AF_PIN_TYPE_USART_CK )		|| \
		( af->fn == AF_FN_UART  && af->type == AF_PIN_TYPE_UART_TX )		\
		)
	{
		mode = GPIO_MODE_AF_PP;
	}
	else if(		\
		( af->fn == AF_FN_I2C && af->type == AF_PIN_TYPE_I2C_SCL )		|| \
		( af->fn == AF_FN_I2C && af->type == AF_PIN_TYPE_I2C_SDA )		\
		)
	{
		mode = GPIO_MODE_AF_OD;
	}
	else if(	\
			mode == MP_HAL_PIN_MODE_INPUT	||
			mode == MP_HAL_PIN_MODE_ANALOG	||
			mode == MP_HAL_PIN_MODE_ADC      )
	{
		mode = GPIO_MODE_INPUT;
	}
	else if(	mode == MP_HAL_PIN_MODE_OUTPUT )
	{
		mode = GPIO_MODE_OUTPUT_PP;
	}
	else if(	mode == MP_HAL_PIN_MODE_ALT )
	{
		mode = GPIO_MODE_AF_PP;
	}
	else if(	mode == MP_HAL_PIN_MODE_OPEN_DRAIN )
	{
		mode = GPIO_MODE_OUTPUT_OD;
	}
	else if(	mode == MP_HAL_PIN_MODE_ALT_OPEN_DRAIN )
	{
		mode = GPIO_MODE_AF_OD;
	}
	
	GPIO_InitTypeDef GPIO_Initure;
	GPIO_Initure.Pin	=	(1 << pin) ;  //GPIO_PIN_0 = 0x00 // GPIO_PIN_1 = 0x01
	GPIO_Initure.Mode	=	mode; 
	GPIO_Initure.Pull	=	pull;
	GPIO_Initure.Speed	=	GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(gpio,&GPIO_Initure); 
		
}
#else
void mp_hal_pin_config(mp_hal_pin_obj_t pin_obj, uint32_t mode, uint32_t pull, uint32_t alt) {
    GPIO_TypeDef *gpio = pin_obj->gpio;  //gpio的暫存器結構體
    uint32_t pin = pin_obj->pin; //0-15
    mp_hal_gpio_clock_enable(gpio);
	gpio->MODER = (gpio->MODER & ~(3 << (2 * pin))) | ((mode & 3) << (2 * pin));
	#if defined(GPIO_ASCR_ASC0)
		// The L4 has a special analog switch to connect the GPIO to the ADC
		gpio->OTYPER = (gpio->OTYPER & ~(1 << pin)) | (((mode >> 2) & 1) << pin);
		gpio->ASCR = (gpio->ASCR & ~(1 << pin)) | ((mode >> 3) & 1) << pin;
	#else
		gpio->OTYPER = (gpio->OTYPER & ~(1 << pin)) | ((mode >> 2) << pin);
	#endif
	gpio->OSPEEDR = (gpio->OSPEEDR & ~(3 << (2 * pin))) | (2 << (2 * pin)); // full speed
	gpio->PUPDR = (gpio->PUPDR & ~(3 << (2 * pin))) | (pull << (2 * pin));
	gpio->AFR[pin >> 3] = (gpio->AFR[pin >> 3] & ~(15 << (4 * (pin & 7)))) | (alt << (4 * (pin & 7)));
}
#endif



bool mp_hal_pin_config_alt(mp_hal_pin_obj_t pin, uint32_t mode, uint32_t pull, uint8_t fn, uint8_t unit) {
    const pin_af_obj_t *af = pin_find_af(pin, fn, unit);
	//pin_find_af at the pin_named_pins.c
    if (af == NULL) {
        return false;
    }
	/*--------------------------------*/
	/*----Add to support stm32f1------*/
	/*--------------------------------*/
	#if defined(STM32F1)
    mp_hal_pin_config(pin, mode, pull, af);
	#else
	mp_hal_pin_config(pin, mode, pull, af->idx);
	#endif
    return true;
}

void mp_hal_pin_config_speed(mp_hal_pin_obj_t pin_obj, uint32_t speed) {
    GPIO_TypeDef *gpio = pin_obj->gpio;
    uint32_t pin = pin_obj->pin;
	/*--------------------------------*/
	/*----Add to support stm32f1------*/
	/*--------------------------------*/
	#if defined(STM32F1)
    if(pin >= 7)	//0-7配置CRL
	{
		gpio->CRL = (  gpio->CRL & ~(3 << (pin * 4))  )  | (speed << (4 * pin));
	}
	else			//8-15配置CRH
	{
		pin -= 7;
		gpio->CRH = (  gpio->CRH & ~(3 << (pin * 4))  )  | (speed << (4 * pin));
	}
	#else
	gpio->OSPEEDR = (gpio->OSPEEDR & ~(3 << (2 * pin))) | (speed << (2 * pin));
	#endif
}

MP_WEAK void mp_hal_get_mac(int idx, uint8_t buf[6]) {
    // Generate a random locally administered MAC address (LAA)
	//設定在mpconfigboard_common.h
    uint8_t *id = (uint8_t *)MP_HAL_UNIQUE_ID_ADDRESS;
    buf[0] = 0x02; // LAA range
    buf[1] = (id[11] << 4) | (id[10] & 0xf);
    buf[2] = (id[9] << 4) | (id[8] & 0xf);
    buf[3] = (id[7] << 4) | (id[6] & 0xf);
    buf[4] = id[2];
    buf[5] = (id[0] << 2) | idx;
}
