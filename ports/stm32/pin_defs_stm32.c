#include "py/obj.h"
#include "pin.h"

// Returns the pin mode. This value returned by this macro should be one of:
// GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_OUTPUT_OD,
// GPIO_MODE_AF_PP, GPIO_MODE_AF_OD, or GPIO_MODE_ANALOG.

uint32_t pin_get_mode(const pin_obj_t *pin) {
	/*--------------------------------*/
	/*----Add to support stm32f1------*/
	/*--------------------------------*/
	#if defined(STM32F1)
	GPIO_TypeDef *gpio = pin->gpio;
	uint32_t pin_name = pin->pin;
	uint32_t mode;
	if ( pin_name <= 7 )
	{
		mode = (gpio->CRL >> (pin_name * 4) ) & 3 ;
	}
	else 
	{
		mode = (gpio->CRH >> (pin_name * 4) ) & 3 ;
	}
    return mode;
	#else
    GPIO_TypeDef *gpio = pin->gpio;
    uint32_t mode = (gpio->MODER >> (pin->pin * 2)) & 3;
    if (mode != GPIO_MODE_ANALOG) {
        if (gpio->OTYPER & pin->pin_mask) {
            mode |= 1 << 4;
        }
    }
    return mode;
	#endif
}

// Returns the pin pullup/pulldown. The value returned by this macro should
// be one of GPIO_NOPULL, GPIO_PULLUP, or GPIO_PULLDOWN.

uint32_t pin_get_pull(const pin_obj_t *pin) {
	/*--------------------------------*/
	/*----Add to support stm32f1------*/
	/*--------------------------------*/
	#if defined(STM32F1)   
	//stm32f1只有輸入時控制上拉下拉的
	//設置 詳看手冊106頁
	return GPIO_PULLUP;
	#else
    return (pin->gpio->PUPDR >> (pin->pin * 2)) & 3;
	#endif
}

// Returns the af (alternate function) index currently set for a pin.

uint32_t pin_get_af(const pin_obj_t *pin) {
	/*--------------------------------*/
	/*----Add to support stm32f1------*/
	/*--------------------------------*/
	#if defined(STM32F1)
	//設置 詳看手冊110頁
	return 0;
	#else
    return (pin->gpio->AFR[pin->pin >> 3] >> ((pin->pin & 7) * 4)) & 0xf;
	#endif
}

