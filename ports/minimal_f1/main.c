#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys.h>
#include "py/compile.h"
#include "py/runtime.h"
#include "py/repl.h"
#include "py/gc.h"
#include "py/mperrno.h"
#include "lib/utils/pyexec.h"

#if MICROPY_ENABLE_COMPILER
void do_str(const char *src, mp_parse_input_kind_t input_kind) {
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_lexer_t *lex = mp_lexer_new_from_str_len(MP_QSTR__lt_stdin_gt_, src, strlen(src), 0);
        qstr source_name = lex->source_name;
        mp_parse_tree_t parse_tree = mp_parse(lex, input_kind);
        mp_obj_t module_fun = mp_compile(&parse_tree, source_name, MP_EMIT_OPT_NONE, true);
        mp_call_function_0(module_fun);
        nlr_pop();
    } else {
        // uncaught exception
        mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
    }
}
#endif

static char *stack_top;
#if MICROPY_ENABLE_GC
static char heap[2048];
#endif

#define LED0 PBout(5)	// DS0
#define LED1 PEout(5)	// DS1	
int main(int argc, char **argv) {
	Stm32_Clock_Init(9);
	
	RCC->APB2ENR|=1<<3;
	RCC->APB2ENR|=1<<6;
	   	 
	GPIOB->CRL&=0XFF0FFFFF;
	GPIOB->CRL|=0X00300000; 
	GPIOB->ODR|=1<<5;
	GPIOE->CRL&=0XFF0FFFFF;
	GPIOE->CRL|=0X00300000;
	GPIOE->ODR|=1<<5;      
	LED0 = 1;
	LED1 = 0;
	
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)( 72 *1000000)/( 115200 *16);//得到USARTDIV
	mantissa=temp;				 //得到整數部分
	fraction=(temp-mantissa)*16; //得到小數部分	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口時鐘  
	RCC->APB2ENR|=1<<14;  //使能串口時鐘 
	GPIOA->CRH&=0XFFFFF00F;//IO狀態設置
	GPIOA->CRH|=0X000008B0;//IO狀態設置 
	RCC->APB2RSTR|=1<<14;   //復位串口1
	RCC->APB2RSTR&=~(1<<14);//停止復位	   	   
	//波特率設置
 	USART1->BRR=mantissa; // 波特率設置	 
	USART1->CR1|=0X200C;  //1位停止,無校驗位.

    int stack_dummy;
    stack_top = (char*)&stack_dummy;

    #if MICROPY_ENABLE_GC
    gc_init(heap, heap + sizeof(heap));
    #endif
    mp_init();
    #if MICROPY_ENABLE_COMPILER
    #if MICROPY_REPL_EVENT_DRIVEN
    pyexec_event_repl_init();
    for (;;) {
        int c = mp_hal_stdin_rx_chr();
        if (pyexec_event_repl_process_char(c)) {
            break;
        }
    }
    #else
    pyexec_friendly_repl();
    #endif
    //do_str("print('hello world!', list(x+1 for x in range(10)), end='eol\\n')", MP_PARSE_SINGLE_INPUT);
    //do_str("for i in range(10):\r\n  print(i)", MP_PARSE_FILE_INPUT);
    #else
    pyexec_frozen_module("frozentest.py");
    #endif
    mp_deinit();
    return 0;
}

void gc_collect(void) {
    // WARNING: This gc_collect implementation doesn't try to get root
    // pointers from CPU registers, and thus may function incorrectly.
    void *dummy;
    gc_collect_start();
    gc_collect_root(&dummy, ((mp_uint_t)stack_top - (mp_uint_t)&dummy) / sizeof(mp_uint_t));
    gc_collect_end();
    gc_dump_info();
}

mp_lexer_t *mp_lexer_new_from_file(const char *filename) {
    mp_raise_OSError(MP_ENOENT);
}

mp_import_stat_t mp_import_stat(const char *path) {
    return MP_IMPORT_STAT_NO_EXIST;
}

mp_obj_t mp_builtin_open(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(mp_builtin_open_obj, 1, mp_builtin_open);

void nlr_jump_fail(void *val) {
    while (1);
}

void NORETURN __fatal_error(const char *msg) {
    while (1);
}

#ifndef NDEBUG
void MP_WEAK __assert_func(const char *file, int line, const char *func, const char *expr) {
    printf("Assertion '%s' failed, at file %s:%d\n", expr, file, line);
    __fatal_error("Assertion failed");
}
#endif

#if MICROPY_MIN_USE_STM32_MCU

// simple GPIO interface
#define GPIO_MODE_IN (0)
#define GPIO_MODE_OUT (1)
#define GPIO_MODE_ALT (2)
#define GPIO_PULL_NONE (1)
#define GPIO_PULL_UP   (2)
#define GPIO_PULL_DOWN (2)
void gpio_init(GPIO_TypeDef *gpio, int pin, int mode, int pull, int alt) {
	if ( mode == 0)
	{
		if ( pin <= 7 )
		{
			gpio->CRL = (gpio->CRL & ~(3 << (4 * pin))) | (mode << (4 * pin));
			gpio->CRL = (gpio->CRL & ~(3 << (4 * pin + 2))) | (pull << (4 * pin + 2)); //set pull
		}
		else
		{
			pin -= 8;
			gpio->CRH = (gpio->CRH & ~(3 << (4 * pin))) | (mode << (4 * pin));
			gpio->CRH = (gpio->CRH & ~(3 << (4 * pin + 2)))     | (pull << (4 * pin + 2));   //set pull
		}
		
	}
	else
	{
		if (mode == GPIO_MODE_OUT) mode = 0;
		if ( pin <= 7 )
		{
			gpio->CRL = (gpio->CRL & ~(3 << (4 * pin + 2))) | (mode << (4 * pin + 2));
			gpio->CRL = (gpio->CRL & ~(3 << (4 * pin)))     | (3 << (4 * pin));   //set as high speed
		}
		else
		{
			pin -= 8;
			gpio->CRH = (gpio->CRH & ~(3 << (4 * pin + 2))) | (mode << (4 * pin + 2));
			gpio->CRH = (gpio->CRH & ~(3 << (4 * pin)))     | (3 << (4 * pin));   //set as high speed
		}
	}
}
#define gpio_get(gpio, pin) ((gpio->IDR >> (pin)) & 1)
#define gpio_set(gpio, pin, value) do { gpio->ODR = (gpio->ODR & ~(1 << (pin))) | (value << pin); } while (0)
#define gpio_low(gpio, pin) do { gpio->BSRR = (1 << (pin)); } while (0)
#define gpio_high(gpio, pin) do { gpio->BSRR = (1 << (pin + 16)); } while (0)


#endif
