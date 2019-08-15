#define MICROPY_HW_BOARD_NAME       "STM32F103ZET6"
#define MICROPY_HW_MCU_NAME         "STM32F103xE"


// HSE is 8MHz not use  
// Preset system clk = 72M , AHB clk = 72M , APB1 = 36M ,APB2 = 72M
// Can be modified by function pyb.freq()



#define MICROPY_HW_FLASH_FS_LABEL "pybflash"

#define MICROPY_HW_HAS_MMA7660      (0)
#define MICROPY_HW_HAS_LIS3DSH      (0)
#define MICROPY_HW_HAS_LCD          (0)
#define MICROPY_HW_HAS_SWITCH       (1)
#define MICROPY_HW_HAS_FLASH        (1)


#define MICROPY_HW_ENABLE_SDCARD         (1)  //if have sdio
#define MICROPY_HW_SDCARD_MOUNT_AT_BOOT  (1)  //auto mount sdcard
// SD card detect switch
// not use
#define MICROPY_HW_SDCARD_DETECT_PIN        (pin_A8)
#define MICROPY_HW_SDCARD_DETECT_PULL       (GPIO_PULLUP)
#define MICROPY_HW_SDCARD_DETECT_PRESENT    (GPIO_PIN_RESET)


#define MICROPY_HW_ENABLE_SERVO     (0)
#define MICROPY_HW_ENABLE_RNG       (0)
#define MICROPY_HW_ENABLE_RTC       (1)
#define MICROPY_HW_ENABLE_TIMER     (1)
#define MICROPY_HW_ENABLE_DAC       (1)
#define MICROPY_HW_ENABLE_USB       (1)

//#define MICROPY_HW_ENABLE_INTERNAL_FLASH_STORAGE (0)


// USB config
#define MICROPY_HW_USB_FS              (1)
//#define MICROPY_HW_USB_VBUS_DETECT_PIN (pin_A9)
//#define MICROPY_HW_USB_HS              (1)
//#define MICROPY_HW_USB_HS_IN_FS        (1)
//#define MICROPY_HW_USB_VBUS_DETECT_PIN (pin_B13)
//#define MICROPY_HW_USB_OTG_ID_PIN      (pin_B12)
//#define MICROPY_HW_SDCARD_MOUNT_AT_BOOT  (0)

// The pyboard has a 32kHz crystal for the RTC
#define MICROPY_HW_RTC_USE_LSE      (1)
#define MICROPY_HW_RTC_USE_US       (0)
#define MICROPY_HW_RTC_USE_CALOUT   (1)
// REPL
#define MICROPY_HW_UART_REPL        PYB_UART_1
#define MICROPY_HW_UART_REPL_BAUD   115200
// USART1
#define MICROPY_HW_UART1_TX     (pin_A9)   // PA9,PB6
#define MICROPY_HW_UART1_RX     (pin_A10)  // PA10,PB7
#define MICROPY_HW_UART1_RTS    (pin_A12)  // PA12
#define MICROPY_HW_UART1_CTS    (pin_A11)  // PA11
// USART2
#define MICROPY_HW_UART2_TX     (pin_A2)  // PA2
#define MICROPY_HW_UART2_RX     (pin_A3)  // PA3
#define MICROPY_HW_UART2_RTS    (pin_A1)  // PA1
#define MICROPY_HW_UART2_CTS    (pin_A0)  // PA0
// USART3
#define MICROPY_HW_UART3_TX     (pin_B10) // PB10
#define MICROPY_HW_UART3_RX     (pin_B11) // PB11
#define MICROPY_HW_UART3_RTS    (pin_B14) // PB14
#define MICROPY_HW_UART3_CTS    (pin_B13) // PB13
// USART4
#define MICROPY_HW_UART4_TX     (pin_C10) // PC10
#define MICROPY_HW_UART4_RX     (pin_C11) // PC11
// USART5
#define MICROPY_HW_UART5_TX     (pin_C12) // PC12
#define MICROPY_HW_UART5_RX     (pin_D2 ) // PD2
// I2C busses
#define MICROPY_HW_I2C1_SCL (pin_B6)  // PB8,PB6
#define MICROPY_HW_I2C1_SDA (pin_B7)  // PB9,PB7
#define MICROPY_HW_I2C2_SCL (pin_B10) // PB10
#define MICROPY_HW_I2C2_SDA (pin_B11) // PB11
// SPI1
#define MICROPY_HW_SPI1_NSS  (pin_A4)  // PA4,PA15
#define MICROPY_HW_SPI1_SCK  (pin_A5)  // PA5,PB3
#define MICROPY_HW_SPI1_MISO (pin_A6)  // PA6,PB4
#define MICROPY_HW_SPI1_MOSI (pin_A7)  // PA7,PB5
// SPI2
#define MICROPY_HW_SPI2_NSS  (pin_B12) // PB12
#define MICROPY_HW_SPI2_SCK  (pin_B13) // PB13
#define MICROPY_HW_SPI2_MISO (pin_B14) // PB14
#define MICROPY_HW_SPI2_MOSI (pin_B15) // PB15
// SPI3
#define MICROPY_HW_SPI3_NSS  (pin_A15) // PA15
#define MICROPY_HW_SPI3_SCK  (pin_B3 ) // PB3
#define MICROPY_HW_SPI3_MISO (pin_B4 ) // PB4
#define MICROPY_HW_SPI3_MOSI (pin_B5 ) // PB5
// CAN1
#define MICROPY_HW_CAN1_TX   (pin_A12)
#define MICROPY_HW_CAN1_RX   (pin_A11)
// LEDs
#define MICROPY_HW_LED1             (pin_E5) // red
#define MICROPY_HW_LED2             (pin_B5) // green
#define MICROPY_HW_LED_ON(pin)      (mp_hal_pin_low(pin))
#define MICROPY_HW_LED_OFF(pin)     (mp_hal_pin_high(pin))
// User Switch
#define MICROPY_HW_USRSW_PIN        (pin_E3)
#define MICROPY_HW_USRSW_PULL       (GPIO_PULLUP)
#define MICROPY_HW_USRSW_EXTI_MODE  (GPIO_MODE_IT_FALLING)
#define MICROPY_HW_USRSW_PRESSED    (0)





