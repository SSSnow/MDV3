#define MICROPY_HW_BOARD_NAME       "OPENMV3"
#define MICROPY_HW_MCU_NAME         "STM32F765"
#define MICROPY_PY_SYS_PLATFORM     "OpenMV3-M7"

#define MICROPY_HW_HAS_SWITCH       (0)
#define MICROPY_HW_HAS_SDCARD       (1)
#define MICROPY_HW_HAS_MMA7660      (0)
#define MICROPY_HW_HAS_LIS3DSH      (0)
#define MICROPY_HW_HAS_LCD          (0)
#define MICROPY_HW_ENABLE_RNG       (1)
#define MICROPY_HW_ENABLE_RTC       (0)
#define MICROPY_HW_ENABLE_TIMER     (1)
#define MICROPY_HW_ENABLE_SERVO     (1)
#define MICROPY_HW_ENABLE_DAC       (1)
#define MICROPY_HW_ENABLE_SPI1      (0)
#define MICROPY_HW_ENABLE_SPI2      (1)
#define MICROPY_HW_ENABLE_SPI3      (0)
#define MICROPY_HW_ENABLE_SPI4      (0)
#define MICROPY_HW_ENABLE_CAN       (1)

#define MICROPY_HW_CLK_PLLM (12)
#define MICROPY_HW_CLK_PLLN (432)
#define MICROPY_HW_CLK_PLLQ (9)
#define MICROPY_HW_CLK_PLLR (2)
#define MICROPY_HW_CLK_PLLP (RCC_PLLP_DIV2)

// UART config
#define MICROPY_HW_UART7_PORT (GPIOE)
#define MICROPY_HW_UART7_PINS (GPIO_PIN_7 | GPIO_PIN_8)

/* yuanjun : UART2 PD5(TX) PD6(RX) */
#define MICROPY_HW_UART2_PORT (GPIOD)
#define MICROPY_HW_UART2_PINS (GPIO_PIN_5 | GPIO_PIN_6)

/* yuanjun : UART3 PD8(TX) PD9(RX) */
#define MICROPY_HW_UART3_PORT (GPIOD)
#define MICROPY_HW_UART3_PINS (GPIO_PIN_8 | GPIO_PIN_9)

/* yuanjun : console */
// #define MICROPY_HW_UART_REPL        PYB_UART_7
#define MICROPY_HW_UART_REPL_BAUD   115200

// I2C buses
#define MICROPY_HW_I2C2_SCL (pin_B10)
#define MICROPY_HW_I2C2_SDA (pin_B11)

// SPI buses
#define MICROPY_HW_SPI2_NSS  (pin_B12)
#define MICROPY_HW_SPI2_SCK  (pin_B13)
#define MICROPY_HW_SPI2_MISO (pin_B14)
#define MICROPY_HW_SPI2_MOSI (pin_B15)

// CAN busses
#define MICROPY_HW_CAN2_NAME "CAN2" // CAN2 on RX,TX = Y5,Y6 = PB12,PB13

// SD card detect switch
#define MICROPY_HW_SDCARD_DETECT_PIN        (pin_A15)
#define MICROPY_HW_SDCARD_DETECT_PULL       (GPIO_PULLUP)
#define MICROPY_HW_SDCARD_DETECT_PRESENT    (GPIO_PIN_RESET)

// USB config
#define MICROPY_HW_USB_VBUS_DETECT_PIN (pin_A9)

// USRSW is pulled low. Pressing the button makes the input go high.
//#define MICROPY_HW_USRSW_PIN        (pin_A5)
//#define MICROPY_HW_USRSW_PULL       (GPIO_NOPULL)
//#define MICROPY_HW_USRSW_EXTI_MODE  (GPIO_MODE_IT_RISING)
//#define MICROPY_HW_USRSW_PRESSED    (1)

// LEDs
#define MICROPY_HW_LED1             (pin_A8)  // green (main)
#define MICROPY_HW_LED2             (pin_D11) // blue  (main)
#define MICROPY_HW_LED3             (pin_D14) // green (lcd)
#define MICROPY_HW_LED4             (pin_D12) // blue  (lcd)
#define MICROPY_HW_LED5             (pin_D13) // red   (lcd)
#define MICROPY_HW_LED_OTYPE        (GPIO_MODE_OUTPUT_PP)
#define MICROPY_HW_LED_ON(pin)      (pin->gpio->BSRR = (pin->pin_mask << 16))
#define MICROPY_HW_LED_OFF(pin)     (pin->gpio->BSRR = pin->pin_mask)
