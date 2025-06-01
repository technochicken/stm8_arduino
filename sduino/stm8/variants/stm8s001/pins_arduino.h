#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#ifndef _BV
#define _BV(X) (1<<(X))
#endif

// STM8S001J3: SO8N package, 5 usable I/Os (pins 1, 5, 6, 7) + PA1 (pin 1) excluded for SWIM (PD1)
#define NUM_DIGITAL_PINS            7
#define NUM_ANALOG_INPUTS           3
#define analogInputToDigitalPin(p)  ((p >= 0 && p < NUM_ANALOG_INPUTS) ? (p + 4) : -1)
#define analogPinToChannel(p)       ((p >= 0 && p < NUM_ANALOG_INPUTS) ? (p + 4) : NO_ANALOG)

// Logical pin mapping
enum portpin {
  PD5, // 0 - Pin 1: AIN5, UART1_TX, TIM2_CH2
  PD3, // 1 - Pin 1: AIN4, TIM2_CH2, ADC_ETR
  PC6, // 2 - Pin 1: SPI_MOSI, TIM1_CH1
  PC5, // 3 - Pin 5: SPI_SCK, TIM2_CH1
  PC4, // 4 - Pin 6: AIN2, TIM1_CH4
  PB4, // 5 - Pin 6: I2C_SCL, ADC_ETR
  PB5  // 6 - Pin 7: I2C_SDA, TIM1_BKIN
};

#define digitalPinHasPWM(p) ((p)==0 || (p)==1 || (p)==2 || (p)==3 || (p)==4 || (p)==6)

#define PIN_LED_BUILTIN (0) // use PD5 for LED
#define LED_BUILTIN     (PIN_LED_BUILTIN)

#define PIN_TX          (0)  // PD5 UART1_TX
#define PIN_RX          (-1) // UART1_RX not accessible (PA1 not bonded out)

#define PIN_SPI_SCK     (3)  // PC5
#define PIN_SPI_MOSI    (2)  // PC6
#define PIN_SPI_MISO    (-1) // Not available
#define PIN_SPI_SS      (-1) // Not available

#define SS              PIN_SPI_SS
#define MOSI            PIN_SPI_MOSI
#define MISO            PIN_SPI_MISO
#define SCK             PIN_SPI_SCK

#define SDA             PB5
#define SCL             PB4
#define NO_ANALOG       0xff

#ifdef ARDUINO_MAIN

const uint16_t PROGMEM port_to_mode_PGM[] = {
  NOT_A_PORT,
  GPIOA_BaseAddress + 2,
  GPIOB_BaseAddress + 2,
  GPIOC_BaseAddress + 2,
  GPIOD_BaseAddress + 2
};

const uint16_t PROGMEM port_to_output_PGM[] = {
  NOT_A_PORT,
  GPIOA_BaseAddress,
  GPIOB_BaseAddress,
  GPIOC_BaseAddress,
  GPIOD_BaseAddress
};

const uint16_t PROGMEM port_to_input_PGM[] = {
  NOT_A_PORT,
  GPIOA_BaseAddress + 1,
  GPIOB_BaseAddress + 1,
  GPIOC_BaseAddress + 1,
  GPIOD_BaseAddress + 1
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
  PD, // 0 - PD5
  PD, // 1 - PD3
  PC, // 2 - PC6
  PC, // 3 - PC5
  PC, // 4 - PC4
  PB, // 5 - PB4
  PB  // 6 - PB5
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
  _BV(5), // PD5
  _BV(3), // PD3
  _BV(6), // PC6
  _BV(5), // PC5
  _BV(4), // PC4
  _BV(4), // PB4
  _BV(5)  // PB5
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
  NOT_ON_TIMER,  // PD5
  NOT_ON_TIMER,  // PD3
  NOT_ON_TIMER,  // PC6
  NOT_ON_TIMER,  // PC5
  NOT_ON_TIMER,  // PC4
  NOT_ON_TIMER,  // PB4
  NOT_ON_TIMER   // PB5
};

#endif

#define SERIAL_PORT_MONITOR   Serial
#define SERIAL_PORT_HARDWARE  Serial

#endif
