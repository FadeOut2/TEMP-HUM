#include "stm32f767xx.h"
#include "dht_data.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdlib.h>


//=== Clock: 16 MHz HSI (default) ===//

//=== I2C OLED SSD1306 Config ===//
#define SSD1306_I2C_ADDR   (0x3C << 1)
#define SSD1306_CMD        0x00
#define SSD1306_DATA       0x40
#define I2C1_TIMING        0x20303E5D  // 100kHz @ 16MHz

//=== App thresholds (Celsius) with hysteresis ===//
#define TEMP_HIGH_C        40   // show ALERT when >= 40C
#define TEMP_RECOVER_C     38   // clear ALERT when <= 38C

#define BUZZER_PC0 0  // Buzzer on PC0
#define GREEN_LED_PF3 3 // Green LED on PF3
#define RED_LED_PC3 3 // Red LED on PC3


// ==== Buzzer control (runtime polarity) ===//
// Control an active-low or active-high buzzer without preprocessor switches.
static volatile uint8_t g_buzzer_active_low = 1; // 1: LOW=ON, 0: HIGH=ON; adjust at runtime
static inline void buzzer_set_active_low(uint8_t yes){ g_buzzer_active_low = yes ? 1u : 0u; }
static inline void buzzer_assert(void){
  if(g_buzzer_active_low){
    GPIOC->BSRR = (1u << (BUZZER_PC0+16)); // drive LOW => ON
  } else {
    GPIOC->BSRR = (1u << BUZZER_PC0);     // drive HIGH => ON
  }
}
static inline void buzzer_deassert(void){
  if(g_buzzer_active_low){
    GPIOC->BSRR = (1u << BUZZER_PC0);     // drive HIGH => OFF
  } else {
    GPIOC->BSRR = (1u << (BUZZER_PC0+16)); // drive LOW => OFF
  }
}
// Backward-compatible aliases
#define BUZZER_ASSERT()   buzzer_assert()
#define BUZZER_DEASSERT() buzzer_deassert()
#define BUZZER_ON()       buzzer_assert()
#define BUZZER_OFF()      buzzer_deassert()
 
// ==== Macros for LEDS ====//
#define GREEN_LED_ON()   (GPIOF->BSRR = (1u << 3))   // PF3 HIGH
#define GREEN_LED_OFF()  (GPIOF->BSRR = (1u << (3+16))) // PF3 LOW
#define RED_LED_ON()   (GPIOC->BSRR = (1u << 3))   // PC3 HIGH
#define RED_LED_OFF()  (GPIOC->BSRR = (1u << (3+16))) // PC3 LOW
#define RED_LED_TOGGLE() (GPIOC->ODR ^= (1u << 3)) // toggle PC3

//=== Function Prototypes ===//
static inline void delay_us(uint32_t us);
static inline void delay_ms(uint32_t ms);
static void GPIO_Init(void);
static void I2C1_Init(void);
static void OLED_Init(void);
static void OLED_ClearScreen(void);
static void OLED_PrintCentered(uint8_t page, const char* str);
static void OLED_PrintChar(uint8_t page, uint8_t col, char c);
static void I2C1_Write2Bytes(uint8_t ctrl, uint8_t data);
static void I2C1_DataBurst(uint8_t ctrl, uint8_t *data, uint16_t size);
static int OLED_Cmd(uint8_t cmd);
static void OLED_SetPageCol(uint8_t page, uint8_t col);
static const uint8_t* glyph(char c);
static void OLED_ClearRow(uint8_t page);
static inline void clear_values(DHT_Data* d);
// Buzzer helpers
static void buzzer_tone(uint32_t freq_hz, uint32_t duration_ms);
static void buzzer_beeps(uint32_t freq_hz, uint32_t on_ms, uint32_t off_ms, uint32_t repeats);
static void buzzer_off(void);

//=== Write Functions ===//
static inline void delay_us(uint32_t us){
  uint32_t cycles = us * 16;  // 16MHz = 16 cycles per microsecond
  while(cycles--) __NOP();
}

static inline void delay_ms(uint32_t ms){
  for(uint32_t i=0; i<ms; i++) delay_us(1000);
}

// Reset DHT reading values to a known state
static inline void clear_values(DHT_Data* d){
  if(!d) return;
  d->temperature = 0;
  d->humidity = 0;
  d->checksum = 0;
}

//=== Main ===//
int main(int argc , char** argv){

  GPIO_Init();
  GPIO_Init();
  I2C1_Init();
  I2C1_Init();
  OLED_Init();

  OLED_ClearScreen();
  OLED_PrintCentered(0 , " DHT11 Ready");
  GREEN_LED_ON();

  DHT_Init();
  delay_ms(2000); // wait for dht11 to be stable

  OLED_ClearScreen();
  for(;;){
    DHT_Data dht;
    char buffer[32];
    int warning_active = 0; //  1 active, 0 not active
    const uint16_t temp_HIGH = 33; // high temp threshold (just for testing)
  uint8_t status = DHT_Read(&dht);

    // Only accept fully valid readings (status == 1). Treat checksum error as failure to avoid garbage.
    if(status != 1){
      OLED_PrintCentered(1, "DHT11 Error");
  
      delay_ms(2000);
      continue;
    }
    while(1){ // will break in case of heating
      snprintf(buffer, sizeof(buffer), "Temp: %dC", dht.temperature);
      OLED_ClearRow(2);
      OLED_PrintCentered(2, buffer);

      snprintf(buffer, sizeof(buffer), "Humidity: %d%%", dht.humidity);
      OLED_ClearRow(4);
      OLED_PrintCentered(4, buffer);
 
      

      if(dht.temperature >= temp_HIGH && warning_active == 0){
        warning_active = 1;
        OLED_ClearRow(6);
        OLED_PrintCentered(6, "!!! OVERHEAT !!!");
        buzzer_beeps(2000, 180, 180, 5); // 2kHz beeps
        OLED_ClearScreen();
      }
      
      // Faster refresh when overheating; slower otherwise
      // Use a shorter delay right after the screen clears/reappears during overheat
      uint32_t next_delay_ms = (dht.temperature >= temp_HIGH) ? 300u : 2000u;
      delay_ms(next_delay_ms);
      clear_values(&dht);
      break;
    }

  }
  return EXIT_SUCCESS;
}

//=== GPIO Init ===//
static void GPIO_Init(void){
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
  
  // PB8: I2C1 SCL (AF4, Open-drain)
  GPIOB->MODER   &= ~(3U << (8*2));
  GPIOB->MODER   |=  (2U << (8*2));    // Alternate function
  GPIOB->OTYPER  |=  (1U << 8);        // Open-drain
  GPIOB->OSPEEDR |=  (3U << (8*2));    // High speed
  GPIOB->PUPDR   &= ~(3U << (8*2));    // No pull
  GPIOB->AFR[1]  &= ~(0xF << 0);
  GPIOB->AFR[1]  |=  (4U << 0);        // AF4
  
  // PB9: I2C1 SDA (AF4, Open-drain)
  GPIOB->MODER   &= ~(3U << (9*2));
  GPIOB->MODER   |=  (2U << (9*2));
  GPIOB->OTYPER  |=  (1U << 9);
  GPIOB->OSPEEDR |=  (3U << (9*2));
  GPIOB->PUPDR   &= ~(3U << (9*2));
  GPIOB->AFR[1]  &= ~(0xF << 4);
  GPIOB->AFR[1]  |=  (4U << 4);

  // GREEN PF3 : Output
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    GPIOF->MODER &= ~(3u << (3*2)); // Clear mode bits
    GPIOF->MODER |=  (1u << (3*2));   // Output mode
    GPIOF->OTYPER &= ~(1u << 3);      // Push-pull
    GPIOF->OSPEEDR |= (3u << (3*2));  // High speed
    GPIOF->PUPDR &= ~(3u << (3*2));   // No pull 

    // RED LED  PC3 as output
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u << (3*2)); // Clear mode
    GPIOC->MODER |=  (1u << (3*2));   // Output mode
    GPIOC->OTYPER &= ~(1u << 3);
    GPIOC->OSPEEDR |= (3u << (3*2)); // high speed
    GPIOC->PUPDR &= ~(3u << (3*2));  // no pull

    // buzzer as output PC0
    GPIOC->MODER &= ~(3u << (0*2)); // clear mode
    GPIOC->MODER |= (1U << (0*2)); // output mode
    GPIOC->OTYPER &= ~(1u << 0); // push-pull
    GPIOC->OSPEEDR |= (3u << (0*2)); // high speed
    GPIOC->PUPDR &= ~(3u << (0*2)); // no pull
    
}

//=== I2C1 Init ===//
static void I2C1_Init(void){
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  I2C1->CR1 &= ~I2C_CR1_PE;
  I2C1->TIMINGR = I2C1_TIMING;
  I2C1->CR1 |= I2C_CR1_PE;
}

static void I2C1_Write2Bytes(uint8_t ctrl, uint8_t data){
  while(I2C1->ISR & I2C_ISR_BUSY);
  
  I2C1->CR2 = SSD1306_I2C_ADDR | (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
  while(!(I2C1->ISR & I2C_ISR_TXIS));
  I2C1->TXDR = ctrl;
  while(!(I2C1->ISR & I2C_ISR_TXIS));
  I2C1->TXDR = data;
  while(!(I2C1->ISR & I2C_ISR_TC));
  I2C1->CR2 |= I2C_CR2_STOP;
  while(!(I2C1->ISR & I2C_ISR_STOPF));
  I2C1->ICR |= I2C_ICR_STOPCF;
}

static void I2C1_DataBurst(uint8_t ctrl, uint8_t *data, uint16_t size){
  if(size == 0) return;
  while(I2C1->ISR & I2C_ISR_BUSY);
  
  I2C1->CR2 = SSD1306_I2C_ADDR | ((size+1) << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
  while(!(I2C1->ISR & I2C_ISR_TXIS));
  I2C1->TXDR = ctrl;
  
  for(uint16_t i=0; i<size; i++){
    while(!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = data[i];
  }
  
  while(!(I2C1->ISR & I2C_ISR_TC));
  I2C1->CR2 |= I2C_CR2_STOP;
  while(!(I2C1->ISR & I2C_ISR_STOPF));
  I2C1->ICR |= I2C_ICR_STOPCF;
}

//=== OLED Functions ===//
static int OLED_Cmd(uint8_t cmd){
  I2C1_Write2Bytes(SSD1306_CMD, cmd);
  return 0;
}

static void OLED_Init(void){
  delay_ms(100);
  OLED_Cmd(0xAE); // Display off
  OLED_Cmd(0xD5); OLED_Cmd(0x80); // Set display clock divide ratio/oscillator frequency
  OLED_Cmd(0xA8); OLED_Cmd(0x3F); // Set multiplex ratio(1 to 64)
  OLED_Cmd(0xD3); OLED_Cmd(0x00); // Set display offset
  OLED_Cmd(0x40); // Set display start line
  OLED_Cmd(0x8D); OLED_Cmd(0x14); // Enable charge pump
  OLED_Cmd(0x20); OLED_Cmd(0x00); // Set memory addressing mode
  OLED_Cmd(0xA1); // Set segment re-map
  OLED_Cmd(0xC8); // Set COM output scan direction
  OLED_Cmd(0xDA); OLED_Cmd(0x12); // Set COM pins hardware configuration
  OLED_Cmd(0x81); OLED_Cmd(0xCF); // Set contrast control
  OLED_Cmd(0xD9); OLED_Cmd(0xF1); // Set pre-charge period
  OLED_Cmd(0xDB); OLED_Cmd(0x40); // Set VCOMH deselect level
  OLED_Cmd(0xA4); // Disable entire display on
  OLED_Cmd(0xA6); // Set normal display
  OLED_Cmd(0xAF); // Display on
  OLED_ClearScreen();
}

static void OLED_SetPageCol(uint8_t page, uint8_t col){
  OLED_Cmd(0xB0 | (page & 0x0F));
  OLED_Cmd(0x00 | (col & 0x0F));
  OLED_Cmd(0x10 | ((col >> 4) & 0x0F));
}

static void OLED_ClearScreen(void){
  for(uint8_t page=0; page<8; page++){
    OLED_SetPageCol(page, 0);
    for(uint8_t col=0; col<128; col++){
      I2C1_Write2Bytes(SSD1306_DATA, 0x00);
    }
  }
}

static void OLED_ClearRow(uint8_t page){
  OLED_SetPageCol(page, 0);
  for(uint8_t col=0; col<128; col++){
    I2C1_Write2Bytes(SSD1306_DATA, 0x00);
  }
}

static void OLED_PrintChar(uint8_t page, uint8_t col, char c){
  if(c < 32) c = 32;
  OLED_SetPageCol(page, col);
  I2C1_DataBurst(SSD1306_DATA, (uint8_t*)glyph(c), 5);
  I2C1_Write2Bytes(SSD1306_DATA, 0x00);
}

static void OLED_PrintCentered(uint8_t page, const char* str){
  int len = 0;
  while(str[len]) len++;
  int start_col = (128 - (len * 6)) / 2;
  for(int i=0; str[i]; i++){
    OLED_PrintChar(page, start_col + i*6, str[i]);
  }
}

//=== 5x7 Font ===//
static const uint8_t font5x7[][5] = {
  {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00},
  {0x14,0x7F,0x14,0x7F,0x14}, {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
  {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00}, {0x00,0x1C,0x22,0x41,0x00},
  {0x00,0x41,0x22,0x1C,0x00}, {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
  {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00},
  {0x20,0x10,0x08,0x04,0x02}, {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
  {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31}, {0x18,0x14,0x12,0x7F,0x10},
  {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00},
  {0x00,0x56,0x36,0x00,0x00}, {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14},
  {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06}, {0x32,0x49,0x79,0x41,0x3E},
  {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
  {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
  {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
  {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
  {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
  {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
  {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
  {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}
};

static const uint8_t* glyph(char c){
  if(c >= 'a' && c <= 'z') c = c - 'a' + 'A';
  if(c >= '0' && c <= '9') return font5x7[c - '0' + 16];
  if(c >= 'A' && c <= 'Z') return font5x7[c - 'A' + 33];
  if(c >= '!' && c <= '/') return font5x7[c - '!' + 1];
  if(c >= ':' && c <= '@') return font5x7[c - ':' + 26];
  return font5x7[0];
}

// --- PASSIVE buzzer tone on PC0: freq Hz for duration ms ---
static void buzzer_tone(uint32_t freq_hz, uint32_t duration_ms){
  if (freq_hz == 0 || duration_ms == 0) return;
  uint32_t half_us = 1000000u / (2u*freq_hz);          // e.g. 2 kHz -> 250 us
  uint32_t edges   = (duration_ms * 1000u) / half_us;  // number of toggles

  for(uint32_t i=0; i<edges; ++i){
    // toggle PC0 using BSRR
    if (GPIOC->ODR & (1u<<BUZZER_PC0))
      GPIOC->BSRR = (1u << (BUZZER_PC0+16));  // HIGH->LOW
    else
      GPIOC->BSRR = (1u << BUZZER_PC0);       // LOW->HIGH
    delay_us(half_us);
  }

  // ensure silent after tone (respect polarity)
  BUZZER_DEASSERT();
}

static void buzzer_beeps(uint32_t freq_hz, uint32_t on_ms, uint32_t off_ms, uint32_t repeats){
  for(uint32_t i=0;i<repeats;i++){
    // LED ON during audible portion
    RED_LED_ON();
    buzzer_tone(freq_hz, on_ms);
    // LED OFF during pause
    RED_LED_OFF();
    delay_ms(off_ms);
  }
}

static void buzzer_off(void){
  BUZZER_DEASSERT();
} 

void _init(void){}
