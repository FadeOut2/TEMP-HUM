#include "dht_data.h"


/* --- Precise delays for 16MHz HSI ---
 * Use ~1 NOP per cycle; empirically ~16 NOPs per microsecond at 16 MHz
 */
static inline void delay_us(uint32_t us){
    uint32_t cycles = us * 16u;
    while(cycles--) __NOP();
}

static inline void delay_ms(uint32_t ms){
    for(uint32_t i=0; i<ms; i++) delay_us(1000);
}

static inline void clear_values(DHT_Data * data){
    data->humidity = 0;
    data->temperature = 0;
    data->checksum = 0;
}

/* --- GPIO control: ONLY pull LOW or release to Hi-Z --- */
static inline void dht_pull_low(void){
    // CRITICAL: Set output LOW first, THEN configure as output
    DHT_PORT->BSRR = (1u << (DHT_PIN+16));     // Set LOW in ODR
    DHT_PORT->OTYPER &= ~(1u << DHT_PIN);      // Push-pull
    DHT_PORT->OSPEEDR |= (3u << (DHT_PIN*2));  // High speed
    DHT_PORT->MODER &= ~(3u << (DHT_PIN*2)); // Clear mode bits
    DHT_PORT->MODER |=  (1u << (DHT_PIN*2));   // Output mode (now drives LOW)
}

static inline void dht_release(void){
    // Switch to input with pull-up to ensure stable HIGH if external pull-up is absent
    DHT_PORT->MODER &= ~(3u << (DHT_PIN*2));   // Input mode
    DHT_PORT->PUPDR &= ~(3u << (DHT_PIN*2));   // clear
    DHT_PORT->PUPDR |=  (1u << (DHT_PIN*2));   // Pull-up (01)
}

static inline uint32_t dht_read_pin(void){ // its either 0 or 1
    return (DHT_PORT->IDR >> DHT_PIN) & 1u; 
}

/* --- Initialization --- */
void DHT_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    dht_release();          // Start in Hi-Z
    delay_ms(2000);         // DHT11 needs 1-2 seconds after power-on
    
    // Try to reset DHT if it's stuck LOW
    dht_pull_low();
    delay_ms(20);
    dht_release();
    delay_ms(100);          // Give it time to return to idle (HIGH)
}

/* --- Read DHT11 --- */
uint8_t DHT_Read(DHT_Data *out){
    uint8_t data[5] = {0};
   // uint8_t i, j;

    // Disable interrupts for critical timing
    __disable_irq();

    /* 1) Start signal: pull LOW for 20ms, then release */
    dht_pull_low();
    __enable_irq();  // Re-enable during long delay
    delay_ms(20);
    __disable_irq();
    dht_release();
    
    /* 2) Wait for DHT response - it should pull LOW then HIGH */
    // Wait up to 100us for DHT to pull LOW
    volatile uint32_t timeout = 1000;
    while(dht_read_pin() && --timeout);
    if(timeout == 0) { __enable_irq(); return 0; }
    
    // Wait for DHT to release (go HIGH)
    timeout = 1000;
    while(!dht_read_pin() && --timeout);
    if(timeout == 0) { __enable_irq(); return 0; }
    
    // Wait for DHT to pull LOW again (start of first bit)
    timeout = 1000;
    while(dht_read_pin() && --timeout);
    if(timeout == 0) { __enable_irq(); return 0; }

    /* 3) Read 40 bits (5 bytes) */
    for(uint8_t i = 0; i < 5; i++){
        for(uint8_t j = 0; j < 8; j++){
            // Each bit: 50us LOW, then 26-28us HIGH (0) or 70us HIGH (1)
            
            // Wait for HIGH (end of LOW pulse)
            timeout = 1000;
            while(!dht_read_pin() && --timeout);
            if(timeout == 0) { __enable_irq(); return 0; }
            
            // Delay 35us then check: if still HIGH = '1', if LOW = '0'
            delay_us(35);
            
            if(dht_read_pin()){
                data[i] |= (1 << (7-j));  // Bit is '1'
            }
            
            // Wait for LOW (start of next bit or end)
            timeout = 1000;
            while(dht_read_pin() && --timeout);
            // Don't check timeout here - last bit may not go LOW
        }
    }
    
    __enable_irq(); // out of critical timing section

    /* 4) Verify checksum */
    uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    
    /* 5) Parse DHT11 data (always return data, even if checksum fails) */
    out->humidity = data[0];
    out->temperature = data[2];
    out->checksum = data[4];
    
    // Return success even if checksum wrong (for debugging)
    if(checksum != data[4]) return 2;  // Return 2 for checksum error
    return 1;  // Return 1 for full success
}
