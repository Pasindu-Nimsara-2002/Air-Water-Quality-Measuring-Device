#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <DHT.c>
#include <ds18b20/ds18b20.h>

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#define PB_CANCEL (1 << PD7)
#define PB_DOWN (1 << PD3)
#define PB_OK (1 << PD6)
#define PB_UP (1 << PD2)
#define DHT11PIN (1 << PD4) 
#define PH_PIN 0 
#define MQ9_PIN 3  
#define MQ135_PIN 4  
#define RL_VALUE 1.0  
#define RO_CLEAN_AIR_FACTOR 9.9  
#define RL_VALUE_MQ135 1.0  
#define RO_CLEAN_AIR_FACTOR_MQ135 3.6  

float Ro = 9.9;  
float LPG_b = 1.8, LPG_m = -0.38;
float CO_b = 1.35, CO_m = -0.37;
float CH4_b = 2.3, CH4_m = -0.47;
float Ro_MQ135 = 3.6;  
float NH3_b = 1.958, NH3_m = -0.86;
float NOx_b = 2.3, NOx_m = -1.2;
float calibration_value = 21.34;
int buffer_arr[10], temp;
float ph_act;
unsigned long avgval;
int8_t temperature_int = 0;
int8_t humidity_int = 0;

void readMQ9();
void readMQ135();
void Air_Temp();
void PH();
void Conductivity();
void Water_Temp();
void setup();
void loop();
void go_to_menu();
void run_mode();
void go_to_menu_air();
void run_mode_in_air();
void go_to_menu_water();
void run_mode_in_water();
void readMQ9();
void readMQ135();
float get_Water_Temp();
float MQ9_calibrate();
float MQ135_calibrate();

// UART functions
void uart_init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
    UCSR0C = (1<<USBS0) | (3<<UCSZ00);
}

void uart_transmit(unsigned char data) {
    while (!( UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

void uart_print(const char *str) {
    while (*str) {
        uart_transmit(*str++);
    }
}

void uart_write(uint8_t data) {
    uart_transmit(data);
    uart_transmit(data);
    uart_transmit(data);
}

// ADC functions
void adc_init() {
    ADMUX = (1<<REFS0); 
    ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); 
}

uint16_t adc_read(uint8_t ch) {
    ch &= 0b00000111; // select the ADC channel (0-7)
    ADMUX = (ADMUX & 0xF8) | ch;
    ADCSRA |= (1<<ADSC); // start conversion
    while (ADCSRA & (1<<ADSC)); // wait for conversion to finish
    return ADC;
}

// Global variables for menu
int current_mode = 0;
int max_modes = 2;
const char* pages[] = {"page 1", "page 2"};

int current_mode_in_air = 0;
int max_modes_in_air = 3;
const char* modes_in_air[] = {"MQ-5", "MQ-135", "Temperature"};

int current_mode_in_water = 0;
int max_modes_in_water = 3;
const char* water_pages[] = {"page 3", "page 4", "page 5"};

int wait_for_button_press() {
    while (1) {
        if (!(PIND & PB_UP)) {
            _delay_ms(500);
            return PB_UP;
        } else if (!(PIND & PB_DOWN)) {
            _delay_ms(500);
            return PB_DOWN;
        } else if (!(PIND & PB_OK)) {
            _delay_ms(500);
            return PB_OK;
        } else if (!(PIND & PB_CANCEL)) {
            _delay_ms(500);
            return PB_CANCEL;
        }
    }
}

void go_to_menu() {
    while (PIND & PB_CANCEL) {
        int pressed = wait_for_button_press();
        if (pressed == PB_UP) {
            _delay_ms(500);
            current_mode = (current_mode + 1) % max_modes;
            uart_print(pages[current_mode]);
            uart_write(0xff);
        } else if (pressed == PB_DOWN) {
            _delay_ms(500);
            current_mode = (current_mode - 1 + max_modes) % max_modes;
            uart_print(pages[current_mode]);
            uart_write(0xff);
        } else if (pressed == PB_OK) {
            _delay_ms(500);
            run_mode(current_mode);
        } else if (pressed == PB_CANCEL) {
            _delay_ms(500);
            break;
        }
    }
}

void PH() {
    while (PIND & PB_CANCEL) {
        for (int i = 0; i < 10; i++) {
            buffer_arr[i] = adc_read(PH_PIN);
            _delay_ms(30);
        }
        for (int i = 0; i < 9; i++) {
            for (int j = i + 1; j < 10; j++) {
                if (buffer_arr[i] > buffer_arr[j]) {
                    temp = buffer_arr[i];
                    buffer_arr[i] = buffer_arr[j];
                    buffer_arr[j] = temp;
                }
            }
        }
        avgval = 0;
        for (int i = 2; i < 8; i++) avgval += buffer_arr[i];
        float volt = (float)avgval * 5.0 / 1024 / 6;
        ph_act = -5.70 * volt + calibration_value;

        char cmd[20];
        snprintf(cmd, sizeof(cmd), "ph.txt=\"%.2f\"", ph_act);
        uart_print(cmd);
        uart_write(0xFF);
        _delay_ms(500);
    }
}

void Conductivity() {
    float waterTemp = get_Water_Temp();
    float rawEc = adc_read(1) * 4.3 / 1024.0;
    float temperatureCoefficient = 1.0 + 0.02 * (waterTemp - 25.0);
    float ec = (rawEc / temperatureCoefficient);
    float tds = (133.42 * pow(ec, 3) - 255.86 * ec * ec + 857.39 * ec) * 0.5;

    char cmd1[20];
    snprintf(cmd1, sizeof(cmd1), "TDS.txt=\"%.2f\"", tds);
    uart_print(cmd1);
    uart_write(0xFF);

    char cmd2[20];
    snprintf(cmd2, sizeof(cmd2), "EC.txt=\"%.2f\"", ec);
    uart_print(cmd2);
    uart_write(0xFF);
}

void Water_Temp() {
    int16_t temp;
    ds18b20convert( &PORTB, &DDRB, &PINB, ( 1 << 1 ), NULL );
    _delay_ms( 1000 );
    ds18b20read( &PORTB, &DDRB, &PINB, ( 1 << 1 ), NULL, &temp );
    float waterTemp = temp;
    char cmd1[20];
    snprintf(cmd1, sizeof(cmd1), "tempWater.txt=\"%.2f\"", waterTemp);
    uart_print(cmd1);
    uart_write(0xFF);
}

float get_Water_Temp(){
    int16_t temp;
    ds18b20convert( &PORTB, &DDRB, &PINB, ( 1 << 1 ), NULL );
    _delay_ms( 1000 );
    ds18b20read( &PORTB, &DDRB, &PINB, ( 1 << 1 ), NULL, &temp );
    float waterTemp = temp;
    return waterTemp;
}

void readMQ9() {
    int sensorValue = adc_read(MQ9_PIN);
    float voltage = sensorValue * (5.0 / 1023.0);
    float RS_gas = ((5.0 * RL_VALUE) / voltage) - RL_VALUE;
    float ratio = RS_gas / Ro;

    float ppmLPG = pow(10, ((log10(ratio) - LPG_b) / LPG_m));
    float ppmCO = pow(10, ((log10(ratio) - CO_b) / CO_m));
    float ppmCH4 = pow(10, ((log10(ratio) - CH4_b) / CH4_m));

    char cmd1[20];
    snprintf(cmd1, sizeof(cmd1), "TDS.txt=\"%.2f\"", ppmLPG);
    uart_print(cmd1);
    uart_write(0xFF);

    char cmd2[20];
    snprintf(cmd2, sizeof(cmd2), "EC.txt=\"%.2f\"", ppmCO);
    uart_print(cmd2);
    uart_write(0xFF);

    char cmd3[20];
    snprintf(cmd3, sizeof(cmd1), "TDS.txt=\"%.2f\"", ppmCH4);
    uart_print(cmd1);
    uart_write(0xFF);

}

void readMQ135() {
    int sensorValue = adc_read(MQ135_PIN);
    float voltage = sensorValue * (5.0 / 1023.0);
    float RS_gas = ((5.0 * RL_VALUE_MQ135) / voltage) - RL_VALUE_MQ135;
    float ratio = RS_gas / Ro_MQ135;

    float ppmNH3 = pow(10, ((log10(ratio) - NH3_b) / NH3_m));
    float ppmNOx = pow(10, ((log10(ratio) - NOx_b) / NOx_m));

    char cmd1[20];
    snprintf(cmd1, sizeof(cmd1), "TDS.txt=\"%.2f\"", ppmNH3);
    uart_print(cmd1);
    uart_write(0xFF);

    char cmd2[20];
    snprintf(cmd2, sizeof(cmd2), "EC.txt=\"%.2f\"", ppmNOx);
    uart_print(cmd2);
    uart_write(0xFF);
}

void Air_Temp(){   
    dht_GetTempUtil(&temperature_int, &humidity_int);
    char cmd1[20];
    snprintf(cmd1, sizeof(cmd1), "temp.txt=\"%d\"", temperature_int);
    uart_print(cmd1);
    char cmd2[20];
    snprintf(cmd2, sizeof(cmd1), "hum.txt=\"%d\"", humidity_int);
    uart_print(cmd2);
    _delay_ms(1500);
}

float MQ9_calibrate() {
    int sensorValue = 0;
    float RS_air = 0.0;
    for (int i = 0; i < 50; i++) {
        sensorValue += adc_read(MQ9_PIN);
        _delay_ms(500);
    }
    sensorValue = sensorValue / 50;
    float voltage = sensorValue * (5.0 / 1023.0);
    RS_air = ((5.0 * RL_VALUE) / voltage) - RL_VALUE;
    return RS_air / RO_CLEAN_AIR_FACTOR;
}

float MQ135_calibrate() {
    int sensorValue = 0;
    float RS_air = 0.0;
    for (int i = 0; i < 50; i++) {
        sensorValue += adc_read(MQ135_PIN);
        _delay_ms(500);
    }
    sensorValue = sensorValue / 50;
    float voltage = sensorValue * (5.0 / 1023.0);
    RS_air = ((5.0 * RL_VALUE_MQ135) / voltage) - RL_VALUE_MQ135;
    return RS_air / RO_CLEAN_AIR_FACTOR_MQ135;
}


void run_mode(int mode) {
    if (mode == 0) {
        uart_print("page 2");
        uart_write(0xff);
        go_to_menu_air();
    } else if (mode == 1) {
        uart_print("page 3");
        uart_write(0xff);
        go_to_menu_water();
    }
}

void go_to_menu_air() {
    while (PIND & PB_CANCEL) {
        int pressed = wait_for_button_press();
        if (pressed == PB_UP) {
            _delay_ms(500);
            current_mode_in_air = (current_mode_in_air + 1) % max_modes_in_air;
        } else if (pressed == PB_DOWN) {
            _delay_ms(500);
            current_mode_in_air = (current_mode_in_air - 1 + max_modes_in_air) % max_modes_in_air;
        } else if (pressed == PB_OK) {
            _delay_ms(500);
            run_mode_in_air(current_mode_in_air);
        } else if (pressed == PB_CANCEL) {
            _delay_ms(500);
            break;
        }
    }
}

void run_mode_in_air(int mode) {
    if (mode == 0) {
        uart_print("MQ-9 Sensor Data:");
        uart_write(0xFF);
        _delay_ms(500);
        readMQ9();
    } else if (mode == 1) {
        uart_print("MQ-135 Sensor Data:");
        uart_write(0xFF);
        _delay_ms(500);
        readMQ135();
    } else if (mode == 2) {
        uart_print("Temperature Sensor Data:");
        uart_write(0xFF);
        _delay_ms(500);
        Air_Temp();
    }
}


void go_to_menu_water() {
    while (PIND & PB_CANCEL) {
        int pressed = wait_for_button_press();
        if (pressed == PB_UP) {
            _delay_ms(500);
            current_mode_in_water = (current_mode_in_water + 1) % max_modes_in_water;
            uart_print(water_pages[current_mode_in_water]);
            uart_write(0xff);
        } else if (pressed == PB_DOWN) {
            _delay_ms(500);
            current_mode_in_water = (current_mode_in_water - 1 + max_modes_in_water) % max_modes_in_water;
            uart_print(water_pages[current_mode_in_water]);
            uart_write(0xff);
        } else if (pressed == PB_OK) {
            _delay_ms(500);
            run_mode_in_water(current_mode_in_water);
        } else if (pressed == PB_CANCEL) {
            _delay_ms(500);
            break;
        }
    }
}

void run_mode_in_water(int mode) {
    if (mode == 0) {
        uart_print("page 6");
        uart_write(0xFF);
        _delay_ms(500);
        PH();
    } else if (mode == 1) {
        uart_print("page 7");
        uart_write(0xff);
        Conductivity();
    } else if (mode == 2) {
        uart_print("page 8");
        uart_write(0xff);
        Water_Temp();
    }
}

void setup() {

    DDRD &= ~(PB_CANCEL | PB_DOWN | PB_OK | PB_UP);
    PORTD |= (PB_CANCEL | PB_DOWN | PB_OK | PB_UP); 

    Ro = MQ9_calibrate();
    Ro_MQ135 = MQ135_calibrate();

    uart_init(MYUBRR);
    adc_init();

    sei(); 
}

void loop() {
    if (!(PIND & PB_OK)) {
        _delay_ms(500);
        uart_print("page 1");
        uart_write(0xff);
        go_to_menu();
    }
}

int main(void) {
    setup();

    while (1) {
        loop();
    }

    return 0;
}
