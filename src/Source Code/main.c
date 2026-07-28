#include <avr/io.h>
#include <stdio.h>

#define PRESCALER_128       ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0))
#define PRESCALER_ADEN      (PRESCALER_128 | (1 << ADEN))
#define VREF_1_1            ((1 << REFS1) | (1 << REFS0))
#define MOTOR_OUTPUTS       ((1 << DDD7) | (1 << DDD6) | (1 << DDD5) | (1 << DDD4))

#define MOTOR_OFF           0x00 // 0b00000000

#define BASE_MOTOR_RESET    0x3F // 0b00111111
#define BASE_MOTOR_CW       0x80 // 0b10000000 (PD7)
#define BASE_MOTOR_CCW      0x40 // 0b01000000 (PD6)
#define BASE_MOTOR_MASK     0xC0 // 0b11000000

#define PANEL_MOTOR_RESET   0xCF // 0b11001111
#define PANEL_MOTOR_CW      0x10 // 0b00010000
#define PANEL_MOTOR_CCW     0x20 // 0b00100000
#define PANEL_MOTOR_MASK    0x30 // 0b00110000

#define MUX_BITS_OFF        0xF0 // 0b11110000

#define U2X                 (1 << U2X0)
#define TX                  (1 << TXEN0)
#define BAUD_RATE           9

// Capacitive touch sensor pins: SEND = Arduino pin 8 (PB0), RECEIVE = Arduino pin 12 (PB4)
#define SEND_OUTPUT          (1 << PB0) // 0b00000001
#define RECEIVE_PULL_UP_OFF  ~(1 << PB4) // 0b11101111
#define RECEIVE_PIN          (1 << PB4) // 0b00010000

char sprint_buf[128];

struct KalmanFilter
{
    int64_t k; // Calculated kalman gain
    int64_t x; // Filtered state estimate
    int64_t p; // Estimate error covariance
    int64_t q; // Process noise covariance
    int64_t r; // Measurement noise covariance
};

struct KalmanFilter base_kalman_filter    = {0, 33554432, 65536, 13107, 655360}; // Base LDR
struct KalmanFilter panel_kalman_filter   = {0, 33554432, 65536, 13107, 655360}; // Panel LDR
struct KalmanFilter battery_kalman_filter = {0, 33554432, 65536, 66,    655360}; // Batery voltage
struct KalmanFilter pot_kalman_filter     = {0, 33554432, 65536, 13107, 655360}; // Potentiometer

// pot limit values (set once by pot_calibration at startup)
uint32_t low_pot_ADC  = 0;
uint32_t high_pot_ADC = 0;
uint32_t pot_filt_ADC = 0;
uint32_t pot_midpoint = 0;

uint8_t UART_enabled = 1;

void Motor_Setup(void)
{
    DDRD |= MOTOR_OUTPUTS;
}

// Configures PD2 (right hall) and PD3 (left hall) as inputs
void inputPin(void)
{
    DDRD &= ~(1 << DDD2);
    DDRD &= ~(1 << DDD3);
}

void setBaseMotor(unsigned char direction)
{
    PORTD = (PORTD & BASE_MOTOR_RESET) | direction;
}

void setPanelMotor(unsigned char direction)
{
    PORTD = (PORTD & PANEL_MOTOR_RESET) | direction;
}

void ADC_Setup(void)
{
    ADMUX |= VREF_1_1;
    ADCSRA |= PRESCALER_ADEN;
}

unsigned int getADCValue(unsigned char channel)
{
    ADMUX = (ADMUX & MUX_BITS_OFF) | channel;
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return (ADC);
}

void UART_Setup(void)
{
    UCSR0A |= U2X;
    UBRR0L = BAUD_RATE;
    UBRR0H = 0;
    UCSR0B = TX;
}

void UART_Transmit(unsigned char buff)
{
    while ((UCSR0A & (1 << UDRE0)) == 0);
    UDR0 = buff;
}

void UART_Transmit_Ptr(char* buff)
{
    while (*buff)
    {
        UART_Transmit(*buff);
        buff++;
    }
}

signed int kalman_update(struct KalmanFilter *select, unsigned int raw_adc_val)
{
    // Scale the raw ADC value by 65536
    int64_t raw_adc_val_multiplied = (int64_t)raw_adc_val * 65536;

    // Predict error covariance (p)
    select->p = select->p + select->q;

    // Calculate Kalman gain (K) which is scaled up by 65536
    select->k = ((select->p) * 65536) / (select->p + select->r);

    // Update state estimate (x)
    select->x = select->x + ((select->k * (raw_adc_val_multiplied - select->x)) / 65536);

    // Update error covariance (p) which is used in the next iteration
    select->p = ((65536 - select->k) * select->p) / 65536;

    // Scale down value by 65536
    signed int filtered_adc_val = (signed int)(select->x / 65536);

    // Return filtered ADC value
    return filtered_adc_val;
}

// Drives the panel motor to each Hall-effect limit to find the pot's high/low
// ADC bounds, then computes the midpoint. Runs once at startup, before the
// main loop, so it reads PIND directly.
void pot_calibration(void)
{
    // 1 - check right hall
    if (!(PIND & 0b100))
    {
        high_pot_ADC = getADCValue(3);
        while (PIND & 0b1000)
        {
            setPanelMotor(PANEL_MOTOR_CCW);
        }
        setPanelMotor(MOTOR_OFF);
        low_pot_ADC = getADCValue(3);
    }
    // 2 - check left hall
    else if (!(PIND & 0b1000))
    {
        low_pot_ADC = getADCValue(3);
        while (PIND & 0b100)
        {
            setPanelMotor(PANEL_MOTOR_CW);
        }
        setPanelMotor(MOTOR_OFF);
        high_pot_ADC = getADCValue(3);
    }
    else
    {
        while (PIND & 0b1000)
        {
            setPanelMotor(PANEL_MOTOR_CCW);
        }
        setPanelMotor(MOTOR_OFF);
        low_pot_ADC = getADCValue(3);
        while (PIND & 0b100)
        {
            setPanelMotor(PANEL_MOTOR_CW);
        }
        setPanelMotor(MOTOR_OFF);
        high_pot_ADC = getADCValue(3);
    }

    pot_midpoint = (high_pot_ADC + low_pot_ADC) / 2;
}

void capacitive_sensor(void)
{
UCSR0B = 0; // turns off TX
UART_enabled = 0;
}

int main(void)
{
    ADC_Setup();
    Motor_Setup();
    inputPin();
    UART_Setup();
    pot_calibration();

    ///////////////////////////////////////////////
    /////////// Capacitive Touch Sensor ///////////
    ///////////////////////////////////////////////

    // ** ONLY ENABLED ONCE ON BOOT

    // Capacitive touch check (SEND = PB0 / Arduino pin 8, RECEIVE = PB4 / Arduino pin 12)
    unsigned int touch_time = 0;

    DDRB = (DDRB & 0b11101110) | SEND_OUTPUT; // Reset PB0 (SEND) PB4 (RECEIVE); set send = 1 (OUTPUT)
    PORTB &= RECEIVE_PULL_UP_OFF; // Set RECEIVE pull-up LOW

    DDRB |= RECEIVE_PIN; // Set RECEIVE to OUTPUT
    PORTB &= 0b11101110; // SEND + RECEIVE pull-up LOW

    DDRB &= ~RECEIVE_PIN; // Set RECEIVE to INPUT
    PORTB |= SEND_OUTPUT; // Set OUTPUT pin pull-up HIGH

    while (!(PINB & RECEIVE_PIN)) // wait for RECEIVE to go HIGH (charge time through mutual capacitance)
    {
        touch_time++;
    }
        if (touch_time > 40) // Determined experimentally, base reading is 14, can be different on other boards
    {
        capacitive_sensor(); // Call capacitive sensor function
    }
    touch_time = 0;

    if (UART_enabled)
    {
        UART_Transmit_Ptr("boot\n");
    }

    // Hall Effect state tracking (edge-detected, so we only print on change)
    char right_last_message = 0;
    char left_last_message = 0;

    // Motor state latches
    int cw_latched = 0;
    int ccw_latched = 0;

    while (1)
    {
        ///////////////////////////////////
        /////// Hall Effect Check /////////
        ///////////////////////////////////
        uint8_t right_hall_high = (PIND >> 2) & 1; // PD2 (Right)
        uint8_t left_hall_high  = (PIND >> 3) & 1; // PD3 (Left)

        if (right_hall_high)
        {
            if (right_last_message != 'A')
            {
                if (UART_enabled) 
                { 
                    UART_Transmit_Ptr("Right Hall effect is HIGH\n"); 
                }
                right_last_message = 'A';
            }
        }
        else
        {
            if (right_last_message != 'B')
            {
                if (UART_enabled) 
                { 
                    UART_Transmit_Ptr("Right Hall effect is LOW\n"); 
                }
                right_last_message = 'B';
            }
        }

        if (left_hall_high)
        {
            if (left_last_message != 'A')
            {
                if (UART_enabled) 
                { 
                    UART_Transmit_Ptr("Left Hall effect is HIGH\n"); 
                }
                left_last_message = 'A';
            }
        }
        else
        {
            if (left_last_message != 'B')
            {
                if (UART_enabled) 
                { 
                    UART_Transmit_Ptr("Left Hall effect is LOW\n"); 
                }
                left_last_message = 'B';
            }
        }

        ///////////////////////////
        /////// Battery ADC ///////
        ///////////////////////////
        unsigned int raw_battery_val = getADCValue(0); // VBAT channel
        signed int battery_adc = kalman_update(&battery_kalman_filter, raw_battery_val);

        sprintf(sprint_buf, "Battery ADC = %u, battery filtADC = %d\n", raw_battery_val, battery_adc);
        if (UART_enabled) 
        { 
            UART_Transmit_Ptr(sprint_buf); 
        }

        ///////////////////////////
        //////// Base ADC /////////
        ///////////////////////////
        unsigned int raw_base_val = getADCValue(1); // base LDR channel
        signed int base_filt_val = kalman_update(&base_kalman_filter, raw_base_val);

        sprintf(sprint_buf, "Base LDR ADC = %u, base filtADC = %d\n", raw_base_val, base_filt_val);
        if (UART_enabled) 
        { 
            UART_Transmit_Ptr(sprint_buf); 
        }

        if (base_filt_val > (3 * battery_adc) / 4)
        {
            if ((PORTD & BASE_MOTOR_CW) == 0)
            {
                setBaseMotor(BASE_MOTOR_CW);
                if (UART_enabled) 
                { 
                    UART_Transmit_Ptr("Base motor is CW\n"); 
                }
            }
        }
        else if (base_filt_val < battery_adc / 4)
        {
            if ((PORTD & BASE_MOTOR_CCW) == 0)
            {
                setBaseMotor(BASE_MOTOR_CCW);
                if (UART_enabled) 
                { 
                    UART_Transmit_Ptr("Base motor is CCW\n"); 
                }
            }
        }
        else
        {
            if (PORTD & BASE_MOTOR_MASK)
            {
                setBaseMotor(MOTOR_OFF);
                if (UART_enabled) 
                { 
                    UART_Transmit_Ptr("Base motor is OFF\n"); 
                }
            }
        }

        /////////////////////////
        //////// Pot ADC ////////
        /////////////////////////
        unsigned int raw_pot_val = getADCValue(3); // pot wiper channel
        pot_filt_ADC = kalman_update(&pot_kalman_filter, raw_pot_val);

        sprintf(sprint_buf, "Pot ADC = %u, pot filtADC = %lu\n", raw_pot_val, pot_filt_ADC);
        if (UART_enabled) 
        { 
            UART_Transmit_Ptr(sprint_buf); 
        }

        ///////////////////////////
        //////// Panel ADC ////////
        ///////////////////////////
        unsigned int raw_panel_val = getADCValue(2); // panel LDR channel
        signed int panel_filt_val = kalman_update(&panel_kalman_filter, raw_panel_val);

        sprintf(sprint_buf, "Panel LDR ADC = %u, panel filtADC = %d\n", raw_panel_val, panel_filt_val);

        if (UART_enabled) 
        { 
            UART_Transmit_Ptr(sprint_buf); 
        }

        if (panel_filt_val > (3 * battery_adc) / 4)
        {
            ccw_latched = 0;
            if ((right_hall_high && (pot_filt_ADC < (high_pot_ADC + 10)) && (PORTD & PANEL_MOTOR_CW) == 0) && (cw_latched == 0))
            {
                setPanelMotor(PANEL_MOTOR_CW);
                if (UART_enabled) 
                { 
                    UART_Transmit_Ptr("Panel motor is CW\n"); 
                }
            }
            else 
            {
                if (!right_hall_high || (pot_filt_ADC >= (high_pot_ADC + 10)))
                {
                    cw_latched = 1;
                    setPanelMotor(MOTOR_OFF);
                }
            }
        }
        else if (panel_filt_val < battery_adc / 4)
        {
            cw_latched = 0;
            if ((left_hall_high && (pot_filt_ADC > (low_pot_ADC - 10)) && (PORTD & PANEL_MOTOR_CCW) == 0) && (ccw_latched == 0))
            {
                setPanelMotor(PANEL_MOTOR_CCW);
                if (UART_enabled) 
                { 
                    UART_Transmit_Ptr("Panel motor is CCW\n"); 
                }
            }
            else if (!left_hall_high || (pot_filt_ADC <= (low_pot_ADC - 10)))
            {
                setPanelMotor(MOTOR_OFF);
            }
        }
        else
        {
            if (PORTD & PANEL_MOTOR_MASK)
            {
                ccw_latched = 1;
                setPanelMotor(MOTOR_OFF);
                if (UART_enabled) 
                { 
                    UART_Transmit_Ptr("Panel motor is OFF\n"); 
                }
            }
        }
    }
}