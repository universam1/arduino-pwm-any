#include <Arduino.h>

#define FADE_LED_PWM_BITS 12
#define FADE_LED_RESOLUTION ((1 << FADE_LED_PWM_BITS) - 1)
double power;

#define BAUCH 1e-4

#define LED_OFF (PORTB &= ~(1 << PORTB5))
#define LED_ON (PORTB |= (1 << PORTB5))

#define TIMER1_ACTIVE_64 (TCCR1B |= (1 << CS11) | (1 << CS10)) // Set prescaler to 64 and starts PWM
#define TIMER1_DISABLED (TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10))) // Stops PWM

double findMaxForPow(uint16_t maxInputValue, double bauch)
{
    double exp = 1;
    double expLast = 0;
    double expadj = 1;
    double v = 0;

    while (exp != expLast)
    {
        expLast = exp;
        v = pow(maxInputValue, exp) * bauch;
        if (v > FADE_LED_RESOLUTION)
        {
            exp -= expadj;
            expadj = expadj / 10;
        }
        exp += expadj;
    }
    Serial.print(F("For B of: "));
    Serial.print(bauch, 10);
    Serial.print(F("  Use: "));
    Serial.println(exp, 10);
    return exp;
}

void LedWrite(uint16_t value)
{
    if (value > 0)
    {
        OCR1A = pow(value - 1, power) * BAUCH;
        TIMER1_ACTIVE_64;
    }
    else
    {
        TIMER1_DISABLED;
        LED_OFF; // Turn LED off
    }
}

void setup()
{
    Serial.begin(115200);
    LED_OFF; // Turn LED off
    pinMode(LED_BUILTIN, OUTPUT);

    cli();        // Disable all interrupts
    TCCR1A = 0;   // Clear all flags in control register A
    TCCR1B = 0;   // Clear all flags in control register B
    TCNT1 = 0;    // Zero timer 1 count
    TIFR1 = 0xFF; // Clear all flags in interrupt flag register

    // Set fast PWM Mode 14
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);

    TIMSK1 |= (1 << OCIE1A); // Enable timer compare interrupt
    TIMSK1 |= (1 << TOIE1);  // Enable timer overflow interrupt

    // Set PWM frequency/top value
    ICR1 = FADE_LED_RESOLUTION;
    OCR1A = 0;

    // Set prescaler to 64 and starts PWM
    TIMER1_ACTIVE_64;
    sei(); // Enable all interrupts
    power = findMaxForPow(100, BAUCH);
}

void loop()
{
    static uint32_t timeout;
    if (Serial.available())
    {
        char buff[32];
        bool po = false;
        int l = Serial.readBytesUntil('\n', buff, sizeof(buff));
        if (l > 0 && buff[l - 1] == '\r')
        {
            l--;         // to remove \r if it is there
            buff[l] = 0; // terminate the string
        }
        if (l > 0 && buff[l - 1] == 'p')
        {
            po = true;
            l--;
            buff[l] = 0; // terminate the string
        }
        uint16_t val = atoi(buff);
        Serial.print("val: ");
        Serial.println(val);
        if (po)
        {
            LedWrite(val);
            Serial.print("power: ");
            Serial.println(OCR1A);
        }
        else
            OCR1A = val;

        timeout = millis() + 20000;
    }
    if (millis() > timeout)
    {
        unsigned int val = 0;
        int v = 1;
        do
        {
            LedWrite(val);
            val += v;
            if (val > 100)
                v = -1;
            delay(5000 / 100);
        } while (val > 0);
    }
}

ISR(TIMER1_OVF_vect)
{           // Timer1 overflow interrupt service routine
    LED_ON; // Turn LED (pin 13) on
}

ISR(TIMER1_COMPA_vect)
{            // Timer1 compare interrupt service routine
    LED_OFF; // Turn LED off
}