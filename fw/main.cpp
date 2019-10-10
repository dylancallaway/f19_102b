#include "mbed.h"

// Helpers
void clamp(float *val, float min, float max);

// Serial
Serial pc(USBTX, USBRX, 115200);

// LED
DigitalOut grn_led(D13);
DigitalOut red_led(D12);
DigitalOut r(PB_13);
DigitalOut g(PB_14);
DigitalOut b(PB_15);
Thread heartbeat_thread(osPriorityLow, 256);
void heartbeat_fcn();

// Encoders
// white is +3.3/5V
// black is gnd
// Encoder 1
InterruptIn enc_1_a(A0); // green
InterruptIn enc_1_b(A1); // blue
void enc_1_isr();
volatile int64_t enc_1_count = 0;

// Motors
// Motor 1
// Short wire side
DigitalOut mot_1_dir(A2);
PwmOut mot_1_pwm(A3);

// Controllers
// Controller 1 (for motor 1)
Ticker controller_1_ticker;
void controller_1_fcn();

int main()
{
    // Encoder 1 init
    enc_1_a.rise(&enc_1_isr);
    enc_1_a.fall(&enc_1_isr);
    enc_1_b.rise(&enc_1_isr);
    enc_1_b.fall(&enc_1_isr);

    // Motor 1 init
    // mot_1_dir = 0 is M1A direction
    mot_1_dir.write(0);
    mot_1_pwm.period_us(25); // 40 kHz
    mot_1_pwm.write(.0f);

    // Controller 1
    controller_1_ticker.attach_us(&controller_1_fcn, 1000);

    // Heartbeat init after other inits
    heartbeat_thread.start(heartbeat_fcn);

    // Status LEDs off
    grn_led.write(0);
    red_led.write(0);

    // Infinite loop
    while (true)
    {
        pc.printf("%d\n", enc_1_count);
        ThisThread::sleep_for(500);
    }
}

void controller_1_fcn()
{
    static int64_t set = 5000;

    static int64_t err = 0;
    static int64_t last_err = 0;
    static float err_sum = 0;

    static float Kp = .005;
    static float Ki = .0005;
    static float Kd = .05;

    static float out = 0;

    // Apply control signal at start of fcn call
    mot_1_pwm.write(abs(out));

    err = set - enc_1_count;

    out = Kp * err; // Proportional (this resets out to a new value, so it does not sum infinitely)

    // Integral
    if (err > 50 || err < -50 || err == 0)
    // anti-windup/saturation
    // only integrate error if near setpoint
    {
        err_sum = 0;
    }
    else
    {
        err_sum += Ki * err;
        // clamp(&err_sum, -1.0, 1.0);
        out += err_sum;
    }

    out += Kd * (err - last_err); // Derivative
    last_err = err;               // Update last_error to be current err for next loop

    // clamp(&out, -1.0, 1.0);

    mot_1_dir.write(out / abs(out) == -1);

    if (err >= -1 && err <= 1)
    {
        out = 0;
    }
}

void enc_1_isr()
{
    static uint8_t enc_val = 0;
    static int8_t enc_lookup_table[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

    enc_val = (enc_val << 2) | ((enc_1_a.read() << 1) | enc_1_b.read());
    enc_1_count += enc_lookup_table[enc_val & 0b1111];
}

void heartbeat_fcn()
{
    static int i = 0;
    while (true)
    {
        switch (i)
        {
        case 0:
            r.write(1);
            g.write(0);
            b.write(0);
            i = 1;
            break;
        case 1:
            r.write(0);
            g.write(1);
            b.write(0);
            i = 2;
            break;
        case 2:
            r.write(0);
            g.write(0);
            b.write(1);
            i = 0;
            break;
        }
        ThisThread::sleep_for(333);
    }
}

void clamp(float *val, float min, float max)
{
    if (*val < min)
    {
        *val = min;
    }
    else if (*val > max)
    {
        *val = max;
    }
}