#include "mbed.h"

// Serial
Serial pc(USBTX, USBRX, 115200);

// LED
DigitalOut red_led(D12);
DigitalOut grn_led(D13);
Thread heartbeat_thread(osPriorityLow, 256);
void heartbeat_fcn();

// Encoders
static int8_t enc_lookup_table[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
// white is +3.3/5V
// black is gnd
// Encoder 1
InterruptIn enc_1_a(PB_14); // green OPPOSITE M2
InterruptIn enc_1_b(PB_13); // blue OPPOSITE M2
void enc_1_isr();
// Encoder 2
InterruptIn enc_2_a(PA_11); // blue OPPOSITE M1
InterruptIn enc_2_b(PB_12); // green OPPOSITE M1
void enc_2_isr();
volatile int64_t enc_count[2] = {0, 0};

// Motors
// Motor 1
// Short wire side
DigitalOut mot_1_dir(D3);
PwmOut mot_pwm[2] = {D4, D6};
DigitalOut mot_2_dir(D5);
// PwmOut mot_pwm[1](D6);

// Controllers
// gains bro
#define Kp .002f
#define Ki .00001f
#define Kd .05f
// Controller 1 (for motor 1)
Ticker controller_1_ticker;
void controller_0_fcn();
int64_t motor_1_set = 0;
// Controller 2 (for motor 2)
Ticker controller_2_ticker;
void controller_1_fcn();
int64_t motor_2_set = 0;

// global flags
bool moving_1 = false;
bool moving_2 = false;

int main()
{
    // Encoder 1 init
    enc_1_a.rise(&enc_1_isr);
    enc_1_a.fall(&enc_1_isr);
    enc_1_b.rise(&enc_1_isr);
    enc_1_b.fall(&enc_1_isr);
    // Encoder 2 init
    enc_2_a.rise(&enc_2_isr);
    enc_2_a.fall(&enc_2_isr);
    enc_2_b.rise(&enc_2_isr);
    enc_2_b.fall(&enc_2_isr);

    // Motor 1 init
    // mot_1_dir = 0 is M1A direction
    mot_1_dir.write(0);
    mot_pwm[0].period_us(25); // 40 kHz
    mot_pwm[0].write(.0f);
    // Motor 2 init
    // mot_2_dir = 0 is M2A direction
    mot_2_dir.write(0);
    mot_pwm[1].period_us(25); // 40 kHz
    mot_pwm[1].write(.0f);

    // Controller 1
    controller_1_ticker.attach_us(&controller_0_fcn, 1000);
    // Controller 2
    controller_2_ticker.attach_us(&controller_1_fcn, 1000);

    // Heartbeat init after other inits
    heartbeat_thread.start(heartbeat_fcn);

    // Status LEDs off
    grn_led.write(0);
    red_led.write(0);

    // action flags

    // action arrays
    uint8_t num_actions = 2;
    int32_t action_delay[num_actions] = {3000, 1000};
    int64_t motor_1_position[num_actions] = {2000, 12000};
    int64_t motor_2_position[num_actions] = {10000, 12000};

    //action loop
    for (int i = 0; i < num_actions; i++)
    {
        motor_1_set = motor_1_position[i];
        motor_2_set = motor_2_position[i];
        moving_1 = true;
        moving_2 = true;
        while (moving_1 == true || moving_2 == true)
        {
            ThisThread::sleep_for(10);
        }
        ThisThread::sleep_for(action_delay[i]);
    }

    // Infinite loop
    while (true)
    {
        ThisThread::sleep_for(500);
    }
}

void controller_0_fcn()
{
    static int64_t err = 0;
    static int64_t last_err = 0;
    static float err_sum = 0;
    static float out = 0;

    // Apply control signal at start of fcn call
    mot_pwm[0].write(abs(out));

    err = motor_1_set - enc_count[0];

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
        out += err_sum;
    }

    out += Kd * (err - last_err); // Derivative
    last_err = err;               // Update last_error to be current err for next loop

    mot_1_dir.write(out / abs(out) == -1); // OPPOSITE TO M2

    if (err >= -1 && err <= 1)
    {
        moving_1 = false;
        out = 0;
    }
}

void controller_1_fcn()
{
    static int64_t err = 0;
    static int64_t last_err = 0;
    static float err_sum = 0;
    static float out = 0;

    // Apply control signal at start of fcn call
    mot_pwm[1].write(abs(out));

    err = motor_2_set - enc_count[1];

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
        out += err_sum;
    }

    out += Kd * (err - last_err); // Derivative
    last_err = err;               // Update last_error to be current err for next loop

    mot_2_dir.write(out / abs(out) == 1); // OPPOSITE TO M1

    if (err >= -1 && err <= 1)
    {
        moving_2 = false;
        out = 0;
    }
}

void enc_1_isr()
{
    static uint8_t enc_val = 0;
    enc_val = (enc_val << 2) | ((enc_1_a.read() << 1) | enc_1_b.read());
    enc_count[0] += enc_lookup_table[enc_val & 0b1111];
}

void enc_2_isr()
{
    static uint8_t enc_val = 0;
    enc_val = (enc_val << 2) | ((enc_2_a.read() << 1) | enc_2_b.read());
    enc_count[1] += enc_lookup_table[enc_val & 0b1111];
}

void heartbeat_fcn()
{
    while (true)
    {
        grn_led.write(!grn_led);
        ThisThread::sleep_for(500);
    }
}
