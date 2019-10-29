#include "mbed.h"

typedef void (*f_t)(); // function pointer typedef

// Serial
Serial pc(USBTX, USBRX, 115200);

// LED
DigitalOut red_led(D12);
DigitalOut grn_led(D13);
Thread heartbeat_thread(osPriorityLow, 256);
void heartbeat_fcn();

// Encoders
// white is +3.3/5V
// black is gnd
static int8_t enc_lookup_table[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
InterruptIn enc[2][2] = {{PB_14, PB_13},  // encoder 0
                         {PA_11, PB_12}}; // encoder 1
void enc_0_isr();
void enc_1_isr();
f_t enc_isr_ptr[2] = {&enc_0_isr, &enc_1_isr};
volatile int64_t enc_count[2] = {0, 0};

// Motors
// Motor 1
DigitalOut mot_dir[2] = {D3, D5};
PwmOut mot_pwm[2] = {D4, D6};

// Controllers
// gains bro
#define Kp .002f
#define Ki .00001f
#define Kd .05f
void controller_0_fcn();
void controller_1_fcn();
f_t controller_fcn[2] = {&controller_0_fcn, &controller_1_fcn};
Ticker controller_0_ticker, controller_1_ticker;
Ticker controller_ticker[2] = {controller_0_ticker, controller_1_ticker};
int64_t motor_set[2] = {0, 0};

// global flags
bool moving[2] = {false, false};

int main()
{
    for (uint8_t m = 0; m < 2; m++) // iterate through motors
    {
        mot_dir[m].write(0);
        mot_pwm[m].period_us(25); // 40 kHz
        mot_pwm[m].write(0.0f);
        controller_ticker[m].attach_us(controller_fcn[m], 1000);
        for (uint8_t p = 0; p < 2; p++) // iterate through encoder pins
        {
            // attach interrupts to all motors+pins on rise and fall
            enc[m][p].rise(enc_isr_ptr[m]);
            enc[m][p].fall(enc_isr_ptr[m]);
        }
    }
    heartbeat_thread.start(heartbeat_fcn); // start heartbeat after succesful inits

    // init with status LEDs off
    grn_led.write(0);
    red_led.write(0);

    // action flags

    // action arrays
    uint8_t num_actions = 2;
    int32_t action_delay[num_actions] = {3000, 1000};
    int64_t motor_position[2][num_actions] = {{1000, 3000},  // motor 0
                                              {2000, 3000}}; // motor 1

    //action loop
    for (int i = 0; i < num_actions; i++)
    {
        for (uint8_t m = 0; m < 2; m++)
        {
            motor_set[m] = motor_position[m][i];
            moving[m] = true;
        }
        while (moving[0] == true || moving[1] == true)
        {
            ThisThread::sleep_for(100);
        }
        ThisThread::sleep_for(action_delay[i]);
    }

    // Infinite loop
    while (true)
    {
        heartbeat_thread.terminate();
        grn_led.write(!grn_led);
        ThisThread::sleep_for(100); // TODO change this to recall heartbeat_fcn with different blink time
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

    err = motor_set[0] - enc_count[0];

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

    mot_dir[0].write(out / abs(out) == -1); // OPPOSITE TO M2

    if (err >= -1 && err <= 1)
    {
        moving[0] = false;
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

    err = motor_set[1] - enc_count[1];

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

    mot_dir[1].write(out / abs(out) == 1); // OPPOSITE TO M1

    if (err >= -1 && err <= 1)
    {
        moving[1] = false;
        out = 0;
    }
}

void enc_0_isr()
{
    static uint8_t enc_val = 0;
    enc_val = (enc_val << 2) | ((enc[0][0].read() << 1) | enc[0][1].read());
    enc_count[0] += enc_lookup_table[enc_val & 0b1111];
}

void enc_1_isr()
{
    static uint8_t enc_val = 0;
    enc_val = (enc_val << 2) | ((enc[1][0].read() << 1) | enc[1][1].read());
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
