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
InterruptIn enc[2][2] = {{PB_13, PB_14},  // encoder 0
                         {PB_12, PA_11}}; // encoder 1
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

// action pins
DigitalOut water = PH_1;
DigitalOut plant = PH_0;
DigitalOut plant_dir = PC_15;
DigitalOut probe = PC_14;
DigitalOut probe_dir = PA_14;

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
    // init with status LEDs off
    grn_led.write(0);
    red_led.write(0);

    heartbeat_thread.start(heartbeat_fcn); // start heartbeat after succesful inits

// action flags

// action arrays
#define STEPS_15DEG 6130 // update for new chassis
#define STEPS_1IN 1128
#define MOVE_PROBE 4 * STEPS_1IN
#define MOVE_WATER 2 * STEPS_1IN
    const uint8_t num_actions = 61;
    uint16_t action_delay[num_actions] = {3000, 1000, 1000};
    int64_t motor_delta[2][num_actions] = {
        // motor 0
        {0, MOVE_PROBE, 0, MOVE_WATER, 0, 24 * STEPS_1IN, 0, MOVE_PROBE, 0, MOVE_WATER, 0, 24 * STEPS_1IN, 0, MOVE_PROBE, 0, MOVE_WATER, 0,
         12 * STEPS_1IN, 6 * STEPS_15DEG, 12 * STEPS_1IN, 6 * STEPS_15DEG, 12 * STEPS_1IN,
         0, MOVE_PROBE, 0, MOVE_WATER, 0, 24 * STEPS_1IN, 0, MOVE_PROBE, 0, MOVE_WATER, 0, 24 * STEPS_1IN, 0, MOVE_PROBE, 0, MOVE_WATER, 0,
         12 * STEPS_1IN, 0, 12 * STEPS_1IN, 0, 12 * STEPS_1IN,
         0, MOVE_PROBE, 0, MOVE_WATER, 0, 24 * STEPS_1IN, 0, MOVE_PROBE, 0, MOVE_WATER, 0, 24 * STEPS_1IN, 0, MOVE_PROBE, 0, MOVE_WATER, 0},

        // motor 1
        {0, MOVE_PROBE, 0, MOVE_WATER, 0, 24 * STEPS_1IN, 0, MOVE_PROBE, 0, MOVE_WATER, 0, 24 * STEPS_1IN, 0, MOVE_PROBE, 0, MOVE_WATER, 0,
         12 * STEPS_1IN, 0, 12 * STEPS_1IN, 0, 12 * STEPS_1IN,
         0, MOVE_PROBE, 0, MOVE_WATER, 0, 24 * STEPS_1IN, 0, MOVE_PROBE, 0, MOVE_WATER, 0, 24 * STEPS_1IN, 0, MOVE_PROBE, 0, MOVE_WATER, 0,
         12 * STEPS_1IN, 6 * STEPS_15DEG, 12 * STEPS_1IN, 6 * STEPS_15DEG, 12 * STEPS_1IN,
         0, MOVE_PROBE, 0, MOVE_WATER, 0, 24 * STEPS_1IN, 0, MOVE_PROBE, 0, MOVE_WATER, 0, 24 * STEPS_1IN, 0, MOVE_PROBE, 0, MOVE_WATER, 0}};

    bool planting[num_actions] = {0, 0, 0};
    bool probing[num_actions] = {0, 0, 0};
    bool watering[num_actions] = {0, 1, 0};

    //action loop
    for (int i = 0; i < num_actions; i++)
    {
        ThisThread::sleep_for(action_delay[i]);

        // plant
        if (planting[i] == true)
        {
            plant_dir.write(1);
            for (int pss = 0; pss > 300; pss++)
            {
                plant.write(1);
                ThisThread::sleep_for(5);
                plant.write(0);
                ThisThread::sleep_for(5);
            }
            ThisThread::sleep_for(500);
            plant_dir.write(0);
            for (int pss = 0; pss > 300; pss++)
            {
                plant.write(1);
                ThisThread::sleep_for(5);
                plant.write(0);
                ThisThread::sleep_for(5);
            }
        }

        // probe
        if (probing[i] == true)
        {
            probe_dir.write(1);
            for (int pss = 0; pss > 300; pss++)
            {
                probe.write(1);
                ThisThread::sleep_for(5);
                probe.write(0);
                ThisThread::sleep_for(5);
            }
            ThisThread::sleep_for(500);
            probe_dir.write(0);
            for (int pss = 0; pss > 300; pss++)
            {
                probe.write(1);
                ThisThread::sleep_for(5);
                probe.write(0);
                ThisThread::sleep_for(5);
            }
        }

        // water
        if (watering[i] == true)
        {
            water.write(1);
            ThisThread::sleep_for(3000);
            water.write(0);
        }

        // move
        for (uint8_t m = 0; m < 2; m++)
        {
            motor_set[m] = enc_count[m] + motor_delta[m][i];
            moving[m] = true;
        }
        while (moving[0] == true || moving[1] == true)
        {
            ThisThread::sleep_for(250);
        }
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

    mot_dir[0].write(out / abs(out) == 1); // OPPOSITE TO M1

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

    mot_dir[1].write(out / abs(out) == -1); // OPPOSITE TO M0

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
