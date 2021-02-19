#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "driver/mcpwm.h"

#define MSECOND ((double)1000000)

//1 = 25 - 27 || 13 - 33
//2 = 12 - 32 || 23 - 5

//3 = 4 - 16 || 18 - 19
//4 = 2 - 15 || 36 - 26

#define MOTORCOUNT 4

#define MOTORPINS 0
#define SENSEPINS 1

#define PULSES_PER_MOTOR_ROTATION 12
#define MOTOR_WHEEL_RATIO 90

#define PULSES_PER_WHEEL_ROTATION (PULSES_PER_MOTOR_ROTATION * MOTOR_WHEEL_RATIO)

uint8_t Motors[MOTORCOUNT][2][2] = {
    {
        {27, 25}, // Motor pins
        {33, 13}, // Sensor pins
    },
    {
        {32, 12}, // Motor pins
        {5, 23},  // Sensor pins
    },
    {
        {16, 4},  // Motor pins
        {18, 19}, // Sensor pins
    },
    {
        {15, 2},  // Motor pins
        {36, 26}, // Sensor pins
    },
};

int64_t lastTime[MOTORCOUNT] = {0};
double measuredspeed[MOTORCOUNT] = {0};
double intendedSpeed[MOTORCOUNT] = {0};
double currentPwmSpeed[MOTORCOUNT] = {0};

int test = 0;

void TaskCore1(void *parameters);
void TaskCore2(void *parameters);

static void IRAM_ATTR event(void *arg)
{
    int64_t currentTime = esp_timer_get_time();
    int pcnt_unit = (int)arg;

    uint32_t status = 0;
    pcnt_get_event_status(pcnt_unit, &status);

    int8_t direction = status == PCNT_EVT_H_LIM ? 1 : -1;

    measuredspeed[pcnt_unit] = MSECOND / (currentTime - lastTime[pcnt_unit]) * 60 / MOTOR_WHEEL_RATIO * direction;
    lastTime[pcnt_unit] = currentTime;
}

void initPCNT()
{

    pcnt_config_t config;
    for (int i = 0; i < MOTORCOUNT; i++)
    {
        config = (pcnt_config_t){
            .pulse_gpio_num = Motors[i][SENSEPINS][0],
            .ctrl_gpio_num = Motors[i][SENSEPINS][1],
            .lctrl_mode = PCNT_MODE_KEEP,
            .hctrl_mode = PCNT_MODE_REVERSE,
            .pos_mode = PCNT_COUNT_INC,
            .neg_mode = PCNT_COUNT_DIS,
            .counter_h_lim = PULSES_PER_MOTOR_ROTATION,
            .counter_l_lim = -PULSES_PER_MOTOR_ROTATION,
            .unit = i,
            .channel = PCNT_CHANNEL_0,
        };

        pcnt_unit_config(&config);

        pcnt_counter_pause(i);
        pcnt_counter_clear(i);

        pcnt_filter_disable(i);

        //pcnt_set_event_value(i, PCNT_EVT_)

        pcnt_event_enable(i, PCNT_EVT_H_LIM);
        pcnt_event_enable(i, PCNT_EVT_L_LIM);
    }

    pcnt_isr_service_install(0);

    for (int i = 0; i < MOTORCOUNT; i++)
    {
        pcnt_isr_handler_add(i, event, (void *)i);
        pcnt_counter_resume(i);
    }
}

void initPWM()
{
    for (int i = 0; i < MOTORCOUNT; i++)
    {
        mcpwm_unit_t unit = i / 2;
        int subUnit = i % 2;
        mcpwm_io_signals_t signalLeft = subUnit * 2;
        mcpwm_io_signals_t signalRight = subUnit * 2 + 1;

        mcpwm_gpio_init(unit, signalLeft, Motors[i][MOTORPINS][0]);
        mcpwm_gpio_init(unit, signalRight, Motors[i][MOTORPINS][1]);
    }

    mcpwm_config_t config;
    for (mcpwm_unit_t unit = 0; unit < MCPWM_UNIT_MAX; unit++)
    {
        for (mcpwm_timer_t timer = 0; timer < 2; timer++)
        {
            config = (mcpwm_config_t){
                .frequency = 50,
                .cmpr_a = 0,
                .cmpr_b = 0,
                .duty_mode = MCPWM_DUTY_MODE_0,
                .counter_mode = MCPWM_UP_COUNTER,
            };

            mcpwm_init(unit, timer, &config);
        }
    }
}

void app_main(void)
{
    initPCNT();
    initPWM();

    xTaskCreatePinnedToCore(
        TaskCore2,
        "CoreTwo",
        10000,
        NULL,
        2,
        NULL,
        1);

    xTaskCreatePinnedToCore(
        TaskCore1,
        "CoreOne",
        10000,
        NULL,
        2,
        NULL,
        0);
}

void writePwmSpeed()
{
    for (int i = 0; i < MOTORCOUNT; i++)
    {
        mcpwm_unit_t unit = i / 2;
        mcpwm_timer_t timer = i % 2;

        mcpwm_set_duty(unit, timer, MCPWM_GEN_A, currentPwmSpeed[i] > 0 ? currentPwmSpeed[i] : 0);
        mcpwm_set_duty(unit, timer, MCPWM_GEN_B, currentPwmSpeed[i] < 0 ? currentPwmSpeed[i] : 0);
    }
}

void TaskCore1(void *parameters)
{
    printf("Hello\n");
    fflush(stdout);

    while (true)
    {
        int64_t time = esp_timer_get_time();
        for (int i = 0; i < MOTORCOUNT; i++)
        {
            if (time - lastTime[i] > 500000)
            {
                measuredspeed[i] = 0;
            }
        }

        writePwmSpeed();
    }
}

void TaskCore2(void *parameters)
{
    while (true)
    {
        for (int i = 0; i < MOTORCOUNT; i++)
        {
            printf("M%d = %f\n", i, measuredspeed[i]);
        }
        fflush(stdout);
        vTaskDelay(100);
    }
}
