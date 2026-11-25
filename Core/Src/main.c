/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "ctype.h"
#include "math.h"
#include "MPU6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t check_status;
uint8_t calib_status = 3;   /* 0 ok, 1 I2C error, 2 moved, 3 not run */
int32_t encoder1_count = 0;
int32_t encoder2_count = 0;

int16_t encoder1_delta = 0;
int16_t encoder2_delta = 0;

int16_t encoder1_previous = 0;
int16_t encoder2_previous = 0;
float rpm1 = 0.0f;
float rpm2 = 0.0f;

/* ---------------- Timing ----------------
 * TIM4 runs at 40 kHz (25 us). It does 3 jobs:
 *   1) software PWM for L298N IN1..IN4  (40 steps -> 1 kHz PWM, 2.5 % resolution)
 *   2) 200 Hz tick  -> angle loop   (every 200 TIM4 ticks = 5 ms)
 *   3) 50  Hz tick  -> speed loop    (every 4th angle tick = 20 ms)
 *   4) 10  Hz tick  -> position loop (every 5th speed tick = 100 ms)
 * The ticks come from the hardware timer, NOT from HAL_GetTick/UART, so UART
 * traffic can never shift the loop period. The heavy work (I2C, PID, printf) is
 * done in main loop, UART uses interrupts + ring buffer so it never blocks.
 */
#define PWM_STEPS        100u      /* TIM4 ticks per PWM period            */
#define TIM4_ARR         9u       /* (71+1)*(9+1)/72MHz = 10 us -> 100 kHz */
#define TICKS_PER_CTRL   500u     /* 500 * 10 us = 5 ms  -> 200 Hz        */
#define VEL_DIV          4u       /* 200 Hz / 4  = 50 Hz                  */
#define POS_DIV          5u       /* 50 Hz  / 5  = 10 Hz                  */
#define ANGLE_DT         0.005f
#define VEL_DT           0.020f
#define POS_DT           0.100f
#define ENCODER_TO_CM 0.00827f
volatile uint8_t  pwm_counter = 0;
volatile uint8_t  motor1_thr = 0;     /* on-time in TIM4 ticks (0..PWM_STEPS) */
volatile uint8_t  motor2_thr = 0;
volatile int8_t   motor1_dir = 0;
volatile int8_t   motor2_dir = 0;

static uint16_t   sched_cnt = 0;
static uint8_t    vel_div = 0;
static uint8_t    pos_div = 0;
volatile uint8_t  tick_flags = 0;     /* bit0 = 200 Hz angle, bit1 = 50 Hz speed, bit2 = 10 Hz position */
volatile uint32_t ctrl_overrun = 0;   /* 200 Hz step not finished in time      */
volatile uint32_t tick_count = 0;
uint32_t step_us = 0, step_us_max = 0;

/* ---------------- Motor ---------------- */
#define MOTOR1_SIGN      (1)          /* flip to -1 if a wheel spins backwards */
#define MOTOR2_SIGN      (-1)
float motor_dz = 0.0f;                /* dead-zone compensation, % (tune: dz=) */

/* ---------------- PID ---------------- */
typedef struct {
    float kp, ki, kd;
    float integ;      /* already multiplied by ki */
    float i_max;
    float out_max;
} PID_t;

/* inner: angle (deg) -> motor % ; derivative uses gyro rate directly */
PID_t pid_ang = { 30.0f, 0.0f, 0.185f, 0.0f, 40.0f, 100.0f };
/* Cascade (position loop 10 Hz, speed loop 50 Hz, angle loop 200 Hz):
 *   position error (cm) --P--> speed setpoint (cm/s, clamped to vel_max)
 *   speed error (cm/s)  --PI-> angle setpoint (deg, clamped to ANGLE_SP_MAX)
 * The speed PI integrator also learns the true balance point, which removes the drift. */
#define ANGLE_SP_MAX     2.6f
float pos_kp  = 1.0f;                 /* 1/s : speed setpoint per cm of error (tune: pp=) */
float vel_max = 10.0f;                /* cm/s: max cruise speed toward target (tune: vm=) */
float vel_sp  = 0.0f;
float vel_now = 0.0f;                 /* low-pass filtered speed, cm/s */
#define VEL_LPF          0.3f        /* 0..1, higher = less filtering */
PID_t pid_vel = { 0.30f, 0.0f, 0.0f, 0.0f, ANGLE_SP_MAX, ANGLE_SP_MAX };

float angle_trim = 0.0f;              /* mechanical balance point offset (deg) */
float angle_balance=0.0f;
float angle_sp   = 0.0f;
float angle_now  = 0.0f;
float last_angle  = 0.0f;
float pos_now    = 0.0f;
float pos_target = 0.0f;
float pos_prev   = 0.0f;
float motor_out  = 0.0f;

#define FALL_ANGLE       35.0f        /* motors off beyond this               */
#define REARM_ANGLE      3.0f         /* re-enable when back within this      */
uint8_t ctrl_enable = 1;
uint8_t armed = 0;
uint8_t tel_enable = 1;
uint8_t tel_due = 0;

/* ---------------- UART2 (non-blocking) ---------------- */
#define TX_BUF_SIZE      512u
#define CMD_LEN          48u
static uint8_t  tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0, tx_tail = 0, tx_chunk = 0;
static volatile uint8_t  tx_busy = 0;
volatile uint32_t tx_dropped = 0;

static uint8_t  rx_byte;
static char     rx_line[CMD_LEN];
static uint8_t  rx_idx = 0;
static char     cmd_buf[CMD_LEN];
static volatile uint8_t cmd_ready = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ======================= Motor (L298N, soft PWM on IN1..IN4) ======================= */
/* PB12/PB13 = motor1 IN1/IN2, PB14/PB15 = motor2 IN3/IN4. ENA/ENB tied high. */
void Motor_Set(int16_t motor1, int16_t motor2)
{
    if (motor1 > 100) motor1 = 100;
    if (motor1 < -100) motor1 = -100;
    if (motor2 > 100) motor2 = 100;
    if (motor2 < -100) motor2 = -100;

    uint8_t t1 = (uint8_t)(((motor1 >= 0) ? motor1 : -motor1) * PWM_STEPS / 100);
    uint8_t t2 = (uint8_t)(((motor2 >= 0) ? motor2 : -motor2) * PWM_STEPS / 100);

    /* thresholds first, then direction (ISR reads both) */
    motor1_thr = t1;
    motor2_thr = t2;
    motor1_dir = (motor1 > 0) ? 1 : (motor1 < 0) ? -1 : 0;
    motor2_dir = (motor2 > 0) ? 1 : (motor2 < 0) ? -1 : 0;
}

static int16_t Apply_Deadzone(float u)
{
    if (u > -0.5f && u < 0.5f) return 0;
    float a = fabsf(u);
    a = motor_dz + a * (100.0f - motor_dz) / 100.0f;
    if (a > 100.0f) a = 100.0f;
    return (u > 0.0f) ? (int16_t)a : -(int16_t)a;
}

void Read_Encoder(void)
{
    int16_t current1 = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t current2 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

    // Count difference, int16 arithmetic handles counter overflow
    encoder1_delta = (int16_t)(current1 - encoder1_previous);
    encoder2_delta = (int16_t)(current2 - encoder2_previous);

    encoder1_previous = current1;
    encoder2_previous = current2;

    encoder1_count += encoder1_delta;
    encoder2_count -= encoder2_delta;
}
#define ENCODER_CPR  1.0f  // Replace with real CPR

void Calculate_RPM(void)
{
    rpm1 = (encoder1_delta * 60.0f) / (ENCODER_CPR * ANGLE_DT);
    rpm2 = (encoder2_delta * 60.0f) / (ENCODER_CPR * ANGLE_DT);
}

/* ======================= PID ======================= */
/* out = kp*err + I - kd*meas_dot   (derivative on measurement, no kick on setpoint change) */
static float PID_Update(PID_t *p, float err, float meas_dot, float dt)
{
    p->integ += p->ki * err * dt;
    if (p->integ >  p->i_max) p->integ =  p->i_max;
    if (p->integ < -p->i_max) p->integ = -p->i_max;

    float out = p->kp * err + p->integ - p->kd * meas_dot;
    if (out >  p->out_max) out =  p->out_max;
    if (out < -p->out_max) out = -p->out_max;
    return out;
}

/* ======================= UART2 non-blocking TX / RX ======================= */
static void Uart_Kick(void)   /* start next TX chunk if idle; call with USART2 IRQ masked or from its ISR */
{
    if (tx_busy || tx_head == tx_tail) return;
    uint16_t chunk = (tx_head > tx_tail) ? (uint16_t)(tx_head - tx_tail)
                                         : (uint16_t)(TX_BUF_SIZE - tx_tail);
    tx_chunk = chunk;
    tx_busy = 1;
    if (HAL_UART_Transmit_IT(&huart2, &tx_buf[tx_tail], chunk) != HAL_OK)
        tx_busy = 0;
}

static void Uart_Write(const char *s, uint16_t len)   /* never blocks; drops if buffer full */
{
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    for (uint16_t i = 0; i < len; i++)
    {
        uint16_t next = (uint16_t)((tx_head + 1u) % TX_BUF_SIZE);
        if (next == tx_tail) { tx_dropped += (uint32_t)(len - i); break; }
        tx_buf[tx_head] = (uint8_t)s[i];
        tx_head = next;
    }
    Uart_Kick();
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        tx_tail = (uint16_t)((tx_tail + tx_chunk) % TX_BUF_SIZE);
        tx_busy = 0;
        Uart_Kick();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        char c = (char)rx_byte;
        if (c == '\n' || c == '\r')
        {
            if (rx_idx > 0 && !cmd_ready)
            {
                rx_line[rx_idx] = 0;
                memcpy(cmd_buf, rx_line, rx_idx + 1u);
                cmd_ready = 1;
            }
            rx_idx = 0;
        }
        else if (rx_idx < CMD_LEN - 1u)
        {
            rx_line[rx_idx++] = c;
        }
        else
        {
            rx_idx = 0;   /* line too long -> discard */
        }
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);   /* recover from overrun/noise */
    }
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/* float -> text without needing printf %f (newlib-nano) */
static int Fmt_F(char *d, size_t n, float v, int dec)
{
    static const int32_t p10[] = {1, 10, 100, 1000, 10000, 100000};
    int32_t sc = p10[dec];
    int neg = (v < 0.0f);
    float a = neg ? -v : v;
    int32_t x = (int32_t)(a * (float)sc + 0.5f);
    if (dec == 0) return snprintf(d, n, "%s%ld", neg ? "-" : "", (long)x);
    return snprintf(d, n, "%s%ld.%0*ld", neg ? "-" : "", (long)(x / sc), dec, (long)(x % sc));
}

/* ======================= Tuning commands over UART2 =======================
 *  ap= ai= ad=   angle  PID  (inner, 200 Hz)
 *  pp= vm=       position P gain (1/s), max speed (cm/s)
 *  vp= vi=       speed PI (outer, 20 Hz)
 *  trim=         angle offset (deg)       tp= absolute position target (cm)
 *  mv=           relative move, e.g. mv=20 -> go 20 cm forward from here
 *  dz=           motor dead-zone (%)
 *  en=0/1        enable control           tel=0/1  telemetry on/off
 *  ?             print all parameters
 */
typedef struct { const char *key; float *ptr; float lo, hi; } Param_t;
static const Param_t params[] = {
    { "ap",   &pid_ang.kp, -500.0f, 500.0f },
    { "ai",   &pid_ang.ki, -500.0f, 500.0f },
    { "ad",   &pid_ang.kd, -500.0f, 500.0f },
    { "pp",   &pos_kp,        0.0f,  20.0f },
    { "vm",   &vel_max,       0.0f, 200.0f },
    { "vp",   &pid_vel.kp,  -10.0f,  10.0f },
    { "vi",   &pid_vel.ki,  -10.0f,  10.0f },
    { "trim", &angle_trim,  -30.0f,  30.0f },
    { "tp",   &pos_target, -1e7f,   1e7f   },
    { "dz",   &motor_dz,     0.0f,   60.0f },
};
#define N_PARAMS (sizeof(params) / sizeof(params[0]))

static void Print_Params(void)
{
    char b[220];
    int n = 0;
    for (unsigned i = 0; i < N_PARAMS && n < (int)sizeof(b) - 24; i++)
    {
        n += snprintf(b + n, sizeof(b) - n, "%s=", params[i].key);
        n += Fmt_F(b + n, sizeof(b) - n, *params[i].ptr, 4);
        n += snprintf(b + n, sizeof(b) - n, " ");
    }
    n += snprintf(b + n, sizeof(b) - n, "en=%u tel=%u\r\n", ctrl_enable, tel_enable);
    Uart_Write(b, (uint16_t)n);
}

static void Cmd_Process(void)
{
    if (!cmd_ready) return;

    char *line = cmd_buf;
    for (char *c = line; *c; c++) *c = (char)tolower((unsigned char)*c);

    if (line[0] == '?')
    {
        Print_Params();
        cmd_ready = 0;
        return;
    }

    char *sep = strchr(line, '=');
    if (!sep) sep = strchr(line, ' ');
    if (!sep) { Uart_Write("ERR fmt\r\n", 9); cmd_ready = 0; return; }
    *sep = 0;
    char *key = line;
    char *end;
    float val = strtof(sep + 1, &end);
    if (end == sep + 1) { Uart_Write("ERR val\r\n", 9); cmd_ready = 0; return; }

    char b[64];
    int n;
    uint8_t ok = 0;

    if (strcmp(key, "en") == 0)       { ctrl_enable = (val != 0.0f); ok = 1; }
    else if (strcmp(key, "tel") == 0) { tel_enable  = (val != 0.0f); ok = 1; }
    else if (strcmp(key, "mv") == 0)  { pos_target = pos_now + val; ok = 1; }
    else
    {
        for (unsigned i = 0; i < N_PARAMS; i++)
        {
            if (strcmp(key, params[i].key) == 0)
            {
                if (val < params[i].lo) val = params[i].lo;
                if (val > params[i].hi) val = params[i].hi;
                *params[i].ptr = val;
                if (key[1] == 'i') { pid_ang.integ = 0.0f; pid_vel.integ = 0.0f; }
                ok = 1;
                break;
            }
        }
    }

    if (ok)
    {
        n = snprintf(b, sizeof(b), "OK %s=", key);
        n += Fmt_F(b + n, sizeof(b) - n, val, 4);
        n += snprintf(b + n, sizeof(b) - n, "\r\n");
    }
    else
    {
        n = snprintf(b, sizeof(b), "ERR key\r\n");
    }
    Uart_Write(b, (uint16_t)n);
    cmd_ready = 0;
}

/* Telemetry CSV @20 Hz:  D,ms,angle,angle_sp,pos,pos_target,motor%,overruns,step_us_max,dropped */
static void Telemetry_Send(void)
{
    char b[120];
    int n = snprintf(b, sizeof(b), "D,%lu,", (unsigned long)HAL_GetTick());
    n += Fmt_F(b + n, sizeof(b) - n, angle_now, 2);  b[n++] = ',';
    n += Fmt_F(b + n, sizeof(b) - n, angle_sp, 2);   b[n++] = ',';
    n += Fmt_F(b + n, sizeof(b) - n, pos_now, 0);    b[n++] = ',';
    n += Fmt_F(b + n, sizeof(b) - n, pos_target, 0); b[n++] = ',';
    n += Fmt_F(b + n, sizeof(b) - n, motor_out, 1);
    n += snprintf(b + n, sizeof(b) - n, ",%lu,%lu,%lu\r\n",
                  (unsigned long)ctrl_overrun, (unsigned long)step_us_max,
                  (unsigned long)tx_dropped);
    Uart_Write(b, (uint16_t)n);
}

/* ======================= Control step (runs in main loop, paced by TIM4 flags) ======================= */
static void Control_Step(uint8_t run_vel, uint8_t run_pos)
{
    uint32_t t0 = DWT->CYCCNT;

    MPU6050_Read_All(&hi2c1, &MPU6050);
    angle_now = (float)MPU6050.AngleRoll - angle_trim;
//    float rate = (float)MPU6050.Gx;                 /* deg/s, d(pitch)/dt */
    float rate =  (angle_now-last_angle)/ANGLE_DT;
    last_angle=angle_now;

    Read_Encoder();
//    Calculate_RPM();
    pos_now = 0.5f * (float)(encoder1_count + encoder2_count)*ENCODER_TO_CM;

    float aabs = fabsf(angle_now);
    if (!ctrl_enable || aabs > FALL_ANGLE)
    {
        armed = 0;
    }
    else if (!armed && aabs < REARM_ANGLE)
    {
        armed = 1;                                   /* robot is upright: start balancing here */
        pid_ang.integ = 0.0f;
        pid_vel.integ = 0.0f;
        vel_now = 0.0f;
        vel_sp = 0.0f;
        pos_target = pos_now;
        pos_prev = pos_now;
        angle_sp =angle_balance;
    }

    if (!armed)
    {
        Motor_Set(0, 0);
        motor_out = 0.0f;
        pos_prev = pos_now;
        angle_sp = angle_balance;
    }
    else
    {
        /* ---- 10 Hz: position P -> speed setpoint (saturated = constant-speed cruise) ---- */
        if (run_pos)
        {
            vel_sp = pos_kp * (pos_target - pos_now);
            if (vel_sp >  vel_max) vel_sp =  vel_max;
            if (vel_sp < -vel_max) vel_sp = -vel_max;
        }

        /* ---- 50 Hz: measured speed (cm/s, low-pass) -> speed PI -> angle setpoint ---- */
        if (run_vel)
        {
            float vel_raw = (pos_now - pos_prev) / VEL_DT;
            pos_prev = pos_now;
            vel_now += VEL_LPF * (vel_raw - vel_now);
            angle_sp = PID_Update(&pid_vel, vel_sp - vel_now, 0.0f, VEL_DT) + angle_balance;
        }

        /* ---- inner loop 200 Hz: angle -> motor ---- */
        motor_out = PID_Update(&pid_ang, angle_sp - angle_now, rate, ANGLE_DT);
        int16_t m = Apply_Deadzone(motor_out);
        Motor_Set(MOTOR1_SIGN * m, MOTOR2_SIGN * m);
    }

    if (run_pos) tel_due = 1;

    step_us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000u);
    if (step_us > step_us_max) step_us_max = step_us;
}

void Soft_PWM_Tick(void)   /* called directly from TIM4_IRQHandler (no HAL overhead) */
{
    {
        /* ---- soft PWM ---- */
        if (++pwm_counter >= PWM_STEPS) pwm_counter = 0;

        uint32_t on = 0;
        if (pwm_counter < motor1_thr)
            on |= (motor1_dir > 0) ? GPIO_PIN_12 : (motor1_dir < 0) ? GPIO_PIN_13 : 0u;
        if (pwm_counter < motor2_thr)
            on |= (motor2_dir > 0) ? GPIO_PIN_14 : (motor2_dir < 0) ? GPIO_PIN_15 : 0u;
        /* one atomic write: set the active pins, reset the others (PB12..15) */
        GPIOB->BSRR = on | ((~on & 0xF000u) << 16);

        /* ---- 200 Hz / 20 Hz scheduler ---- */
        if (++sched_cnt >= TICKS_PER_CTRL)
        {
            sched_cnt = 0;
            tick_count++;
            if (tick_flags & 1u) ctrl_overrun++;     /* previous step still pending */
            tick_flags |= 1u;
            if (++vel_div >= VEL_DIV)
            {
                vel_div = 0;
                tick_flags |= 2u;                    /* 50 Hz speed tick */
                if (++pos_div >= POS_DIV)
                {
                    pos_div = 0;
                    tick_flags |= 4u;                /* 10 Hz position tick */
                }
            }
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* cycle counter: microsecond timing for the IMU filter and step-time measurement */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Encoders */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  encoder1_previous = 0;
  encoder2_previous = 0;

  /* MPU6050 */
  check_status = MPU6050_Init(&hi2c1);

  /* Gyro bias calibration: keep the robot still for ~1 s after power-up.
   * Retries if it was moving; after 5 failures it runs uncalibrated. */
  calib_status = 3;
  for (uint8_t try = 0; try < 5 && calib_status != 0; try++)
      calib_status = MPU6050_Calibrate(&hi2c1);

  /* UART2: interrupt driven TX (ring buffer) + RX line parser */
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);   /* below TIM4 (prio 0) */
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  Uart_Write("\r\nBalance robot ready. '?' = list params\r\n", 42);
  if (calib_status == 0) Uart_Write("IMU calib OK\r\n", 14);
  else                   Uart_Write("IMU calib FAILED (moved?)\r\n", 27);

  /* TIM4 -> 40 kHz (25 us). If you regenerate with CubeMX set TIM4 Period = 24 there. */
  __HAL_TIM_SET_AUTORELOAD(&htim4, TIM4_ARR);
  HAL_TIM_Base_Start_IT(&htim4);             /* start last: begins soft PWM + 200/20 Hz ticks */
  uint8_t count_now=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* take the pending tick flags atomically */
	  __disable_irq();
	  uint8_t flags = tick_flags;
	  tick_flags = 0;
	  __enable_irq();

	  if (flags & 1u)
	  {
		  /* 200 Hz angle loop; bit1 = 50 Hz speed loop, bit2 = 10 Hz position loop */
		  Control_Step((flags & 2u) ? 1u : 0u, (flags & 4u) ? 1u : 0u);
	  }
	  else
	  {
		  /* idle time between control ticks: serial work only */
		  Cmd_Process();
		  if (tel_due)
		  {
			  tel_due = 0;
        count_now++;
        if(count_now>=5)
        {
          count_now=0;
           if (tel_enable) Telemetry_Send();
        }
			 
		  }
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 49;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
