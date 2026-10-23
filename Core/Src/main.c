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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { ST_IDLE, ST_JOG, ST_MOVING, ST_SETTLE, ST_COAST } run_state_t;
typedef enum { EVT_NONE, EVT_ARRIVED, EVT_STALL, EVT_ABORT } run_evt_t;

typedef struct {                 /* one editable number */
  const char *label;
  const char *unit;
  uint8_t  nd;                   /* number of digits shown while editing */
  uint8_t  dec;                  /* how many of them are after the decimal point */
  uint32_t val;                  /* value without the decimal point (30.00 mm -> 3000) */
  uint32_t minv;                 /* smallest value accepted when ENTER is pressed */
} field_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IN1_HIGH()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define IN1_LOW()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define IN2_HIGH()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define IN2_LOW()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)

#define MOTOR_TO_ZERO()  do { IN1_HIGH(); IN2_LOW();  } while (0)
#define MOTOR_TO_TOP()   do { IN1_LOW();  IN2_HIGH(); } while (0)
#define MOTOR_STOP()     do { IN1_LOW();  IN2_LOW();  } while (0)

#define COUNTS_PER_MM      780.6
#define MAX_COUNTS         61667     /* largest single dispense accepted */
#define MAX_TRAVEL_MM      75.0      /* above this -> OUT RANGE */
#define ERR_TOL            300       /* result is "ERR" if |actual - theory| is larger */
#define SHORT_MOVE_COUNTS  781       /* ~1 mm: shorter moves use brake offset 0 */
#define STILL_MS           500       /* stall time */
#define STILL_TOL          2
#define SETTLE_MS          300       /* wait after stopping before reading final count */
#define POS()              (-(int32_t)__HAL_TIM_GET_COUNTER(&htim2))  /* + toward top */

#define JOG_FWD   1
#define JOG_REV   2
#define JOG_STOP  3

#define B_UP      0x01
#define B_DOWN    0x02
#define B_LEFT    0x04
#define B_RIGHT   0x08
#define B_ENTER   0x10
#define DEBOUNCE_MS  20

#define PI_VAL    3.14159265358979
#define VALX      70                 /* x position of the values on the screen */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
/* ---- motion state machine (runs in the 1 kHz TIM6 interrupt) ---- */
static volatile run_state_t state = ST_IDLE;
static volatile run_evt_t   evt = EVT_NONE;
static volatile int32_t  evt_a = 0, evt_b = 0;
static volatile uint8_t  req_jog = 0, req_disp = 0, aborted = 0;
static volatile int32_t  req_delta = 0;
static volatile int32_t  target = 0, move_start = 0, brake_eff = 0;
static volatile int32_t  brake_offset = 495;   /* counts, used for moves >= 1 mm */
static volatile uint16_t still_ms = 0;
static int32_t  last_count = 0;

/* ---- buttons (debounced in the same interrupt) ---- */
static volatile uint8_t btn_evt = 0;           /* press events, cleared by main loop */
static volatile uint8_t btn_down = 0;          /* buttons currently held */
static volatile uint8_t jog_page = 0;          /* 1 while the JOG screen is shown */

/* ---- user interface ---- */
enum { F_SDIA, F_RBIG, F_RSML, F_VOL };
static field_t fld[4] = {
  { "SMALL DIA", "MM", 5, 2, 3000, 1 },        /* 30.00 mm */
  { "RATIO BIG", "",   2, 0,    1, 1 },        /* big : small = 1 : 1 */
  { "RATIO SML", "",   2, 0,    1, 1 },
  { "VOLUME",    "ML", 5, 2,  500, 1 },        /* 5.00 ml */
};
static uint8_t  page = 0;                      /* 0 SETUP, 1 DISPENSE, 2 JOG */
static uint8_t  sel[3] = {0, 0, 0};            /* selected row on every page */
static uint8_t  editing = 0, cursor = 0;       /* digit editing */
static volatile uint8_t ui_busy = 0;           /* a dispense is running */
static uint8_t  dirty = 1;                     /* screen needs a redraw */
static uint32_t last_draw = 0, busy_t0 = 0;
static char     status[24] = "READY";
static uint8_t  status_bad = 0;
static uint8_t  info_mode = 0;                 /* 0 = ACT/TOTAL lines, 1 = range error lines */
static int32_t  pend_theory = 0;
static double   area = 0.0;                    /* mm^2 of both bottles, set when dispense starts */
static uint32_t last_ul = 0, total_ul = 0;     /* microlitres (1 ml = 1000) */
static uint8_t  have_last = 0;
static uint32_t range_max_hund = 0, range_need_x10 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ====================== number formatting (no float printf) ====================== */
static const uint32_t P10[] = {1u, 10u, 100u, 1000u, 10000u, 100000u, 1000000u, 10000000u, 100000000u, 1000000000u};

/* zero padded: nd digits, decimal point before the last 'dec' digits ("030.00") */
static uint8_t fmt_pad(char *o, uint32_t v, uint8_t nd, uint8_t dec)
{
  char tmp[10];
  for (int i = nd - 1; i >= 0; i--) { tmp[i] = (char)('0' + (v % 10)); v /= 10; }
  uint8_t n = 0;
  for (uint8_t i = 0; i < nd; i++)
  {
    if (dec && i == nd - dec) o[n++] = '.';
    o[n++] = tmp[i];
  }
  o[n] = 0;
  return n;
}

/* plain number with 'dec' decimals, no leading zeros ("-12.34") */
static uint8_t fmt_dec(char *o, int32_t v, uint8_t dec)
{
  uint8_t n = 0;
  if (v < 0) { o[n++] = '-'; v = -v; }
  uint32_t ip = (uint32_t)v / P10[dec], fp = (uint32_t)v % P10[dec];
  char t[11];
  uint8_t k = 0;
  do { t[k++] = (char)('0' + (ip % 10)); ip /= 10; } while (ip);
  while (k) o[n++] = t[--k];
  if (dec)
  {
    o[n++] = '.';
    for (int i = dec - 1; i >= 0; i--) o[n++] = (char)('0' + ((fp / P10[i]) % 10));
  }
  o[n] = 0;
  return n;
}

/* ====================== buttons: called from the 1 kHz interrupt ====================== */
static uint8_t btn_raw(void)
{
  uint8_t m = 0;
  if (HAL_GPIO_ReadPin(BTN_UP_GPIO_Port,    BTN_UP_Pin)    == GPIO_PIN_RESET) m |= B_UP;
  if (HAL_GPIO_ReadPin(BTN_DOWN_GPIO_Port,  BTN_DOWN_Pin)  == GPIO_PIN_RESET) m |= B_DOWN;
  if (HAL_GPIO_ReadPin(BTN_LEFT_GPIO_Port,  BTN_LEFT_Pin)  == GPIO_PIN_RESET) m |= B_LEFT;
  if (HAL_GPIO_ReadPin(BTN_RIGHT_GPIO_Port, BTN_RIGHT_Pin) == GPIO_PIN_RESET) m |= B_RIGHT;
  if (HAL_GPIO_ReadPin(BTN_ENTER_GPIO_Port, BTN_ENTER_Pin) == GPIO_PIN_RESET) m |= B_ENTER;
  return m;
}

static void jog_update(void)         /* UP = forward, DOWN = retreat, release = stop */
{
  uint8_t d = btn_down;
  if ((d & B_UP) && !(d & B_DOWN))        req_jog = JOG_FWD;
  else if ((d & B_DOWN) && !(d & B_UP))   req_jog = JOG_REV;
  else                                    req_jog = JOG_STOP;
}

static void buttons_scan(void)
{
  static uint8_t cnt[5];
  uint8_t raw = btn_raw();
  for (uint8_t i = 0; i < 5; i++)
  {
    uint8_t bit = (uint8_t)(1u << i);
    if (((raw & bit) != 0) == ((btn_down & bit) != 0)) { cnt[i] = 0; continue; }
    if (++cnt[i] < DEBOUNCE_MS) continue;
    cnt[i] = 0;
    btn_down ^= bit;                                   /* debounced state changed */
    if (btn_down & bit)                                /* ---- pressed ---- */
    {
      btn_evt |= bit;
      if (state == ST_MOVING) req_jog = JOG_STOP;      /* any key aborts a running dispense */
      else if (jog_page && (bit & (B_UP | B_DOWN))) jog_update();
    }
    else if (jog_page && (bit & (B_UP | B_DOWN)))      /* ---- released ---- */
    {
      jog_update();
    }
  }
}

static uint8_t btn_take(void)        /* main loop: fetch and clear press events */
{
  __disable_irq();
  uint8_t e = btn_evt;
  btn_evt = 0;
  __enable_irq();
  return e;
}

/* ====================== maths (all on the STM32) ====================== */
static uint32_t rsml_safe(void) { return fld[F_RSML].val ? fld[F_RSML].val : 1; }

/* area of both bottles: pi/4 * (d_small^2 + d_big^2), d_big = d_small * ratio_big / ratio_small */
static double calc_area(void)
{
  double ds = fld[F_SDIA].val / 100.0;
  double db = ds * fld[F_RBIG].val / rsml_safe();
  return (PI_VAL / 4.0) * (ds * ds + db * db);
}

static void set_status(const char *s, uint8_t bad)
{
  strncpy(status, s, sizeof(status) - 1);
  status[sizeof(status) - 1] = 0;
  status_bad = bad;
}

static void do_dispense(void)
{
  info_mode = 0;
  if (state != ST_IDLE) { set_status("BUSY", 1); return; }

  area = calc_area();
  if (area < 0.001) { set_status("BAD DIA", 1); return; }

  double  h = (fld[F_VOL].val / 100.0) * 1000.0 / area;      /* V = area * h  ->  travel in mm */
  int32_t d = (int32_t)(h * COUNTS_PER_MM + 0.5);

  if (d < 1) { set_status("TOO SMALL", 1); return; }
  if (h > MAX_TRAVEL_MM || d > MAX_COUNTS)
  {
    range_max_hund = (uint32_t)(MAX_TRAVEL_MM * area / 10.0 + 0.5);   /* max ml x 100 */
    range_need_x10 = (uint32_t)(h * 10.0 + 0.5);                       /* needed mm x 10 */
    info_mode = 1;
    set_status("OUT RANGE", 1);
    return;
  }

  pend_theory = d;
  busy_t0 = HAL_GetTick();
  ui_busy = 1;
  set_status("MOVING", 0);
  req_delta = d;                       /* hand over to the interrupt */
  req_disp = 1;
}

static void process_event(void)       /* result of a move, produced by the interrupt */
{
  if (evt == EVT_NONE) return;
  run_evt_t e = evt;
  int32_t a = evt_a, b = evt_b;
  evt = EVT_NONE;

  int32_t actual = (e == EVT_STALL) ? (a - move_start) : (b - move_start);
  int32_t error  = actual - pend_theory;
  if (actual < 0) actual = 0;
  uint32_t ul = (uint32_t)(actual / COUNTS_PER_MM * area + 0.5);   /* mm^3 = microlitres */

  if (e == EVT_STALL)            set_status("STALLED", 1);
  else if (e == EVT_ABORT)       set_status("ABORTED", 1);
  else if (labs((long)error) > ERR_TOL)
  {
    snprintf(status, sizeof(status), "ERR %c%ld", error < 0 ? '-' : '+', (long)labs((long)error));
    status_bad = 1;
  }
  else                           set_status("REACHED", 0);

  last_ul = ul;
  have_last = 1;
  total_ul += ul;
  ui_busy = 0;
  info_mode = 0;
  dirty = 1;
}

static void ui_watchdog(void)         /* never stay "busy" forever */
{
  if (!ui_busy) return;
  uint32_t t = HAL_GetTick() - busy_t0;
  if (t > 50 && state == ST_IDLE && !req_disp && evt == EVT_NONE)      /* request was dropped (stop key) */
  {
    ui_busy = 0; set_status("ABORTED", 1); dirty = 1;
  }
  else if (t > 90000)                                                  /* no result after 90 s */
  {
    req_jog = JOG_STOP; ui_busy = 0; set_status("NO RESULT", 1); dirty = 1;
  }
}

/* ====================== drawing ====================== */
static void draw_header(const char *title)        /* top 16 rows = yellow band */
{
  oled_text(2, 1, title, 2, 1);
  for (uint8_t i = 0; i < 3; i++)                 /* page dots, filled = current page */
  {
    if (i == page) oled_rect(106 + 7 * i, 5, 4, 6, 1);
    else           oled_frame(106 + 7 * i, 5, 4, 6, 1);
  }
}

static void draw_field_row(int y, uint8_t fi, uint8_t selected)
{
  field_t *f = &fld[fi];
  char v[12];
  uint8_t n = fmt_pad(v, f->val, f->nd, f->dec);
  uint8_t edit_here = editing && selected;
  uint8_t inv = selected && !editing;
  uint8_t col = inv ? 0 : 1;

  if (inv) oled_rect(0, y, 128, 12, 1);
  oled_text(4, y + 2, f->label, 1, col);
  oled_text(VALX, y + 2, v, 1, col);
  if (f->unit[0]) oled_text(VALX + 6 * n + 4, y + 2, f->unit, 1, col);
  if (edit_here)
  {
    uint8_t ci = cursor + ((f->dec && cursor >= f->nd - f->dec) ? 1 : 0);   /* skip the '.' */
    char one[2] = { v[ci], 0 };
    oled_frame(0, y, 128, 12, 1);                                           /* row frame = edit mode */
    oled_text_inv(VALX + 6 * ci, y + 2, one, 1);                            /* digit being edited */
  }
}

static void draw_setup(void)
{
  for (uint8_t i = 0; i < 3; i++) draw_field_row(16 + 12 * i, i, sel[0] == i);

  char v[14];                                       /* big bottle diameter (calculated) */
  uint32_t bd = (uint32_t)(fld[F_SDIA].val * (double)fld[F_RBIG].val / rsml_safe() + 0.5);
  uint8_t n = fmt_dec(v, (int32_t)bd, 2);
  oled_text(4, 54, "BIG DIA", 1, 1);
  oled_text(VALX, 54, v, 1, 1);
  oled_text(VALX + 6 * n + 4, 54, "MM", 1, 1);
}

static void draw_dispense(void)
{
  char buf[24], num[14];

  draw_field_row(16, F_VOL, sel[1] == 0);

  const char *bl = ui_busy ? "STOP" : "DISPENSE";
  int w = oled_text_width(bl, 1) + 8;
  if (ui_busy || (sel[1] == 1 && !editing)) { oled_rect(3, 28, w, 11, 1); oled_text(7, 30, bl, 1, 0); }
  else                                      { oled_frame(3, 28, w, 11, 1); oled_text(7, 30, bl, 1, 1); }
  if (status_bad) oled_text_inv(65, 30, status, 1);
  else            oled_text(65, 30, status, 1, 1);

  if (ui_busy)                                      /* progress bar */
  {
    int32_t span = target - move_start, done = POS() - move_start;
    int fill = (span > 0) ? (int)(done * 118.0 / span) : 0;
    if (fill < 0) fill = 0;
    if (fill > 118) fill = 118;
    oled_frame(4, 42, 120, 8, 1);
    oled_rect(5, 43, fill, 6, 1);
  }
  else if (info_mode == 1)
  {
    fmt_dec(num, (int32_t)range_max_hund, 2);
    snprintf(buf, sizeof(buf), "MAX %s ML", num);
    oled_text(4, 42, buf, 1, 1);
  }
  else if (have_last)
  {
    fmt_dec(num, (int32_t)last_ul, 3);
    snprintf(buf, sizeof(buf), "ACT %s ML", num);
    oled_text(4, 42, buf, 1, 1);
  }
  else oled_text(4, 42, "ACT --- ML", 1, 1);

  if (info_mode == 1 && !ui_busy)
  {
    fmt_dec(num, (int32_t)range_need_x10, 1);
    snprintf(buf, sizeof(buf), "NEED %s MM", num);
  }
  else
  {
    fmt_dec(num, (int32_t)total_ul, 3);
    snprintf(buf, sizeof(buf), "TOTAL %s ML", num);
  }
  oled_text(4, 54, buf, 1, 1);
}

static void draw_jog(void)
{
  uint8_t d = btn_down;
  if (d & B_UP)   oled_text_inv(4, 18, "UP   : FORWARD", 1); else oled_text(4, 18, "UP   : FORWARD", 1, 1);
  if (d & B_DOWN) oled_text_inv(4, 29, "DOWN : RETREAT", 1); else oled_text(4, 29, "DOWN : RETREAT", 1, 1);

  double mm100 = (double)POS() * 100.0 / COUNTS_PER_MM;            /* position in mm x 100 */
  int32_t v = (int32_t)(mm100 >= 0 ? mm100 + 0.5 : mm100 - 0.5);
  char buf[14];
  uint8_t n = fmt_dec(buf, v, 2);
  oled_text(4, 41, buf, 2, 1);
  oled_text(4 + 12 * n + 4, 48, "MM", 1, 1);
  oled_text(4, 56, "ENTER: ZERO POS", 1, 1);
}

static void draw_ui(void)
{
  static const char *titles[3] = { "SETUP", "DISPENSE", "JOG" };
  oled_clear();
  draw_header(titles[page]);
  if (page == 0)      draw_setup();
  else if (page == 1) draw_dispense();
  else                draw_jog();
  oled_flush();
}

static void draw_splash(void)
{
  oled_clear();
  oled_text(16, 1, "ADHESIVE", 2, 1);
  oled_text(10, 24, "DISPENSER", 2, 1);
  oled_text(19, 50, "LINEAR ACTUATOR", 1, 1);
  oled_flush();
}

/* ====================== button handling (menu logic) ====================== */
static field_t *cur_field(void) { return (page == 0) ? &fld[sel[0]] : &fld[F_VOL]; }

static void change_page(int dir)
{
  if (page == 2) req_jog = JOG_STOP;               /* never leave the actuator running */
  page = (uint8_t)((page + 3 + dir) % 3);
  jog_page = (page == 2);
}

static void ui_key(uint8_t b)
{
  if (ui_busy) return;                             /* (the interrupt already aborted the move) */
  dirty = 1;

  if (editing)                                     /* ---- editing a number ---- */
  {
    field_t *f = cur_field();
    uint32_t place = P10[f->nd - 1 - cursor];
    uint32_t dg = (f->val / place) % 10;
    switch (b)
    {
      case B_LEFT:  if (cursor > 0)         cursor--; break;
      case B_RIGHT: if (cursor < f->nd - 1) cursor++; break;
      case B_UP:    f->val = f->val - dg * place + ((dg + 1) % 10) * place; break;
      case B_DOWN:  f->val = f->val - dg * place + ((dg + 9) % 10) * place; break;
      case B_ENTER: if (f->val < f->minv) f->val = f->minv; editing = 0; break;
    }
    return;
  }

  switch (b)                                       /* ---- browsing ---- */
  {
    case B_LEFT:  change_page(-1); break;
    case B_RIGHT: change_page(+1); break;
    case B_UP:
    case B_DOWN:
    {
      uint8_t n = (page == 0) ? 3 : (page == 1) ? 2 : 0;
      if (n) sel[page] = (b == B_UP) ? (uint8_t)((sel[page] + n - 1) % n) : (uint8_t)((sel[page] + 1) % n);
      break;
    }
    case B_ENTER:
      if (page == 0 || (page == 1 && sel[1] == 0)) { editing = 1; cursor = 0; set_status("READY", 0); info_mode = 0; }
      else if (page == 1)                          do_dispense();
      else if (state == ST_IDLE)                   __HAL_TIM_SET_COUNTER(&htim2, 0);   /* zero the position display */
      break;
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
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  MOTOR_STOP();
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim2, 0);          /* power-on position = 0 */
  oled_init(&hi2c1);
  draw_splash();
  HAL_Delay(1500);
  HAL_TIM_Base_Start_IT(&htim6);             /* 1 kHz: buttons + motion state machine */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    process_event();

	    uint8_t be = btn_take();
	    for (uint8_t i = 0; i < 5; i++)
	      if (be & (1u << i)) ui_key((uint8_t)(1u << i));

	    ui_watchdog();

	    uint32_t now = HAL_GetTick();
	    if (dirty || ((ui_busy || page == 2) && (now - last_draw) >= 100))   /* live progress / position */
	    {
	      dirty = 0;
	      last_draw = now;
	      draw_ui();
	    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_UP_Pin BTN_DOWN_Pin BTN_LEFT_Pin BTN_RIGHT_Pin
                           BTN_ENTER_Pin */
  GPIO_InitStruct.Pin = BTN_UP_Pin|BTN_DOWN_Pin|BTN_LEFT_Pin|BTN_RIGHT_Pin
                          |BTN_ENTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void start_move(int32_t tgt, int32_t p)
{
  target = tgt;
  move_start = p;
  last_count = p;
  still_ms = 0;
  aborted = 0;
  brake_eff = ((tgt - p) >= SHORT_MOVE_COUNTS) ? brake_offset : 0;  /* <1 mm: no offset */
  if ((tgt - brake_eff) <= p) { state = ST_SETTLE; return; }
  MOTOR_TO_TOP();
  state = ST_MOVING;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)      /* B1 on the board = emergency stop */
{
  static uint32_t last_press = 0;
  if (GPIO_Pin == GPIO_PIN_13)
  {
    uint32_t now = HAL_GetTick();
    if ((now - last_press) > 300) { last_press = now; req_jog = JOG_STOP; }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   /* 1 kHz */
{
  if (htim->Instance != TIM6) return;
  int32_t p = POS();

  buttons_scan();

  if (req_jog)                       /* F/R/S win over everything, in every state */
  {
    uint8_t j = req_jog;
    req_jog = 0;
    req_disp = 0;                    /* drop any dispense that was queued */
    if (j == JOG_FWD)      { MOTOR_TO_TOP();  state = ST_JOG; }
    else if (j == JOG_REV) { MOTOR_TO_ZERO(); state = ST_JOG; }
    else
    {
      MOTOR_STOP();
      last_count = p;
      still_ms = 0;
      if (state == ST_MOVING)      { aborted = 1; state = ST_SETTLE; }   /* report how far it got */
      else if (state != ST_SETTLE) { state = ST_COAST; }
    }
    return;
  }

  switch (state)
  {
    case ST_IDLE:
      if (req_disp) { req_disp = 0; start_move(p + req_delta, p); }  /* relative to NOW */
      break;

    case ST_JOG:                     /* no stall check: runs until released */
      break;

    case ST_COAST:                   /* wait for the coast to end before the next move */
      if (abs(p - last_count) <= STILL_TOL) still_ms++; else { still_ms = 0; last_count = p; }
      if (still_ms >= SETTLE_MS) state = ST_IDLE;
      break;

    case ST_MOVING:
      if (p >= target - brake_eff)
      {
        MOTOR_STOP();
        last_count = p;
        still_ms = 0;
        state = ST_SETTLE;
      }
      else
      {
        if (abs(p - last_count) <= STILL_TOL) still_ms++; else { still_ms = 0; last_count = p; }
        if (still_ms >= STILL_MS)
        {
          MOTOR_STOP();
          evt_a = p;
          evt = EVT_STALL;
          state = ST_IDLE;
        }
      }
      break;

    case ST_SETTLE:
      if (abs(p - last_count) <= STILL_TOL) still_ms++; else { still_ms = 0; last_count = p; }
      if (still_ms >= SETTLE_MS)
      {
        evt_a = target;
        evt_b = p;
        evt = aborted ? EVT_ABORT : EVT_ARRIVED;
        aborted = 0;
        state = ST_IDLE;
      }
      break;
  }
}
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
