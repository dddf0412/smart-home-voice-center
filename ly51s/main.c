/* =====================================================================
 * LY-51S (STC89C52RC @ 12MHz) — 智能家居感知 + 执行主控
 * （新版板：XPT2046 SPI ADC，显示用 LCD1602）
 *
 * 引脚分配（全部飞线，按此表接线）：
 *   P0            LCD1602 数据 DB0~7
 *   P1.0          DHT11 数据
 *   P1.1          蜂鸣器（高电平响）
 *   P1.2~P1.5     步进电机 A/B/C/D 相
 *   P1.6          继电器（灯，高电平吸合）
 *   P2.2 / P2.3   超声波 TRIG / ECHO
 *   P2.4/2.5/2.6  LCD1602 RS / RW / E
 *   P3.0 / P3.1   UART RX / TX（9600，Timer2）
 *   P3.2~P3.5     XPT2046：CLK / CS / DIN / DOUT
 * ===================================================================== */
#include <reg52.h>
#include <intrins.h>

/* ---------- 引脚 ---------- */
#define LCD_DATA P0
sbit LCD_RS = P2^4;
sbit LCD_RW = P2^5;
sbit LCD_EN = P2^6;

sbit DHT11_PIN = P1^0;
sbit BUZZER    = P1^1;
sbit LAMP      = P1^6;

sbit MOTOR_A = P1^2;  /* 步进电机 A相 */
sbit MOTOR_B = P1^3;  /* B相 */
sbit MOTOR_C = P1^4;  /* C相 */
sbit MOTOR_D = P1^5;  /* D相 */

sbit TRIG = P2^2;
sbit ECHO = P2^3;

sbit XP_CLK  = P3^2;  /* XPT2046 时钟 */
sbit XP_CS   = P3^3;  /* XPT2046 片选 */
sbit XP_DIN  = P3^4;  /* XPT2046 数据输入 */
sbit XP_DOUT = P3^5;  /* XPT2046 数据输出 */

/* ---------- 可调参数 ---------- */
#define TEMP_ALARM_H  35
#define TEMP_ALARM_L  5
#define HUM_ALARM_H   85
#define HUM_ALARM_L   20
#define LIGHT_DARK    30
#define LIGHT_BRIGHT  70
#define FAN_TEMP_ON   30    /* 温度 >30 自动开风扇 */
#define FAN_TEMP_OFF  27    /* 温度 <27 自动关风扇 */
#define FAN_HUM_ON    80    /* 湿度 >80 自动开风扇 */
#define FAN_HUM_OFF   65    /* 湿度 <65 自动关风扇 */
#define FAN_SPEED_MS  2     /* 风扇每步延时 */
#define SENSE_TICKS   40    /* 40 × 50ms = 2s 采集/上报周期 */
#define DIST_ALARM_CM 10
#define DIST_NO_ECHO  0xFFFF

/* ---------- 全局状态 ---------- */
unsigned char g_temp = 0;
unsigned char g_hum  = 0;
unsigned char g_light = 0;
unsigned int g_adc = 0;   /* 原始 ADC 值（调试） */
unsigned int g_dist = 0;
bit g_lamp = 0, g_auto = 0, g_lamp_fault = 0;
bit g_fan = 0;          /* 风扇状态（步进电机） */
bit g_alarm = 0;        /* 报警状态（上报用） */
unsigned char g_fan_phase = 0;   /* 风扇当前相（4拍循环） */

/* UART 接收 */
unsigned char rx_line[32];
unsigned char rx_len = 0;
bit rx_ready = 0, tx_busy = 0;

/* 定时 */
volatile unsigned char tick50 = 0;
bit sense_flag = 0;

/* ============ 延时（12MHz 校准） ============ */
void delay_us2x(unsigned char t) { while (--t); }   /* 约 2us/次 */
void delay_ms(unsigned int t) {
  while (t--) { delay_us2x(245); delay_us2x(245); }
}

/* ============ LCD1602 ============ */
void lcd_cmd(unsigned char c) {
  LCD_RS = 0; LCD_RW = 0;
  LCD_DATA = c;
  LCD_EN = 1; delay_ms(1); LCD_EN = 0;
  delay_ms(2);
}
void lcd_dat(unsigned char d) {
  LCD_RS = 1; LCD_RW = 0;
  LCD_DATA = d;
  LCD_EN = 1; delay_ms(1); LCD_EN = 0;
  delay_ms(1);
}
void lcd_init(void) {
  delay_ms(20);
  lcd_cmd(0x38); lcd_cmd(0x0C); lcd_cmd(0x06); lcd_cmd(0x01);
  delay_ms(2);
}
void lcd_xy(unsigned char r, unsigned char c) {
  lcd_cmd(0x80 + (r ? 0x40 : 0x00) + c);
}
void lcd_str(unsigned char r, unsigned char c, unsigned char *s) {
  lcd_xy(r, c);
  while (*s) lcd_dat(*s++);
}
void lcd_str_code(unsigned char r, unsigned char c, unsigned char code *s) {
  lcd_xy(r, c);
  while (*s) lcd_dat(*s++);
}

/* 无符号整数转字符串（无前导零），返回长度 */
unsigned char num_str(unsigned int v, unsigned char *buf) {
  unsigned char tmp[5], i, len = 0;
  do { tmp[len++] = '0' + (v % 10); v /= 10; } while (v);
  for (i = 0; i < len; i++) buf[i] = tmp[len - 1 - i];
  buf[len] = 0;
  return len;
}

/* ============ UART（9600, Timer2，中断接收） ============ */
void uart_init(void) {
  SCON = 0x50;
  RCAP2H = 0xFF; RCAP2L = 0xD9;
  TH2 = 0xFF; TL2 = 0xD9;
  T2CON = 0x34;
  ES = 1;
  EA = 1;
}
void uart_isr(void) interrupt 4 {
  if (RI) {
    RI = 0;
    { unsigned char c = SBUF;
      if (!rx_ready) {
        if (c == '\n') { rx_line[rx_len] = 0; rx_ready = 1; rx_len = 0; }
        else if (c != '\r') { if (rx_len < 31) rx_line[rx_len++] = c; }
      }
    }
  }
  if (TI) { TI = 0; tx_busy = 0; }
}
void uart_send(unsigned char c) {
  while (tx_busy);
  tx_busy = 1;
  SBUF = c;
}
void uart_send_str(unsigned char *s) { while (*s) uart_send(*s++); }

/* ============ 定时器0（50ms） ============ */
void timer0_init(void) {
  TMOD = 0x11;   /* Timer0/Timer1 均模式1（Timer1 用于测距计时） */
  TH0 = 0x3C; TL0 = 0xB0;   /* 50ms @ 12MHz */
  ET0 = 1;
  TR0 = 1;
}
void timer0_isr(void) interrupt 1 {
  TH0 = 0x3C; TL0 = 0xB0;
  tick50++;
  if (tick50 >= SENSE_TICKS) { tick50 = 0; sense_flag = 1; }
}

/* ============ XPT2046（SPI ADC） ============ */
unsigned int xpt2046_read(unsigned char cmd) {
  unsigned char i;
  unsigned int val = 0;
  XP_CS = 0;
  XP_CLK = 0;
  for (i = 0; i < 8; i++) {           /* 发 8 位控制字（MSB 先） */
    XP_DIN = (cmd & 0x80) ? 1 : 0;
    cmd <<= 1;
    XP_CLK = 0;
    XP_CLK = 1;
  }
  for (i = 6; i > 0; i--);            /* 等转换完成 */
  XP_CLK = 1; _nop_(); _nop_();
  XP_CLK = 0; _nop_(); _nop_();
  for (i = 0; i < 12; i++) {          /* 读 12 位结果 */
    val <<= 1;
    XP_CLK = 1;
    XP_CLK = 0;
    val |= XP_DOUT;
  }
  XP_CS = 1;
  return val;
}

/* 光照 0~100%。此板光敏「亮→低ADC、暗→高ADC」，两点线性映射。
 * 实测：最亮≈275、最暗≈3500；两点各留余量。 */
#define LIGHT_ADC_BRIGHT 250   /* 映射到 100% 的 ADC（比最亮 275 略小） */
#define LIGHT_ADC_DARK   3600  /* 映射到 0% 的 ADC（比最暗 3500 略大） */

unsigned char light_read(void) {
  unsigned int adc = xpt2046_read(0x94);   /* 通道0(X+) */
  g_adc = adc;   /* 保存原始值（调试） */
  if (adc < LIGHT_ADC_BRIGHT) adc = LIGHT_ADC_BRIGHT;
  if (adc > LIGHT_ADC_DARK)   adc = LIGHT_ADC_DARK;
  return (unsigned char)((unsigned long)(LIGHT_ADC_DARK - adc) * 100
                         / (LIGHT_ADC_DARK - LIGHT_ADC_BRIGHT));
}

/* ============ DHT11 ============ */
bit dht11_read(unsigned char *temp, unsigned char *hum) {
  unsigned char d[5], i, k, t;
  bit ok = 0;

  EA = 0;   /* 关中断，保证单总线时序 */
  DHT11_PIN = 0; delay_ms(18);
  DHT11_PIN = 1;

  t = 0; while (!DHT11_PIN) { if (++t == 0) goto done; }  /* 释放拉高 */
  t = 0; while (DHT11_PIN)  { if (++t == 0) goto done; }  /* 响应拉低 */
  t = 0; while (!DHT11_PIN) { if (++t == 0) goto done; }  /* 就绪拉高 */
  t = 0; while (DHT11_PIN)  { if (++t == 0) goto done; }  /* 就绪结束拉低 */

  for (i = 0; i < 5; i++) {
    unsigned char val = 0;
    for (k = 0; k < 8; k++) {
      unsigned char cnt = 0;
      t = 0;
      while (!DHT11_PIN) { if (++t == 0) goto done; }
      while (DHT11_PIN)  { if (++cnt > 60) break; }
      val <<= 1;
      if (cnt > 6) val |= 1;
    }
    d[i] = val;
  }

  if ((unsigned char)(d[0] + d[1] + d[2] + d[3]) == d[4]) {
    *hum = d[0];
    *temp = d[2];
    ok = 1;
  }
done:
  DHT11_PIN = 1;
  EA = 1;
  return ok;
}

/* ============ 超声波测距（HC-SR04，Timer1） ============ */
unsigned int distance_cm(void) {
  unsigned int t, timeout;

  TRIG = 0;
  _nop_(); _nop_();
  TRIG = 1;
  _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
  _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
  _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
  TRIG = 0;

  timeout = 0;
  while (!ECHO) { if (++timeout > 20000) return DIST_NO_ECHO; }

  TH1 = 0; TL1 = 0;
  TR1 = 1;
  timeout = 0;
  while (ECHO) { if (++timeout > 30000) { TR1 = 0; return DIST_NO_ECHO; } }
  TR1 = 0;

  t = ((unsigned int)TH1 << 8) | TL1;
  return t / 58;   /* 距离 cm */
}

/* ============ 执行器 ============ */
void beep(unsigned int ms) {
  BUZZER = 1; delay_ms(ms); BUZZER = 0;
}

/* 风扇驱动：g_fan=1 时每调用一次前进一步（A→B→C→D 循环），否则停 */
void fan_drive(void) {
  if (!g_fan) {
    MOTOR_A = 0; MOTOR_B = 0; MOTOR_C = 0; MOTOR_D = 0;
    return;
  }
  g_fan_phase = (g_fan_phase + 1) & 3;
  MOTOR_A = (g_fan_phase == 0) ? 1 : 0;
  MOTOR_B = (g_fan_phase == 1) ? 1 : 0;
  MOTOR_C = (g_fan_phase == 2) ? 1 : 0;
  MOTOR_D = (g_fan_phase == 3) ? 1 : 0;
}

/* 手动开关灯（云端/语音/网页）：直接切换，不做光敏验证（演示中灯是模拟的） */
void set_lamp(bit on) {
  LAMP = on;
  g_lamp = on;
}

/* 自动模式开关灯：阈值判断 + 光敏闭环验证（验证逻辑只在自动模式用） */
void auto_lamp(bit on) {
  unsigned char before = light_read();
  LAMP = on;
  g_lamp = on;
  delay_ms(500);   /* 等继电器/光敏稳定 */
  { unsigned char after = light_read();
    /* 光照变化需超过 3%，否则判故障（过滤噪声/响应慢） */
    if (on)  g_lamp_fault = (after <= before + 3);
    else     g_lamp_fault = (before <= after + 3);
  }
}

/* ============ 上报 ============ */
void report(void) {
  unsigned char buf[16], p;

  p = 0; buf[p++] = 'D'; buf[p++] = ':';
  p += num_str(g_temp, buf + p);
  buf[p++] = ':';
  p += num_str(g_hum, buf + p);
  buf[p++] = '\n'; buf[p] = 0;
  uart_send_str(buf);

  p = 0; buf[p++] = 'L'; buf[p++] = ':';
  p += num_str(g_light, buf + p);
  buf[p++] = '\n'; buf[p] = 0;
  uart_send_str(buf);

  p = 0; buf[p++] = 'A'; buf[p++] = ':';
  buf[p++] = g_lamp ? '1' : '0';
  buf[p++] = '\n'; buf[p] = 0;
  uart_send_str(buf);

  p = 0; buf[p++] = 'R'; buf[p++] = ':';
  p += num_str((g_dist == DIST_NO_ECHO) ? 0 : g_dist, buf + p);
  buf[p++] = '\n'; buf[p] = 0;
  uart_send_str(buf);

  p = 0; buf[p++] = 'W'; buf[p++] = ':';
  buf[p++] = g_alarm ? '1' : '0';
  buf[p++] = '\n'; buf[p] = 0;
  uart_send_str(buf);

  p = 0; buf[p++] = 'F'; buf[p++] = ':';
  buf[p++] = g_fan ? '1' : '0';
  buf[p++] = '\n'; buf[p] = 0;
  uart_send_str(buf);
}

/* ============ LCD 显示 ============ */
void lcd_update(void) {
  unsigned char buf[17], p;

  p = 0;
  buf[p++] = 'T'; p += num_str(g_temp, buf + p);  buf[p++] = 'C'; buf[p++] = ' ';
  buf[p++] = 'H'; p += num_str(g_hum, buf + p);   buf[p++] = '%'; buf[p++] = ' ';
  buf[p++] = 'L'; p += num_str(g_light, buf + p); buf[p++] = '%';
  while (p < 16) buf[p++] = ' ';
  buf[16] = 0;
  lcd_str(0, 0, buf);

  p = 0;
  buf[p++] = 'D'; p += num_str((g_dist == DIST_NO_ECHO) ? 0 : g_dist, buf + p);
  buf[p++] = 'c'; buf[p++] = 'm'; buf[p++] = ' ';
  buf[p++] = 'L'; buf[p++] = ':';
  if (g_lamp_fault) { buf[p++] = 'E'; buf[p++] = 'R'; buf[p++] = 'R'; }
  else if (g_lamp)  { buf[p++] = 'O'; buf[p++] = 'N'; }
  else              { buf[p++] = 'O'; buf[p++] = 'F'; buf[p++] = 'F'; }
  buf[p++] = ' ';
  buf[p++] = 'A'; buf[p++] = ':';
  buf[p++] = g_auto ? '1' : '0';
  while (p < 16) buf[p++] = ' ';
  buf[16] = 0;
  lcd_str(1, 0, buf);
}

/* ============ 命令解析 ============ */
bit str_eq(unsigned char *s, unsigned char code *c) {
  while (*c) { if (*s != *c) return 0; s++; c++; }
  return (*s == 0);
}

void process_command(unsigned char *cmd) {
  if (str_eq(cmd, "C:lamp:on"))         set_lamp(1);
  else if (str_eq(cmd, "C:lamp:off"))   set_lamp(0);
  else if (str_eq(cmd, "C:fan:on"))     g_fan = 1;
  else if (str_eq(cmd, "C:fan:off"))    g_fan = 0;
  else if (str_eq(cmd, "C:auto:1"))     g_auto = 1;
  else if (str_eq(cmd, "C:auto:0"))     g_auto = 0;
}

/* ============ 周期任务（每 2s） ============ */
void sense_and_control(void) {
  unsigned char t, h;
  bit too_close;

  if (dht11_read(&t, &h)) { g_temp = t; g_hum = h; }
  g_light = light_read();
  g_dist = distance_cm();
  too_close = (g_dist != DIST_NO_ECHO && g_dist < DIST_ALARM_CM);

  if (g_auto) {
    if (g_light < LIGHT_DARK && !g_lamp)        auto_lamp(1);
    else if (g_light > LIGHT_BRIGHT && g_lamp)  auto_lamp(0);

    /* 风扇：温湿度联动（带滞回） */
    if ((g_temp > FAN_TEMP_ON || g_hum > FAN_HUM_ON) && !g_fan)     g_fan = 1;
    else if (g_temp < FAN_TEMP_OFF && g_hum < FAN_HUM_OFF && g_fan) g_fan = 0;
  }

  g_alarm = (g_temp > TEMP_ALARM_H || g_temp < TEMP_ALARM_L ||
             g_hum > HUM_ALARM_H || g_hum < HUM_ALARM_L ||
             g_lamp_fault || too_close);

  if (g_alarm) { BUZZER = 1; }   /* 报警只响蜂鸣器 */
  else         { BUZZER = 0; }

  lcd_update();
  report();
}

/* ============ 主程序 ============ */
void main(void) {
  uart_init();
  timer0_init();
  lcd_init();

  lcd_str_code(0, 0, "  Smart Home    ");
  lcd_str_code(1, 0, "   Init...      ");
  delay_ms(1000);

  while (1) {
    if (rx_ready) { process_command(rx_line); rx_ready = 0; rx_len = 0; }
    if (sense_flag) { sense_flag = 0; sense_and_control(); }
    fan_drive();
    delay_ms(FAN_SPEED_MS);   /* 风扇转速 */
  }
}
