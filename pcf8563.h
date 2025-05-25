#ifndef PCF8563_H
#define PCF8563_H
#include "driver/i2c.h"
#include "sys/time.h"
#include "esp_log.h"

//iic设备一般使用7位地址，最低位为读写标志位，0表示写，1表示读
#define PCF8563_ADDR 0x51
#define PCF8563_WRITE_ADDR PCF8563_ADDR << 1
#define PCF8563_READ_ADDR (PCF8563_ADDR << 1) | 1

//十进制转BCD码，BCD码每个4位二进制数表示十进制数中的一位
#define DEC2BCD(x) (((x) / 10 << 4) + ((x) % 10))
#define BCD2DEC(x, flag) ((((x) >> 4) & flag) * 10  + ((x) & 0x0F))
#define PCF8563_CLEAR_TIMER_FLAG pcf8563_clear_control(0x01, 0xFB) //清除定时器中断标志
#define PCF8563_CLEAR_ALARM_FLAG pcf8563_clear_control(0x01, 0xF7) //清除报警器中断标志
//需要注意当alarm和timer同时被使能而触发中断时，仅清除一个标志位会导致另一个标志位无法清除，导致中断无法终止
//需要明确自己是否使能了alarm和timer，同时使能的情况下优先使用此宏完成清理。或者清理前先调用pcf8563_get_intterrupt_type再判断。
#define PCF8563_CLEAR_TIMER_ALARM_FLAG pcf8563_clear_control(0x01, 0xF3) //同时清除定时器和报警器中断标

typedef enum {
    PCF8563_TIMER_MODE_DISABLE = 0,
    PCF8563_TIMER_MODE_ENABLE = 1
} pcf8563_timer_mode_t;

typedef enum {
    PCF8563_TIMER_4096HZ = 0,
    PCF8563_TIMER_64HZ = 1,
    PCF8563_TIMER_1HZ = 2,
    PCF8563_TIMER_1DIV60HZ = 3
} pcf8563_timer_fraquence_t;

typedef enum {
    PCF8563_ALARM_MODE_DISABLE = 0,
    PCF8563_ALARM_MODE_ENABLE = 1,
} pcf8563_alarm_mode_t;

typedef enum {
    PCF8563_ALARM_MINUTE = 0,
    PCF8563_ALARM_HOUR = 1,
    PCF8563_ALARM_DAY = 2,
    PCF8563_ALARM_WEEK = 3
} pcf8563_alarm_type_t;

typedef enum {
    PCF8563_NONE_INTERRUPT = 0,
    PCF8563_TIMER = 1,
    PCF8563_ALARM = 2,
    PCF8563_TIMER_ALARM = 3
} pcf8563_interrupt_type_t;

typedef enum {
    PCF8563_FREQUENCY_32KHZ = 0,
    PCF8563_FREQUENCY_1024HZ = 1,
    PCF8563_FREQUENCY_32HZ = 2,
    PCF8563_FREQUENCY_1HZ = 3,
} pcf8563_clkout_frquence_t;

typedef enum {
    PCF8563_CLKOUT_DISABLE = 0,
    PCF8563_CLKOUT_ENABLE = 1
} pcf8563_clkout_mode_t;

int pcf8563_init(gpio_num_t sda, gpio_num_t scl);
int pcf8563_set_time(time_t time);
time_t pcf8563_get_time();
//倒计数周期=n/时钟频率
esp_err_t pcf8563_set_timer(pcf8563_timer_mode_t mode, pcf8563_timer_fraquence_t fraquence, uint8_t count);
esp_err_t pcf8563_get_timer(pcf8563_timer_mode_t *mode, pcf8563_timer_fraquence_t *fraquence, uint8_t *count);
//闹钟
esp_err_t pcf8563_set_alarm_mode(pcf8563_alarm_mode_t mode);
esp_err_t pcf8563_set_alarm_type(pcf8563_alarm_type_t type, uint8_t value, bool disable);
esp_err_t pcf8563_get_alarm_mode(pcf8563_alarm_mode_t *mode);
esp_err_t pcf8563_get_alarm_typeinfo(pcf8563_alarm_type_t type, uint8_t *value, bool *disabled);
//中断
esp_err_t pcf8563_get_intterrupt_type(pcf8563_interrupt_type_t *type);
// 脉冲
esp_err_t pcf8563_set_clkout(pcf8563_clkout_mode_t mode, pcf8563_clkout_frquence_t fraquence);
esp_err_t pcf8563_get_clkout(pcf8563_clkout_mode_t *mode, pcf8563_clkout_frquence_t *fraquence);
//定时器
esp_err_t pcf8563_get_control(uint8_t address,uint8_t *control);
esp_err_t pcf8563_set_control(uint8_t address,uint8_t control);
esp_err_t pcf8563_clear_control(uint8_t address,uint8_t clearflag);
#endif
