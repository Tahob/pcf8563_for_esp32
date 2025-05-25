#include "pcf8563.h"

static esp_err_t pcf8563_read_register(uint8_t address, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) return 0;
    esp_err_t err = i2c_master_start(cmd);
    if (err != ESP_OK) goto fatal;
    err = i2c_master_write_byte(cmd, PCF8563_WRITE_ADDR, true);
    if (err != ESP_OK) goto fatal;
    err = i2c_master_write_byte(cmd, address, true);
    if (err != ESP_OK) goto fatal;
    err = i2c_master_start(cmd);
    if (err != ESP_OK) goto fatal;
    err = i2c_master_write_byte(cmd, PCF8563_READ_ADDR, true);
    if (err != ESP_OK) goto fatal;
    err = i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    if (err != ESP_OK) goto fatal;
    err = i2c_master_stop(cmd);
    if (err != ESP_OK) goto fatal;
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) goto fatal;
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
fatal:
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t pcf8563_write_register(uint8_t address,const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) return ESP_ERR_NO_MEM;
    esp_err_t err = i2c_master_start(cmd);
    if (err != ESP_OK) goto fatal;
    //固定流程，先向写寄存器写入要操作的寄存器地址，再向寄存器写入或读取数据
    err = i2c_master_write_byte(cmd, PCF8563_WRITE_ADDR, true);
    if (err != ESP_OK) goto fatal;
    err = i2c_master_write_byte(cmd, address, true);
    if (err != ESP_OK) goto fatal;
    err = i2c_master_write(cmd, data, len, true);
    if (err != ESP_OK) goto fatal;
    err = i2c_master_stop(cmd);
    if (err != ESP_OK) goto fatal;
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) goto fatal;
    i2c_cmd_link_delete(cmd);
    return ESP_OK;

fatal:
    i2c_cmd_link_delete(cmd);
    return err;
}

int pcf8563_init(gpio_num_t sda, gpio_num_t scl)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000, // 100kHz
    };
    esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        return err;
    }
    //if you don't want to use alarm, use 0x00 replace the 0x10
    const uint8_t data[2] = {0x00, 0x10};
    return pcf8563_write_register(0x00, data, 2);
}

int pcf8563_set_time(time_t time)
{
    struct tm timeinfo;
    localtime_r(&time, &timeinfo);
    uint8_t data[7] = {0};
    data[0] = DEC2BCD(timeinfo.tm_sec) & 0x7F;
    data[1] = DEC2BCD(timeinfo.tm_min) & 0x7F;
    data[2] = DEC2BCD(timeinfo.tm_hour) & 0x3F;
    data[3] = DEC2BCD(timeinfo.tm_mday)  & 0x3F;
    data[4] = DEC2BCD(timeinfo.tm_wday) & 0x07;
    data[5] = DEC2BCD(timeinfo.tm_mon) & 0x1F;//最高位表示世纪，0为21世纪，1为20世纪
    //timeinfo.tm_year以1900年为基准
    if (timeinfo.tm_year >= 100) {
		data[6] = DEC2BCD(timeinfo.tm_year - 100);
	} else {
        data[5] |= 0x80;
		data[6] = DEC2BCD(timeinfo.tm_year);
	}

    return pcf8563_write_register(0x02, data, sizeof(data));
}

time_t pcf8563_get_time()
{
    uint8_t data[7] = {0,0,0,0,0,0,0};
    if (pcf8563_read_register(0x02, data, sizeof(data)) != ESP_OK) {
        return 0;
    }
    struct tm timeinfo;
    timeinfo.tm_sec = BCD2DEC(data[0], 0x07);
    timeinfo.tm_min = BCD2DEC(data[1], 0x07);
    timeinfo.tm_hour = BCD2DEC(data[2], 0x03);
    timeinfo.tm_mday = BCD2DEC(data[3], 0x03);
    timeinfo.tm_mon = BCD2DEC(data[5], 0x01);
    timeinfo.tm_year = BCD2DEC(data[6], 0x0F);
    if (!(data[5] & 0x80)) //月寄存器最高位非1，表示21世纪
	{
		timeinfo.tm_year += 100;
	}

    time_t t = mktime(&timeinfo);
    if (t == -1) return 0;
    return t;
}

esp_err_t pcf8563_set_timer(pcf8563_timer_mode_t mode, pcf8563_timer_fraquence_t fraquence, uint8_t count)
{
    esp_err_t err;
    uint8_t control2 = 0;
    err = pcf8563_get_control(0x01,&control2);
    if (err != ESP_OK) return err;
    control2 |= 1;//使能定时器中断
    err = pcf8563_set_control(0x01, control2);
    if (err != ESP_OK) return err;
    uint8_t data[2] = {0};
    data[0] = (mode << 7 | fraquence) & 0xFF;
    data[1] = count & 0xFF;
    err = pcf8563_write_register(0x0E, data, sizeof(data));
    return err;
}

esp_err_t pcf8563_get_timer(pcf8563_timer_mode_t *mode, pcf8563_timer_fraquence_t *fraquence, uint8_t *count)
{
    esp_err_t err;
    uint8_t data[2] = {0};
    if (mode == NULL || fraquence == NULL || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    err = pcf8563_read_register(0x0E, data, sizeof(data));
    if (err != ESP_OK) return err;
    *mode = (pcf8563_timer_mode_t)((data[0] >> 7) & 0x01);
    *fraquence = (pcf8563_timer_fraquence_t)(data[0] & 0x03);
    *count = data[1];
    return ESP_OK;
}

esp_err_t pcf8563_set_alarm_mode(pcf8563_alarm_mode_t mode)
{
    esp_err_t err;
    uint8_t control2 = 0;
    err = pcf8563_get_control(0x01, &control2);
    if (err != ESP_OK) return err;
    control2 &= ~(1 << 3); //先清除报警标志
    if (mode == PCF8563_ALARM_MODE_ENABLE) {
        control2 |= (1 << 1); //使能报警器中断
    } else {
        control2 &= ~(1 << 1); //清除报警器中断使能
    }
    err = pcf8563_set_control(0x01, control2);
    return err;
}

esp_err_t pcf8563_set_alarm_type(pcf8563_alarm_type_t type, uint8_t value, bool disable)
{
    esp_err_t err;
    uint8_t data = 0;
        switch (type) {
            case PCF8563_ALARM_MINUTE:
                data = DEC2BCD(value) & 0x7F;
                break;
            case PCF8563_ALARM_HOUR:
                data = DEC2BCD(value) & 0x3F;
                break;
            case PCF8563_ALARM_DAY:
                data = DEC2BCD(value) & 0x3F;
                break;
            case PCF8563_ALARM_WEEK:
                data = DEC2BCD(value) & 0x07;
                break;
            default:
                return ESP_ERR_INVALID_ARG;
        }
        data |= (disable << 7); //设置报警器使能
        uint8_t address = 0x09 + type;
        err = pcf8563_write_register(address, &data, sizeof(data));
        if (err != ESP_OK) return err;
    return err;
}

esp_err_t pcf8563_get_alarm_mode(pcf8563_alarm_mode_t *mode)
{
    esp_err_t err;
    uint8_t data = 0;
    if (mode == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    err = pcf8563_get_control(0x01, &data);
    if (err != ESP_OK) return err;
    *mode = (pcf8563_alarm_mode_t)((data >> 1) & 0x01);
    return err;
}

esp_err_t pcf8563_get_alarm_typeinfo(pcf8563_alarm_type_t type, uint8_t *value, bool *disabled)
{
    uint8_t data = 0;
    if (disabled == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = pcf8563_read_register(0x09 + type, &data, sizeof(data));
    if (err != ESP_OK) return err;
    switch(type) {
        case PCF8563_ALARM_MINUTE:
            *value = BCD2DEC(data, 0x07);
            break;
        case PCF8563_ALARM_HOUR:
            *value = BCD2DEC(data, 0x03);
            break;
        case PCF8563_ALARM_DAY:
            *value = BCD2DEC(data, 0x03);
            break;
        case PCF8563_ALARM_WEEK:
            *value = BCD2DEC(data, 0x00);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    *disabled = data >> 7 & 0x01;
    return err;
}

esp_err_t pcf8563_get_intterrupt_type(pcf8563_interrupt_type_t *type)
{
    esp_err_t err;
    uint8_t data = 0;
    if (type == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *type = PCF8563_NONE_INTERRUPT;
    err = pcf8563_read_register(0x01, &data, sizeof(data));
    if (err != ESP_OK) return err;
    if ((data & (1 << 3)) != 0) {
        *type += PCF8563_ALARM;
    } 
    if ((data & (1 << 2)) != 0) {
        *type += PCF8563_TIMER;
    }
    return err;
}

esp_err_t pcf8563_set_clkout(pcf8563_clkout_mode_t mode, pcf8563_clkout_frquence_t fraquence)
{
    esp_err_t err;
    uint8_t data = 0;
    data |= (mode << 7) & 0x80;
    data |= (fraquence & 0x03);
    err = pcf8563_write_register(0x0D, &data, sizeof(data));
    return err;
}

esp_err_t pcf8563_get_clkout(pcf8563_clkout_mode_t *mode, pcf8563_clkout_frquence_t *fraquence)
{
    esp_err_t err;
    uint8_t data = 0;
    if (mode == NULL || fraquence == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    err = pcf8563_read_register(0x0D, &data, sizeof(data));
    if (err != ESP_OK) return err;
    *mode = (pcf8563_clkout_mode_t)((data >> 7) & 0x01);
    *fraquence = (pcf8563_clkout_frquence_t)(data & 0x03);
    return err;
}

esp_err_t pcf8563_get_control(uint8_t address, uint8_t *control)
{
    esp_err_t err;
    uint8_t data[1] = {0};
    err = pcf8563_read_register(address, data, sizeof(data));
    if (err == ESP_OK) {
        *control = data[0];
    }
    return err;
}

esp_err_t pcf8563_set_control(uint8_t address,uint8_t control)
{
    uint8_t data[1] = {0};
    data[0] = control & 0xFF;
    return pcf8563_write_register(address, data, sizeof(data));
}

esp_err_t pcf8563_clear_control(uint8_t address,uint8_t clearflag)
{
    esp_err_t err;
    uint8_t data[1] = {0};
    err = pcf8563_read_register(address, data, sizeof(data));
    if (err == ESP_OK) {
        data[0] &= clearflag;
        err = pcf8563_write_register(address, data, sizeof(data));
    }
    return err;
}
