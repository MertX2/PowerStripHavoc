#ifndef _BQ2589X_H

#define _BQ2589X_H

#include <Wire.h>

#include "bq2589x_reg.h"

#define BQ25895_ADDR (0x6A);

#define I2C_OK 0 //0:success
#define I2C_ERR 1

#define SCL_PIN 5
#define SDA_PIN 4

#define BQ2589X_OK 0
#define BQ2589X_ERR 1 //  ERR>0

typedef enum bq2589x_vbus_type
{
    BQ2589X_VBUS_NONE,
    BQ2589X_VBUS_USB_SDP,
    BQ2589X_VBUS_USB_CDP, /*CDP for bq25890, Adapter for bq25892*/
    BQ2589X_VBUS_USB_DCP,
    BQ2589X_VBUS_MAXC,
    BQ2589X_VBUS_UNKNOWN,
    BQ2589X_VBUS_NONSTAND,
    BQ2589X_VBUS_OTG,
    BQ2589X_VBUS_TYPE_NUM,
} bq2589x_vbus_type;

typedef enum bq2589x_part_no
{
    BQ25890 = 0x03,
    BQ25892 = 0x00,
    BQ25895 = 0x07,
} bq2589x_part_no;



class bq2589x
{
private:
    TwoWire *_wire;
    uint8_t _i2caddr;

    /* data */
public:
    bq2589x(/* args */);
    ~bq2589x();
    int begin();
    int begin(TwoWire *theWire);
    int begin(uint8_t addr);
    int begin(uint8_t addr, TwoWire *theWire);
    int read_byte(uint8_t *data, uint8_t reg);
    int write_byte(uint8_t reg, uint8_t data);
    int update_bits(uint8_t reg, uint8_t mask, uint8_t data);
    bq2589x_vbus_type get_vbus_type();
    int enable_otg();
    int disable_otg();
    int set_otg_volt(uint16_t volt);
    int set_otg_current(int curr);
    int enable_charger();
    int disable_charger();
    int adc_start(bool oneshot);
    int adc_stop();
    int adc_read_battery_volt();
    int adc_read_sys_volt();
    int adc_read_vbus_volt();
    int adc_read_temperature();
    int adc_read_charge_current();
    int set_charge_current(int curr);
    int set_term_current(int curr);
    int set_prechg_current(int curr);
    int set_chargevoltage(int volt);
    int set_input_volt_limit(int volt);
    int set_input_current_limit(int curr);
    int set_vindpm_offset(int offset);
    int get_charging_status();
    void bq2589x_set_otg(int enable);
    int set_watchdog_timer(uint8_t timeout);
    int disable_watchdog_timer();
    int reset_watchdog_timer();
    int force_dpdm();
    int reset_chip();
    int enter_ship_mode();
    int enter_hiz_mode();
    int exit_hiz_mode();
    int get_hiz_mode(uint8_t *state);
    int pumpx_enable(int enable);
    int pumpx_increase_volt();
    int pumpx_increase_volt_done();
    int pumpx_decrease_volt();
    int pumpx_decrease_volt_done();
    int force_ico();
    int check_force_ico_done();
    int enable_term(bool enable);
    int enable_auto_dpdm(bool enable);
    int use_absolute_vindpm(bool enable);
    int enable_ico(bool enable);
    int read_idpm_limit();
    bool is_charge_done();
    int init_device();
    int detect_device(bq2589x_part_no *part_no, int *revision);
    int enable_max_charge(bool enable);
};

#endif
