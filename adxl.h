#include "stdint.h"

#ifndef __ADXL355_H__
#define __ADXL355_H__

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/* SPI commands */
#define ADXL355_SPI_READ          0x01
#define ADXL355_SPI_WRITE         0x00



#define BIT_0							(1 << 0)
#define BIT_1							(1 << 1)
#define BIT_2							(1 << 2)
#define BIT_3							(1 << 3)
#define BIT_4							(1 << 4)
#define BIT_5							(1 << 5)
#define BIT_6							(1 << 6)
#define BIT_7							(1 << 7)


/* ADXL355 REG ADDR*/
#define ADXL355_DEVID_AD                 0x00
#define ADXL355_DEVID_MST                0x01
#define ADXL355_PARTID                   0x02
#define ADXL355_REVID                    0x03
#define ADXL355_STATUS                   0x04
#define ADXL355_FIFO_ENTRIES             0x05
#define ADXL355_TEMP2                    0x06
#define ADXL355_TEMP1                    0x07
#define ADXL355_XDATA3                   0x08
#define ADXL355_XDATA2                   0x09
#define ADXL355_XDATA1                   0x0A
#define ADXL355_YDATA3                   0x0B
#define ADXL355_YDATA2                   0x0C
#define ADXL355_YDATA1                   0x0D
#define ADXL355_ZDATA3                   0x0E
#define ADXL355_ZDATA2                   0x0F
#define ADXL355_ZDATA1                   0x10
#define ADXL355_FIFO_DATA                0x11
#define ADXL355_OFFSET_X_H               0x1E
#define ADXL355_OFFSET_X_L               0x1F
#define ADXL355_OFFSET_Y_H               0x20
#define ADXL355_OFFSET_Y_L               0x21
#define ADXL355_OFFSET_Z_H               0x22
#define ADXL355_OFFSET_Z_L               0x23
#define ADXL355_ACT_EN                   0x24
#define ADXL355_ACT_THRESH_H             0x25
#define ADXL355_ACT_THRESH_L             0x26
#define ADXL355_ACT_COUNT                0x27
#define ADXL355_FILTER                   0x28
#define ADXL355_FIFO_SAMPLES             0x29
#define ADXL355_INT_MAP                  0x2A
#define ADXL355_SYNC                     0x2B
#define ADXL355_RANGE                    0x2C
#define ADXL355_POWER_CTL                0x2D
#define ADXL355_SELF_TEST                0x2E
#define ADXL355_ADX_RESET                0x2F
#define ADXL355_SHADOW_REGISTER			 0x50

//Default register values
#define PD_DEVID_AD                     0xAD
#define PD_DEVID_MST                    0x1D
#define PD_PARTID                       0xED
#define PD_REVID                        0x01
#define PD_STATUS						0x00
#define PD_FIFO_ENTRIES					0x00
#define PD_OFFSET_X_H					0x00
#define PD_OFFSET_X_L					0x00
#define PD_OFFSET_Y_H					0x00
#define PD_OFFSET_Y_L					0x00
#define PD_OFFSET_Z_H					0x00
#define PD_OFFSET_Z_L					0x00
#define PD_ACT_EN						0x00
#define PD_ACT_THRESH_H					0x00
#define PD_ACT_THRESH_L					0x00
#define PD_ACT_COUNT					0x01
#define PD_FILTER						0x00
#define PD_FIFO_SAMPLES					0x60
#define PD_INT_MAP						0x00
#define PD_RANGE						0x81
#define PD_POWER_CTL					0x01
#define PD_SELF_TEST					0x00


#define ADXL355_RESET_CMD               0x52
#define ADXL355_SELF_TEST_CMD			0x03

//Register masks
#define ADXL355_RANGE_FIELD_MSK			(BIT_1 | BIT_0) 
#define ADXL355_INT_POL_FIELD_MSK		(BIT_6)
#define ADXL355_HPF_FIELD_MSK			(BIT_6 | BIT_5 | BIT_4)
#define ADXL355_ODR_LPF_FIELD_MSK		(BIT_3 | BIT_2 | BIT_1 | BIT_0)
#define ADXL355_NEG_ACC_MSK
#define ADXL355_PWR_CTR_STDBY_MSK		(BIT_0)
#define ADXL355_PWR_CTR_DRDY_MSK		(BIT_1)
#define ADXL355_PWR_CTR_TEMP_MSK		(BIT_2)



#define ADXL355_TEMP_BIAS               (float)1852.0     /* 25degC 时的截距 */
#define ADXL355_TEMP_SLOPE              (float)-9.05      /* 斜率(LSB/degC) */

#define ADXL355_SCALE_2G                256000.0
#define ADXL355_SCALE_4G                128000.0
#define ADXL355_SCALE_8G                64000.0




enum adxl355_op_mode {
	ADXL355_MEAS_TEMP_ON_DRDY_ON = 0,
	ADXL355_STDBY_TEMP_ON_DRDY_ON = 1,
	ADXL355_MEAS_TEMP_OFF_DRDY_ON = 2,
	ADXL355_STDBY_TEMP_OFF_DRDY_ON = 3,
	ADXL355_MEAS_TEMP_ON_DRDY_OFF = 4,
	ADXL355_STDBY_TEMP_ON_DRDY_OFF = 5,
	ADXL355_MEAS_TEMP_OFF_DRDY_OFF = 6,
	ADXL355_STDBY_TEMP_OFF_DRDY_OFF = 7
};

enum adxl355_comm_type {
	ADXL355_SPI_COMM,
	ADXL355_I2C_COMM
};

enum adxl355_hpf_corner {
	ADXL355_HPF_OFF,
	ADXL355_HPF_24_7,
	ADXL355_HPF_6_2084,
	ADXL355_HPF_1_5545,
	ADXL355_HPF_0_3862,
	ADXL355_HPF_0_0954,
	ADXL355_HPF_0_0238
};

enum adxl355_odr_lpf {
	ADXL355_ODR_4000HZ,
	ADXL355_ODR_2000HZ,
	ADXL355_ODR_1000HZ,
	ADXL355_ODR_500HZ,
	ADXL355_ODR_250HZ,
	ADXL355_ODR_125HZ,
	ADXL355_ODR_62_5HZ,
	ADXL355_ODR_31_25HZ,
	ADXL355_ODR_15_625HZ,
	ADXL355_ODR_7_813HZ,
	ADXL355_ODR_3_906HZ
};

enum adxl355_range {
	ADXL355_RANGE_2G = 1,
	ADXL359_RANGE_10G = 1,
	ADXL355_RANGE_4G = 2,
	ADXL359_RANGE_20G = 2,
	ADXL355_RANGE_8G = 3,
	ADXL359_RANGE_40G = 3,
};

enum adxl355_int_pol {
	ADXL355_INT_ACTIVE_LOW = 0,
	ADXL355_INT_ACTIVE_HIGH = 1
};

struct _adxl355_int_mask {
	uint8_t RDY_EN1 : 1;
	uint8_t FULL_EN1 : 1;
	uint8_t OVR_EN1 : 1;
	uint8_t ACT_EN1 : 1;
	uint8_t RDY_EN2 : 1;
	uint8_t FULL_EN2 : 1;
	uint8_t OVR_EN2 : 1;
	uint8_t ACT_EN2 : 1;
};

union adxl355_int_mask {
	struct _adxl355_int_mask fields;
	uint8_t value;
};

struct _adxl355_sts_reg_flags {
	uint8_t DATA_RDY : 1;
	uint8_t FIFO_FULL : 1;
	uint8_t FIFO_OVR : 1;
	uint8_t Activity : 1;
	uint8_t NVM_BUSY : 1;
	uint8_t reserved : 3;
};

union adxl355_sts_reg_flags {
	struct _adxl355_sts_reg_flags fields;
	uint8_t value;
};

struct _adxl355_act_en_flags {
	uint8_t ACT_X    : 1;
	uint8_t ACT_Y    : 1;
	uint8_t ACT_Z    : 1;
	uint8_t reserved : 4;
};

union adxl355_act_en_flags {
	struct _adxl355_act_en_flags fields;
	uint8_t value;
};

struct adxl355_frac_repr {
	int64_t integer;
	int32_t fractional;
} ;

typedef int32_t(*dev_write_reg_ptr)(void*, uint8_t, uint8_t*, uint16_t);
typedef int32_t(*dev_read_reg_ptr)(void*, uint8_t, uint8_t*, uint16_t);

typedef struct {
    void* handle;
    dev_read_reg_ptr read_reg;
    dev_write_reg_ptr write_reg;

	enum adxl355_op_mode op_mode;
	enum adxl355_odr_lpf odr_lpf;
	enum adxl355_hpf_corner hpf_corner;
	enum adxl355_range range;
	uint16_t x_offset;
	uint16_t y_offset;
	uint16_t z_offset;
	uint8_t fifo_samples;
	union adxl355_act_en_flags act_en;
	uint8_t act_cnt;
	uint16_t act_thr;
	uint8_t shadow_reg[5];

} dev_ctx_t;
/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

int32_t adxl355_read_reg(dev_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len);
int32_t adxl355_write_reg(dev_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len);


int32_t adxl355_init(dev_ctx_t* ctx, void *read_reg, void *write_reg);
int32_t adxl355_deinit(dev_ctx_t* ctx);

int adxl355_set_op_mode(dev_ctx_t* ctx, enum adxl355_op_mode op_mode);
int adxl355_get_op_mode(dev_ctx_t* ctx, enum adxl355_op_mode *read_op_mode);

int32_t adxl355_reset(dev_ctx_t* ctx);
int32_t adxl355_set_self_test(dev_ctx_t* ctx);
int32_t adxl355_set_range(dev_ctx_t* ctx, enum adxl355_range range_val);
int32_t adxl355_get_range(dev_ctx_t* ctx, enum adxl355_range);
int32_t adxl355_set_odr_lpf(dev_ctx_t* ctx, enum adxl355_odr_lpf odr_lpf_val);
int32_t adxl355_get_odr_lpf(dev_ctx_t* ctx, enum adxl355_odr_lpf odr_lpf_val);
int32_t adxl355_set_hpf_corner(dev_ctx_t* ctx, enum adxl355_hpf_corner hpf_corner_val);
int32_t adxl355_get_hpf_corner(dev_ctx_t* ctx, enum adxl355_hpf_corner hpf_corner_val );
int32_t adxl355_set_offset(dev_ctx_t* ctx, uint16_t x_offset, uint16_t y_offset, uint16_t z_offset);
int32_t adxl355_get_raw_xyz(dev_ctx_t* ctx, uint32_t *pBuff);
int32_t adxl355_get_xyz(dev_ctx_t* ctx, struct adxl355_frac_repr *x, struct adxl355_frac_repr *y, struct adxl355_frac_repr *z);
int32_t adxl355_get_raw_temp(dev_ctx_t* ctx, uint16_t *raw_temp);
int32_t adxl355_get_temp(dev_ctx_t* ctx, struct adxl355_frac_repr *temp);
int32_t adxl355_get_stat_reg(dev_ctx_t* ctx, 
			union adxl355_sts_reg_flags *status_flags);
int32_t adxl355_get_num_of_fifo_entries(dev_ctx_t* ctx, 
			uint8_t *reg_value);
int32_t adxl355_set_fifo_samples(dev_ctx_t* ctx, uint8_t *reg_value);
int32_t adxl355_get_raw_fifo_data(dev_ctx_t* ctx, 
			uint8_t *fifo_entries,uint32_t *pBuff);
int32_t adxl355_get_fifo_data(dev_ctx_t* ctx, uint8_t *fifo_entries,
			struct adxl355_frac_repr *x, struct adxl355_frac_repr *y,
			struct adxl355_frac_repr *z);
int32_t adxl355_conf_act_en(dev_ctx_t* ctx, 
			union adxl355_act_en_flags act_config);
int32_t adxl355_conf_act_thr(dev_ctx_t* ctx, uint16_t act_thr);
int32_t adxl355_set_act_cnt_reg(dev_ctx_t* ctx, uint8_t act_cnt);
int32_t adxl355_config_int_path(dev_ctx_t* ctx, 
			union adxl355_int_mask int_conf);
int32_t adxl355_set_int_pol(dev_ctx_t* ctx, enum adxl355_int_pol int_pol);

#endif /* __ADXL355_H__ */