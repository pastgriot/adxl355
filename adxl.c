#include <stdlib.h>
#include <errno.h>
#include "adxl.h"

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

 /******************************************************************************
 * @brief Reads from the device.
 *
 * @param ctx - The device handler structure.
 * @param reg_val - Address of the base register.
 *  * @param data  - pointer to the read data buffer.
 * @param len - The number of bytes to be read and returned in read_data.
 *
 * @return ret         - Result of the reading procedure.
*******************************************************************************/
int32_t adxl355_read_reg( dev_ctx_t *ctx, uint8_t reg_val, uint8_t* data, uint16_t len ) {

    int32_t ret;
    reg_val = ( reg_val << 1 ) | ADXL355_SPI_READ;
    ret = ctx->read_reg( ctx->handle, reg_val, data, len + 1 );
    
    return ret;
}

 /******************************************************************************
 * @brief Writes to the device.
 *
 * @param ctx - The device handler structure.
 * @param reg_val - Address of the base register.
 * @param data - pointer data buffer to be written.
 * @param len - The number of bytes to be write.
 *
 * @return ret         - Result of the reading procedure.
*******************************************************************************/
int32_t adxl355_write_reg( dev_ctx_t *ctx, uint8_t reg_val, uint8_t* data, uint16_t len ) {
    int32_t ret;
    
    reg_val = ( reg_val << 1 ) | ADXL355_SPI_WRITE;
    ret = ctx->write_reg( ctx->handle, reg_val, data, len + 1 );
    
    return ret;
}

/***************************************************************************
 * @brief Initializes the communication peripheral and checks if the ADXL355
 *        part is present.
 *
 * @param ctx - Device handler 
 * @param read_reg - function pointer to MCU implementation of SPI read.
 * @param write_reg - function pointer to MCU implementation of SPI write.
 * @return ret       - Result of the initialization procedure.
*******************************************************************************/
int32_t adxl355_init(dev_ctx_t *ctx, void *read_reg, void *write_reg) {
    int32_t ret;
    uint8_t temp_val;

    ctx->read_reg = read_reg;
    ctx->write_reg = write_reg;

    ret = adxl355_read_reg( ctx, ADXL355_DEVID_AD, &temp_val, 1 );    
    if ( !ret || ( temp_val != PD_DEVID_AD ) )
        return -1;

    ret = adxl355_read_reg( ctx, ADXL355_DEVID_MST, &temp_val, 1 );
    if ( !ret || (temp_val != PD_DEVID_MST ) )
        return -1;

    ret = adxl355_read_reg( ctx, ADXL355_PARTID, &temp_val, 1 );
    if ( !ret || (temp_val != PD_PARTID ) )
        return -1;
    
    // Get and store shadow register values for later soft reset validations
    ret = adxl355_read_reg( ctx, ADXL355_SHADOW_REGISTER, &ctx->shadow_reg[0], 5 );
    if ( !ret )
        return -1;

    // Lets reset just to be shure
    ret = adxl355_reset( ctx );
    if ( !ret )
        return -1;

    //Set default register values to the handler
    ctx->op_mode = PD_POWER_CTL;
    ctx->range = PD_RANGE & ADXL355_RANGE_FIELD_MSK;
    ctx->odr_lpf = PD_FILTER & ADXL355_ODR_LPF_FIELD_MSK;
    ctx->act_cnt = PD_ACT_COUNT;
    ctx->x_offset = 0x00;
    ctx->y_offset = 0x00;
    ctx->z_offset = 0x00;

    return 0;
}

int32_t adxl355_deinit(dev_ctx_t* ctx) {

}

/***************************************************************************
 * @brief Performs a soft reset of the device.
 * @param ctx  - Handler of device structure.
 * @return ret - Result of the soft reset procedure.
*******************************************************************************/
int32_t adxl355_reset( dev_ctx_t *ctx ) {
    int32_t ret;
    uint8_t shadow_temp_val[5] = 0, nb_of_retries = 255, temp_val = ADXL355_RESET_CMD;
	union adxl355_sts_reg_flags flags;


    ret = adxl355_write_reg( ctx, ADXL355_ADX_RESET, temp_val, 1 );
    if ( !ret )
        return -1;
    // After soft reset, the data in the shadow registers will be valid only after NVM is not busy anymore
	ret = adxl355_get_sts_reg( ctx, &flags );
    if ( !ret )
        return -1;
	while ( flags.fields.NVM_BUSY && nb_of_retries ) {
		if ( !adxl355_get_sts_reg( ctx, &flags ) )
            return -1;
		nb_of_retries--;
	}
    if ( nb_of_retries == 0 )
        return -1;

    ret = adxl355_read_reg( ctx, ADXL355_ADX_RESET, &shadow_temp_val[0], NELEMS( shadow_temp_val ) );
    if ( !ret )
        return 1;
    
    // compare the current shadow registers with the pre-reset ones
    for( uint8_t i = 0; i < NELEMS( shadow_temp_val ); i++ )
        if ( shadow_temp_val[i] != ctx->shadow_reg[i] )
            return -1;
    
    return 0;
}

/***************************************************************************
 * @brief Triggers the self-test feature.
 * @param ctx  - The handle device structure.
 * @return ret - Result of the writing procedure.
*******************************************************************************/
int32_t adxl355_set_self_test( dev_ctx_t *ctx ) {

    uint8_t temp_val = PD_SELF_TEST; 
    int32_t ret;
    
    ret = adxl355_write_reg( ctx, ADXL355_SELF_TEST, &temp_val, 1 );
    if ( !ret )
        return -1;
    return 0;
}

/***************************************************************************
 * @brief Sets the measurement range register value.
 *
 * @param ctx       - The handler device structure.
 * @param range_val - Selected range.
 *
 * @return ret      - Result of the writing procedure.
*******************************************************************************/
int32_t adxl355_set_range( dev_ctx_t *ctx, enum adxl355_range range_val ) {
    uint8_t temp_val;
    int32_t ret;

    enum adxl355_op_mode curr_mode = ctx->op_mode;

    if (!(curr_mode & ADXL355_PWR_CTR_STDBY_MSK)) {
    	ret = adxl355_set_op_mode( ctx, ADXL355_STDBY_TEMP_OFF_DRDY_OFF );
		if ( !ret )
			return -1;
    }

    ret = adxl355_read_reg( ctx, ADXL355_RANGE, &temp_val, 1 );
    if ( !ret )
        return -1;

    temp_val &= ~ADXL355_RANGE_FIELD_MSK;
    temp_val |= range_val & ADXL355_RANGE_FIELD_MSK;

    ret = adxl355_write_reg( ctx, ADXL355_RANGE, &temp_val, 1 );
    if ( !ret )
        return -1;
    ctx->range = range_val;
    
    //resume whatever state op_mode was set to.
    ret = adxl355_set_op_mode( ctx, &curr_mode );
    if ( !ret )
        return -1;

    return 0;
}

/***************************************************************************
 * @brief Gets the measurement range register value.
 *
 * @param ctx       - The handler device structure.
 * @param range_val - Buffer of the range to be read.
 *
 * @return ret      - Result of the writing procedure.
*******************************************************************************/
int32_t adxl355_get_range( dev_ctx_t *ctx, enum adxl355_range range_val ) {
    uint8_t temp_val;
    int32_t ret;

    ret = adxl355_read_reg( ctx, ADXL355_RANGE, &temp_val, 1 );
    if ( !ret )
        return -1;

    range_val |= temp_val & ADXL355_RANGE_FIELD_MSK;

    return 0;
}


/***************************************************************************
 * @brief Writes the low-pass filter settings.
 *
 * @param ctx         - The handler device structure.
 * @param odr_lpf_val - Low-pass filter settings.
 *
 * @return ret        - Result of the writing procedure.
*******************************************************************************/
int32_t adxl355_set_odr_lpf( dev_ctx_t *ctx, enum adxl355_odr_lpf odr_lpf_val ) {
    uint8_t temp_val;
    int32_t ret;

    // The lpf settings can be changed only when the device's operation mode is
	// Standby.
    enum adxl355_op_mode curr_mode = ctx->op_mode;

    if (!(curr_mode & ADXL355_PWR_CTR_STDBY_MSK)) {
    	ret = adxl355_set_op_mode( ctx, ADXL355_STDBY_TEMP_OFF_DRDY_OFF );
		if ( !ret )
			return -1;
    }
    ret = adxl355_read_reg( ctx, ADXL355_FILTER, &temp_val, 1 );
    if ( !ret )
        return -1;
    
    temp_val &= ~( ADXL355_ODR_LPF_FIELD_MSK );
    temp_val |= odr_lpf_val & ADXL355_ODR_LPF_FIELD_MSK;

    ret = adxl355_write_reg( ctx, ADXL355_FILTER, &temp_val, 1 );
    if ( !ret )
        return -1;
    
    ctx->odr_lpf = odr_lpf_val;

    //resume whatever state op_mode was set to.
    ret = adxl355_set_op_mode( ctx, &curr_mode );
    if ( !ret )
        return -1;

    return 0;
}

/***************************************************************************
 * @brief Reads the low-pass filter settings.
 *
 * @param ctx         - The handler device structure.
 * @param odr_lpf_val - Low-pass filter settings.
 *
 * @return ret        - Result of the writing procedure.
*******************************************************************************/
int32_t adxl355_get_odr_lpf( dev_ctx_t *ctx, enum adxl355_odr_lpf odr_lpf_val ) {
    uint8_t temp_val;
    int32_t ret;

    ret = adxl355_read_reg( ctx, ADXL355_FILTER, &temp_val, 1 );
    if ( !ret )
        return -1;

    odr_lpf_val |= temp_val & ADXL355_RANGE_FIELD_MSK;

    return 0;
}

/***************************************************************************//**
 * @brief Writes the high-pass filter settings.
 *
 * @param ctx            - The handler device structure.
 * @param hpf_corner_val - High-pass filter settings.
 *
 * @return ret           - Result of the writing procedure.
*******************************************************************************/
int32_t adxl355_set_hpf_corner( dev_ctx_t *ctx, enum adxl355_hpf_corner hpf_corner_val ) {
    uint8_t temp_val;
    int32_t ret;

    // Stop any operation previous change of hpf.
    enum adxl355_op_mode curr_mode = ctx->op_mode;
    if (!(curr_mode & ADXL355_PWR_CTR_STDBY_MSK)) {
    	ret = adxl355_set_op_mode(ctx, ADXL355_STDBY_TEMP_OFF_DRDY_OFF);
		if ( !ret )
			return -1;
    }

    ret = adxl355_read_reg( ctx, ADXL355_FILTER, &temp_val, 1 );
    if ( !ret )
        return -1;

    temp_val &= ~( ADXL355_HPF_FIELD_MSK );
    temp_val |= hpf_corner_val & ADXL355_HPF_FIELD_MSK;

    ret = adxl355_write_reg( ctx, ADXL355_FILTER, &temp_val, 1 );
    if ( !ret )
        return -1;
    
    ctx->hpf_corner = hpf_corner_val;

    //resume whatever state op_mode was set to.
    ret = adxl355_set_op_mode( ctx, &curr_mode );
    if ( !ret )
        return -1;

    return 0;
}

/*******************************************************************************
 * @brief Reads the high-pass filter settings.
 *
 * @param ctx            - The handler device structure.
 * @param hpf_corner_val - High-pass filter settings.
 *
 * @return ret           - Result of the writing procedure.
*******************************************************************************/
int32_t adxl355_get_hpf_corner(dev_ctx_t* ctx, enum adxl355_hpf_corner hpf_corner_val ) {
    int32_t ret;
    uint8_t temp_val;

    ret = adxl355_read_reg( ctx, ADXL355_FILTER, &temp_val, 1 );
    if ( !ret )
        return -1;
    hpf_corner_val |= temp_val & ADXL355_HPF_FIELD_MSK;

    return 0;
}

/*******************************************************************************
 * @brief Sets an offset value for each axis (Offset Calibration).
 *
 * @param ctx      - The hanlder device structure.
 * @param x_offset - X-axis's offset.
 * @param y_offset - Y-axis's offset.
 * @param z_offset - Z-axis's offset.
 *
 * @return ret     - Result of the writing procedure.
*******************************************************************************/
int32_t adxl355_set_offset( dev_ctx_t *ctx, uint16_t x_offset, uint16_t y_offset, uint16_t z_offset ) {

	int32_t ret;

	uint8_t data_offset_x[2] = {x_offset >> 8, (uint8_t)x_offset};
	uint8_t data_offset_y[2] = {y_offset >> 8, (uint8_t)y_offset};
	uint8_t data_offset_z[2] = {z_offset >> 8, (uint8_t)z_offset};

    // Stop any operation previous change of hpf.
    enum adxl355_op_mode curr_mode = ctx->op_mode;
    if (!(curr_mode & ADXL355_PWR_CTR_STDBY_MSK)) {
    	ret = adxl355_set_op_mode(ctx, ADXL355_STDBY_TEMP_OFF_DRDY_OFF);
		if ( !ret )
			return -1;
    }

    ret = adxl355_write_reg( ctx, ADXL355_OFFSET_X_H, &data_offset_x[0], 2 );
    if ( !ret )
        return -1;
    ret = adxl355_write_reg( ctx, ADXL355_OFFSET_Y_H, &data_offset_y[0], 2 );
    if ( !ret )
        return -1;
    ret = adxl355_write_reg( ctx, ADXL355_OFFSET_Z_H, &data_offset_z[0], 2 );
    if ( !ret )
        return -1;

    ctx->x_offset = x_offset;
    ctx->y_offset = y_offset;
    ctx->z_offset = z_offset;

    //resume whatever state op_mode was set to.
    ret = adxl355_set_op_mode( ctx, &curr_mode );
    if ( !ret )
        return -1;

    return 0;
}

/*******************************************************************************
 * @brief Reads the raw output data.
 *
 * @param ctx   - The handler device structure.
 * @param pBuff - Pointer to 9 byte array xyz
 *
 * @return ret  - Result of the reading procedure.
*******************************************************************************/
int32_t adxl355_get_raw_xyz( dev_ctx_t *ctx, uint32_t *pBuff ) {
    int32_t ret;

    uint8_t pBuff[9];
    ret = adxl355_read_reg( ctx, ADXL355_XDATA3, &pBuff[0] , 9 );
    if ( !ret )
        return 1;

    return 0;
}

int32_t adxl355_get_xyz(    dev_ctx_t* ctx, 
                            struct adxl355_frac_repr *x,
		                    struct adxl355_frac_repr *y, 
                            struct adxl355_frac_repr *z ) {

    return 0;
}

/*******************************************************************************
 * @brief Reads the raw temperature.
 *
 * @param dev      - The device structure.
 * @param raw_temp - 2 byte pointer Raw temperature output data.
 *
 * @return ret     - Result of the reading procedure.
*******************************************************************************/
int32_t adxl355_get_raw_temp( dev_ctx_t *ctx, uint16_t *raw_temp ) {
    int32_t ret;
    uint8_t temp_val[2];

    ret = adxl355_read_reg( ctx, ADXL355_TEMP2, &temp_val[0], 2 );
    if ( !ret )
        return -1;
    // raw_data[0] bits [7-4]: reserved
    // raw_data[0] bits [3-0]: DATA bits [11: 8]
    // raw_data[1] bits [7-0]: DATA bits [ 7: 0]
    *raw_temp = ( ( temp_val[0] & 0xf ) << 8 ) | temp_val [1];
    return 0;
}

int32_t adxl355_get_temp(dev_ctx_t* ctx, struct adxl355_frac_repr *temp) {

    return 0;
}

/*******************************************************************************
 * @brief Reads the status register value.
 *
 * @param dev          - The device structure.
 * @param status_flags - Register value.
 *
 * @return ret         - Result of the reading procedure.
*******************************************************************************/
int32_t adxl355_get_stat_reg(dev_ctx_t* ctx, union adxl355_sts_reg_flags *status_flags) {
    int32_t ret;
    uint8_t temp_val;

    ret = adxl355_read_reg( ctx, ADXL355_STATUS, &temp_val, 1);
    if ( !ret )
        return -1;

    status_flags->value = temp_val;
    return 0;
}

/*******************************************************************************
 * @brief Reads the number of FIFO entries register value.
 *
 * @param ctx       - The handler device structure.
 * @param reg_value - Register value.
 *
 * @return ret      - Result of the reading procedure.
*******************************************************************************/
int32_t adxl355_get_num_of_fifo_entries( dev_ctx_t *ctx, uint8_t *reg_value ) {
    int32_t ret;

    ret = adxl355_read_reg( ctx, ADXL355_FIFO_ENTRIES, reg_value, 1);
    if ( !ret )
        return -1;
    
    return 0;
}

/*******************************************************************************
 * @brief Sets the number of FIFO samples register value.
 *
 * @param ctx       - The handler device structure.
 * @param reg_value - Register value.
 *
 * @return ret      - Result of the writing procedure.
*******************************************************************************/
int32_t adxl355_set_fifo_samples( dev_ctx_t* ctx, uint8_t *reg_value ) {
    int32_t ret;

	if ( reg_value > PD_FIFO_SAMPLES )
		return -1;

    ret = adxl355_write_reg( ctx, ADXL355_FIFO_SAMPLES, reg_value, 1) ;
	if ( !ret )
        return -1;

	ctx->fifo_samples = reg_value;
    return 0;
}


int32_t adxl355_get_raw_fifo_data(dev_ctx_t* ctx, 
			uint8_t *fifo_entries,uint32_t *pBuff);


/*******************************************************************************
 * @brief Reads fifo data and returns the values converted in m/s^2.
 *
 * @param ctx          - The handler device structure.
 * @param fifo_entries - The number of fifo entries.
 * @param x            - Converted x-axis data.
 * @param y            - Converted y-axis data.
 * @param z            - Converted z-axis data.
 *
 * @return ret         - Result of the configuration procedure.
*******************************************************************************/
int32_t adxl355_get_fifo_data(  dev_ctx_t* ctx, 
                                uint8_t *fifo_entries,
			                    struct adxl355_frac_repr *x, 
                                struct adxl355_frac_repr *y,
			                    struct adxl355_frac_repr *z ) {

    int32_t ret;


    return 0;
}

/******************************************************************************
 * @brief Configures the activity enable register.
 *
 * @param ctx         - The handler device structure.
 * @param act_config  - Activity enable mapping.
 *
 * @return ret         - Result of the configuration procedure.
*******************************************************************************/
int32_t adxl355_conf_act_en(dev_ctx_t* ctx,  union adxl355_act_en_flags act_config) {
    int32_t ret;
    uint8_t reg_val = act_config.value;

    ret = adxl355_write_reg( ctx, ADXL355_ACT_EN, &reg_val, 1);
	if (!ret)
        return -1;

	ctx->act_en = act_config;
    return 0;
}

/***************************************************************************
 * @brief Configures the activity threshold registers.
 *
 * @param ctx     - The handler device structure.
 * @param act_thr - Activity threshold value.
 *
 * @return ret    - Result of the configuration procedure.
*******************************************************************************/
int32_t adxl355_conf_act_thr(dev_ctx_t* ctx, uint16_t act_thr) {
    int32_t ret;

   	ret = adxl355_write_reg( ctx, ADXL355_ACT_THRESH_H, &act_thr, 2);
	if ( !ret )
        return -1;
    
    ctx->act_thr = act_thr;
    return 0;
}

/***************************************************************************//**
 * @brief Writes the activity count register value.
 *
 * @param ctx     - The handler device structure.
 * @param act_cnt - Register value.
 *
 * @return ret    - Result of the writing procedure.
*******************************************************************************/
int32_t adxl355_set_act_cnt_reg(dev_ctx_t* ctx, uint8_t act_cnt) {
    int32_t ret;
    uint8_t temp_val;

	ret = adxl355_write_reg( ctx, ADXL355_ACT_COUNT, &act_cnt, 1);
	if ( !ret )
        return -1;
    
    ctx->act_cnt = act_cnt;
	return 0;
}

/***************************************************************************
 * @brief Configures the interrupt map for INT1 and INT2 pins.
 *
 * @param dev      - The handler device structure.
 * @param int_conf - Interrupt mapping.
 *
 * @return ret     - Result of the configuration procedure.
*******************************************************************************/
int32_t adxl355_config_int_path( dev_ctx_t *ctx, union adxl355_int_mask int_conf ) {
    int32_t ret;
    uint8_t wr_val = int_conf.value, rd_val;

    ret = adxl355_write_reg( ctx, ADXL355_INT_MAP, &wr_val, 1 );
    if ( !ret )
        return -1;

    ret = adxl355_read_reg( ctx, ADXL355_INT_MAP,  &rd_val, 1);
    if ( !ret )
        return -1;

    if (rd_val != wr_val)
        return -1;

    return 0;
}

/***************************************************************************
 * @brief Sets the interrupt polarity.
 *
 * @param ctx     - The handler device structure.
 * @param int_pol - Interrupt polarity to be set.
 *
 * @return ret    - Result of the reading procedure.
*******************************************************************************/
int32_t adxl355_set_int_pol( dev_ctx_t *ctx, enum adxl355_int_pol int_pol ) {
    int32_t ret;
    uint8_t temp_val;

    ret = adxl355_read_reg( ctx, ADXL355_RANGE, &temp_val, 1);
    if ( !ret )
        return -1;
    
    temp_val &= ~( ADXL355_INT_POL_FIELD_MSK );
    temp_val |= ( int_pol << 6 ) & ADXL355_INT_POL_FIELD_MSK;
    
    ret = adxl355_write_reg( ctx, ADXL355_RANGE, &temp_val, 1 );
    if ( !ret )
        return -1;

    return 0;
}
