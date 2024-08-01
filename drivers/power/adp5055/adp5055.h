/***************************************************************************//**
 *   @file   adp5055.h
 *   @brief  Header file for the ADP5055 Driver
 *   @author Jose Ramon San Buenaventura (jose.sanbuenaventura@analog.com)
********************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef __ADP_5055_H__
#define __ADP_5055_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "no_os_gpio.h"
#include "no_os_i2c.h"
#include "no_os_pwm.h"
#include "no_os_util.h"
#include "no_os_units.h"

#define ADP5055_EXTENDED_COMMAND		0xE2
#define ADP5055_MSB_MASK			NO_OS_GENMASK(15, 8)
#define ADP5055_LSB_MASK			NO_OS_GENMASK(7, 0)

/* PMBus Addresses */
#define ADP5055_PMBUS_GROUNDED_CFG2_ADDRESS	0x70
#define ADP5055_PMBUS_14P3KOHM_CFG2_ADDRESS	0x71
#define ADP5055_PMBUS_16P9KOHM_CFG2_ADDRESS	0x72
#define ADP5055_PMBUS_20KOHM_CFG2_ADDRESS	0x73
#define ADP5055_PMBUS_23P7KOHM_CFG2_ADDRESS	0x70
#define ADP5055_PMBUS_32P4KOHM_CFG2_ADDRESS	0x71
#define ADP5055_PMBUS_39P2KOHM_CFG2_ADDRESS	0x73
#define ADP5055_PMBUS_OPEN_CFG2_ADDRESS		0x70
#define ADP5055_PMBUS_47P5KOHM_CFG2_ADDRESS	0x71
#define ADP5055_PMBUS_57P6KOHM_CFG2_ADDRESS	0x72
#define ADP5055_PMBUS_71P5KOHM_CFG2_ADDRESS	0x73
#define ADP5055_PMBUS_90P9KOHM_CFG2_ADDRESS	0x70
#define ADP5055_PMBUS_127KOHM_CFG2_ADDRESS	0x71
#define ADP5055_PMBUS_200KOHM_CFG2_ADDRESS	0x72
#define ADP5055_PMBUS_511KOHM_CFG2_ADDRESS	0x73

/* PMBus COMMAND SET */
#define ADP5055_CAPABILITY			0x19
#define ADP5055_STATUS_CML			0x7E
#define ADP5055_MODEL_ID			0xD0
#define ADP5055_CTRL123				0xD1
#define ADP5055_VID_GO				0xD2
#define ADP5055_CTRL_MODE1			0xD3
#define ADP5055_CTRL_MODE2			0xD4
#define ADP5055_DLY1				0xD5
#define ADP5055_DLY2				0xD6
#define ADP5055_DLY3				0xD7
#define ADP5055_VID1				0xD8
#define ADP5055_VID2				0xD9
#define ADP5055_VID3				0xDA
#define ADP5055_DVS_CFG				0xDB
#define ADP5055_DVS_LIM1			0xDC
#define ADP5055_DVS_LIM2			0xDD
#define ADP5055_DVS_LIM3			0xDE
#define ADP5055_FT_CFG				0xDF
#define ADP5055_PG_CFG				0xE0
#define ADP5055_PG_READ				0xE1
#define ADP5055_STATUS_LCH			0xE2

/* MASKS */
#define ADP5055_PEC_CAPABILITY			NO_OS_BIT(7)
#define ADP5055_MAX_BUS_SPEED_CAPABILITY	NO_OS_GENMASK(6, 5)
#define ADP5055_SMB_ALRT_CAPABILITY		NO_OS_BIT(4)

#define ADP5055_STATUS_CML_CMD_ERR		NO_OS_BIT(7)
#define ADP5055_STATUS_CML_DATA_ERR		NO_OS_BIT(6)
#define ADP5055_STATUS_CML_PEC_ERR		NO_OS_BIT(5)
#define ADP5055_STATUS_CML_CRC_ERR		NO_OS_BIT(4)
#define ADP5055_STATUS_CML_COMM_ERR		NO_OS_BIT(1)

#define ADP5055_CTRL123_CH(ch)			NO_OS_BIT(ch - 1)

#define ADP5055_VID_GO_CH(ch)			NO_OS_BIT(ch - 1)

#define ADP5055_CTRL_MODE1_DVS_AUTO		NO_OS_BIT(4)
#define ADP5055_CTRL_MODE1_EN_MODE		NO_OS_GENMASK(1, 0)

#define ADP5055_CTRL_MODE2_OCP_BLANKING		NO_OS_BIT(7)	
#define ADP5055_CTRL_MODE2_PSM_ON(ch)		NO_OS_BIT(ch + 3)
#define ADP5055_CTRL_MODE2_DSCHG_ON(ch)		NO_OS_BIT(ch - 1)

#define ADP5055_DIS_DLY				NO_OS_GENMASK(6, 4)
#define ADP5055_EN_DLY				NO_OS_GENMASK(2, 0)

#define ADP5055_DVS_INTVAL(ch)			NO_OS_GENMASK(2 * ch - 1, 2 * ch - 2)

#define ADP5055_VID_HIGH			NO_OS_GENMASK(7, 4)
#define ADP5055_VID_LOW				NO_OS_GENMASK(3, 0)

#define ADP5055_FT_CFG_TH(ch)			NO_OS_GENMASK(2 * ch - 1, 2 * ch - 2)

#define ADP5055_PG_CFG_PG_ON(ch)		NO_OS_BIT(ch - 1)
#define ADP5055_PG_CFG_PWRGD_DLY		NO_OS_BIT(4)

#define ADP5055_PG_READ_PWRGD(ch)		NO_OS_BIT(ch - 1)

#define ADP5055_STATUS_OCP_LCH(ch)		NO_OS_BIT(ch + 3)
#define ADP5055_STATUS_INT_LCH			NO_OS_BIT(7)
#define ADP5055_STATUS_PG_LCH(ch)		NO_OS_BIT(ch - 1)
#define ADP5055_STATUS_TSD_LCH			NO_OS_BIT(3)

/* Others */
#define ADP5055_MIN_CHANNELS			1
#define ADP5055_MAX_CHANNELS			3
#define ADP5055_VID_OFFSET			408e-3
#define ADP5055_VID_STEP			1.5e-3
#define ADP5055_VID_LOW_LIM_OFFSET		-190.5e-3
#define ADP5055_VID_HIGH_LIM_OFFSET		192e-3
#define ADP5055_VID_LIM_STEP			12e-3
#define ADP5055_VID_DEFAULT			600e-3
#define VREF_TRIM				0

/* Enums */
enum adp5055_cfg_valid_resistances {
	ADP5055_ZERO_OHM,
	ADP5055_14P3KOHM,
	ADP5055_16P9KOHM,
	ADP5055_20KOHM,
	ADP5055_23P7KOHM,
	ADP5055_32P4KOHM,
	ADP5055_39P2KOHM,
	ADP5055_47P5KOHM,
	ADP5055_57P6KOHM,
	ADP5055_71P5KOHM,
	ADP5055_90P9KOHM,
	ADP5055_127KOHM,
	ADP5055_200KOHM,
	ADP5055_511KOHM,
	ADP5055_OPEN,	
};

enum adp5055_output_capability {
	ADP5055_7A,
	ADP5055_3A,
	ADP5055_3P5A,
	ADP5055_1P5A,
	ADP5055_INTERLEAVED_PARALLEL_14A,
	ADP5055_IN_PHASE_PARALLEL_14A,
};

enum adp5055_status_cml_errors {
	ADP5055_CMD_ERR,
	ADP5055_DATA_ERR,
	ADP5055_PEC_ERR,
	ADP5055_CRC_ERR,
	ADP5055_COMM_ERR,
};

enum adp5055_en_mode {
	ADP5055_ENX_CONTROL,
	ADP5055_CHX_CONTROL,
	ADP5055_ENX_CHX_CONTROL,
	ADP5055_ENX_OR_CHX_CONTROL,
};

enum adp5055_disable_delays {
	ADP5055_DISABLE_DELAY_0MS,
	ADP5055_DISABLE_DELAY_2X_TSET,
	ADP5055_DISABLE_DELAY_4X_TSET,
	ADP5055_DISABLE_DELAY_6X_TSET,
	ADP5055_DISABLE_DELAY_8X_TSET,
	ADP5055_DISABLE_DELAY_10X_TSET,
	ADP5055_DISABLE_DELAY_12X_TSET,
	ADP5055_DISABLE_DELAY_14X_TSET,
};

enum adp5055_enable_delays {
	ADP5055_ENABLE_DELAY_0MS,
	ADP5055_ENABLE_DELAY_1X_TSET,
	ADP5055_ENABLE_DELAY_2X_TSET,
	ADP5055_ENABLE_DELAY_3X_TSET,
	ADP5055_ENABLE_DELAY_4X_TSET,
	ADP5055_ENABLE_DELAY_5X_TSET,
	ADP5055_ENABLE_DELAY_6X_TSET,
	ADP5055_ENABLE_DELAY_7X_TSET,
};

enum adp5055_dvs_interval {
	ADP5055_125US_12MV_PER_MS_SLEW,
	ADP5055_62P5US_24MV_PER_MS_SLEW,
	ADP5055_31P25US_48MV_PER_MS_SLEW,
	ADP5055_15P625US_96MV_PER_MS_SLEW,
};

enum adp5055_fast_transient_sensitivity {
	ADP5055_NO_FAST_TRANSIENT,
	ADP5055_1P5_WINDOW_3GM,
	ADP5055_1P5_WINDOW_5GM,
	ADP5055_2P5_WINDOW_5GM,
};

enum adp5055_gpio_mode {
	ADP5055_GPIO_SYNC_MODE,
	ADP5055_GPIO_CLOCK_OUT,
};

/**
 * @brief Initialization parameter for the ADP5055 device.
*/
struct adp5055_init_param {
	struct no_os_i2c_init_param *i2c_param;
	struct no_os_gpio_init_param *en_param[ADP5055_MAX_CHANNELS];
	struct no_os_gpio_init_param *pwrgd_gpio_param;
	enum adp5055_cfg_valid_resistances rcfg1;
	enum adp5055_cfg_valid_resistances rcfg2;
	float *vid_low;
	float *vid_high;
	float rtop[ADP5055_MAX_CHANNELS];
	float rbottom[ADP5055_MAX_CHANNELS];
};

/**
 * @brief Device descriptor for ADP5055.
*/
struct adp5055_desc {
	struct no_os_i2c_desc *i2c_desc;
	struct no_os_gpio_desc *en_desc[ADP5055_MAX_CHANNELS];
	struct no_os_gpio_desc *pwrgd_gpio_desc;
	enum adp5055_output_capability out_capability[ADP5055_MAX_CHANNELS];
	enum adp5055_gpio_mode sync_mode;
	float vid[ADP5055_MAX_CHANNELS];
	float vout[ADP5055_MAX_CHANNELS];
	uint16_t tset_us;
	uint8_t *vid_low;
	uint8_t *vid_high;
	bool is_fast_transient_enabled;
};

/** Send command to ADP5055 device. */
int adp5055_send_command(struct adp5055_desc *desc, uint16_t command);

/** Read command from ADP5055 device. */
int adp5055_read(struct adp5055_desc *desc, uint16_t command, uint8_t *data,
 uint8_t bytes_number);

/** Write command to ADP5055 device. */
int adp5055_write(struct adp5055_desc *desc, uint16_t command, uint16_t data,
  uint8_t bytes_number);

/** Initialize the ADP5055 device descriptor. */
int adp5055_init(struct adp5055_desc **desc,
 struct adp5055_init_param *init_param);

/** Remove resources allocated by the init function. */
int adp5055_remove(struct adp5055_desc *desc);

/** Read PEC capability from ADP5055 device. */
int adp5055_read_pec_capability(struct adp5055_desc *desc, bool *pec);

int adp5055_read_max_bus_speed(struct adp5055_desc *desc, bool *is_400kHz);

int adp5055_read_smb_alert_capability(struct adp5055_desc *desc, bool *smb_alert);

int adp5055_read_status_cml(struct adp5055_desc *desc, enum adp5055_status_cml_errors status,
			bool *is_err_raised);

int adp5055_read_model_id(struct adp5055_desc *desc, uint8_t *model_id);

int adp5055_read_channel_enable(struct adp5055_desc *desc, uint8_t channel,
			 bool *is_enabled);

int adp5055_set_channel_enable(struct adp5055_desc *desc, uint8_t channel,
			 bool is_enabled);

int adp5055_gpio_set_enable(struct adp5055_desc *desc, uint8_t channel, bool is_enabled);

int adp5055_set_vid_go(struct adp5055_desc *desc, uint8_t channel);

int adp5055_read_vidx_execution_mode(struct adp5055_desc *desc, bool *is_write_init);

int adp5055_set_vidx_execution_mode(struct adp5055_desc *desc, bool is_write_init);

int adp5055_read_en_mode(struct adp5055_desc *desc, enum adp5055_en_mode *mode);

int adp5055_set_en_mode(struct adp5055_desc *desc, enum adp5055_en_mode mode);

int adp5055_read_ocp_blanking(struct adp5055_desc *desc, bool *ocp_blanking);

int adp5055_set_ocp_blanking(struct adp5055_desc *desc, bool ocp_blanking);

int adp5055_read_psm_on(struct adp5055_desc *desc, uint8_t channel, bool *psm_on);

int adp5055_set_psm_on(struct adp5055_desc *desc, uint8_t channel, bool psm_on);

int adp5055_read_dschg_on(struct adp5055_desc *desc, uint8_t channel, bool *dschg_on);

int adp5055_set_dschg_on(struct adp5055_desc *desc, uint8_t channel, bool dschg_on);

int adp5055_read_disable_delay(struct adp5055_desc *desc, uint8_t channel,
	enum adp5055_disable_delays *delay);

int adp5055_set_disable_delay(struct adp5055_desc *desc, uint8_t channel,
	enum adp5055_disable_delays delay);

int adp5055_read_enable_delay(struct adp5055_desc *desc, uint8_t channel,
	enum adp5055_enable_delays *delay);

int adp5055_set_enable_delay(struct adp5055_desc *desc, uint8_t channel,
	enum adp5055_enable_delays delay);

int adp5055_read_raw_vid(struct adp5055_desc *desc, uint8_t channel, uint8_t *vid);

int adp5055_read_converted_vid(struct adp5055_desc *desc, uint8_t channel, float *vid);

int adp5055_set_raw_vid(struct adp5055_desc *desc, uint8_t channel, uint8_t vid);

int adp5055_set_converted_vid(struct adp5055_desc *desc, uint8_t channel, float vid);

int adp5055_read_dvs_intval(struct adp5055_desc *desc, uint8_t channel,
		enum adp5055_dvs_interval *dvs_intval);

int adp5055_set_dvs_intval(struct adp5055_desc *desc, uint8_t channel,
		enum adp5055_dvs_interval dvs_intval);

int adp5055_read_vid_high_lim(struct adp5055_desc *desc, uint8_t channel, uint8_t *lim);

int adp5055_read_converted_vid_high_lim(struct adp5055_desc *desc, uint8_t channel, float *lim);

int adp5055_set_vid_high_lim(struct adp5055_desc *desc, uint8_t channel, uint8_t lim);

int adp5055_set_converted_vid_high_lim(struct adp5055_desc *desc, uint8_t channel, float lim);

int adp5055_read_vid_low_lim(struct adp5055_desc *desc, uint8_t channel, uint8_t *lim);

int adp5055_read_converted_vid_low_lim(struct adp5055_desc *desc, uint8_t channel, float *lim);

int adp5055_set_vid_low_lim(struct adp5055_desc *desc, uint8_t channel, uint8_t lim);

int adp5055_set_converted_vid_low_lim(struct adp5055_desc *desc, uint8_t channel, float lim);

int adp5055_read_fast_transient_settings(struct adp5055_desc *desc, uint8_t channel,
	enum adp5055_fast_transient_sensitivity *ft_cfg);

int adp5055_set_fast_transient_settings(struct adp5055_desc *desc, uint8_t channel,
	enum adp5055_fast_transient_sensitivity ft_cfg);

int adp5055_read_pwrgd_delay(struct adp5055_desc *desc, bool *pwrgd_delay_enabled);

int adp5055_set_pwrgd_delay(struct adp5055_desc *desc, bool pwrgd_delay_enabled);

int adp5055_read_pg_mask(struct adp5055_desc *desc, uint8_t channel, bool *is_masked);

int adp5055_set_pg_mask(struct adp5055_desc *desc, uint8_t channel, bool is_masked);

int adp5055_read_pwrgd_status(struct adp5055_desc *desc, uint8_t channel, 
	bool *is_out_nominal);

int adp5055_read_init_proc_latch_status(struct adp5055_desc *desc, bool *int_lch);

int adp5055_clear_init_proc_latch_status(struct adp5055_desc *desc);

int adp5055_read_over_current_hiccup_latch_status(struct adp5055_desc *desc, uint8_t channel, 
		bool *hiccup);

int adp5055_clear_over_current_hiccup(struct adp5055_desc *desc, uint8_t channel);

int adp5055_read_thermal_shutdown_latch_status(struct adp5055_desc *desc, bool *tsd_lch);

int adp5055_clear_thermal_shutdown_latch_status(struct adp5055_desc *desc);

int adp5055_read_power_good_latch_status(struct adp5055_desc *desc, uint8_t channel,
		 bool *pg_lch);

int adp5055_clear_power_good_latch_status(struct adp5055_desc *desc, uint8_t channel);

#endif /** __ADP_5055_H__ */
