/***************************************************************************//**
 *   @file   adp5055.c
 *   @brief  Source file for the ADP5055 Driver
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
#include <stdlib.h>
#include <string.h>
#include "adp5055.h"
#include "no_os_alloc.h"
#include "no_os_delay.h"
#include "no_os_error.h"

static const uint8_t adp5055_commands[] = {
	ADP5055_CAPABILITY,
	ADP5055_STATUS_CML,
	ADP5055_MODEL_ID,
	ADP5055_CTRL123,
	ADP5055_VID_GO,
	ADP5055_CTRL_MODE1,
	ADP5055_CTRL_MODE2,
	ADP5055_DLY1,
	ADP5055_DLY2,
	ADP5055_DLY3,
	ADP5055_VID1,
	ADP5055_VID2,
	ADP5055_VID3,
	ADP5055_DVS_CFG,
	ADP5055_DVS_LIM1,
	ADP5055_DVS_LIM2,
	ADP5055_DVS_LIM3,
	ADP5055_FT_CFG,
	ADP5055_PG_CFG,
	ADP5055_PG_READ,
	ADP5055_STATUS_LCH,
};

/**
 * @brief Send command byte/word to ADP5055
 * @param desc - ADP5055 device descriptor
 * @param command - Value of the command.
 * @return 0 in case of succes, negative error code otherwise
*/
int adp5055_send_command(struct adp5055_desc *desc, uint16_t command)
{
	int i;
	uint8_t command_val;

	if (!desc)
		return -EINVAL;

	for (i = 0; i < NO_OS_ARRAY_SIZE(adp5055_commands); i++) {
		if (adp5055_commands[i] == command)
			break;
	}

	if (i >= NO_OS_ARRAY_SIZE(adp5055_commands))
		return -EINVAL;

	command_val = no_os_field_get(ADP5055_LSB_MASK, command);

	return no_os_i2c_write(desc->i2c_desc, &command_val, 1, 1);
}

/**
 * @brief Read data from ADP5055
 * @param desc - ADP5055 device descriptor
 * @param command - Command value.
 * @param data - Buffer with received data.
 * @param bytes_number - Number of bytes to read.
 * @return 0 in case of succes, negative error code otherwise
*/
int adp5055_read(struct adp5055_desc *desc, uint16_t command, uint8_t *data,
		 uint8_t bytes_number)
{
	int i, ret;
	uint8_t command_val[1] = {0};

	if (!desc)
		return -EINVAL;

	for (i = 0; i < NO_OS_ARRAY_SIZE(adp5055_commands); i++) {
		if (adp5055_commands[i] == command)
			break;
	}

	if (i >= NO_OS_ARRAY_SIZE(adp5055_commands))
		return -EINVAL;

	command_val[0] = no_os_field_get(ADP5055_LSB_MASK, command);

	ret = no_os_i2c_write(desc->i2c_desc, command_val, 1, 0);
	if (ret)
		return ret;

	return no_os_i2c_read(desc->i2c_desc, data, bytes_number, 1);
}

/**
 * @brief Write data to ADP5055
 * @param desc - ADP5055 device descriptor
 * @param command - Command value
 * @param data - Data value to write to the ADP5055. Can be either just a byte
 * 		 or a word.
 * @param bytes_number - Number of bytes to write.
 * @return 0 in case of succes, negative error code otherwise
*/
int adp5055_write(struct adp5055_desc *desc, uint16_t command, uint16_t data,
		  uint8_t bytes_number)
{
	int i;
	uint8_t val[2] = {0, 0};

	if (!desc)
		return -EINVAL;

	for (i = 0; i < NO_OS_ARRAY_SIZE(adp5055_commands); i++) {
		if (adp5055_commands[i] == command)
			break;
	}

	if (i >= NO_OS_ARRAY_SIZE(adp5055_commands))
		return -EINVAL;

	val[0] = no_os_field_get(ADP5055_LSB_MASK, command);
	val[1] = no_os_field_get(ADP5055_LSB_MASK, data);

	return no_os_i2c_write(desc->i2c_desc, val, bytes_number + 1, 1);
}

/**
 * @brief Updates the ADP5055 device with the specified command, mask, and value.
 *
 * This function updates the ADP5055 device with the specified command, mask, and value.
 * It first reads the current data from the specified command, then applies the mask and value
 * to the data, and finally writes the updated data back to the device.
 *
 * @param desc Pointer to the ADP5055 device descriptor.
 * @param command The command to update.
 * @param mask The mask to apply to the data.
 * @param value The value to set in the data.
 * @return 0 on success, or a negative error code on failure.
 *         -EINVAL is returned if the descriptor is NULL.
 */
int adp5055_update(struct adp5055_desc *desc, uint16_t command, uint16_t mask,
		   uint16_t value)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	ret = adp5055_read(desc, command, &data, 1);
	if (ret)
		return ret;

	data &= ~mask;
	data |= value;
	
	return adp5055_write(desc, command, data, 1);
}


/**
 * @brief Generates the configuration value for the ADP5055 device.
 *
 * This function generates the configuration value for the ADP5055 device based on the specified
 * valid resistances for CFG1 and CFG2. The generated configuration value is stored in the `addr`
 * parameter.
 *
 * @param desc Pointer to the ADP5055 device descriptor.
 * @param cfg1 The valid resistances for CFG1.
 * @param cfg2 The valid resistances for CFG2.
 * @param addr Pointer to store the corresponding i2c address
 * @return None.
 */
int adp5055_rcfg_gen(struct adp5055_desc *desc, enum adp5055_cfg_valid_resistances cfg1,
	enum adp5055_cfg_valid_resistances cfg2, uint8_t *addr)
{
	if (!desc)
		return -EINVAL;

	/* tset setting */
	if (cfg2 == ADP5055_OPEN || cfg2 < ADP5055_47P5KOHM)
		desc->tset_us = 2600;
	else
		desc->tset_us = 20800;

	/* cfg1 based parameters */
	switch (cfg1) {
	case ADP5055_ZERO_OHM:
		desc->out_capability[0] = ADP5055_7A;
		desc->out_capability[1] = ADP5055_7A;
		desc->out_capability[2] = ADP5055_3A;
		desc->sync_mode = ADP5055_GPIO_SYNC_MODE;
		break;
	case ADP5055_14P3KOHM:
		desc->out_capability[0] = ADP5055_7A;
		desc->out_capability[1] = ADP5055_7A;
		desc->out_capability[2] = ADP5055_1P5A;
		desc->sync_mode = ADP5055_GPIO_SYNC_MODE;
		break;
	case ADP5055_16P9KOHM:
		desc->out_capability[0] = ADP5055_7A;
		desc->out_capability[1] = ADP5055_3P5A;
		desc->out_capability[2] = ADP5055_3A;
		desc->sync_mode = ADP5055_GPIO_SYNC_MODE;
		break;
	case ADP5055_20KOHM:
		desc->out_capability[0] = ADP5055_7A;
		desc->out_capability[1] = ADP5055_3P5A;
		desc->out_capability[2] = ADP5055_1P5A;
		desc->sync_mode = ADP5055_GPIO_SYNC_MODE;
		break;
	case ADP5055_23P7KOHM:
		desc->out_capability[0] = ADP5055_INTERLEAVED_PARALLEL_14A;
		desc->out_capability[1] = ADP5055_INTERLEAVED_PARALLEL_14A;
		desc->out_capability[2] = ADP5055_3A;
		desc->sync_mode = ADP5055_GPIO_SYNC_MODE;
		break;
	case ADP5055_OPEN:
		desc->out_capability[0] = ADP5055_3P5A;
		desc->out_capability[1] = ADP5055_3P5A;
		desc->out_capability[2] = ADP5055_1P5A;
		desc->sync_mode = ADP5055_GPIO_SYNC_MODE;
		break;
	case ADP5055_32P4KOHM:
		desc->out_capability[0] = ADP5055_IN_PHASE_PARALLEL_14A;
		desc->out_capability[1] = ADP5055_IN_PHASE_PARALLEL_14A;
		desc->out_capability[2] = ADP5055_3A;
		desc->sync_mode = ADP5055_GPIO_SYNC_MODE;
		break;
	case ADP5055_39P2KOHM:
		desc->out_capability[0] = ADP5055_7A;
		desc->out_capability[1] = ADP5055_7A;
		desc->out_capability[2] = ADP5055_3A;
		desc->sync_mode = ADP5055_GPIO_CLOCK_OUT;
		break;
	case ADP5055_47P5KOHM:
		desc->out_capability[0] = ADP5055_7A;
		desc->out_capability[1] = ADP5055_7A;
		desc->out_capability[2] = ADP5055_1P5A;
		desc->sync_mode = ADP5055_GPIO_CLOCK_OUT;
		break;
	case ADP5055_57P6KOHM:
		desc->out_capability[0] = ADP5055_7A;
		desc->out_capability[1] = ADP5055_3P5A;
		desc->out_capability[2] = ADP5055_3A;
		desc->sync_mode = ADP5055_GPIO_CLOCK_OUT;
		break;
	case ADP5055_71P5KOHM:
		desc->out_capability[0] = ADP5055_7A;
		desc->out_capability[1] = ADP5055_3P5A;
		desc->out_capability[2] = ADP5055_1P5A;
		desc->sync_mode = ADP5055_GPIO_CLOCK_OUT;
		break;
	case ADP5055_90P9KOHM:
		desc->out_capability[0] = ADP5055_3P5A;
		desc->out_capability[1] = ADP5055_7A;
		desc->out_capability[2] = ADP5055_3A;
		desc->sync_mode = ADP5055_GPIO_CLOCK_OUT;
		break;
	case ADP5055_127KOHM:
		desc->out_capability[0] = ADP5055_INTERLEAVED_PARALLEL_14A;
		desc->out_capability[1] = ADP5055_INTERLEAVED_PARALLEL_14A;
		desc->out_capability[2] = ADP5055_3A;
		desc->sync_mode = ADP5055_GPIO_CLOCK_OUT;
		break;
	case ADP5055_200KOHM:
		desc->out_capability[0] = ADP5055_3P5A;
		desc->out_capability[1] = ADP5055_3P5A;
		desc->out_capability[2] = ADP5055_1P5A;
		desc->sync_mode = ADP5055_GPIO_CLOCK_OUT;
		break;
	case ADP5055_511KOHM:
		desc->out_capability[0] = ADP5055_IN_PHASE_PARALLEL_14A;
		desc->out_capability[1] = ADP5055_IN_PHASE_PARALLEL_14A;
		desc->out_capability[2] = ADP5055_3A;
		desc->sync_mode = ADP5055_GPIO_CLOCK_OUT;
		break;
	default:
		return -EINVAL;
	}

	/* cfg2 based parameters */
	switch (cfg2) {
	case ADP5055_ZERO_OHM:
		desc->is_fast_transient_enabled = false;
		*addr = 0x70;
		break;
	case ADP5055_14P3KOHM:
		desc->is_fast_transient_enabled = false;
		*addr = 0x71;
		break;
	case ADP5055_16P9KOHM:
		desc->is_fast_transient_enabled = false;
		*addr = 0x72;
		break;
	case ADP5055_20KOHM:
		desc->is_fast_transient_enabled = false;
		*addr = 0x73;
		break;
	case ADP5055_23P7KOHM:
		desc->is_fast_transient_enabled = true;
		*addr = 0x70;
		break;
	case ADP5055_32P4KOHM:
		desc->is_fast_transient_enabled = true;
		*addr = 0x71;
		break;
	case ADP5055_39P2KOHM:
		desc->is_fast_transient_enabled = true;
		*addr = 0x73;
		break;
	case ADP5055_OPEN:
		desc->is_fast_transient_enabled = false;
		*addr = 0x70;
		break;
	case ADP5055_47P5KOHM:
		desc->is_fast_transient_enabled = false;
		*addr = 0x71;
		break;
	case ADP5055_57P6KOHM:
		desc->is_fast_transient_enabled = false;
		*addr = 0x72;
		break;
	case ADP5055_71P5KOHM:
		desc->is_fast_transient_enabled = false;
		*addr = 0x73;
		break;
	case ADP5055_90P9KOHM:
		desc->is_fast_transient_enabled = true;
		*addr = 0x70;
		break;
	case ADP5055_127KOHM:
		desc->is_fast_transient_enabled = true;
		*addr = 0x71;
		break;
	case ADP5055_200KOHM:
		desc->is_fast_transient_enabled = true;
		*addr = 0x72;
		break;
	case ADP5055_511KOHM:
		desc->is_fast_transient_enabled = true;
		*addr = 0x73;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief Initialize the ADP5055 device.
 * @param desc - ADP5055 device descriptor
 * @param init_param - Initialization parameter containing information about the
 * 		       ADP5055 device to be initialized.
 * @return 0 in case of succes, negative error code otherwise
*/
int adp5055_init(struct adp5055_desc **desc,
		 struct adp5055_init_param *init_param)
{
	struct adp5055_desc *descriptor;
	int i, j, ret;

	descriptor = (struct adp5055_desc *)no_os_calloc(sizeof(*descriptor), 1);
	if (!descriptor)
		return -ENOMEM;

	ret = adp5055_rcfg_gen(descriptor, init_param->rcfg1, init_param->rcfg2,
		&init_param->i2c_param->slave_address);
	if (ret)
		goto free_i2c_desc;

	/* Initialize i2c block */
	ret = no_os_i2c_init(&descriptor->i2c_desc, init_param->i2c_param);
	if (ret)
		goto free_i2c_desc;

	/* check for enable gpios specified */
	for (i = 0; i < ADP5055_MAX_CHANNELS; i++) {
		ret = no_os_gpio_get_optional(&descriptor->en_desc[i], init_param->en_param[i]);
		if (ret)
			goto free_en_gpio_desc;
	}

	/* PWRGD GPIO init */
	ret = no_os_gpio_get_optional(&descriptor->pwrgd_gpio_desc, init_param->pwrgd_gpio_param);
	if (ret)
		goto free_pwrgd_gpio_desc;

	/* Generate a VOUT table based on RTOP and RBOTTOM */
	/* Need RTOP and RBOTTOM info for initial VOUT state */
	if (!init_param->rtop || !init_param->rbottom)
		return -EINVAL;
	
	/* Assumes default VREF of 0.6V for all channels */		
	for (i = 0; i < ADP5055_MAX_CHANNELS; i++) {
		ret = adp5055_read_converted_vid(descriptor, i + 1, &descriptor->vout[i]);
		if (ret)
			goto free_pwrgd_gpio_desc;

		descriptor->vout[i] = descriptor->vout[i] * (1 + 
			(init_param->rtop[i] / init_param->rbottom[i]));
	}

	/* Configure VID HIGH and LOW if not empty */
	for (i = 1; i < ADP5055_MAX_CHANNELS + 1; i++) {
		if (init_param->vid_low && init_param->vid_high) {
			ret = adp5055_set_converted_vid_high_lim(descriptor, i, init_param->vid_high[i - 1]);
			if (ret)
				goto free_pwrgd_gpio_desc;
			descriptor->vid_high[i] = (init_param->vid_high[i - 1] - VREF_TRIM - ADP5055_VID_HIGH_LIM_OFFSET)
					/ -ADP5055_VID_LIM_STEP;

			ret = adp5055_set_converted_vid_low_lim(descriptor, i, init_param->vid_low[i - 1]);
			if (ret)
				goto free_pwrgd_gpio_desc;
			descriptor->vid_low[i] = (init_param->vid_low[i - 1] - VREF_TRIM - ADP5055_VID_LOW_LIM_OFFSET)
					/ ADP5055_VID_LIM_STEP;
		}
		else {
			ret = adp5055_set_vid_high_lim(descriptor, i, 0xf);
			if (ret)
				goto free_pwrgd_gpio_desc;
			descriptor->vid_low[i - 1] = 0xf;

			ret = adp5055_set_vid_low_lim(descriptor, i, 0x0);
			if (ret)
				goto free_pwrgd_gpio_desc;
			descriptor->vid_high[i - 1] = 0;
		}
	}

	/* Set vid members to default */
	for (i = 0; i < ADP5055_MAX_CHANNELS; i++)
		descriptor->vid[i] = ADP5055_VID_DEFAULT;

	*desc = descriptor;

	return 0;

free_pwrgd_gpio_desc:
	no_os_gpio_remove(descriptor->en_desc[i]);
free_en_gpio_desc:
	for (j = 0; j < i; j++)
		no_os_gpio_remove(descriptor->en_desc[j]);
free_i2c_desc:
	no_os_i2c_remove(descriptor->i2c_desc);

	no_os_free(descriptor);

	return ret;
}

/**
 * @brief Free the resources allocated by the adp5055_init()
 * @param desc - ADP5055 device descriptor
 * @return- 0 in case of succes, negative error code otherwise
*/
int adp5055_remove(struct adp5055_desc *desc)
{
	int i;

	if (!desc)
		return -ENODEV;

	no_os_i2c_remove(desc->i2c_desc);

	for (i = 0; i < ADP5055_MAX_CHANNELS; i++) 
		no_os_gpio_remove(desc->en_desc[i]);

	no_os_free(desc);

	return 0;
}

/**
 * @brief Reads the PEC (Packet Error Checking) capability of the ADP5055 device.
 * @param desc - ADP5055 device descriptor.
 * @param pec - Pointer to a boolean where the PEC capability status will be stored.
 *              True if PEC is supported, false otherwise.
 * @return 0 in case of success, negative error code otherwise.
 */
int adp5055_read_pec_capability(struct adp5055_desc *desc, bool *pec)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_CAPABILITY, &data, 1);
	if (ret)
		return ret;

	*pec = (bool) no_os_field_get(ADP5055_PEC_CAPABILITY, data);

	return 0;
}

/**
 * @brief Reads the maximum bus speed capability of the ADP5055 device.
 * @param desc - ADP5055 device descriptor.
 * @param is_400kHz - Pointer to a boolean where the maximum bus speed capability will be stored.
 *                    True if the device supports 400kHz, false otherwise.
 * @return 0 in case of success, negative error code otherwise.
 */
int adp5055_read_max_bus_speed(struct adp5055_desc *desc, bool *is_400kHz)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_CAPABILITY, &data, 1);
	if (ret)
		return ret;

	*is_400kHz = (bool) no_os_field_get(ADP5055_MAX_BUS_SPEED_CAPABILITY, data);

	return 0;
}

/**
 * @brief Reads the SMBus alert capability of the ADP5055 device.
 * @param desc - ADP5055 device descriptor.
 * @param smb_alert - Pointer to a boolean where the SMBus alert capability status will be stored.
 *                    True if SMBus alert is supported, false otherwise.
 * @return 0 in case of success, negative error code otherwise.
 */
int adp5055_read_smb_alert_capability(struct adp5055_desc *desc, bool *smb_alert)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_CAPABILITY, &data, 1);
	if (ret)
		return ret;

	*smb_alert = (bool) no_os_field_get(ADP5055_SMB_ALRT_CAPABILITY, data);

	return 0;
}

/**
 * @brief Reads the specified Communication, Memory, or Logic (CML) error status from the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param status - The specific CML error status to check, as defined by the enum adp5055_status_cml_errors.
 * @param is_err_raised - Pointer to a boolean where the result will be stored. True if the specified CML error is raised, false otherwise.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL or the status parameter is invalid, or other negative error code for communication errors.
 */
int adp5055_read_status_cml(struct adp5055_desc *desc, enum adp5055_status_cml_errors status,
			bool *is_err_raised)
{
	int ret;
	uint8_t data;

	if (!desc || !status)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_STATUS_CML, &data, 1);
	if (ret)
		return ret;

	switch (status) {
	case ADP5055_CMD_ERR:
		*is_err_raised = (bool) no_os_field_get(ADP5055_STATUS_CML_CMD_ERR, data);
		break;
	case ADP5055_DATA_ERR:
		*is_err_raised = (bool) no_os_field_get(ADP5055_STATUS_CML_DATA_ERR, data);
		break;
	case ADP5055_PEC_ERR:
		*is_err_raised = (bool) no_os_field_get(ADP5055_STATUS_CML_PEC_ERR, data);
		break;
	case ADP5055_CRC_ERR:
		*is_err_raised = (bool) no_os_field_get(ADP5055_STATUS_CML_CRC_ERR, data);
		break;
	case ADP5055_COMM_ERR:
		*is_err_raised = (bool) no_os_field_get(ADP5055_STATUS_CML_COMM_ERR, data);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief Reads the model ID of the ADP5055 device.
 * @param desc - ADP5055 device descriptor.
 * @param model_id - Pointer to a uint8_t where the model ID will be stored.
 * @return 0 in case of success, negative error code otherwise.
 */
int adp5055_read_model_id(struct adp5055_desc *desc, uint8_t *model_id)
{
	if (!desc || !model_id)
		return -EINVAL;

	return adp5055_read(desc, ADP5055_MODEL_ID, model_id, 1);
}

/**
 * @brief Reads the enable status of a specific channel on the ADP5055 device.
 * @param desc - ADP5055 device descriptor.
 * @param channel - The channel number to check the enable status for.
 * @param is_enabled - Pointer to a boolean where the enable status of the channel will be stored.
 *                     True if the channel is enabled, false otherwise.
 * @return 0 in case of success, negative error code otherwise.
 */
int adp5055_read_channel_enable(struct adp5055_desc *desc, uint8_t channel,
			 bool *is_enabled)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_CTRL123, &data, 1);
	if (ret)
		return ret;

	*is_enabled = (bool) no_os_field_get(ADP5055_CTRL123_CH(channel), data);

	return 0;
}

/**
 * @brief Sets the enable status of a specific channel on the ADP5055 device.
 * @param desc - ADP5055 device descriptor.
 * @param channel - The channel number to set the enable status for.
 * @param is_enabled - Boolean indicating the desired enable status of the channel.
 *                     True to enable the channel, false to disable it.
 * @return 0 in case of success, negative error code otherwise.
 */
int adp5055_set_channel_enable(struct adp5055_desc *desc, uint8_t channel,
			 bool is_enabled)
{
	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	return adp5055_update(desc, ADP5055_CTRL123, ADP5055_CTRL123_CH(channel),
		no_os_field_prep(ADP5055_CTRL123_CH(channel), is_enabled));
}

/**
 * @brief Sets the enable state of a GPIO for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to set the GPIO enable state.
 *                  Valid channels are defined by ADP5055_MIN_CHANNELS and ADP5055_MAX_CHANNELS.
 * @param is_enabled - Boolean indicating the desired enable state of the GPIO for the specified channel.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or the channel's enable descriptor is NULL.
 */
int adp5055_gpio_set_enable(struct adp5055_desc *desc, uint8_t channel, bool is_enabled)
{
	if (!desc || !desc->en_desc[channel])
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	return no_os_gpio_set_value(desc->en_desc[channel], is_enabled);
}

/**
 * @brief Triggers the Voltage transition for a specified channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which the VID update process is to be triggered.
 *                  Must be within the range defined by ADP5055_MIN_CHANNELS and ADP5055_MAX_CHANNELS.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL or the channel number is out of range.
 */
int adp5055_set_vid_go(struct adp5055_desc *desc, uint8_t channel)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	// check if DVS_AUTO is enabled
	ret = adp5055_read_vidx_execution_mode(desc, (bool *) &data);
	if (ret)
		return ret;
	
	// if DVS_AUTO is disabled, then the VIDx_GO trigger attempt is ignored
	if (!data)
		return -EINVAL;

	return adp5055_update(desc, ADP5055_VID_GO, ADP5055_VID_GO_CH(channel),
		no_os_field_prep(ADP5055_VID_GO_CH(channel), 1));
}

/**
 * @brief Reads the VIDx execution mode from the ADP5055 device.
 *
 * This function reads the VIDx execution mode from the ADP5055 device and
 * updates the value of the boolean variable `is_write_init` accordingly.
 *
 * @param desc Pointer to the ADP5055 device descriptor.
 * @param is_write_init Pointer to a boolean variable that will be updated
 *                      with the VIDx execution mode.
 * @return 0 on success, negative error code on failure.
 *         -EINVAL if `desc` is NULL.
 *         Error code returned by `adp5055_read()` if the read operation fails.
 */
int adp5055_read_vidx_execution_mode(struct adp5055_desc *desc, bool *is_write_init)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_CTRL_MODE1, &data, 1);
	if (ret)
		return ret;

	*is_write_init = (bool) no_os_field_get(ADP5055_CTRL_MODE1_DVS_AUTO, data);

	return 0;
}

/**
 * @brief Sets the VIDX execution mode on the ADP5055 device.
 * @param desc - ADP5055 device descriptor.
 * @param is_write_init - Boolean indicating the desired VIDX execution mode.
 *                        True for VIDx_GO bit initiated transition.
 * 			  False for ignoring VIDx_GO bit.
 * @return 0 in case of success, negative error code otherwise.
 */
int adp5055_set_vidx_execution_mode(struct adp5055_desc *desc, bool is_write_init)
{
	if (!desc)
		return -EINVAL;

	return adp5055_update(desc, ADP5055_CTRL_MODE1, ADP5055_CTRL_MODE1_DVS_AUTO,
		no_os_field_prep(ADP5055_CTRL_MODE1_DVS_AUTO, is_write_init));
}

/**
 * @brief Reads the enable mode setting from the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param mode - Pointer to an enum adp5055_en_mode variable where the enable mode will be stored.
 *               The enable mode determines how the device is enabled or disabled.
 * @return 0 in case of success, negative error code otherwise. Returns -EINVAL if the descriptor is NULL.
 */
int adp5055_read_en_mode(struct adp5055_desc *desc, enum adp5055_en_mode *mode)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_CTRL_MODE1, &data, 1);
	if (ret)
		return ret;

	*mode = no_os_field_get(ADP5055_CTRL_MODE1_EN_MODE, data);

	return 0;
}

/**
 * Sets the enable mode of the ADP5055 device.
 *
 * @param desc Pointer to the ADP5055 device descriptor.
 * @param mode The enable mode to be set.
 * @return 0 on success, negative error code on failure.
 */
int adp5055_set_en_mode(struct adp5055_desc *desc, enum adp5055_en_mode mode)
{
	if (!desc)
		return -EINVAL;

	if (mode < ADP5055_ENX_CONTROL || mode > ADP5055_ENX_OR_CHX_CONTROL)
		return -EINVAL;

	return adp5055_update(desc, ADP5055_CTRL_MODE1, ADP5055_CTRL_MODE1_EN_MODE,
		no_os_field_prep(ADP5055_CTRL_MODE1_EN_MODE, mode));
}


/**
 * @brief Reads the OCP_BLANKING setting from the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param ocp_blanking - Pointer to a boolean where the OCP_BLANKING setting will be stored.
 *                       True if OCP_BLANKING is enabled, false otherwise.
 * @return 0 in case of success, negative error code otherwise. Returns -EINVAL if the descriptor is NULL.
 */
int adp5055_read_ocp_blanking(struct adp5055_desc *desc, bool *ocp_blanking)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_CTRL_MODE2, &data, 1);
	if (ret)
		return ret;

	*ocp_blanking = (bool) no_os_field_get(ADP5055_CTRL_MODE2_OCP_BLANKING, data);

	return 0;
}

/**
 * @brief Updates the OCP_BLANKING setting on the ADP5055 device.
 * @param desc - ADP5055 device descriptor.
 * @param ocp_blanking - Boolean indicating the desired OCP_BLANKING setting.
 *                       True to enable OCP_BLANKING, false to disable it.
 * @return 0 in case of success, negative error code otherwise.
 */
int adp5055_set_ocp_blanking(struct adp5055_desc *desc, bool ocp_blanking)
{
	if (!desc)
		return -EINVAL;

	return adp5055_update(desc, ADP5055_CTRL_MODE2, ADP5055_CTRL_MODE2_OCP_BLANKING,
		no_os_field_prep(ADP5055_CTRL_MODE2_OCP_BLANKING, ocp_blanking));
}

/**
 * @brief Reads the PSM_ON setting from the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number to read the PSM_ON setting for.
 * @param psm_on - Pointer to a boolean where the PSM_ON setting will be stored.
 *                 True if auto PWM/PSM is enabled, false for FPWM mode.
 * @return 0 in case of success, negative error code otherwise. Returns -EINVAL if the descriptor is NULL.
 */
int adp5055_read_psm_on(struct adp5055_desc *desc, uint8_t channel, bool *psm_on)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_CTRL_MODE2, &data, 1);
	if (ret)
		return ret;

	*psm_on = (bool) no_os_field_get(ADP5055_CTRL_MODE2_PSM_ON(channel), data);

	return 0;
}

/**
 * @brief Updates the PSM_ON setting on the ADP5055 device.
 * @param desc - ADP5055 device descriptor.
 * @param channel - The channel number to update the PSM_ON setting for.
 * @param psm_on - Boolean indicating the desired PSM_ON setting.
 *                 True to enable auto PWM/PSM, false for FPWM mode.
 * @return 0 in case of success, negative error code otherwise.
 */
int adp5055_set_psm_on(struct adp5055_desc *desc, uint8_t channel, bool psm_on)
{
	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	return adp5055_update(desc, ADP5055_CTRL_MODE2, ADP5055_CTRL_MODE2_PSM_ON(channel),
		no_os_field_prep(ADP5055_CTRL_MODE2_PSM_ON(channel), psm_on));
}

/**
 * @brief Reads the DSCHG_ON setting from the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number to read the DSCHG_ON setting for.
 * @param dschg_on - Pointer to a boolean where the DSCHG_ON setting will be stored.
 *                   True if the channel is enabled, false otherwise.
 * @return 0 in case of success, negative error code otherwise. Returns -EINVAL if the descriptor is NULL.
 */
int adp5055_read_dschg_on(struct adp5055_desc *desc, uint8_t channel, bool *dschg_on)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_CTRL_MODE2, &data, 1);
	if (ret)
		return ret;

	*dschg_on = (bool) no_os_field_get(ADP5055_CTRL_MODE2_DSCHG_ON(channel), data);

	return 0;
}

/**
 * @brief Updates the DSCHG_ON setting on the ADP5055 device.
 * @param desc - ADP5055 device descriptor.
 * @param channel - The channel number to update the DSCHG_ON setting for.
 * @param dschg_on - Boolean indicating the desired DSCHG_ON setting.
 *                   True to enable the channel, false otherwise.
 * @return 0 in case of success, negative error code otherwise.
 */
int adp5055_set_dschg_on(struct adp5055_desc *desc, uint8_t channel, bool dschg_on)
{
	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	return adp5055_update(desc, ADP5055_CTRL_MODE2, ADP5055_CTRL_MODE2_DSCHG_ON(channel),
		no_os_field_prep(ADP5055_CTRL_MODE2_DSCHG_ON(channel), dschg_on));
}

/**
 * @brief Reads the disable delay setting for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to read the disable delay setting.
 *                  Valid channels are 1, 2, and 3.
 * @param delay - Pointer to an enum adp5055_disable_delays variable where the disable delay setting will be stored.
 *                The disable delay determines the delay before the device disables the specified channel.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL or the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_read_disable_delay(struct adp5055_desc *desc, uint8_t channel,
	enum adp5055_disable_delays *delay)
{
	int ret;

	if (!desc)
		return -EINVAL;

	switch (channel) {
	case 1:
		ret = adp5055_read(desc, ADP5055_DLY1, delay, 1);
		break;
	case 2:
		ret = adp5055_read(desc, ADP5055_DLY2, delay, 1);
		break;
	case 3:
		ret = adp5055_read(desc, ADP5055_DLY3, delay, 1);
		break;
	default:
		return -EINVAL;
	}
	if (ret)
		return ret;

	*delay = no_os_field_get(ADP5055_DIS_DLY, *delay);

	return 0;
}

/**
 * @brief Sets the disable delay for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to set the disable delay. Valid channels are 1, 2, and 3.
 * @param delay - The disable delay setting to apply, as defined by the enum adp5055_disable_delays.
 *                This setting determines the delay before the device disables the specified channel.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or the delay setting is not supported.
 */
int adp5055_set_disable_delay(struct adp5055_desc *desc, uint8_t channel,
	enum adp5055_disable_delays delay)
{
	if (!desc)
		return -EINVAL;

	switch (channel) {
	case 1:
		return adp5055_update(desc, ADP5055_DLY1, ADP5055_DIS_DLY,
			no_os_field_prep(ADP5055_DIS_DLY, delay));
	case 2:
		return adp5055_update(desc, ADP5055_DLY2, ADP5055_DIS_DLY,
			no_os_field_prep(ADP5055_DIS_DLY, delay));
	case 3:
		return adp5055_update(desc, ADP5055_DLY3, ADP5055_DIS_DLY,
			no_os_field_prep(ADP5055_DIS_DLY, delay));
	default:
		return -EINVAL;
	}
}

/**
 * @brief Reads the enable delay setting for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to read the enable delay setting.
 *                  Valid channels are 1, 2, and 3.
 * @param delay - Pointer to an enum adp5055_enable_delays variable where the enable delay setting will be stored.
 *                The enable delay determines the delay before the device enables the specified channel.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL or the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_read_enable_delay(struct adp5055_desc *desc, uint8_t channel,
	enum adp5055_enable_delays *delay)
{
	int ret;

	if (!desc)
		return -EINVAL;

	switch (channel) {
	case 1:
		ret = adp5055_read(desc, ADP5055_DLY1, delay, 1);
		break;
	case 2:
		ret = adp5055_read(desc, ADP5055_DLY2, delay, 1);
		break;
	case 3:
		ret = adp5055_read(desc, ADP5055_DLY3, delay, 1);
		break;
	default:
		return -EINVAL;
	}
	if (ret)
		return ret;

	*delay = no_os_field_get(ADP5055_EN_DLY, *delay);

	return 0;
}

/**
 * @brief Sets the enable delay for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to set the enable delay. Valid channels are 1, 2, and 3.
 * @param delay - The enable delay setting to apply, as defined by the enum adp5055_enable_delays.
 *                This setting determines the delay before the device enables the specified channel.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or the delay setting is not supported.
 */
int adp5055_set_enable_delay(struct adp5055_desc *desc, uint8_t channel,
	enum adp5055_enable_delays delay)
{
	if (!desc)
		return -EINVAL;

	switch (channel) {
	case 1:
		return adp5055_update(desc, ADP5055_DLY1, ADP5055_EN_DLY,
			no_os_field_prep(ADP5055_EN_DLY, delay));
	case 2:
		return adp5055_update(desc, ADP5055_DLY2, ADP5055_EN_DLY,
			no_os_field_prep(ADP5055_EN_DLY, delay));
	case 3:
		return adp5055_update(desc, ADP5055_DLY3, ADP5055_EN_DLY,
			no_os_field_prep(ADP5055_EN_DLY, delay));
	default:
		return -EINVAL;
	}
}


/**
 * @brief Reads the raw VID setting for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to read the VID setting.
 *                  Valid channels are 1, 2, and 3.
 * @param vid - Pointer to a uint8_t where the VID setting will be stored.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL or the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_read_raw_vid(struct adp5055_desc *desc, uint8_t channel, uint8_t *vid)
{
	int ret;

	if (!desc)
		return -EINVAL;

	switch (channel) {
	case 1:
		ret = adp5055_read(desc, ADP5055_VID1, vid, 1);
		break;
	case 2:
		ret = adp5055_read(desc, ADP5055_VID2, vid, 1);
		break;
	case 3:
		ret = adp5055_read(desc, ADP5055_VID3, vid, 1);
		break;
	default:
		return -EINVAL;
	}
	if (ret)
		return ret;

	desc->vid[channel - 1] = ADP5055_VID_OFFSET + (*vid * ADP5055_VID_STEP);
	
	return 0;
}

/**
 * @brief Reads the converted VID setting for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to read the VID setting.
 *                  Valid channels are 1, 2, and 3.
 * @param vid - Pointer to a float where the converted VID setting will be stored.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL or the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_read_converted_vid(struct adp5055_desc *desc, uint8_t channel, float *vid)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	ret = adp5055_read_raw_vid(desc, channel, &data);
	if (ret)
		return ret;

	*vid = ((float) data * ADP5055_VID_STEP) + ADP5055_VID_OFFSET;

	return 0;
}

/**
 * @brief Sets the raw VID setting for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to set the VID setting.
 *                  Valid channels are 1, 2, and 3.
 * @param vid - The VID setting to apply.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL or the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_set_raw_vid(struct adp5055_desc *desc, uint8_t channel, uint8_t vid)
{
	int ret;

	if (!desc)
		return -EINVAL;

	/* TODO: Compare vid w/ VIDx_LOW and VIDx_HIGH */
	
	switch (channel) {
	case 1:
		ret = adp5055_write(desc, ADP5055_VID1, vid, 1);
		break;
	case 2:
		ret = adp5055_write(desc, ADP5055_VID2, vid, 1);
		break;
	case 3:
		ret = adp5055_write(desc, ADP5055_VID3, vid, 1);
		break;
	default:
		return -EINVAL;
	}
	if (ret)
		return ret;

	/* Change in VID results to change in VOUT as well */
	desc->vout[channel - 1] = ((ADP5055_VID_OFFSET + (vid * ADP5055_VID_STEP)) 
		/ desc->vid[channel - 1]) * desc->vout[channel - 1];

	return 0;
}

/**
 * @brief Sets the converted VID setting for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to set the VID setting.
 *                  Valid channels are 1, 2, and 3.
 * @param vid - The VID setting to apply.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL or the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_set_converted_vid(struct adp5055_desc *desc, uint8_t channel, float vid)
{
	uint8_t data;

	if (!desc)
		return -EINVAL;
	
	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	data = (uint8_t) ((vid - ADP5055_VID_OFFSET) / ADP5055_VID_STEP);

	return adp5055_set_raw_vid(desc, channel, data);
}


/**
 * @brief Reads the DVS (Dynamic Voltage Scaling) interval setting for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to read the DVS interval setting.
 *                  Valid channels are defined by ADP5055_MIN_CHANNELS and ADP5055_MAX_CHANNELS.
 * @param dvs_intval - Pointer to an enum adp5055_dvs_interval variable where the DVS interval setting will be stored.
 *                     The DVS interval determines the time interval for dynamic voltage scaling operations.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_read_dvs_intval(struct adp5055_desc *desc, uint8_t channel,
		enum adp5055_dvs_interval *dvs_intval)
{
	int ret;

	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_DVS_CFG, dvs_intval, 1);
	if (ret)
		return ret;

	*dvs_intval = no_os_field_get(ADP5055_DVS_INTVAL(channel), *dvs_intval);

	return 0;
}

/**
 * @brief Sets the DVS (Dynamic Voltage Scaling) interval for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to set the DVS interval.
 *                  Valid channels are defined by ADP5055_MIN_CHANNELS and ADP5055_MAX_CHANNELS.
 * @param dvs_intval - The DVS interval setting to apply, as defined by the enum adp5055_dvs_interval.
 *                     This setting determines the time interval for dynamic voltage scaling operations.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_set_dvs_intval(struct adp5055_desc *desc, uint8_t channel,
		enum adp5055_dvs_interval dvs_intval)
{
	int ret;

	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	return adp5055_update(desc, ADP5055_DVS_CFG, ADP5055_DVS_INTVAL(channel),
		no_os_field_prep(ADP5055_DVS_INTVAL(channel), dvs_intval));
}

/**
 * @brief Reads the VID high limit setting for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to read the VID high limit setting.
 *                  Valid channels are 1, 2, and 3.
 * @param lim - Pointer to a uint8_t where the VID high limit setting will be stored.
 *              This setting determines the maximum VID limit for the specified channel.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_read_vid_high_lim(struct adp5055_desc *desc, uint8_t channel, uint8_t *lim)
{
	int ret;

	if (!desc)
		return -EINVAL;

	switch (channel) {
	case 1:
		ret = adp5055_read(desc, ADP5055_DVS_LIM1, lim, 1);
		break;
	case 2:
		ret = adp5055_read(desc, ADP5055_DVS_LIM2, lim, 1);
		break;
	case 3:
		ret = adp5055_read(desc, ADP5055_DVS_LIM3, lim, 1);
		break;
	default:
		return -EINVAL;
	}
	if (ret)
		return ret;

	*lim = no_os_field_get(ADP5055_VID_HIGH, *lim);
	
	return 0;
}

/**
 * @brief Reads the converted VID high limit setting for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to read the VID high limit setting.
 *                  Valid channels are 1, 2, and 3.
 * @param lim - Pointer to a float where the converted VID high limit setting will be stored.
 *              This setting determines the maximum VID limit for the specified channel.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_read_converted_vid_high_lim(struct adp5055_desc *desc, uint8_t channel, float *lim)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	ret = adp5055_read_vid_high_lim(desc, channel, &data);
	if (ret)
		return ret;

	/* TODO: Add Vref trim */
	*lim = ADP5055_VID_HIGH_LIM_OFFSET - (float) data * ADP5055_VID_LIM_STEP;

	return 0;
}

/**
 * @brief Sets the VID high limit for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to set the VID high limit.
 *                  Valid channels are 1, 2, and 3.
 * @param lim - The VID high limit setting to apply.
 *              This setting determines the maximum VID limit for the specified channel.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_set_vid_high_lim(struct adp5055_desc *desc, uint8_t channel, uint8_t lim)
{
	if (!desc)
		return -EINVAL;

	switch (channel) {
	case 1:
		return adp5055_update(desc, ADP5055_DVS_LIM1, ADP5055_VID_HIGH,
			no_os_field_prep(ADP5055_VID_HIGH, lim));
	case 2:
		return adp5055_update(desc, ADP5055_DVS_LIM2, ADP5055_VID_HIGH,
			no_os_field_prep(ADP5055_VID_HIGH, lim));
	case 3:
		return adp5055_update(desc, ADP5055_DVS_LIM3, ADP5055_VID_HIGH,
			no_os_field_prep(ADP5055_VID_HIGH, lim));
	default:
		return -EINVAL;
	}
}

/**
 * @brief Sets the converted VID high limit for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to set the VID high limit.
 *                  Valid channels are 1, 2, and 3.
 * @param lim - The VID high limit setting to apply.
 *              This setting determines the maximum VID limit for the specified channel.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_set_converted_vid_high_lim(struct adp5055_desc *desc, uint8_t channel, float lim)
{
	uint8_t data;

	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	/* TODO: subtract Vref trim */
	data = (uint8_t) ((lim - ADP5055_VID_HIGH_LIM_OFFSET) / -ADP5055_VID_LIM_STEP);

	return adp5055_set_vid_high_lim(desc, channel, data);
}

/**
 * @brief Reads the VID low limit setting for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to read the VID low limit setting.
 *                  Valid channels are 1, 2, and 3.
 * @param lim - Pointer to a uint8_t where the VID low limit setting will be stored.
 *              This setting determines the minimum VID limit for the specified channel.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_read_vid_low_lim(struct adp5055_desc *desc, uint8_t channel, uint8_t *lim)
{
	int ret;

	if (!desc)
		return -EINVAL;

	switch (channel) {
	case 1:
		ret = adp5055_read(desc, ADP5055_DVS_LIM1, lim, 1);
		break;
	case 2:
		ret = adp5055_read(desc, ADP5055_DVS_LIM2, lim, 1);
		break;
	case 3:
		ret = adp5055_read(desc, ADP5055_DVS_LIM3, lim, 1);
		break;
	default:
		return -EINVAL;
	}
	if (ret)
		return ret;

	*lim = no_os_field_get(ADP5055_VID_LOW, *lim);
	
	return 0;
}

/**
 * @brief Reads the converted VID low limit setting for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to read the VID low limit setting.
 *                  Valid channels are 1, 2, and 3.
 * @param lim - Pointer to a float where the converted VID low limit setting will be stored.
 *              This setting determines the minimum VID limit for the specified channel.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_read_converted_vid_low_lim(struct adp5055_desc *desc, uint8_t channel, float *lim)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	ret = adp5055_read_vid_low_lim(desc, channel, &data);
	if (ret)
		return ret;

	/* TODO: Add Vref trim */
	*lim = ADP5055_VID_HIGH_LIM_OFFSET - (float) data * ADP5055_VID_LIM_STEP;

	return 0;
}

/**
 * @brief Sets the VID high limit for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to set the VID low limit.
 *                  Valid channels are 1, 2, and 3.
 * @param lim - The VID high limit setting to apply.
 *              This setting determines the minimum VID limit for the specified channel.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_set_vid_low_lim(struct adp5055_desc *desc, uint8_t channel, uint8_t lim)
{
	if (!desc)
		return -EINVAL;

	switch (channel) {
	case 1:
		return adp5055_update(desc, ADP5055_DVS_LIM1, ADP5055_VID_HIGH,
			no_os_field_prep(ADP5055_VID_HIGH, lim));
	case 2:
		return adp5055_update(desc, ADP5055_DVS_LIM2, ADP5055_VID_HIGH,
			no_os_field_prep(ADP5055_VID_HIGH, lim));
	case 3:
		return adp5055_update(desc, ADP5055_DVS_LIM3, ADP5055_VID_HIGH,
			no_os_field_prep(ADP5055_VID_HIGH, lim));
	default:
		return -EINVAL;
	}
}

/**
 * @brief Sets the converted VID low limit for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to set the VID low limit.
 *                  Valid channels are 1, 2, and 3.
 * @param lim - The VID low limit setting to apply.
 *              This setting determines the minimum VID limit for the specified channel.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_set_converted_vid_low_lim(struct adp5055_desc *desc, uint8_t channel, float lim)
{
	uint8_t data;

	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	/* TODO: subtract Vref trim */
	data = (uint8_t) ((lim - ADP5055_VID_HIGH_LIM_OFFSET) / -ADP5055_VID_LIM_STEP);

	return adp5055_set_vid_low_lim(desc, channel, data);
}

/**
 * @brief Reads the fast transient settings for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to read the fast transient settings.
 *                  Valid channels are defined by ADP5055_MIN_CHANNELS and ADP5055_MAX_CHANNELS.
 * @param ft_cfg - Pointer to an enum adp5055_fast_transient_sensitivity variable where the fast transient sensitivity setting will be stored.
 *                 This setting determines the sensitivity to fast transients for the specified channel.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_read_fast_transient_settings(struct adp5055_desc *desc, uint8_t channel,
	enum adp5055_fast_transient_sensitivity *ft_cfg)
{
	int ret;

	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_FT_CFG, ft_cfg, 1);
	if (ret)
		return ret;
	
	*ft_cfg = no_os_field_get(ADP5055_FT_CFG_TH(channel), *ft_cfg);

	return 0;
}

/**
 * @brief Sets the fast transient settings for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to set the fast transient settings.
 *                  Valid channels are defined by ADP5055_MIN_CHANNELS and ADP5055_MAX_CHANNELS.
 * @param ft_cfg - The fast transient sensitivity setting to apply, as defined by the enum adp5055_fast_transient_sensitivity.
 *                 This setting determines the sensitivity to fast transients for the specified channel.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_set_fast_transient_settings(struct adp5055_desc *desc, uint8_t channel,
	enum adp5055_fast_transient_sensitivity ft_cfg)
{
	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	return adp5055_update(desc, ADP5055_FT_CFG, ADP5055_FT_CFG_TH(channel),
		no_os_field_prep(ADP5055_FT_CFG_TH(channel), ft_cfg));
}

/**
 * @brief Reads the power-good delay setting from the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param pwrgd_delay_enabled - Pointer to a boolean where the power-good delay enabled status will be stored.
 *                              True if the power-good delay is enabled, false otherwise.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, or other negative error code for communication errors.
 */
int adp5055_read_pwrgd_delay(struct adp5055_desc *desc, bool *pwrgd_delay_enabled)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_PG_CFG, &data, 1);
	if (ret)
		return ret;

	*pwrgd_delay_enabled = no_os_field_get(ADP5055_PG_CFG_PWRGD_DLY, data);

	return 0;
}

/**
 * @brief Sets the power-good delay setting on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param pwrgd_delay_enabled - Boolean indicating the desired power-good delay setting.
 *                              True to enable the power-good delay, false to disable it.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, or other negative error code for communication errors.
 */
int adp5055_set_pwrgd_delay(struct adp5055_desc *desc, bool pwrgd_delay_enabled)
{
	if (!desc)
		return -EINVAL;

	return adp5055_update(desc, ADP5055_PG_CFG, ADP5055_PG_CFG_PWRGD_DLY,
		no_os_field_prep(ADP5055_PG_CFG_PWRGD_DLY, pwrgd_delay_enabled));
}

/**
 * @brief Reads the power-good (PG) mask setting for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to read the PG mask setting.
 *                  Valid channels are defined by ADP5055_MIN_CHANNELS and ADP5055_MAX_CHANNELS.
 * @param is_masked - Pointer to a boolean where the PG mask status will be stored.
 *                    True if the power-good signal is masked to external PWRGD pin.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is out of range, or other negative error code for communication errors.
 */
int adp5055_read_pg_mask(struct adp5055_desc *desc, uint8_t channel, bool *is_masked)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_PG_CFG, &data, 1);
	if (ret)
		return ret;

	*is_masked = (bool) no_os_field_get(ADP5055_PG_CFG_PG_ON(channel), data);

	return 0;
}

/**
 * @brief Sets the power-good (PG) mask setting for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to set the PG mask setting.
 *                  Valid channels are defined by ADP5055_MIN_CHANNELS and ADP5055_MAX_CHANNELS.
 * @param is_masked - Boolean indicating the desired PG mask setting.
 *                    True to mask the power-good signal to the external PWRGD pin, false otherwise.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is out of range, or other negative error code for communication errors.
 */
int adp5055_set_pg_mask(struct adp5055_desc *desc, uint8_t channel, bool is_masked)
{
	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	return adp5055_update(desc, ADP5055_PG_CFG, ADP5055_PG_CFG_PG_ON(channel),
		no_os_field_prep(ADP5055_PG_CFG_PG_ON(channel), is_masked));
}

/**
 * @brief Reads the power-good (PWGD) status for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to read the PWGD status.
 *                  Valid channels are defined by ADP5055_MIN_CHANNELS and ADP5055_MAX_CHANNELS.
 * @param is_out_nominal - Pointer to a boolean where the PWGD status will be stored.
 *                         True if the output is in a nominal condition, false otherwise.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, or other negative error code for communication errors.
 */
int adp5055_read_pwrgd_status(struct adp5055_desc *desc, uint8_t channel, 
	bool *is_out_nominal)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_PG_READ, &data, 1);
	if (ret)
		return ret;

	*is_out_nominal = (bool) no_os_field_get(ADP5055_PG_READ_PWRGD(channel), data);

	return 0;
}

/**
 * @brief Reads the INT_LCH value from the STATUS_LCH register on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param int_lch - Pointer to a variable where the INT_LCH value will be stored.
 * 			True if chip init failure detected.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is out of range, or other negative error code for communication errors.
 */
int adp5055_read_init_proc_latch_status(struct adp5055_desc *desc, bool *int_lch)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_STATUS_LCH, &data, 1);
	if (ret)
		return ret;

	*int_lch = (bool) no_os_field_get(ADP5055_STATUS_INT_LCH, data);

	return 0;
}

/**
 * @brief Clears the INT_LCH status on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, or other negative error code for communication errors.
 */
int adp5055_clear_init_proc_latch_status(struct adp5055_desc *desc)
{
	if (!desc)
		return -EINVAL;

	return adp5055_update(desc, ADP5055_STATUS_LCH, ADP5055_STATUS_INT_LCH,
		 no_os_field_prep(ADP5055_STATUS_INT_LCH, 1));
}

/**
 * @brief Reads the over-current hiccup status for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to read the over-current hiccup status.
 *                  Valid channels are defined by ADP5055_MIN_CHANNELS and ADP5055_MAX_CHANNELS.
 * @param hiccup - Pointer to a boolean where the over-current hiccup status will be stored.
 *                 True if an over-current hiccup failure is detected, false otherwise.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, or other negative error code for communication errors.
 */
int adp5055_read_over_current_hiccup_latch_status(struct adp5055_desc *desc, uint8_t channel, 
		bool *hiccup)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_STATUS_LCH, &data, 1);
	if (ret)
		return ret;

	*hiccup = (bool) no_os_field_get(ADP5055_STATUS_OCP_LCH(channel), data);

	return 0;
}

/**
 * @brief Clears the over-current hiccup status for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to clear the over-current hiccup status.
 *                  Valid channels are defined by ADP5055_MIN_CHANNELS and ADP5055_MAX_CHANNELS.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, or other negative error code for communication errors.
 */
int adp5055_clear_over_current_hiccup(struct adp5055_desc *desc, uint8_t channel)
{
	if (!desc)
		return -EINVAL;

	return adp5055_update(desc, ADP5055_STATUS_LCH, ADP5055_STATUS_OCP_LCH(channel),
		 no_os_field_prep(ADP5055_STATUS_OCP_LCH(channel), 1));
}


/**
 * @brief Reads the thermal shutdown latch status from the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param tsd_lch - Pointer to a boolean where the thermal shutdown latch status will be stored.
 *                     True if a thermal shutdown has occurred and the status is latched, false otherwise.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, or other negative error code for communication errors.
 */
int adp5055_read_thermal_shutdown_latch_status(struct adp5055_desc *desc, bool *tsd_lch)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_STATUS_LCH, &data, 1);
	if (ret)
		return ret;

	*tsd_lch = (bool) no_os_field_get(ADP5055_STATUS_TSD_LCH, data);

	return 0;
}

/**
 * @brief Clears the thermal shutdown latch status on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, or other negative error code for communication errors.
 */
int adp5055_clear_thermal_shutdown_latch_status(struct adp5055_desc *desc)
{
	if (!desc)
		return -EINVAL;

	return adp5055_update(desc, ADP5055_STATUS_LCH, ADP5055_STATUS_TSD_LCH,
		 no_os_field_prep(ADP5055_STATUS_TSD_LCH, 1));
}

/**
 * @brief Reads the power-good latch status for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to read the power-good latch status.
 *                  Valid channels are defined by the device specification.
 * @param pg_lch - Pointer to a boolean where the power-good latch status will be stored.
 *                 True if a power-good failure has occurred and the status is latched, false otherwise.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_read_power_good_latch_status(struct adp5055_desc *desc, uint8_t channel,
		 bool *pg_lch)
{
	int ret;
	uint8_t data;

	if (!desc)
		return -EINVAL;
	
	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	ret = adp5055_read(desc, ADP5055_STATUS_LCH, &data, 1);
	if (ret)
		return ret;

	*pg_lch = (bool) no_os_field_get(ADP5055_STATUS_PG_LCH(channel), data);

	return 0;
}

/**
 * @brief Clears the power-good latch status for a specific channel on the ADP5055 device.
 * @param desc - Pointer to the ADP5055 device descriptor.
 * @param channel - The channel number for which to clear the power-good latch status.
 *                  Valid channels are defined by the device specification.
 * @return 0 in case of success, -EINVAL if the descriptor is NULL, the channel number is invalid, or other negative error code for communication errors.
 */
int adp5055_clear_power_good_latch_status(struct adp5055_desc *desc, uint8_t channel)
{
	if (!desc)
		return -EINVAL;

	if (channel < ADP5055_MIN_CHANNELS || channel > ADP5055_MAX_CHANNELS)
		return -EINVAL;

	return adp5055_update(desc, ADP5055_STATUS_LCH, ADP5055_STATUS_PG_LCH(channel),
		 no_os_field_prep(ADP5055_STATUS_PG_LCH(channel), 1));
}
