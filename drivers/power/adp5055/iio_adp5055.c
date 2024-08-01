/***************************************************************************//**
 *   @file   iio_adp5055.c
 *   @brief  Source file for the ADP5055 IIO Driver
 *   @author Jose San Buenaventura (jose.sanbuenaventura@analog.com)
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
#include <stdio.h>
#include <string.h>
#include "no_os_alloc.h"
#include "no_os_error.h"
#include "no_os_units.h"
#include "no_os_util.h"

#include "adp5055.h"
#include "iio_adp5055.h"

static int adp5055_iio_reg_read(struct adp5055_iio_desc *dev, uint32_t reg,
				 uint32_t *readval);
				 
static int adp5055_iio_reg_write(struct adp5055_iio_desc *dev, uint32_t reg,
				 uint32_t *writeval);

static int adp5055_iio_read_status(void *dev, char *buf, uint32_t len,
				   const struct iio_ch_info *channel,
				   intptr_t priv);

static struct iio_attribute adp5055_debug_attrs[] = {
	{
		.name = "status_cml",
		.show = adp5055_iio_read_status,
		.priv = ADP5055_STATUS_CML
	},
	{
		.name = "status_lch",
		.show = adp5055_iio_read_status,
		.priv = ADP5055_STATUS_LCH
	},
	END_ATTRIBUTES_ARRAY
};

static struct iio_device adp5055_iio_dev = {
	.debug_reg_read = (int32_t (*)())adp5055_iio_reg_read,
        .debug_reg_write = (int32_t (*)())adp5055_iio_reg_write,
	.debug_attributes = adp5055_debug_attrs,
};

/**
 * @brief adp5055 reg read wrapper
 * @param dev - The iio device structure.
 * @param reg - Register address
 * @param readval - Register value
 * 
 * @return 0 in case of success, errno errors otherwise
 */
static int adp5055_iio_reg_read(struct adp5055_iio_desc *dev, uint32_t reg,
				 uint32_t *readval)
{
        if (!dev)
		return -ENODEV;

	return adp5055_read(dev->adp5055_desc, (uint16_t)reg , 
                (uint8_t *) readval, 1);
}

/**
 * @brief adp5055 reg write wrapper
 * @param dev - The iio device structure.
 * @param reg - Register address
 * @param readval - Register value
 * 
 * @return 0 in case of success, errno errors otherwise
 */
static int adp5055_iio_reg_write(struct adp5055_iio_desc *dev, uint32_t reg,
				 uint32_t *writeval)
{
        if (!dev)
		return -ENODEV;

	return adp5055_write(dev->adp5055_desc, (uint16_t)reg, 
                (uint16_t ) *writeval, 1);
}

/**
 * @brief Initializes the ADP5055 IIO descriptor.
 * @param iio_desc - The iio device descriptor.
 * @param init_param - The structure that contains the device initial parameters.
 * @return 0 in case of success, an error code otherwise.
 */
int adp5055_iio_init(struct adp5055_iio_desc **iio_desc,
		     struct adp5055_iio_desc_init_param *init_param)
{
	struct adp5055_iio_desc *descriptor;
	int ret;

	if (!init_param || !init_param->adp5055_init_param)
		return -EINVAL;

	descriptor = no_os_calloc(1, sizeof(*descriptor));
	if (!descriptor)
		return -ENOMEM;

	ret = adp5055_init(&descriptor->adp5055_desc,
			   init_param->adp5055_init_param);
	if (ret)
		goto free_desc;

	descriptor->iio_dev = &adp5055_iio_dev;

	*iio_desc = descriptor;

	return 0;

free_desc:
	adp5055_iio_remove(descriptor);

	return ret;
}

/**
 * @brief Free resources allocated by the init function.
 * @param iio_desc - The iio device descriptor.
 * @return 0 in case of success, an error code otherwise.
 */
int adp5055_iio_remove(struct adp5055_iio_desc *iio_desc)
{
	if (!iio_desc)
		return -ENODEV;

	// no_os_free(iio_desc->iio_dev->channels);
	adp5055_remove(iio_desc->adp5055_desc);
	no_os_free(iio_desc);

	return 0;
}

/**
 * @brief Handles the read request for status debug attribute.
 * @param dev     - The iio device structure.
 * @param buf	  - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return ret    - Result of the reading procedure.
 * 		    In case of success, the size of the read data is returned.
*/
static int adp5055_iio_read_status(void *dev, char *buf, uint32_t len,
				   const struct iio_ch_info *channel,
				   intptr_t priv)
{
	struct adp5055_iio_desc *iio_adp5055 = dev;
	struct adp5055_desc *adp5055 = iio_adp5055->adp5055_desc;
	int ret;
	int32_t status;

	ret = adp5055_read(adp5055, priv, (uint8_t *) &status, 1);
	if (ret)
		return ret;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &status);
}
