/***************************************************************************//**
 *   @file   basic_example.c
 *   @brief  Implementation of IIO example for ad463x_fmcz project.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
 *   @author Axel Haslam (ahaslam@baylibre.com)
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "basic_example.h"
#include "common_data.h"
#include "no_os_print_log.h"

#define BITS_PER_SAMPLE 32
#define REAL_BITS 24
/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief basic example main execution.
 *
 * @return ret - Result of the example execution. If working correctly, will
 *               execute print the sample data.
*******************************************************************************/
int basic_example_main()
{
	struct ad463x_dev *dev;
	uint32_t *buf = ADC_DDR_BASEADDR;
	int32_t ret, i;
	int32_t tmp;

	ret = ad463x_init(&dev, &ad463x_init_param);
	if (ret)
		return ret;

#if ADAQ4224_DEV
	/* Apply a gain of 1/3 */
#define AD4224_INITIAL_GAIN_IDX 0

	ret = ad463x_fill_scale_tbl(dev);
	if (ret)
		goto out;

	ret = ad463x_set_pgia_gain(dev, AD4224_INITIAL_GAIN_IDX);
	if (ret)
		goto out;

#endif
	ret = ad463x_exit_reg_cfg_mode(dev);
	if (ret)
		goto out;

	ret = ad463x_read_data(dev, buf, SAMPLES_PER_CHANNEL);
	if (ret)
		goto out;

	for (i = 0; i < SAMPLES_PER_CHANNEL; i+=2) {
		tmp = no_os_sign_extend32(buf[i], REAL_BITS - 1);
		pr_info("ADC sample ch1: %lu : %ld \r\n", i, tmp);
	}

	for (i = 1; i < SAMPLES_PER_CHANNEL; i+=2) {
		tmp = no_os_sign_extend32(buf[i], REAL_BITS - 1);
		pr_info("ADC sample ch2: %lu : %ld \r\n", i, tmp);
	}

out:
	return ad463x_remove(dev);
}
