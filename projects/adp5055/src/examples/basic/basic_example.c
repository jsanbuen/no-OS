/***************************************************************************//**
 *   @file   basic_example.c
 *   @brief  Basic example source file for adp5055 project.
 *   @author Jose Ramon San Buenaventura (jose.sanbuenaventura@analog.com
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
#include "common_data.h"
#include "basic_example.h"
#include "no_os_delay.h"
#include "no_os_i2c.h"
#include "no_os_print_log.h"
#include "no_os_util.h"
#include "no_os_pwm.h"
#include "adp5055.h"

int basic_example_main()
{
	float f_dat;
	int i, ret;
	struct adp5055_desc *adp5055_desc;
	uint8_t data;

	ret = adp5055_init(&adp5055_desc, &adp5055_ip);
	if (ret)
		goto exit;

	for (i = 0; i < ADP5055_MAX_CHANNELS; i++) {
		ret = adp5055_read_converted_vid(adp5055_desc, i + 1, &f_dat);
		if (ret)
			goto exit;
		
		pr_info("Read VID%d = %0.4f\n", i + 1, f_dat);
	}

	pr_info("\n\n");

	for (i = 0; i < ADP5055_MAX_CHANNELS; i++)
		pr_info("Channel %d VOUT: %0.4f\n", i + 1, adp5055_desc->vout[i]);
	
	pr_info("\n\n");		

	for (i = 0; i < ADP5055_MAX_CHANNELS; i++) {
		ret = adp5055_set_converted_vid(adp5055_desc, i + 1, new_vid[i]);
		if (ret)
			goto exit;

		pr_info("Configured VID%d to %0.4f\n", i + 1, new_vid[i]);
	}

	no_os_mdelay(5000);
	
	pr_info("\n\n");
	
	for (i = 0; i < ADP5055_MAX_CHANNELS; i++)
		pr_info("New Channel %d VOUT: %0.4f\n", i + 1, adp5055_desc->vout[i]);

	pr_info("\n\n");

	ret = adp5055_read(adp5055_desc, ADP5055_STATUS_CML, &data, 1);
	if (ret)
		goto exit;

	pr_info("STATUS_CML: 0x%x\n", data);

	ret = adp5055_read(adp5055_desc, ADP5055_STATUS_LCH, &data, 1);
	if (ret)
		goto exit;

	pr_info("STATUS_LCH: 0x%x\n", data);
	
exit:
	if (ret)
		pr_info("Error\n");

	adp5055_remove(adp5055_desc);

	return ret;
}
