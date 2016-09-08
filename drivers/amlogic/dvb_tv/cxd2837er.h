/*
 * cxd2837er.h
 *
 * Sony CXD2837ER digital demodulator driver
 *
 * Copyright (C) 2015 Sasa Savic <sasa.savic.sr@gmail.com>
 *
 * Based on CXD2841ER driver
 *
 * Copyright 2012 Sony Corporation
 * Copyright (C) 2014 NetUP Inc.
 * Copyright (C) 2014 Sergey Kozlov <serjk@netup.ru>
 * Copyright (C) 2014 Abylay Ospan <aospan@netup.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
  */

#ifndef CXD2837ER_H
#define CXD2837ER_H

#include <linux/kconfig.h>
#include <linux/dvb/frontend.h>

struct cxd2837er_config {
	u8	i2c_addr;
	u8 	if_agc;
	u8	ifagc_adc_range;
	u8	ts_error_polarity;
	u8	clock_polarity;
	u8	mxl603;	
};

#define I2C_SLVX			0
#define I2C_SLVT			1

#define CXD2837ER_CHIP_ID		0xb1

enum cxd2837er_dvbt2_profile_t {
	DVBT2_PROFILE_ANY = 0,
	DVBT2_PROFILE_BASE = 1,
	DVBT2_PROFILE_LITE = 2
};


extern struct dvb_frontend *cxd2837er_attach(struct cxd2837er_config *cfg,
					       struct i2c_adapter *i2c);

#endif
