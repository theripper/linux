/*
 * cxd2837er.c
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/log2.h>
#include <linux/dynamic_debug.h>

#include "dvb_math.h"
#include "dvb_frontend.h"
#include "cxd2837er.h"

#define MAX_WRITE_REGSIZE	16

enum cxd2837er_state {
	STATE_SHUTDOWN = 0,
	STATE_SLEEP_TC,
	STATE_ACTIVE_TC
};

struct cxd2837er_priv {
	struct dvb_frontend		frontend;
	struct i2c_adapter		*i2c;
	u8				i2c_addr_slvx;
	u8				i2c_addr_slvt;
	const struct cxd2837er_config	*config;
	enum cxd2837er_state		state;
	u8				system;
};


#define MAKE_IFFREQ_CONFIG(iffreq) ((u32)(((iffreq)/41.0)*16777216.0 + 0.5))

static int cxd2837er_write_regs(struct cxd2837er_priv *priv,
				u8 addr, u8 reg, const u8 *data, u32 len)
{
	int ret;
	u8 buf[MAX_WRITE_REGSIZE + 1];
	u8 i2c_addr = (addr == I2C_SLVX ?
		priv->i2c_addr_slvx : priv->i2c_addr_slvt);
	struct i2c_msg msg[1] = {
		{
			.addr = i2c_addr,
			.flags = 0,
			.len = len + 1,
			.buf = buf,
		}
	};

	if (len + 1 >= sizeof(buf)) {
		dev_warn(&priv->i2c->dev,"wr reg=%04x: len=%d is too big!\n",
			 reg, len + 1);
		return -E2BIG;
	}


	buf[0] = reg;
	memcpy(&buf[1], data, len);

	ret = i2c_transfer(priv->i2c, msg, 1);
	if (ret >= 0 && ret != 1)
		ret = -EIO;
	if (ret < 0) {
		dev_warn(&priv->i2c->dev,
			"i2c wr failed=%d addr=%02x reg=%02x len=%d\n",
			ret, i2c_addr, reg, len);
		return ret;
	}
	return 0;
}

static int cxd2837er_write_reg(struct cxd2837er_priv *priv,
			       u8 addr, u8 reg, u8 val)
{
	return cxd2837er_write_regs(priv, addr, reg, &val, 1);
}

static int cxd2837er_read_regs(struct cxd2837er_priv *priv,
			       u8 addr, u8 reg, u8 *val, u32 len)
{
	int ret;
	u8 i2c_addr = (addr == I2C_SLVX ?
		priv->i2c_addr_slvx : priv->i2c_addr_slvt);
	struct i2c_msg msg[2] = {
		{
			.addr = i2c_addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		}, {
			.addr = i2c_addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = val,
		}
	};

	ret = i2c_transfer(priv->i2c, &msg[0], 1);
	if (ret >= 0 && ret != 1)
		ret = -EIO;
	if (ret < 0) {
		dev_warn(&priv->i2c->dev,
			"i2c rw failed=%d addr=%02x reg=%02x\n",
			ret, i2c_addr, reg);
		return ret;
	}
	ret = i2c_transfer(priv->i2c, &msg[1], 1);
	if (ret >= 0 && ret != 1)
		ret = -EIO;
	if (ret < 0) {
		dev_warn(&priv->i2c->dev,
			"i2c rd failed=%d addr=%02x reg=%02x\n",
			ret, i2c_addr, reg);
		return ret;
	}
	return 0;
}

static int cxd2837er_read_reg(struct cxd2837er_priv *priv,
			      u8 addr, u8 reg, u8 *val)
{
	return cxd2837er_read_regs(priv, addr, reg, val, 1);
}

static int cxd2837er_set_reg_bits(struct cxd2837er_priv *priv,
				  u8 addr, u8 reg, u8 data, u8 mask)
{
	int res;
	u8 rdata;

	if (mask != 0xff) {
		res = cxd2837er_read_reg(priv, addr, reg, &rdata);
		if (res)
			return res;
		data = ((data & mask) | (rdata & (mask ^ 0xFF)));
	}
	return cxd2837er_write_reg(priv, addr, reg, data);
}

static void cxd2837er_set_ts_clock_mode(struct cxd2837er_priv *priv,
					u8 system);

static int cxd2837er_sleep_tc_to_active_t_band(struct cxd2837er_priv *priv,
					       u32 bandwidth);

static int cxd2837er_sleep_tc_to_active_t2_band(struct cxd2837er_priv *priv,
						u32 bandwidth);

static int cxd2837er_sleep_tc_to_active_c_band(struct cxd2837er_priv *priv,
					       u32 bandwidth);

static int cxd2837er_retune_active(struct cxd2837er_priv *priv,
				   struct dtv_frontend_properties *p)
{
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* disable TS output */
	cxd2837er_write_reg(priv, I2C_SLVT, 0xc3, 0x01);
	if (priv->state == STATE_ACTIVE_TC) {
		switch (priv->system) {
		case SYS_DVBT:
			return cxd2837er_sleep_tc_to_active_t_band(
					priv, p->bandwidth_hz);
		case SYS_DVBT2:
			return cxd2837er_sleep_tc_to_active_t2_band(
					priv, p->bandwidth_hz);
		case SYS_DVBC_ANNEX_A:
			return cxd2837er_sleep_tc_to_active_c_band(
					priv, 8000000);
		}
	}
	dev_dbg(&priv->i2c->dev, "%s(): invalid delivery system %d\n",
		__func__, priv->system);
	return -EINVAL;
}

static int cxd2837er_sleep_tc_to_shutdown(struct cxd2837er_priv *priv)
{
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_SLEEP_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid demod state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-X Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* Disable oscillator */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x15, 0x01);
	/* Set demod mode */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x17, 0x01);
	priv->state = STATE_SHUTDOWN;
	return 0;
}

static int cxd2837er_active_t_to_sleep_tc(struct cxd2837er_priv *priv)
{
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_err(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* disable TS output */
	cxd2837er_write_reg(priv, I2C_SLVT, 0xc3, 0x01);
	/* enable Hi-Z setting 1 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x80, 0x3f);
	/* enable Hi-Z setting 2 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x81, 0xff);
	/* Set SLV-X Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* disable ADC 1 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x18, 0x01);
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* Disable ADC 2 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x43, 0x0a);
	/* Disable ADC 3 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x41, 0x0a);
	/* Disable ADC clock */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x30, 0x00);
	/* Disable RF level monitor */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x2f, 0x00);
	/* Disable demod clock */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x2c, 0x00);
	priv->state = STATE_SLEEP_TC;
	return 0;
}

static int cxd2837er_active_t2_to_sleep_tc(struct cxd2837er_priv *priv)
{
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_err(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* disable TS output */
	cxd2837er_write_reg(priv, I2C_SLVT, 0xc3, 0x01);
	/* enable Hi-Z setting 1 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x80, 0x3f);
	/* enable Hi-Z setting 2 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x81, 0xff);
	/* Cancel DVB-T2 setting */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x13);
	cxd2837er_write_reg(priv, I2C_SLVT, 0x83, 0x40);
	cxd2837er_write_reg(priv, I2C_SLVT, 0x86, 0x21);
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x9e, 0x09, 0x0f);
	cxd2837er_write_reg(priv, I2C_SLVT, 0x9f, 0xfb);
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x2a);
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x38, 0x00, 0x0f);
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x2b);
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x11, 0x00, 0x3f);
	/* Set SLV-X Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* disable ADC 1 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x18, 0x01);
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* Disable ADC 2 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x43, 0x0a);
	/* Disable ADC 3 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x41, 0x0a);
	/* Disable ADC clock */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x30, 0x00);
	/* Disable RF level monitor */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x2f, 0x00);
	/* Disable demod clock */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x2c, 0x00);
	priv->state = STATE_SLEEP_TC;
	return 0;
}

static int cxd2837er_active_c_to_sleep_tc(struct cxd2837er_priv *priv)
{
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_err(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* disable TS output */
	cxd2837er_write_reg(priv, I2C_SLVT, 0xc3, 0x01);
	/* enable Hi-Z setting 1 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x80, 0x3f);
	/* enable Hi-Z setting 2 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x81, 0xff);
	/* Cancel DVB-C setting */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x11);
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xa3, 0x00, 0x1f);
	/* Set SLV-X Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* disable ADC 1 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x18, 0x01);
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* Disable ADC 2 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x43, 0x0a);
	/* Disable ADC 3 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x41, 0x0a);
	/* Disable ADC clock */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x30, 0x00);
	/* Disable RF level monitor */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x2f, 0x00);
	/* Disable demod clock */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x2c, 0x00);
	priv->state = STATE_SLEEP_TC;
	return 0;
}

static int cxd2837er_shutdown_to_sleep_tc(struct cxd2837er_priv *priv)
{
	u8 data[2] = { 0x00, 0x00 };
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_SHUTDOWN) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid demod state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-X Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* Clear all demodulator registers */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x02, 0x00);
	usleep_range(3000, 5000);
	/* Set SLV-X Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* Set demod SW reset */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x10, 0x01);
	/* Set X'tal clock to 20.5Mhz */
	cxd2837er_write_regs(priv, I2C_SLVX, 0x13, data, 2);	
	/* Clear demod SW reset */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x10, 0x00);
	usleep_range(1000, 2000);
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* TADC Bias On */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x43, 0x0a);
	cxd2837er_write_reg(priv, I2C_SLVT, 0x41, 0x0a);

	priv->state = STATE_SLEEP_TC;
	return 0;
}

static int cxd2837er_tune_done(struct cxd2837er_priv *priv)
{
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0, 0);
	/* SW Reset */
	cxd2837er_write_reg(priv, I2C_SLVT, 0xfe, 0x01);
	/* Enable TS output */
	cxd2837er_write_reg(priv, I2C_SLVT, 0xc3, 0x00);
	return 0;
}

/* Set TS parallel mode */
static void cxd2837er_set_ts_clock_mode(struct cxd2837er_priv *priv,
					u8 system)
{
	u8 serial_ts, ts_rate_ctrl_off, ts_in_off;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	cxd2837er_read_reg(priv, I2C_SLVT, 0xc4, &serial_ts);
	cxd2837er_read_reg(priv, I2C_SLVT, 0xd3, &ts_rate_ctrl_off);
	cxd2837er_read_reg(priv, I2C_SLVT, 0xde, &ts_in_off);
	dev_dbg(&priv->i2c->dev, "%s(): ser_ts=0x%02x rate_ctrl_off=0x%02x in_off=0x%02x\n",
		__func__, serial_ts, ts_rate_ctrl_off, ts_in_off);

	/*
	 * slave    Bank    Addr    Bit    default    Name
	 * <SLV-T>  00h     D9h     [7:0]  8'h08      OTSCKPERIOD
	 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0xd9, 0x08);
	/*
	 * Disable TS IF Clock
	 * slave    Bank    Addr    Bit    default    Name
	 * <SLV-T>  00h     32h     [0]    1'b1       OREG_CK_TSIF_EN
	 */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x32, 0x00, 0x01);
	/*
	 * slave    Bank    Addr    Bit    default    Name
	 * <SLV-T>  00h     33h     [1:0]  2'b01      OREG_CKSEL_TSIF
	 */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x33, 0x00, 0x03);
	/*
	 * Enable TS IF Clock
	 * slave    Bank    Addr    Bit    default    Name
	 * <SLV-T>  00h     32h     [0]    1'b1       OREG_CK_TSIF_EN
	 */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x32, 0x01, 0x01);

	if (system == SYS_DVBT) {
		/* Enable parity period for DVB-T */
		cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
		cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x66, 0x01, 0x01);
	} else if (system == SYS_DVBC_ANNEX_A) {
		/* Enable parity period for DVB-C */
		cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x40);
		cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x66, 0x01, 0x01);
	}
}

static u8 cxd2837er_chip_id(struct cxd2837er_priv *priv)
{
	u8 chip_id;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	cxd2837er_write_reg(priv, I2C_SLVT, 0, 0);
	cxd2837er_read_reg(priv, I2C_SLVT, 0xfd, &chip_id);
	return chip_id;
}

static int cxd2837er_read_status_t_t2(struct cxd2837er_priv *priv,
				      u8 *sync, u8 *tslock, u8 *unlock)
{
	u8 data = 0;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC)
		return -EINVAL;
	if (priv->system == SYS_DVBT) {
		/* Set SLV-T Bank : 0x10 */
		cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	} else {
		/* Set SLV-T Bank : 0x20 */
		cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x20);
	}
	cxd2837er_read_reg(priv, I2C_SLVT, 0x10, &data);
	if ((data & 0x07) == 0x07) {
		dev_dbg(&priv->i2c->dev,
			"%s(): invalid hardware state detected\n", __func__);
		*sync = 0;
		*tslock = 0;
		*unlock = 0;
	} else {
		*sync = ((data & 0x07) == 0x6 ? 1 : 0);
		*tslock = ((data & 0x20) ? 1 : 0);
		*unlock = ((data & 0x10) ? 1 : 0);
	}
	return 0;
}

static int cxd2837er_read_status_c(struct cxd2837er_priv *priv, u8 *tslock)
{
	u8 data;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC)
		return -EINVAL;
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x40);
	cxd2837er_read_reg(priv, I2C_SLVT, 0x88, &data);
	if ((data & 0x01) == 0) {
		*tslock = 0;
	} else {
		cxd2837er_read_reg(priv, I2C_SLVT, 0x10, &data);
		*tslock = ((data & 0x20) ? 1 : 0);
	}
	return 0;
}

static int cxd2837er_read_status_tc(struct dvb_frontend *fe,
				    enum fe_status *status)
{
	int ret = 0;
	u8 sync = 0;
	u8 tslock = 0;
	u8 unlock = 0;
	struct cxd2837er_priv *priv = fe->demodulator_priv;

	*status = 0;
	if (priv->state == STATE_ACTIVE_TC) {
		if (priv->system == SYS_DVBT || priv->system == SYS_DVBT2) {
			ret = cxd2837er_read_status_t_t2(
				priv, &sync, &tslock, &unlock);
			if (ret)
				goto done;
			if (unlock)
				goto done;
			if (sync)
				*status = FE_HAS_SIGNAL |
					FE_HAS_CARRIER |
					FE_HAS_VITERBI |
					FE_HAS_SYNC;
			if (tslock)
				*status |= FE_HAS_LOCK;
		} else if (priv->system == SYS_DVBC_ANNEX_A) {
			ret = cxd2837er_read_status_c(priv, &tslock);
			if (ret)
				goto done;
			if (tslock)
				*status = FE_HAS_SIGNAL |
					FE_HAS_CARRIER |
					FE_HAS_VITERBI |
					FE_HAS_SYNC |
					FE_HAS_LOCK;
		}
	}
done:
	dev_dbg(&priv->i2c->dev, "%s(): status 0x%x\n", __func__, *status);
	return ret;
}

static int cxd2837er_get_carrier_offset_t2(struct cxd2837er_priv *priv,
					   u32 bandwidth, int *offset)
{
	u8 data[4];

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	if (priv->system != SYS_DVBT2) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid delivery system %d\n",
			__func__, priv->system);
		return -EINVAL;
	}
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x20);
	cxd2837er_read_regs(priv, I2C_SLVT, 0x4c, data, sizeof(data));
	*offset = -1 * sign_extend32(
		((u32)(data[0] & 0x0F) << 24) | ((u32)data[1] << 16) |
		((u32)data[2] << 8) | (u32)data[3], 27);
	switch (bandwidth) {
	case 1712000:
		*offset /= 582;
		break;
	case 5000000:
	case 6000000:
	case 7000000:
	case 8000000:
		*offset *= (bandwidth / 1000000);
		*offset /= 940;
		break;
	default:
		dev_dbg(&priv->i2c->dev, "%s(): invalid bandwidth %d\n",
			__func__, bandwidth);
		return -EINVAL;
	}
	return 0;
}

static int cxd2837er_get_carrier_offset_c(struct cxd2837er_priv *priv,
					  int *offset)
{
	u8 data[2];

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	if (priv->system != SYS_DVBC_ANNEX_A) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid delivery system %d\n",
			__func__, priv->system);
		return -EINVAL;
	}
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x40);
	cxd2837er_read_regs(priv, I2C_SLVT, 0x15, data, sizeof(data));
	*offset = div_s64(41000LL * sign_extend32((((u32)data[0] & 0x3f) << 8)
						| (u32)data[1], 13), 16384);
	return 0;
}

static int cxd2837er_read_packet_errors_t(
		struct cxd2837er_priv *priv, u32 *penum)
{
	u8 data[3];

	*penum = 0;
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	cxd2837er_read_regs(priv, I2C_SLVT, 0xea, data, sizeof(data));
	if (data[2] & 0x01)
		*penum = ((u32)data[0] << 8) | (u32)data[1];
	return 0;
}

static int cxd2837er_read_packet_errors_t2(
		struct cxd2837er_priv *priv, u32 *penum)
{
	u8 data[3];

	*penum = 0;
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x24);
	cxd2837er_read_regs(priv, I2C_SLVT, 0xfd, data, sizeof(data));
	if (data[0] & 0x01)
		*penum = ((u32)data[1] << 8) | (u32)data[2];
	return 0;
}

static int cxd2837er_read_ber_t2(struct cxd2837er_priv *priv, u32 *ber)
{
	u8 data[4];
	u32 div, q, r;
	u32 bit_err, period_exp, n_ldpc;

	*ber = 0;
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev,
			"%s(): invalid state %d\n", __func__, priv->state);
		return -EINVAL;
	}
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x20);
	cxd2837er_read_regs(priv, I2C_SLVT, 0x39, data, sizeof(data));
	if (!(data[0] & 0x10)) {
		dev_dbg(&priv->i2c->dev,
			"%s(): no valid BER data\n", __func__);
		return 0;
	}
	bit_err = ((u32)(data[0] & 0x0f) << 24) |
		((u32)data[1] << 16) |
		((u32)data[2] << 8) |
		(u32)data[3];
	cxd2837er_read_reg(priv, I2C_SLVT, 0x6f, data);
	period_exp = data[0] & 0x0f;
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x22);
	cxd2837er_read_reg(priv, I2C_SLVT, 0x5e, data);
	n_ldpc = ((data[0] & 0x03) == 0 ? 16200 : 64800);
	if (bit_err > ((1U << period_exp) * n_ldpc)) {
		dev_dbg(&priv->i2c->dev,
			"%s(): invalid BER value\n", __func__);
		return -EINVAL;
	}
	if (period_exp >= 4) {
		div = (1U << (period_exp - 4)) * (n_ldpc / 200);
		q = div_u64_rem(3125ULL * bit_err, div, &r);
	} else {
		div = (1U << period_exp) * (n_ldpc / 200);
		q = div_u64_rem(50000ULL * bit_err, div, &r);
	}
	*ber = (r >= div / 2) ? q + 1 : q;
	return 0;
}

static int cxd2837er_read_ber_t(struct cxd2837er_priv *priv, u32 *ber)
{
	u8 data[2];
	u32 div, q, r;
	u32 bit_err, period;

	*ber = 0;
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev,
			"%s(): invalid state %d\n", __func__, priv->state);
		return -EINVAL;
	}
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	cxd2837er_read_reg(priv, I2C_SLVT, 0x39, data);
	if (!(data[0] & 0x01)) {
		dev_dbg(&priv->i2c->dev,
			"%s(): no valid BER data\n", __func__);
		return 0;
	}
	cxd2837er_read_regs(priv, I2C_SLVT, 0x22, data, sizeof(data));
	bit_err = ((u32)data[0] << 8) | (u32)data[1];
	cxd2837er_read_reg(priv, I2C_SLVT, 0x6f, data);
	period = ((data[0] & 0x07) == 0) ? 256 : (4096 << (data[0] & 0x07));
	div = period / 128;
	q = div_u64_rem(78125ULL * bit_err, div, &r);
	*ber = (r >= div / 2) ? q + 1 : q;
	return 0;
}

static int cxd2837er_read_snr_t(struct cxd2837er_priv *priv, u32 *snr)
{
	u32 reg;
	u8 data[2];

	*snr = 0;
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev,
			"%s(): invalid state %d\n", __func__, priv->state);
		return -EINVAL;
	}
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	cxd2837er_read_regs(priv, I2C_SLVT, 0x28, data, sizeof(data));
	reg = ((u32)data[0] << 8) | (u32)data[1];
	if (reg == 0) {
		dev_dbg(&priv->i2c->dev,
			"%s(): reg value out of range\n", __func__);
		return 0;
	}
	if (reg > 4996)
		reg = 4996;
	*snr = 10000 * ((intlog10(reg) - intlog10(5350 - reg)) >> 24) + 28500;
	return 0;
}

static int cxd2837er_read_snr_t2(struct cxd2837er_priv *priv, u32 *snr)
{
	u32 reg;
	u8 data[2];

	*snr = 0;
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev,
			"%s(): invalid state %d\n", __func__, priv->state);
		return -EINVAL;
	}
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x20);
	cxd2837er_read_regs(priv, I2C_SLVT, 0x28, data, sizeof(data));
	reg = ((u32)data[0] << 8) | (u32)data[1];
	if (reg == 0) {
		dev_dbg(&priv->i2c->dev,
			"%s(): reg value out of range\n", __func__);
		return 0;
	}
	if (reg > 10876)
		reg = 10876;
	*snr = 10000 * ((intlog10(reg) -
		intlog10(12600 - reg)) >> 24) + 32000;
	return 0;
}

static u16 cxd2837er_read_agc_gain_t_t2(struct cxd2837er_priv *priv,
					u8 delsys)
{
	u8 data[2];

	cxd2837er_write_reg(
		priv, I2C_SLVT, 0x00, (delsys == SYS_DVBT ? 0x10 : 0x20));
	cxd2837er_read_regs(priv, I2C_SLVT, 0x26, data, 2);
	return ((((u16)data[0] & 0x0F) << 8) | (u16)(data[1] & 0xFF)) << 4;
}

static int cxd2837er_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct cxd2837er_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	*ber = 0;
	switch (p->delivery_system) {
	case SYS_DVBT:
		return cxd2837er_read_ber_t(priv, ber);
	case SYS_DVBT2:
		return cxd2837er_read_ber_t2(priv, ber);
	default:
		*ber = 0;
		break;
	}
	return 0;
}

static int cxd2837er_read_signal_strength(struct dvb_frontend *fe,
					  u16 *strength)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct cxd2837er_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	switch (p->delivery_system) {
	case SYS_DVBT:
	case SYS_DVBT2:
		*strength = 65535 - cxd2837er_read_agc_gain_t_t2(
			priv, p->delivery_system);
		break;
	default:
		*strength = 0;
		break;
	}
	return 0;
}

static int cxd2837er_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	u32 tmp = 0;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct cxd2837er_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	switch (p->delivery_system) {
	case SYS_DVBT:
		cxd2837er_read_snr_t(priv, &tmp);
		break;
	case SYS_DVBT2:
		cxd2837er_read_snr_t2(priv, &tmp);
		break;
	default:
		dev_dbg(&priv->i2c->dev, "%s(): unknown delivery system %d\n",
			__func__, p->delivery_system);
		break;
	}
	*snr = tmp & 0xffff;
	return 0;
}

static int cxd2837er_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct cxd2837er_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	switch (p->delivery_system) {
	case SYS_DVBT:
		cxd2837er_read_packet_errors_t(priv, ucblocks);
		break;
	case SYS_DVBT2:
		cxd2837er_read_packet_errors_t2(priv, ucblocks);
		break;
	default:
		*ucblocks = 0;
		break;
	}
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	return 0;
}

static int cxd2837er_dvbt2_set_profile(
	struct cxd2837er_priv *priv, enum cxd2837er_dvbt2_profile_t profile)
{
	u8 tune_mode;
	u8 seq_not2d_time;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	switch (profile) {
	case DVBT2_PROFILE_BASE:
		tune_mode = 0x01;
		seq_not2d_time = 12;
		break;
	case DVBT2_PROFILE_LITE:
		tune_mode = 0x05;
		seq_not2d_time = 40;
		break;
	case DVBT2_PROFILE_ANY:
		tune_mode = 0x00;
		seq_not2d_time = 40;
		break;
	default:
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x2E */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x2e);
	/* Set profile and tune mode */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x10, tune_mode, 0x07);
	/* Set SLV-T Bank : 0x2B */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x2b);
	/* Set early unlock detection time */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x9d, seq_not2d_time);
	return 0;
}

static int cxd2837er_dvbt2_set_plp_config(struct cxd2837er_priv *priv,
					  u8 is_auto, u8 plp_id)
{
	if (is_auto) {
		dev_dbg(&priv->i2c->dev,
			"%s() using auto PLP selection\n", __func__);
	} else {
		dev_dbg(&priv->i2c->dev,
			"%s() using manual PLP selection, ID %d\n",
			__func__, plp_id);
	}
	/* Set SLV-T Bank : 0x23 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x23);
	if (!is_auto) {
		/* Manual PLP selection mode. Set the data PLP Id. */
		cxd2837er_write_reg(priv, I2C_SLVT, 0xaf, plp_id);
	}
	/* Auto PLP select (Scanning mode = 0x00). Data PLP select = 0x01. */
	cxd2837er_write_reg(priv, I2C_SLVT, 0xad, (is_auto ? 0x00 : 0x01));
	return 0;
}

static int cxd2837er_sleep_tc_to_active_t2_band(struct cxd2837er_priv *priv,
						u32 bandwidth)
{
	u32 iffreq;
	u8 b20_9f[5];
	u8 b10_a6[14];
	u8 b10_b6[3];
	u8 b10_d7;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	switch (bandwidth) {
	case 8000000:
		/* bank 0x20, reg 0x9f */
		b20_9f[0] = 0x11;
		b20_9f[1] = 0xf0;
		b20_9f[2] = 0x00;
		b20_9f[3] = 0x00;
		b20_9f[4] = 0x00;
		/* bank 0x10, reg 0xa6 */
		b10_a6[0] = 0x26;
		b10_a6[1] = 0xaf;
		b10_a6[2] = 0x06;
		b10_a6[3] = 0xcd;
		b10_a6[4] = 0x13;
		b10_a6[5] = 0xbb;
		b10_a6[6] = 0x28;
		b10_a6[7] = 0xba;
		b10_a6[8] = 0x23;
		b10_a6[9] = 0xa9;
		b10_a6[10] = 0x1f;
		b10_a6[11] = 0xa8;
		b10_a6[12] = 0x2c;
		b10_a6[13] = 0xc8;
		iffreq = MAKE_IFFREQ_CONFIG(4.80);
		b10_d7 = 0x00;
		break;
	case 7000000:
		/* bank 0x20, reg 0x9f */
		b20_9f[0] = 0x14;
		b20_9f[1] = 0x80;
		b20_9f[2] = 0x00;
		b20_9f[3] = 0x00;
		b20_9f[4] = 0x00;
		/* bank 0x10, reg 0xa6 */
		b10_a6[0] = 0x2C;
		b10_a6[1] = 0xBD;
		b10_a6[2] = 0x02;
		b10_a6[3] = 0xCF;
		b10_a6[4] = 0x04;
		b10_a6[5] = 0xF8;
		b10_a6[6] = 0x23;
		b10_a6[7] = 0xA6;
		b10_a6[8] = 0x29;
		b10_a6[9] = 0xB0;
		b10_a6[10] = 0x26;
		b10_a6[11] = 0xA9;
		b10_a6[12] = 0x21;
		b10_a6[13] = 0xA5;
		iffreq = MAKE_IFFREQ_CONFIG(4.2);
		b10_d7 = 0x02;
		break;
	case 6000000:
		/* bank 0x20, reg 0x9f */
		b20_9f[0] = 0x17;
		b20_9f[1] = 0xEA;
		b20_9f[2] = 0xAA;
		b20_9f[3] = 0xAA;
		b20_9f[4] = 0xAA;
		/* bank 0x10, reg 0xa6 */
		b10_a6[0] = 0x27;
		b10_a6[1] = 0xA7;
		b10_a6[2] = 0x28;
		b10_a6[3] = 0xB3;
		b10_a6[4] = 0x02;
		b10_a6[5] = 0xF0;
		b10_a6[6] = 0x01;
		b10_a6[7] = 0xE8;
		b10_a6[8] = 0x00;
		b10_a6[9] = 0xCF;
		b10_a6[10] = 0x00;
		b10_a6[11] = 0xE6;
		b10_a6[12] = 0x23;
		b10_a6[13] = 0xA4;
		iffreq = MAKE_IFFREQ_CONFIG(3.6);
		b10_d7 = 0x04;
		break;
	case 5000000:
		/* bank 0x20, reg 0x9f */
		b20_9f[0] = 0x1C;
		b20_9f[1] = 0xB3;
		b20_9f[2] = 0x33;
		b20_9f[3] = 0x33;
		b20_9f[4] = 0x33;
		/* bank 0x10, reg 0xa6 */
		b10_a6[0] = 0x27;
		b10_a6[1] = 0xA7;
		b10_a6[2] = 0x28;
		b10_a6[3] = 0xB3;
		b10_a6[4] = 0x02;
		b10_a6[5] = 0xF0;
		b10_a6[6] = 0x01;
		b10_a6[7] = 0xE8;
		b10_a6[8] = 0x00;
		b10_a6[9] = 0xCF;
		b10_a6[10] = 0x00;
		b10_a6[11] = 0xE6;
		b10_a6[12] = 0x23;
		b10_a6[13] = 0xA4;
		iffreq = MAKE_IFFREQ_CONFIG(3.6);
		b10_d7 = 0x06;
		break;
	case 1712000:
		/* bank 0x20, reg 0x9f */
		b20_9f[0] = 0x58;
		b20_9f[1] = 0xE2;
		b20_9f[2] = 0xAF;
		b20_9f[3] = 0xE0;
		b20_9f[4] = 0xBC;
		/* bank 0x10, reg 0xa6 */
		b10_a6[0] = 0x25;
		b10_a6[1] = 0xA0;
		b10_a6[2] = 0x36;
		b10_a6[3] = 0x8D;
		b10_a6[4] = 0x2E;
		b10_a6[5] = 0x94;
		b10_a6[6] = 0x28;
		b10_a6[7] = 0x9B;
		b10_a6[8] = 0x32;
		b10_a6[9] = 0x90;
		b10_a6[10] = 0x2C;
		b10_a6[11] = 0x9D;
		b10_a6[12] = 0x29;
		b10_a6[13] = 0x99;
		iffreq = MAKE_IFFREQ_CONFIG(3.5);
		b10_d7 = 0x03;
		break;
	default:
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x20 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x00, 0x20);
	cxd2837er_write_regs(priv, I2C_SLVT, 0x9f, b20_9f, sizeof(b20_9f));
	/* Set SLV-T Bank : 0x27 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x27);
	cxd2837er_set_reg_bits(
		priv, I2C_SLVT, 0x7a,
		(bandwidth == 1712000 ? 0x03 : 0x00), 0x0f);
	/* Set SLV-T Bank : 0x10 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	/* Group delay equaliser sett. for ASCOT2E */
	cxd2837er_write_regs(priv, I2C_SLVT, 0xa6, b10_a6, sizeof(b10_a6));
	/* <IF freq setting> */
	b10_b6[0] = (u8) ((iffreq >> 16) & 0xff);
	b10_b6[1] = (u8)((iffreq >> 8) & 0xff);
	b10_b6[2] = (u8)(iffreq & 0xff);
	cxd2837er_write_regs(priv, I2C_SLVT, 0xb6, b10_b6, sizeof(b10_b6));
	/* System bandwidth setting */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xd7, b10_d7, 0x07);
	return 0;
}

static int cxd2837er_sleep_tc_to_active_t_band(
		struct cxd2837er_priv *priv, u32 bandwidth)
{
	u8 b13_9c[2] = { 0x01, 0x14 };
	u8 bw8mhz_b10_9f[] = { 0x11, 0xF0, 0x00, 0x00, 0x00 };
	u8 bw8mhz_b10_a6[] = { 0x26, 0xAF, 0x06, 0xCD, 0x13, 0xBB,
			0x28, 0xBA, 0x23, 0xA9, 0x1F, 0xA8, 0x2C, 0xC8 };
	u8 bw8mhz_b10_d9[] = { 0x01, 0xE0 };
	u8 bw8mhz_b17_38[] = { 0x01, 0x02 };
	u8 bw7mhz_b10_9f[] = { 0x14, 0x80, 0x00, 0x00, 0x00 };
	u8 bw7mhz_b10_a6[] = { 0x2C, 0xBD, 0x02, 0xCF, 0x04, 0xF8,
			0x23, 0xA6, 0x29, 0xB0, 0x26, 0xA9, 0x21, 0xA5 };
	u8 bw7mhz_b10_d9[] = { 0x12, 0xF8 };
	u8 bw7mhz_b17_38[] = { 0x00, 0x03 };
	u8 bw6mhz_b10_9f[] = { 0x17, 0xEA, 0xAA, 0xAA, 0xAA };
	u8 bw6mhz_b10_a6[] = { 0x27, 0xA7, 0x28, 0xB3, 0x02, 0xF0,
			0x01, 0xE8, 0x00, 0xCF, 0x00, 0xE6, 0x23, 0xA4 };
	u8 bw6mhz_b10_d9[] = { 0x1F, 0xDC };
	u8 bw6mhz_b17_38[] = { 0x00, 0x03 };
	u8 bw5mhz_b10_9f[] = { 0x1C, 0xB3, 0x33, 0x33, 0x33 };
	u8 bw5mhz_b10_a6[] = { 0x27, 0xA7, 0x28, 0xB3, 0x02, 0xF0,
			0x01, 0xE8, 0x00, 0xCF, 0x00, 0xE6, 0x23, 0xA4 };
	u8 bw5mhz_b10_d9[] = { 0x26, 0x3C };
	u8 bw5mhz_b17_38[] = { 0x00, 0x03 };
	u8 b10_b6[3];
	u8 d7val;
	u32 iffreq;
	u8 *b10_9f;
	u8 *b10_a6;
	u8 *b10_d9;
	u8 *b17_38;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x13);
	/* Echo performance optimization setting */
	cxd2837er_write_regs(priv, I2C_SLVT, 0x9c, b13_9c, sizeof(b13_9c));
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x10);

	switch (bandwidth) {
	case 8000000:
		b10_9f = bw8mhz_b10_9f;
		b10_a6 = bw8mhz_b10_a6;
		b10_d9 = bw8mhz_b10_d9;
		b17_38 = bw8mhz_b17_38;
		d7val = 0;
		iffreq = MAKE_IFFREQ_CONFIG(4.80);
		break;
	case 7000000:
		b10_9f = bw7mhz_b10_9f;
		b10_a6 = bw7mhz_b10_a6;
		b10_d9 = bw7mhz_b10_d9;
		b17_38 = bw7mhz_b17_38;
		d7val = 2;
		iffreq = MAKE_IFFREQ_CONFIG(4.20);
		break;
	case 6000000:
		b10_9f = bw6mhz_b10_9f;
		b10_a6 = bw6mhz_b10_a6;
		b10_d9 = bw6mhz_b10_d9;
		b17_38 = bw6mhz_b17_38;
		d7val = 4;
		iffreq = MAKE_IFFREQ_CONFIG(3.60);
		break;
	case 5000000:
		b10_9f = bw5mhz_b10_9f;
		b10_a6 = bw5mhz_b10_a6;
		b10_d9 = bw5mhz_b10_d9;
		b17_38 = bw5mhz_b17_38;
		d7val = 6;
		iffreq = MAKE_IFFREQ_CONFIG(3.60);
		break;
	default:
		dev_dbg(&priv->i2c->dev, "%s(): invalid bandwidth %d\n",
			__func__, bandwidth);
		return -EINVAL;
	}
	/* <IF freq setting> */
	b10_b6[0] = (u8) ((iffreq >> 16) & 0xff);
	b10_b6[1] = (u8)((iffreq >> 8) & 0xff);
	b10_b6[2] = (u8)(iffreq & 0xff);
	cxd2837er_write_regs(
		priv, I2C_SLVT, 0x9f, b10_9f, sizeof(bw8mhz_b10_9f));
	cxd2837er_write_regs(
		priv, I2C_SLVT, 0xa6, b10_a6, sizeof(bw8mhz_b10_a6));
	cxd2837er_write_regs(priv, I2C_SLVT, 0xb6, b10_b6, sizeof(b10_b6));
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xd7, d7val, 0x7);
	cxd2837er_write_regs(
		priv, I2C_SLVT, 0xd9, b10_d9, sizeof(bw8mhz_b10_d9));
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x17);
	cxd2837er_write_regs(
		priv, I2C_SLVT, 0x38, b17_38, sizeof(bw8mhz_b17_38));
	return 0;
}

static int cxd2837er_sleep_tc_to_active_c_band(struct cxd2837er_priv *priv,
					       u32 bandwidth)
{
	u8 bw7_8mhz_b10_a6[] = {
		0x2D, 0xC7, 0x04, 0xF4, 0x07, 0xC5, 0x2A, 0xB8,
		0x27, 0x9E, 0x27, 0xA4, 0x29, 0xAB };
	u8 bw6mhz_b10_a6[] = {
		0x27, 0xA7, 0x28, 0xB3, 0x02, 0xF0, 0x01, 0xE8,
		0x00, 0xCF, 0x00, 0xE6, 0x23, 0xA4 };
	u8 b10_b6[3];
	u32 iffreq;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	switch (bandwidth) {
	case 8000000:
	case 7000000:
		cxd2837er_write_regs(
			priv, I2C_SLVT, 0xa6,
			bw7_8mhz_b10_a6, sizeof(bw7_8mhz_b10_a6));
		iffreq = MAKE_IFFREQ_CONFIG(4.9);
		break;
	case 6000000:
		cxd2837er_write_regs(
			priv, I2C_SLVT, 0xa6,
			bw6mhz_b10_a6, sizeof(bw6mhz_b10_a6));
		iffreq = MAKE_IFFREQ_CONFIG(3.7);
		break;
	default:
		dev_dbg(&priv->i2c->dev, "%s(): unsupported bandwidth %d\n",
			__func__, bandwidth);
		return -EINVAL;
	}
	/* <IF freq setting> */
	b10_b6[0] = (u8) ((iffreq >> 16) & 0xff);
	b10_b6[1] = (u8)((iffreq >> 8) & 0xff);
	b10_b6[2] = (u8)(iffreq & 0xff);
	cxd2837er_write_regs(priv, I2C_SLVT, 0xb6, b10_b6, sizeof(b10_b6));
	/* Set SLV-T Bank : 0x11 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x11);
	switch (bandwidth) {
	case 8000000:
	case 7000000:
		cxd2837er_set_reg_bits(
			priv, I2C_SLVT, 0xa3, 0x00, 0x1f);
		break;
	case 6000000:
		cxd2837er_set_reg_bits(
			priv, I2C_SLVT, 0xa3, 0x14, 0x1f);
		break;
	}
	/* Set SLV-T Bank : 0x40 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x40);
	switch (bandwidth) {
	case 8000000:
		cxd2837er_set_reg_bits(
			priv, I2C_SLVT, 0x26, 0x0b, 0x0f);
		cxd2837er_write_reg(priv, I2C_SLVT,  0x27, 0x3e);
		break;
	case 7000000:
		cxd2837er_set_reg_bits(
			priv, I2C_SLVT, 0x26, 0x09, 0x0f);
		cxd2837er_write_reg(priv, I2C_SLVT,  0x27, 0xd6);
		break;
	case 6000000:
		cxd2837er_set_reg_bits(
			priv, I2C_SLVT, 0x26, 0x08, 0x0f);
		cxd2837er_write_reg(priv, I2C_SLVT,  0x27, 0x6e);
		break;
	}
	return 0;
}

static int cxd2837er_sleep_tc_to_active_t(struct cxd2837er_priv *priv,
					  u32 bandwidth)
{
	u8 data[2] = { 0x09, 0x54 };

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	cxd2837er_set_ts_clock_mode(priv, SYS_DVBT);
	/* Set SLV-X Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* Set demod mode */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x17, 0x01);
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* Enable demod clock */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x2c, 0x01);
	/* Disable RF level monitor */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x2f, 0x00);
	/* Enable ADC clock */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x30, 0x00);
	/* Enable ADC 1 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x41, 0x1a);
	/* xtal freq 20.5MHz */
	cxd2837er_write_regs(priv, I2C_SLVT, 0x43, data, 2);
	/* Enable ADC 4 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x18, 0x00);
	/* Set SLV-T Bank : 0x10 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	/* IFAGC gain settings */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xd2, 0x0c, 0x1f);
	/* Set SLV-T Bank : 0x11 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x11);
	/* BBAGC TARGET level setting */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x6a, 0x50);
	/* Set SLV-T Bank : 0x10 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	/* ASCOT setting ON */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xa5, 0x01, 0x01);
	/* Set SLV-T Bank : 0x18 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x18);
	/* Pre-RS BER moniter setting */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x36, 0x40, 0x07);
	/* FEC Auto Recovery setting */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x30, 0x01, 0x01);
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x31, 0x01, 0x01);
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* TSIF setting */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xce, 0x01, 0x01);
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xcf, 0x01, 0x01);
	cxd2837er_sleep_tc_to_active_t_band(priv, bandwidth);
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* Disable HiZ Setting 1 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x80, 0x28);
	/* Disable HiZ Setting 2 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x81, 0x00);
	priv->state = STATE_ACTIVE_TC;
	return 0;
}

static int cxd2837er_sleep_tc_to_active_t2(struct cxd2837er_priv *priv,
					   u32 bandwidth)
{
	u8 data[2] = { 0x09, 0x54 };

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	cxd2837er_set_ts_clock_mode(priv, SYS_DVBT2);
	/* Set SLV-X Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* Set demod mode */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x17, 0x02);
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* Enable demod clock */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x2c, 0x01);
	/* Disable RF level monitor */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x2f, 0x00);
	/* Enable ADC clock */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x30, 0x00);
	/* Enable ADC 1 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x41, 0x1a);
	/* xtal freq 20.5MHz */
	cxd2837er_write_regs(priv, I2C_SLVT, 0x43, data, 2);
	/* Enable ADC 4 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x18, 0x00);
	/* Set SLV-T Bank : 0x10 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	/* IFAGC gain settings */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xd2, 0x0c, 0x1f);
	/* Set SLV-T Bank : 0x11 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x11);
	/* BBAGC TARGET level setting */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x6a, 0x50);
	/* Set SLV-T Bank : 0x10 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	/* ASCOT setting ON */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xa5, 0x01, 0x01);
	/* Set SLV-T Bank : 0x20 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x20);
	/* Acquisition optimization setting */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x8b, 0x3c);
	/* Set SLV-T Bank : 0x2b */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x2b);
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x76, 0x20, 0x70);
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* TSIF setting */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xce, 0x01, 0x01);
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xcf, 0x01, 0x01);
	/* DVB-T2 initial setting */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x13);
	cxd2837er_write_reg(priv, I2C_SLVT, 0x83, 0x10);
	cxd2837er_write_reg(priv, I2C_SLVT, 0x86, 0x34);
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x9e, 0x09, 0x0f);
	cxd2837er_write_reg(priv, I2C_SLVT, 0x9f, 0xd8);
	/* Set SLV-T Bank : 0x2a */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x2a);
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x38, 0x04, 0x0f);
	/* Set SLV-T Bank : 0x2b */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x2b);
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0x11, 0x20, 0x3f);

	cxd2837er_sleep_tc_to_active_t2_band(priv, bandwidth);

	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* Disable HiZ Setting 1 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x80, 0x28);
	/* Disable HiZ Setting 2 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x81, 0x00);
	priv->state = STATE_ACTIVE_TC;
	return 0;
}

static int cxd2837er_sleep_tc_to_active_c(struct cxd2837er_priv *priv,
					  u32 bandwidth)
{
	u8 data[2] = { 0x09, 0x54 };

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	cxd2837er_set_ts_clock_mode(priv, SYS_DVBC_ANNEX_A);
	/* Set SLV-X Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* Set demod mode */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x17, 0x04);
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* Enable demod clock */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x2c, 0x01);
	/* Disable RF level monitor */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x2f, 0x00);
	/* Enable ADC clock */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x30, 0x00);
	/* Enable ADC 1 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x41, 0x1a);
	/* xtal freq 20.5MHz */
	cxd2837er_write_regs(priv, I2C_SLVT, 0x43, data, 2);
	/* Enable ADC 4 */
	cxd2837er_write_reg(priv, I2C_SLVX, 0x18, 0x00);
	/* Set SLV-T Bank : 0x10 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	/* IFAGC gain settings */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xd2, 0x09, 0x1f);
	/* Set SLV-T Bank : 0x11 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x11);
	/* BBAGC TARGET level setting */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x6a, 0x48);
	/* Set SLV-T Bank : 0x10 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	/* ASCOT setting ON */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xa5, 0x01, 0x01);
	/* Set SLV-T Bank : 0x40 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x40);
	/* Demod setting */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xc3, 0x00, 0x04);
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* TSIF setting */
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xce, 0x01, 0x01);
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xcf, 0x01, 0x01);

	cxd2837er_sleep_tc_to_active_c_band(priv, 8000000);
	/* Set SLV-T Bank : 0x00 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* Disable HiZ Setting 1 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x80, 0x28);
	/* Disable HiZ Setting 2 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x81, 0x00);
	priv->state = STATE_ACTIVE_TC;
	return 0;
}

static int cxd2837er_get_frontend(struct dvb_frontend *fe)
{
	enum fe_status status = 0;
	u16 strength = 0, snr = 0;
	u32 errors = 0, ber = 0;
	struct cxd2837er_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state == STATE_ACTIVE_TC)
		cxd2837er_read_status_tc(fe, &status);

	if (status & FE_HAS_LOCK) {
		cxd2837er_read_signal_strength(fe, &strength);
		p->strength.len = 1;
		p->strength.stat[0].scale = FE_SCALE_RELATIVE;
		p->strength.stat[0].uvalue = strength;
		cxd2837er_read_snr(fe, &snr);
		p->cnr.len = 1;
		p->cnr.stat[0].scale = FE_SCALE_DECIBEL;
		p->cnr.stat[0].svalue = snr;
		cxd2837er_read_ucblocks(fe, &errors);
		p->block_error.len = 1;
		p->block_error.stat[0].scale = FE_SCALE_COUNTER;
		p->block_error.stat[0].uvalue = errors;
		cxd2837er_read_ber(fe, &ber);
		p->post_bit_error.len = 1;
		p->post_bit_error.stat[0].scale = FE_SCALE_COUNTER;
		p->post_bit_error.stat[0].uvalue = ber;
	} else {
		p->strength.len = 1;
		p->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		p->cnr.len = 1;
		p->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		p->block_error.len = 1;
		p->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		p->post_bit_error.len = 1;
		p->post_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	}
	return 0;
}

static int cxd2837er_set_frontend_tc(struct dvb_frontend *fe)
{
	int ret = 0, timeout;
	enum fe_status status;
	struct cxd2837er_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (p->delivery_system == SYS_DVBT) {
		priv->system = SYS_DVBT;
		switch (priv->state) {
		case STATE_SLEEP_TC:
			ret = cxd2837er_sleep_tc_to_active_t(
				priv, p->bandwidth_hz);
			break;
		case STATE_ACTIVE_TC:
			ret = cxd2837er_retune_active(priv, p);
			break;
		default:
			dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
				__func__, priv->state);
			ret = -EINVAL;
		}
	} else if (p->delivery_system == SYS_DVBT2) {
		priv->system = SYS_DVBT2;
		cxd2837er_dvbt2_set_plp_config(priv,
			(int)(p->stream_id > 255), p->stream_id);
		cxd2837er_dvbt2_set_profile(priv, DVBT2_PROFILE_BASE);
		switch (priv->state) {
		case STATE_SLEEP_TC:
			ret = cxd2837er_sleep_tc_to_active_t2(priv,
				p->bandwidth_hz);
			break;
		case STATE_ACTIVE_TC:
			ret = cxd2837er_retune_active(priv, p);
			break;
		default:
			dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
				__func__, priv->state);
			ret = -EINVAL;
		}
	} else if (p->delivery_system == SYS_DVBC_ANNEX_A ||
			p->delivery_system == SYS_DVBC_ANNEX_C) {
		priv->system = SYS_DVBC_ANNEX_A;
		switch (priv->state) {
		case STATE_SLEEP_TC:
			ret = cxd2837er_sleep_tc_to_active_c(
				priv, p->bandwidth_hz);
			break;
		case STATE_ACTIVE_TC:
			ret = cxd2837er_retune_active(priv, p);
			break;
		default:
			dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
				__func__, priv->state);
			ret = -EINVAL;
		}
	} else {
		dev_dbg(&priv->i2c->dev,
			"%s(): invalid delivery system %d\n",
			__func__, p->delivery_system);
		ret = -EINVAL;
	}
	if (ret)
		goto done;
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);
	if (fe->ops.tuner_ops.set_params)
		fe->ops.tuner_ops.set_params(fe);
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);
	cxd2837er_tune_done(priv);
	timeout = 2500;
	while (timeout > 0) {
		ret = cxd2837er_read_status_tc(fe, &status);
		if (ret)
			goto done;
		if (status & FE_HAS_LOCK)
			break;
		msleep(20);
		timeout -= 20;
	}
	if (timeout < 0)
		dev_dbg(&priv->i2c->dev,
			"%s(): LOCK wait timeout\n", __func__);
done:
	return ret;
}

static int cxd2837er_tune_tc(struct dvb_frontend *fe,
			     bool re_tune,
			     unsigned int mode_flags,
			     unsigned int *delay,
			     enum fe_status *status)
{
	int ret, carrier_offset;
	struct cxd2837er_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;

	dev_dbg(&priv->i2c->dev, "%s(): re_tune %d\n", __func__, re_tune);
	if (re_tune) {
		ret = cxd2837er_set_frontend_tc(fe);
		if (ret)
			return ret;
		cxd2837er_read_status_tc(fe, status);
		if (*status & FE_HAS_LOCK) {
			switch (priv->system) {
			case SYS_DVBT:
			case SYS_DVBT2:
				ret = cxd2837er_get_carrier_offset_t2(
					priv, p->bandwidth_hz,
					&carrier_offset);
				break;
			case SYS_DVBC_ANNEX_A:
				ret = cxd2837er_get_carrier_offset_c(
					priv, &carrier_offset);
				break;
			default:
				dev_dbg(&priv->i2c->dev,
					"%s(): invalid delivery system %d\n",
					__func__, priv->system);
				return -EINVAL;
			}
			if (ret)
				return ret;
			dev_dbg(&priv->i2c->dev, "%s(): carrier offset %d\n",
				__func__, carrier_offset);
			p->frequency += carrier_offset;
			ret = cxd2837er_set_frontend_tc(fe);
			if (ret)
				return ret;
		}
	}
	*delay = HZ / 5;
	return cxd2837er_read_status_tc(fe, status);
}

static int cxd2837er_sleep_tc(struct dvb_frontend *fe)
{
	struct cxd2837er_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state == STATE_ACTIVE_TC) {
		switch (priv->system) {
		case SYS_DVBT:
			cxd2837er_active_t_to_sleep_tc(priv);
			break;
		case SYS_DVBT2:
			cxd2837er_active_t2_to_sleep_tc(priv);
			break;
		case SYS_DVBC_ANNEX_A:
			cxd2837er_active_c_to_sleep_tc(priv);
			break;
		default:
			dev_warn(&priv->i2c->dev,
				"%s(): unknown delivery system %d\n",
				__func__, priv->system);
		}
	}
	if (priv->state != STATE_SLEEP_TC) {
		dev_err(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	cxd2837er_sleep_tc_to_shutdown(priv);
	return 0;
}

static void cxd2837er_release(struct dvb_frontend *fe)
{
	struct cxd2837er_priv *priv  = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	kfree(priv);
}

static int cxd2837er_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct cxd2837er_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s(): enable=%d\n", __func__, enable);
	cxd2837er_set_reg_bits(
		priv, I2C_SLVX, 0x8, (enable ? 0x01 : 0x00), 0x01);
	return 0;
}

static enum dvbfe_algo cxd2837er_get_algo(struct dvb_frontend *fe)
{
	struct cxd2837er_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	return DVBFE_ALGO_HW;
}

static int cxd2837er_init_tc(struct dvb_frontend *fe)
{
	struct cxd2837er_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	cxd2837er_shutdown_to_sleep_tc(priv);
	/* SONY_DEMOD_CONFIG_IFAGCNEG = 1 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xcb, 
						priv->config->if_agc ? 0x40 : 0x00, 0x40);
	/* SONY_DEMOD_CONFIG_IFAGC_ADC_FS = 0 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0xcd, 
						priv->config->ifagc_adc_range);
	/* SONY_DEMOD_CONFIG_PARALLEL_SEL = 1 */
	cxd2837er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xCB, 
					priv->config->ts_error_polarity ? 0x00 : 0x01, 0x01);
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xC5, 
					priv->config->clock_polarity ? 0x01 : 0x00, 0x01);
	cxd2837er_set_reg_bits(priv, I2C_SLVT, 0xc4, 0x00, 0x80);
	return 0;
}

static struct dvb_frontend_ops cxd2837er_ops;

struct dvb_frontend *cxd2837er_attach(struct cxd2837er_config *cfg,
					     struct i2c_adapter *i2c)
{
	u8 chip_id = 0;
	struct cxd2837er_priv *priv = NULL;

	/* allocate memory for the internal state */
	priv = kzalloc(sizeof(struct cxd2837er_priv), GFP_KERNEL);
	if (!priv)
		return NULL;
	priv->i2c = i2c;
	priv->config = cfg;
	priv->i2c_addr_slvx = cfg->i2c_addr + 2;
	priv->i2c_addr_slvt = cfg->i2c_addr;

	memcpy(&priv->frontend.ops,
		&cxd2837er_ops,
		sizeof(struct dvb_frontend_ops));
		
	priv->frontend.demodulator_priv = priv;
	dev_info(&priv->i2c->dev,
		"%s(): attaching CXD2837 frontend\n",
		__func__);
	dev_info(&priv->i2c->dev,
		"%s(): I2C adapter %p SLVX addr %x SLVT addr %x\n",
		__func__, priv->i2c,
		priv->i2c_addr_slvx, priv->i2c_addr_slvt);
	chip_id = cxd2837er_chip_id(priv);
	if (chip_id != CXD2837ER_CHIP_ID) {
		dev_err(&priv->i2c->dev, "%s(): invalid chip ID 0x%02x\n",
			__func__, chip_id);
		priv->frontend.demodulator_priv = NULL;
		kfree(priv);
		return NULL;
	}
	dev_info(&priv->i2c->dev, "%s(): chip ID 0x%02x OK.\n",
		__func__, chip_id);
	return &priv->frontend;
}

static struct  dvb_frontend_ops cxd2837er_ops = {
	.delsys = { SYS_DVBT, SYS_DVBT2, SYS_DVBC_ANNEX_A },
	.info = {
		.name	= "Sony CXD2837 DVB-T/T2/C demodulator",
		.caps = FE_CAN_FEC_1_2 |
			FE_CAN_FEC_2_3 |
			FE_CAN_FEC_3_4 |
			FE_CAN_FEC_5_6 |
			FE_CAN_FEC_7_8 |
			FE_CAN_FEC_AUTO |
			FE_CAN_QPSK |
			FE_CAN_QAM_16 |
			FE_CAN_QAM_32 |
			FE_CAN_QAM_64 |
			FE_CAN_QAM_128 |
			FE_CAN_QAM_256 |
			FE_CAN_QAM_AUTO |
			FE_CAN_INVERSION_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO |
			FE_CAN_HIERARCHY_AUTO |
			FE_CAN_MUTE_TS |
			FE_CAN_2G_MODULATION,
		.frequency_min = 42000000,
		.frequency_max = 1002000000
	},
	.init = cxd2837er_init_tc,
	.read_ber = cxd2837er_read_ber,
	.sleep = cxd2837er_sleep_tc,
	.release = cxd2837er_release,
	.set_frontend = cxd2837er_set_frontend_tc,
	.get_frontend = cxd2837er_get_frontend,
	.read_status = cxd2837er_read_status_tc,
	.tune = cxd2837er_tune_tc,
	.i2c_gate_ctrl = cxd2837er_i2c_gate_ctrl,
	.get_frontend_algo = cxd2837er_get_algo
};

MODULE_DESCRIPTION("Sony CXD2837ER DVB-C/T/T2/ demodulator driver");
MODULE_AUTHOR("sasa.savic.sr@gmail.com");
MODULE_LICENSE("GPL");
