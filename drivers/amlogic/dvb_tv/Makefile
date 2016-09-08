#
# Makefile for the WetekPlay driver.
#

obj-$(CONFIG_AM_DVB) += wetekplay.o

wetekplay-objs = nimdetect.o mxl603.o avl6211.o mn88436.o cxd2837er.o ascot3.o

EXTRA_CFLAGS += -Idrivers/media/dvb-core -Idrivers/media/tuners
EXTRA_CFLAGS += -I.
