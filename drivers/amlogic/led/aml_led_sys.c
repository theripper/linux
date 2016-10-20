/*
 * drivers/amlogic/led/aml_led_sys.h
 *
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
*/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/slab.h>

#include "aml_led_sys.h"


#define AML_DEV_NAME		"sysled"
#define AML_LED_NAME		"led-sys"

#define DEBUG

#ifndef DEBUG
#define INFO(format, arg...)
#define ERR(format,  arg...)
#else
#define INFO(format, arg...) pr_info("%s: " format, __func__ , ## arg)
#define ERR(format,  arg...) pr_err("%s: "  format, __func__ , ## arg)
#endif

#ifdef CONFIG_LEDS_TRIGGER_NETWORK

#define GPIO_OWNER_WIFILED "WIFILED"
#define GPIO_OWNER_ETHLED "ETHLED"

int wifiLedGpio = -1;
int ethLedGpio = -1;

static void wetekplay_wifiled_set(struct led_classdev *cdev,
			  enum led_brightness brightness)
{
	if (wifiLedGpio > 0)
		gpio_direction_output(wifiLedGpio, brightness > 0 ? 1 : 0);
}

static enum led_brightness wetekplay_wifiled_get(struct led_classdev *cdev)
{
	if (wifiLedGpio > 0 && gpio_get_value(wifiLedGpio))
	    return 255;
	else
	    return 0;
}
static void wetekplay_ethled_set(struct led_classdev *cdev,
			  enum led_brightness brightness)
{
	if (ethLedGpio > 0)
		gpio_direction_output(ethLedGpio, brightness > 0 ? 1 : 0);
}

static enum led_brightness wetekplay_ethled_get(struct led_classdev *cdev)
{
	if (ethLedGpio > 0 && gpio_get_value(ethLedGpio))
	    return 255;
	else
	    return 0;
}
static struct led_classdev wetekplay_wifiled = {
	.name = "wetek:blue:wifiled",
	.brightness_set = wetekplay_wifiled_set,
	.brightness_get = wetekplay_wifiled_get,
	.default_trigger = "wifilink",
};

static struct led_classdev wetekplay_ethled = {
	.name = "wetek:blue:ethled",
	.brightness_set = wetekplay_ethled_set,
	.brightness_get = wetekplay_ethled_get,
	.default_trigger = "ethlink",
};
#endif

static void aml_sysled_work(struct work_struct *work)
{
	struct aml_sysled_dev *ldev;
	unsigned int level;

	ldev = container_of(work, struct aml_sysled_dev, work);

	mutex_lock(&ldev->lock);

	if (ldev->new_brightness == LED_OFF)
		level = 0;
	else
		level = 1;

	if (ldev->d.active_low)
		level = !level;

	gpio_direction_output(ldev->d.pin, level);

	mutex_unlock(&ldev->lock);
}


static void aml_sysled_brightness_set(struct led_classdev *cdev,
	enum led_brightness value)
{
	struct aml_sysled_dev *ldev;
	struct platform_device *pdev;

	pdev = to_platform_device(cdev->dev->parent);
	ldev = platform_get_drvdata(pdev);
	ldev->new_brightness = value;
	schedule_work(&ldev->work);
}


static int aml_sysled_dt_parse(struct platform_device *pdev)
{
	struct device_node *node;
	struct aml_sysled_dev *ldev;
	struct gpio_desc *desc;
	unsigned int val;
	int ret;

	ldev = platform_get_drvdata(pdev);
	node = pdev->dev.of_node;
	if (!node) {
		ERR("failed to find node for %s\n", AML_DEV_NAME);
		return -ENODEV;
	}

	desc = of_get_named_gpiod_flags(node, "led_gpio", 0, NULL);
	ldev->d.pin = desc_to_gpio(desc);
	gpio_request(ldev->d.pin, AML_DEV_NAME);

	ret = of_property_read_u32(node, "led_active_low", &val);
	if (ret) {
		ERR("faild to get active_low\n");
		/* set default active_low */
		val = 1;
	}
	INFO("active_low = %u\n", val);
	ldev->d.active_low = !!val;
#ifdef CONFIG_LEDS_TRIGGER_NETWORK
	{
		int ret;
		const char *str;
		
		ret = of_property_read_string(node,
			"eth_gpio", &str);
		if (!ret) {
			desc = of_get_named_gpiod_flags(node,
				"eth_gpio", 0, NULL);
			ethLedGpio = desc_to_gpio(desc);
			gpio_request(ethLedGpio, GPIO_OWNER_ETHLED);
		}
		
		ret = of_property_read_string(node,
			"wifi_gpio", &str);
		if (!ret) {						
			desc = of_get_named_gpiod_flags(node,
				"wifi_gpio", 0, NULL);
			wifiLedGpio = desc_to_gpio(desc);
			gpio_request(wifiLedGpio, GPIO_OWNER_WIFILED);
		}
	}
#endif	
	return 0;
}



static const struct of_device_id aml_sysled_dt_match[] = {
	{
		.compatible = "amlogic, sysled",
	},
	{},
};


static int aml_sysled_probe(struct platform_device *pdev)
{
	struct aml_sysled_dev *ldev;
	int ret;

	ldev = kzalloc(sizeof(struct aml_sysled_dev), GFP_KERNEL);
	if (!ldev) {
		ERR("kzalloc error\n");
		return -ENOMEM;
	}

	/* set driver data */
	platform_set_drvdata(pdev, ldev);

	/* parse dt param */
	ret = aml_sysled_dt_parse(pdev);
	if (ret)
		return ret;

	/* register led class device */
	ldev->cdev.name = AML_LED_NAME;
	ldev->cdev.brightness_set = aml_sysled_brightness_set;
	mutex_init(&ldev->lock);
	INIT_WORK(&ldev->work, aml_sysled_work);
	ret = led_classdev_register(&pdev->dev, &ldev->cdev);
	if (ret < 0) {
		kfree(ldev);
		return ret;
	}
#ifdef CONFIG_LEDS_TRIGGER_NETWORK	
	led_classdev_register(&pdev->dev, &wetekplay_wifiled);
	led_classdev_register(&pdev->dev, &wetekplay_ethled);
	
	if (ethLedGpio > 0)
		gpio_direction_output(ethLedGpio, 0);
	if (wifiLedGpio > 0)
		gpio_direction_output(wifiLedGpio, 0);
#endif	

	INFO("module probed ok\n");
	return 0;
}


static int __exit aml_sysled_remove(struct platform_device *pdev)
{
	struct aml_sysled_dev *ldev = platform_get_drvdata(pdev);

	led_classdev_unregister(&ldev->cdev);
	cancel_work_sync(&ldev->work);
	gpio_free(ldev->d.pin);
	platform_set_drvdata(pdev, NULL);
	kfree(ldev);
#ifdef CONFIG_LEDS_TRIGGER_NETWORK	
	led_classdev_unregister(&wetekplay_wifiled);
	led_classdev_unregister(&wetekplay_ethled);
#endif	
	return 0;
}


static struct platform_driver aml_sysled_driver = {
	.driver = {
		.name = AML_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = aml_sysled_dt_match,
	},
	.probe = aml_sysled_probe,
	.remove = __exit_p(aml_sysled_remove),
};


static int __init aml_sysled_init(void)
{
	INFO("module init\n");
	if (platform_driver_register(&aml_sysled_driver)) {
		ERR("failed to register driver\n");
		return -ENODEV;
	}

	return 0;
}


static void __exit aml_sysled_exit(void)
{
	INFO("module exit\n");
	platform_driver_unregister(&aml_sysled_driver);
}


module_init(aml_sysled_init);
module_exit(aml_sysled_exit);

MODULE_DESCRIPTION("Amlogic sys led driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Amlogic, Inc.");

