/*
 *  Copyright (C) 2019, Matt Hsu <matt.hsu@ubnt.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/delay.h>

#include <linux/power/ubnt-hdd-pwrctl.h>

#define UBNT_HDD_PWRCTL_DELAY (HZ * 2)

struct ubnt_hdd_pwrctl {
	struct device *dev;

	struct delayed_work timeout;
	const struct ubnt_hdd_pwrctl_platform_data *pdata;
	bool present_state[UBNT_HDD_NUM];
	bool fault_state[UBNT_HDD_NUM];
};

static void
ubnt_hdd_pwrctl_work(struct work_struct *work)
{
	struct ubnt_hdd_pwrctl *pwrctl = container_of(work,
							struct ubnt_hdd_pwrctl, timeout.work);
	const struct ubnt_hdd_pwrctl_platform_data *pdata;
	int i, ret;

	pdata = pwrctl->pdata;

	for (i = 0; i < UBNT_HDD_NUM; i++) {
		/* present_gpio is LOW active */
		dev_dbg(pwrctl->dev, "present_gpio# %d, present_gpio %d, present_state %d\n",
				(pdata->present_gpio + i),
				gpio_get_value_cansleep(pdata->present_gpio + i),
				pwrctl->present_state[i]);

		if ((!gpio_get_value_cansleep(pdata->present_gpio + i)) != pwrctl->present_state[i]) {
			msleep(pdata->pwren_delay);

			ret = gpio_direction_output((pdata->pwren_gpio + i), !pwrctl->present_state[i]);
			if (ret) {
				dev_err(pwrctl->dev, "Failed to set gpio# %d to output: %d\n",
								(pdata->pwren_gpio + i), ret);
			}
			pwrctl->present_state[i] = !pwrctl->present_state[i];
			dev_dbg(pwrctl->dev, "set pwren_gpio# %d as %d, update state %d\n",
						(pdata->pwren_gpio + i),
						!pwrctl->present_state[i],
						pwrctl->present_state[i]);
		}
	}

	schedule_delayed_work(&pwrctl->timeout, UBNT_HDD_PWRCTL_DELAY);
}

static
struct ubnt_hdd_pwrctl_platform_data *ubnt_hdd_pwrctl_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct ubnt_hdd_pwrctl_platform_data *pdata;
	int ret;

	if (!np)
		return ERR_PTR(-ENOENT);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_u32(np, "hdds-pwren", &pdata->pwren_gpio);
	if (ret) {
		dev_err(dev, "failed to get hdds-pwren %d\n", ret);
	}

	ret = of_property_read_u32(np, "hdds-present", &pdata->present_gpio);
	if (ret) {
		dev_err(dev, "failed to get hdds-present %d\n", ret);
	}

	ret = of_property_read_u32(np, "hdds-fault-led", &pdata->fault_led_gpio);
	if (ret) {
		dev_err(dev, "failed to get hdds-fault-led %d\n", ret);
	}

	ret = of_property_read_u32(np, "pwren-delay", &pdata->pwren_delay);
	if (ret) {
		dev_err(dev, "failed to get power enable delay %d\n", ret);
		pdata->pwren_delay = 500;
	}

	dev_info(dev, "pwren %u, present %u, fault-led %u, pwren delay %u\n",
					pdata->pwren_gpio,
					pdata->present_gpio,
					pdata->fault_led_gpio,
					pdata->pwren_delay);

	return pdata;
}

static ssize_t show_hdd_present(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	int i;
	struct ubnt_hdd_pwrctl *pwrctl = dev_get_drvdata(dev);
	char present_state[32];
	char *p = present_state;

	for (i = 0; i < UBNT_HDD_NUM; i++) {
		if (pwrctl->present_state[i])
			p += sprintf(p, "%d ", i);
	}
	p += sprintf(p, "\n");

	return snprintf(buf, PAGE_SIZE, "%s", present_state);
}

static ssize_t show_hdd_fault_led(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned int hdd_idx;
	const char *p = attr->attr.name + strlen(attr->attr.name);
	struct ubnt_hdd_pwrctl *pwrctl = dev_get_drvdata(dev);
	const struct ubnt_hdd_pwrctl_platform_data *pdata = pwrctl->pdata;

	ret = kstrtouint((p - 1), 10, &hdd_idx);
	if (ret < 0)
		return ret;

    return snprintf(buf, PAGE_SIZE, "%d\n",
				gpio_get_value_cansleep(pdata->fault_led_gpio + hdd_idx));
}

static ssize_t store_hdd_fault_led(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;
	unsigned int hdd_idx;
	const char *p = attr->attr.name + strlen(attr->attr.name);
	struct ubnt_hdd_pwrctl *pwrctl = dev_get_drvdata(dev);
	const struct ubnt_hdd_pwrctl_platform_data *pdata = pwrctl->pdata;

	ret = kstrtoul(buf, 10, &val);
	if (ret < 0)
		return ret;

	ret = kstrtouint((p - 1), 10, &hdd_idx);
	if (ret < 0)
		return ret;

	ret = gpio_direction_output((pdata->fault_led_gpio + (hdd_idx & 0x3)), !!val);
	if (ret) {
		dev_err(pwrctl->dev, "Failed to set gpio# %u to output: %d\n",
					(pdata->fault_led_gpio + (hdd_idx & 0x3)), ret);
	}
	pwrctl->fault_state[hdd_idx] = (!!val);
	return count;
}
static ssize_t show_hdd_power(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	int i;
	struct ubnt_hdd_pwrctl *pwrctl = dev_get_drvdata(dev);
	const struct ubnt_hdd_pwrctl_platform_data *pdata = pwrctl->pdata;
	char power_state[32];
	char *p = power_state;

	for (i = 0; i < UBNT_HDD_NUM; i++) {
		if (gpio_get_value_cansleep(pdata->pwren_gpio + i))
			p += sprintf(p, "%d ", i);
	}

	p += sprintf(p, "\n");
	return snprintf(buf, PAGE_SIZE, "%s", power_state);
}

static ssize_t force_poweron(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long hdd_idx;
	struct ubnt_hdd_pwrctl *pwrctl = dev_get_drvdata(dev);
	const struct ubnt_hdd_pwrctl_platform_data *pdata = pwrctl->pdata;

	ret = kstrtoul(buf, 10, &hdd_idx);
	if (ret < 0)
		return ret;

	ret = gpio_direction_output((pdata->pwren_gpio + (hdd_idx & 0x3)), 1);
	if (ret) {
		dev_err(pwrctl->dev, "Failed to set gpio# %ld to output: %d\n",
					(pdata->pwren_gpio + (hdd_idx & 0x3)), ret);
	}

	return count;
}

static ssize_t force_poweroff(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long hdd_idx;
	struct ubnt_hdd_pwrctl *pwrctl = dev_get_drvdata(dev);
	const struct ubnt_hdd_pwrctl_platform_data *pdata = pwrctl->pdata;

	ret = kstrtoul(buf, 10, &hdd_idx);
	if (ret < 0)
		return ret;

	ret = gpio_direction_output((pdata->pwren_gpio + (hdd_idx & 0x3)), 0);
	if (ret) {
		dev_err(pwrctl->dev, "Failed to set gpio# %ld to output: %d\n",
					(pdata->pwren_gpio + (hdd_idx & 0x3)), ret);
	}

	return count;
}

static DEVICE_ATTR(hdd_present, S_IRUSR, show_hdd_present, NULL);
static DEVICE_ATTR(hdd_fault_led0, S_IRUGO | S_IWUSR, show_hdd_fault_led, store_hdd_fault_led);
static DEVICE_ATTR(hdd_fault_led1, S_IRUGO | S_IWUSR, show_hdd_fault_led, store_hdd_fault_led);
static DEVICE_ATTR(hdd_fault_led2, S_IRUGO | S_IWUSR, show_hdd_fault_led, store_hdd_fault_led);
static DEVICE_ATTR(hdd_fault_led3, S_IRUGO | S_IWUSR, show_hdd_fault_led, store_hdd_fault_led);
static DEVICE_ATTR(hdd_power, S_IRUSR, show_hdd_power, NULL);
static DEVICE_ATTR(hdd_force_poweron, S_IWUSR, NULL, force_poweron);
static DEVICE_ATTR(hdd_force_poweroff, S_IWUSR, NULL, force_poweroff);

static struct attribute *ubnt_hdd_pwrctl_attr[] = {
    &dev_attr_hdd_present.attr,
    &dev_attr_hdd_fault_led0.attr,
    &dev_attr_hdd_fault_led1.attr,
    &dev_attr_hdd_fault_led2.attr,
    &dev_attr_hdd_fault_led3.attr,
    &dev_attr_hdd_power.attr,
    &dev_attr_hdd_force_poweron.attr,
    &dev_attr_hdd_force_poweroff.attr,
    NULL,
};

static const struct attribute_group ubnt_hdd_pwrctl_attr_group = {
    .attrs = ubnt_hdd_pwrctl_attr,
};

struct ubnt_hdd_pwrctl *g_pwrctl = NULL;

int ubnt_hdd_fault_led_enabled(int fault_led_gpio)
{
	int i;
	const struct ubnt_hdd_pwrctl_platform_data *pdata;

	if (!g_pwrctl)
		return 0;

	pdata = g_pwrctl->pdata;
	for (i = 0; i < UBNT_HDD_NUM; i++) {
		if ((pdata->fault_led_gpio + i) == fault_led_gpio)
			return g_pwrctl->fault_state[i];
	}

	return 0;
}
EXPORT_SYMBOL(ubnt_hdd_fault_led_enabled);

static int ubnt_hdd_pwrctl_probe(struct platform_device *pdev)
{
	int ret, i;
	const struct ubnt_hdd_pwrctl_platform_data *pdata = pdev->dev.platform_data;
	struct ubnt_hdd_pwrctl *pwrctl;

	if (!pdata) {
		pdata = ubnt_hdd_pwrctl_parse_dt(&pdev->dev);
		if (IS_ERR(pdata)) {
			ret = PTR_ERR(pdata);
			if (ret != -EPROBE_DEFER)
				dev_err(&pdev->dev, "No platform data\n");
			return ret;
		}
	}

	pwrctl = devm_kzalloc(&pdev->dev, sizeof(*pwrctl),
					GFP_KERNEL);
	if (!pwrctl) {
		dev_err(&pdev->dev, "Failed to alloc driver structure\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&pwrctl->timeout, ubnt_hdd_pwrctl_work);

	for (i = 0; i < UBNT_HDD_NUM; i++) {
		ret = gpio_request((pdata->present_gpio + i), dev_name(&pdev->dev));
		if (ret) {
			dev_err(&pdev->dev, "Failed to request gpio %u pin: %d\n", (pdata->present_gpio + i), ret);
			goto err_free;
		}

		ret = gpio_direction_input((pdata->present_gpio + i));
		if (ret) {
			dev_err(&pdev->dev, "Failed to set gpio to input: %d\n", ret);
			goto err_gpio_present_free;
		}

		ret = gpio_request((pdata->pwren_gpio + i), dev_name(&pdev->dev));
		if (ret) {
			dev_err(&pdev->dev, "Failed to request gpio %u pin: %d\n", (pdata->pwren_gpio + i), ret);
			goto err_gpio_present_free;
		}

		ret = gpio_request((pdata->fault_led_gpio + i), dev_name(&pdev->dev));
		if (ret) {
			dev_err(&pdev->dev, "Failed to request gpio %u pin: %d\n", (pdata->fault_led_gpio + i), ret);
			goto err_gpio_pwren_free;
		}
		pwrctl->fault_state[i] = false;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &ubnt_hdd_pwrctl_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs group\n");
		goto err_gpio_fault_led_free;
	}

	pwrctl->dev = &pdev->dev;
	pwrctl->pdata = pdata;
	platform_set_drvdata(pdev, pwrctl);
	g_pwrctl = pwrctl;

	schedule_delayed_work(&pwrctl->timeout, UBNT_HDD_PWRCTL_DELAY);

	return 0;

err_gpio_fault_led_free:
	for (i = 0; i < UBNT_HDD_NUM; i++)
		gpio_free(pdata->fault_led_gpio + i);
err_gpio_pwren_free:
	for (i = 0; i < UBNT_HDD_NUM; i++)
		gpio_free(pdata->pwren_gpio + i);
err_gpio_present_free:
	for (i = 0; i < UBNT_HDD_NUM; i++)
		gpio_free(pdata->present_gpio + i);
err_free:
	return ret;
}

static int ubnt_hdd_pwrctl_remove(struct platform_device *pdev)
{
	const struct ubnt_hdd_pwrctl_platform_data *pdata = pdev->dev.platform_data;
	struct ubnt_hdd_pwrctl *pwrctl = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < UBNT_HDD_NUM; i++) {
		gpio_free((pdata->pwren_gpio + i));
		gpio_free((pdata->present_gpio + i));
		gpio_free((pdata->fault_led_gpio + i));
	}

	flush_delayed_work(&pwrctl->timeout);
	return 0;
}

static const struct of_device_id ubnt_hdd_pwrctl_match[] = {
	{ .compatible = "ubnt, hdd-pwrctl" },
	{ }
};
MODULE_DEVICE_TABLE(of, ubnt_hdd_pwrctl_match);

static struct platform_driver ubnt_hdd_pwrctl_driver = {
	.probe = ubnt_hdd_pwrctl_probe,
	.remove = ubnt_hdd_pwrctl_remove,
	.driver = {
		.name = "ubnt-hdd-pwrctl",
		.of_match_table = ubnt_hdd_pwrctl_match,
	},
};

module_platform_driver(ubnt_hdd_pwrctl_driver);
