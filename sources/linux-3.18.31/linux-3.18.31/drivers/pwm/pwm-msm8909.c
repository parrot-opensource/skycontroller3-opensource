/**
 * Copyright (c) 2016 Parrot SA
 *
 * @file pwm-msm8909.c
 * @brief Qualcomm msm8909 PWM driver
 * This file contains code to configure the msm8909 PWM
 * @author Remi Pommarel <remi.pommarel@parrot.com>
 * @version 0.1
 * @date 2016-05-30
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/clk.h>
#include <linux/clk/msm-clk.h>
#include <linux/platform_device.h>

#define PWMNR 1

struct msm8909_pwm {
	struct pwm_chip chip;
	struct clk *clk;
	struct clk *div;
	int duty_ns;
	int period_ns;
	bool enable;
};

static inline struct msm8909_pwm *to_msm8909_pwm(struct pwm_chip *chip)
{
	return container_of(chip, struct msm8909_pwm, chip);
}

static int msm8909_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	return 0;
}

static void msm8909_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
}

static int msm8909_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
		int duty_ns, int period_ns)
{
	struct msm8909_pwm *pc = to_msm8909_pwm(chip);
	int ret = 0;

	if(period_ns == 0)
		goto out;

	pr_debug("msm8909_pwm: Config %d duty %d period\n", duty_ns,
			period_ns);

	if(duty_ns == 0 && pc->enable) {
		clk_disable(pc->clk);
		pc->enable = 0;
	}

	if(duty_ns == 0)
		goto out;

	ret = clk_set_duty_cycle(pc->div, duty_ns, period_ns);
	if(ret != 0) {
		pr_err("ms8909_pwm: cannot set duty cycle\n");
		goto out;
	}

	if(!pc->enable) {
		clk_enable(pc->clk);
		pc->enable = 1;
	}
out:
	return ret;
}

static int msm8909_pwm_set_polarity(struct pwm_chip *chip,
		struct pwm_device *pwm, enum pwm_polarity polarity)
{
	return 0;
}

static int msm8909_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	return 0;
}

static void msm8909_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
}

static const struct pwm_ops msm8909_pwm_ops = {
	.request = msm8909_pwm_request,
	.free = msm8909_pwm_free,
	.config = msm8909_pwm_config,
	.enable = msm8909_pwm_enable,
	.disable = msm8909_pwm_disable,
	.set_polarity = msm8909_pwm_set_polarity,
	.owner = THIS_MODULE,
};

static int msm8909_pwm_probe(struct platform_device *dev)
{
	struct msm8909_pwm *pc;
	struct clk *clk;
	u32 rate;
	int ret;

	pr_debug("msm8909_pwm_probe() called\n");

	pc = devm_kzalloc(&dev->dev, sizeof(*pc), GFP_KERNEL);
	if(!pc)
		return -ENOMEM;

	clk = devm_clk_get(&dev->dev, "branch_clk");
	if(IS_ERR(clk)) {
		dev_err(&dev->dev, "clock not found: %ld\n", PTR_ERR(clk));
		ret = PTR_ERR(clk);
		goto error;
	}

	ret = of_property_read_u32(dev->dev.of_node, "qcom,pwm-clock-rate", &rate);
	if(ret != 0) {
		dev_err(&dev->dev, "cannot get freq from dt\n");
		goto error;
	}

	ret = clk_set_rate(clk, rate);
	if(ret != 0) {
		dev_err(&dev->dev, "cannot set freq\n");
		goto error;
	}

	ret = clk_prepare(clk);
	if(ret) {
		dev_err(&dev->dev, "cannot enable clock\n");
		goto error;
	}

	pc->clk = clk;

	clk = devm_clk_get(&dev->dev, "div_clk");
	if(IS_ERR(clk)) {
		dev_err(&dev->dev, "PWM clock not found: %ld\n", PTR_ERR(clk));
		ret = PTR_ERR(clk);
		goto error;
	}
	pc->div = clk;

	pc->chip.dev = &dev->dev;
	pc->chip.ops = &msm8909_pwm_ops;
	pc->chip.base = -1;
	pc->chip.npwm = PWMNR;

	platform_set_drvdata(dev, pc);

	ret = pwmchip_add(&pc->chip);
	if(ret < 0) {
		dev_err(&dev->dev, "pwmchip_add() failure\n");
		goto error;
	}

	return 0;

error:
	if(pc->clk)
		clk_unprepare(pc->clk);

	return ret;
}

static int msm8909_pwm_remove(struct platform_device *dev)
{
	struct msm8909_pwm *pc = platform_get_drvdata(dev);

	pr_debug("msm8909_pwm_remove() called\n");

	clk_unprepare(pc->clk);

	return pwmchip_remove(&pc->chip);
}

static const struct of_device_id msm8909_pwm_of_match[] = {
	{
		.compatible = "qcom,msm8909-pwm",
	},
	{
		/* sentinel */
	},
};
MODULE_DEVICE_TABLE(of, msm8909_pwm_of_match);

static struct platform_driver msm8909_pwm_driver = {
	.driver = {
		.name = "msm8909-pwm",
		.of_match_table = msm8909_pwm_of_match,
	},
	.probe = msm8909_pwm_probe,
	.remove = msm8909_pwm_remove,
};
module_platform_driver(msm8909_pwm_driver);

MODULE_AUTHOR("Remi Pommarel <remi.pommarel@parrot.com>");
MODULE_DESCRIPTION("MSM 8909 PWM driver");
MODULE_LICENSE("GPL");
