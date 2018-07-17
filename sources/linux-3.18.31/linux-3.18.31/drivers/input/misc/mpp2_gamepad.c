/*
 * Copyright © 2016 Parrot S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/debugfs.h>
#include <linux/atomic.h>
#include <linux/uaccess.h>

#define MPP2_DRIVER_NAME "mpp2_gamepad"
#define MAX_RETRIES 3

/* ADC */
#define ADC_NUMBER_OF_BITS	8 /* for TI adc088s022 & TI ads7959 */

/* Virtual buttons codes (EV_KEY)*/
#define VIRTUAL_ROTR_R		298
#define VIRTUAL_ROTR_L		299
#define VIRTUAL_LEFT_R		300
#define VIRTUAL_LEFT_L		301
#define VIRTUAL_LEFT_U		302
#define VIRTUAL_LEFT_D		303
/* see BTN_TRIGGER_HAPPY17 to BTN_TRIGGER_HAPPY22 */
#define VIRTUAL_RIGHT_R		720
#define VIRTUAL_RIGHT_L		721
#define VIRTUAL_RIGHT_U		722
#define VIRTUAL_RIGHT_D		723
#define VIRTUAL_ROTL_R		724
#define VIRTUAL_ROTL_L		725

/* Sampling (rate, maximum samples) */
#define DEFAULT_REFRESH		50 /* ms (20 Hz)*/
#define MAX_AVERAGE_SAMPLES	10 /* */

/* Joysticks/Slider */
#define AXIS_MAX_VALUE		100
#define AXIS_MAX_THRESHOLD	75
/* deadzone (applied AFTER saturation) */
#define JOYSTICK_DEADZONE_ABS	10
#define SLIDER_DEADZONE_ABS	12

enum gamepad_adc_index {
	ADC_LLR_INDEX = 0,	/* JOYSTICK_LEFT_X  */
	ADC_LUD_INDEX,		/* JOYSTICK_LEFT_Y  */
	ADC_RLR_INDEX,		/* JOYSTICK_RIGHT_X */
	ADC_RUD_INDEX,		/* JOYSTICK_RIGHT_Y */
	ADC_ROTL_INDEX,		/* ROTARY_LEFT      */
	ADC_ROTR_INDEX,		/* ROTARY_RIGHT     */
	ADC_MAX_INDEX
};

enum chan_attribute {
	MIN_ATTR = 0,
	MAX_CHAN,
	THRESHOLD_ATTR,
	CENTER_ATTR,
	DEADZONE_ATTR,
};

struct mmp2_gamepad_state {
	struct spi_device *spi;
	struct spi_message msg;
	struct spi_transfer xfer[ADC_MAX_INDEX + 2];
	u16 spi_chan[ADC_MAX_INDEX];
	u16 spi_data[ADC_MAX_INDEX] ____cacheline_aligned;;
	atomic_t retry;

	struct input_dev *input;
	uint32_t expire;
	uint32_t max_samples;
	uint32_t nr_samples;
	uint32_t samples[ADC_MAX_INDEX][MAX_AVERAGE_SAMPLES];
	struct timer_list adc_rdr;

	unsigned long channels_map;

	int16_t min[ADC_MAX_INDEX];
	int16_t max[ADC_MAX_INDEX];
	int16_t threshold[ADC_MAX_INDEX];
	int16_t center[ADC_MAX_INDEX];
	int16_t deadzone[ADC_MAX_INDEX];
	uint8_t raw_mode;
	spinlock_t lock;

	/* debugfs interface used for to generate input events
	   When used the reading of adc is disabled
	   The correction made on deadzone, center, threshold, min and max
	   are alsa disabled */
	struct dentry *debug_dir;
	atomic_t debug_active;
	int32_t abs_code;
	int32_t abs_value;
	int32_t key_code;
	int32_t key_value;
};

static ssize_t show_raw_mode(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	struct mmp2_gamepad_state *st = dev_get_drvdata(dev);

	spin_lock(&st->lock);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", st->raw_mode);
	spin_unlock(&st->lock);
	return ret;
}

static ssize_t set_raw_mode(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int ret;
	uint8_t val;
	struct mmp2_gamepad_state *st = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 10, &val);
	if (ret)
		return ret;

	if ((val != 1) && (val != 0))
		return -EINVAL;

	spin_lock(&st->lock);
	st->raw_mode = val;
	spin_unlock(&st->lock);
	return count;
}

static DEVICE_ATTR(raw_mode, S_IRUGO | S_IWUSR,
		show_raw_mode, set_raw_mode);

static ssize_t show_channel_settings(struct device *dev,
			struct device_attribute *attr,
			char *buf,
			enum gamepad_adc_index chan,
			enum chan_attribute setting)
{
	struct mmp2_gamepad_state *st = dev_get_drvdata(dev);
	int count;

	spin_lock(&st->lock);
	switch (setting) {
	case MIN_ATTR:
		count = snprintf(buf, PAGE_SIZE, "%d\n", st->min[chan]);
		break;

	case MAX_CHAN:
		count = snprintf(buf, PAGE_SIZE, "%d\n", st->max[chan]);
		break;

	case THRESHOLD_ATTR:
		count = snprintf(buf, PAGE_SIZE, "%d\n", st->threshold[chan]);
		break;

	case CENTER_ATTR:
		count = snprintf(buf, PAGE_SIZE, "%d\n", st->center[chan]);
		break;

	case DEADZONE_ATTR:
		count = snprintf(buf, PAGE_SIZE, "%d\n", st->deadzone[chan]);
		break;

	default:
		count = -EINVAL;
	}

	spin_unlock(&st->lock);
	return count;
}

static ssize_t set_channel_settings(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count,
			enum gamepad_adc_index chan,
			enum chan_attribute setting)
{
	int ret;
	int16_t val;
	struct mmp2_gamepad_state *st = dev_get_drvdata(dev);

	ret = kstrtos16(buf, 10, &val);
	if (ret)
		return ret;

	if ((val > AXIS_MAX_VALUE)
		|| (val < -AXIS_MAX_VALUE))
		return -EINVAL;

	ret = count;

	spin_lock(&st->lock);
	switch (setting) {
	case MIN_ATTR:
		st->min[chan] = val;
		break;

	case MAX_CHAN:
		st->max[chan] = val;
		break;

	case THRESHOLD_ATTR:
		st->threshold[chan] = val;
		break;

	case CENTER_ATTR:
		st->center[chan] = val;
		break;

	case DEADZONE_ATTR:
		st->deadzone[chan] = val;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	spin_unlock(&st->lock);
	return ret;
}

#define MPP2_DEVICE_SETTINGS_SHOW(_name, _channel, _setting) \
static ssize_t show_##_name(struct device *dev, \
		struct device_attribute *attr, char *buf) \
{ \
	return show_channel_settings(dev, attr, buf, _channel, _setting); \
}

#define MPP2_DEVICE_SETTINGS_SET(_name, _channel, _setting) \
static ssize_t set_##_name(struct device *dev, \
		 struct device_attribute *attr, \
		 const char *buf, size_t count) \
{ \
	return set_channel_settings(dev, attr, buf, \
				count, _channel, _setting); \
}

#define MPP2_DEVICE_SETTINGS(_name, _channel, _setting) \
MPP2_DEVICE_SETTINGS_SHOW(_name, _channel, _setting) \
MPP2_DEVICE_SETTINGS_SET(_name, _channel, _setting) \
static DEVICE_ATTR(_name, S_IWUSR | S_IRUGO, show_##_name, set_##_name);

MPP2_DEVICE_SETTINGS(joystick_left_x_max,	ADC_LLR_INDEX, MAX_CHAN);
MPP2_DEVICE_SETTINGS(joystick_left_x_min,	ADC_LLR_INDEX, MIN_ATTR);
MPP2_DEVICE_SETTINGS(joystick_left_x_center,	ADC_LLR_INDEX, CENTER_ATTR);
MPP2_DEVICE_SETTINGS(joystick_left_x_threshold,	ADC_LLR_INDEX, THRESHOLD_ATTR);
MPP2_DEVICE_SETTINGS(joystick_left_x_deadzone,	ADC_LLR_INDEX, DEADZONE_ATTR);

MPP2_DEVICE_SETTINGS(joystick_left_y_max,	ADC_LUD_INDEX, MAX_CHAN);
MPP2_DEVICE_SETTINGS(joystick_left_y_min,	ADC_LUD_INDEX, MIN_ATTR);
MPP2_DEVICE_SETTINGS(joystick_left_y_center,	ADC_LUD_INDEX, CENTER_ATTR);
MPP2_DEVICE_SETTINGS(joystick_left_y_threshold,	ADC_LUD_INDEX, THRESHOLD_ATTR);
MPP2_DEVICE_SETTINGS(joystick_left_y_deadzone,	ADC_LUD_INDEX, DEADZONE_ATTR);

MPP2_DEVICE_SETTINGS(joystick_right_x_max,	ADC_RLR_INDEX, MAX_CHAN);
MPP2_DEVICE_SETTINGS(joystick_right_x_min,	ADC_RLR_INDEX, MIN_ATTR);
MPP2_DEVICE_SETTINGS(joystick_right_x_center,	ADC_RLR_INDEX, CENTER_ATTR);
MPP2_DEVICE_SETTINGS(joystick_right_x_threshold,
				ADC_RLR_INDEX, THRESHOLD_ATTR);
MPP2_DEVICE_SETTINGS(joystick_right_x_deadzone,	ADC_RLR_INDEX, DEADZONE_ATTR);

MPP2_DEVICE_SETTINGS(joystick_right_y_max,	ADC_RUD_INDEX, MAX_CHAN);
MPP2_DEVICE_SETTINGS(joystick_right_y_min,	ADC_RUD_INDEX, MIN_ATTR);
MPP2_DEVICE_SETTINGS(joystick_right_y_center,	ADC_RUD_INDEX, CENTER_ATTR);
MPP2_DEVICE_SETTINGS(joystick_right_y_threshold,
				ADC_RUD_INDEX, THRESHOLD_ATTR);
MPP2_DEVICE_SETTINGS(joystick_right_y_deadzone,	ADC_RUD_INDEX, DEADZONE_ATTR);

MPP2_DEVICE_SETTINGS(rotary_left_max,		ADC_ROTL_INDEX, MAX_CHAN);
MPP2_DEVICE_SETTINGS(rotary_left_min,		ADC_ROTL_INDEX, MIN_ATTR);
MPP2_DEVICE_SETTINGS(rotary_left_center,	ADC_ROTL_INDEX, CENTER_ATTR);
MPP2_DEVICE_SETTINGS(rotary_left_threshold,	ADC_ROTL_INDEX, THRESHOLD_ATTR);
MPP2_DEVICE_SETTINGS(rotary_left_deadzone,	ADC_ROTL_INDEX, DEADZONE_ATTR);

MPP2_DEVICE_SETTINGS(rotary_right_max,		ADC_ROTR_INDEX, MAX_CHAN);
MPP2_DEVICE_SETTINGS(rotary_right_min,		ADC_ROTR_INDEX, MIN_ATTR);
MPP2_DEVICE_SETTINGS(rotary_right_center,	ADC_ROTR_INDEX, CENTER_ATTR);
MPP2_DEVICE_SETTINGS(rotary_right_threshold,	ADC_ROTR_INDEX, THRESHOLD_ATTR);
MPP2_DEVICE_SETTINGS(rotary_right_deadzone,	ADC_ROTR_INDEX, DEADZONE_ATTR);

struct mpp2_gamepad_chan_attr {
	struct attribute const **attr;
	size_t nr_attr;
};

#define MPP2_GAMEPAD_ATTR(_index, _attr)	\
	[_index] = {				\
		.attr = _attr,			\
		.nr_attr = ARRAY_SIZE(_attr), }

static const struct attribute *joystick_left_x_attr[] = {
	&dev_attr_joystick_left_x_max.attr,
	&dev_attr_joystick_left_x_min.attr,
	&dev_attr_joystick_left_x_center.attr,
	&dev_attr_joystick_left_x_threshold.attr,
	&dev_attr_joystick_left_x_deadzone.attr,
};

static const struct attribute *joystick_left_y_attr[] = {
	&dev_attr_joystick_left_y_max.attr,
	&dev_attr_joystick_left_y_min.attr,
	&dev_attr_joystick_left_y_center.attr,
	&dev_attr_joystick_left_y_threshold.attr,
	&dev_attr_joystick_left_y_deadzone.attr,
};

static const struct attribute *joystick_right_x_attr[] = {
	&dev_attr_joystick_right_x_max.attr,
	&dev_attr_joystick_right_x_min.attr,
	&dev_attr_joystick_right_x_center.attr,
	&dev_attr_joystick_right_x_threshold.attr,
	&dev_attr_joystick_right_x_deadzone.attr,
};

static const struct attribute *joystick_right_y_attr[] = {
	&dev_attr_joystick_right_y_max.attr,
	&dev_attr_joystick_right_y_min.attr,
	&dev_attr_joystick_right_y_center.attr,
	&dev_attr_joystick_right_y_threshold.attr,
	&dev_attr_joystick_right_y_deadzone.attr,
};

static const struct attribute *rotary_left_attr[] = {
	&dev_attr_rotary_left_max.attr,
	&dev_attr_rotary_left_min.attr,
	&dev_attr_rotary_left_center.attr,
	&dev_attr_rotary_left_threshold.attr,
	&dev_attr_rotary_left_deadzone.attr,
};

static const struct attribute *rotary_right_attr[] = {
	&dev_attr_rotary_right_max.attr,
	&dev_attr_rotary_right_min.attr,
	&dev_attr_rotary_right_center.attr,
	&dev_attr_rotary_right_threshold.attr,
	&dev_attr_rotary_right_deadzone.attr,
};

static const struct mpp2_gamepad_chan_attr
	mpp2_gamepad_attrib[ADC_MAX_INDEX] = {
	MPP2_GAMEPAD_ATTR(ADC_LLR_INDEX, joystick_left_x_attr),
	MPP2_GAMEPAD_ATTR(ADC_LUD_INDEX, joystick_left_y_attr),
	MPP2_GAMEPAD_ATTR(ADC_RLR_INDEX, joystick_right_x_attr),
	MPP2_GAMEPAD_ATTR(ADC_RUD_INDEX, joystick_right_y_attr),
	MPP2_GAMEPAD_ATTR(ADC_ROTL_INDEX, rotary_left_attr),
	MPP2_GAMEPAD_ATTR(ADC_ROTR_INDEX, rotary_right_attr),
};

static ssize_t read_s32(struct file *file, char __user *user_buf,
			      size_t count, loff_t *ppos)
{
	char buf[12];
	size_t buf_size;
	int32_t *val = file->private_data;

	/* TODO need lock */
	buf_size = scnprintf(buf, ARRAY_SIZE(buf), "%d\n", *val);
	buf[buf_size++] = 0x0;

	pr_debug("%s: val %d (%p)\n", __func__, *val, val);
	return simple_read_from_buffer(user_buf, count, ppos, buf, buf_size);
}

/* Debugfs */
static ssize_t write_s32(struct file *file, const char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	char buf[12];
	size_t buf_size;
	int32_t *val = file->private_data;

	buf_size = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	/* TODO need lock */
	if (sscanf(buf, "%d", val) != 1)
		return -EINVAL;

	pr_debug("%s: val %d (%p)\n", __func__, *val, val);
	return buf_size;
}

static const struct file_operations fops_s32 = {
	.read =		read_s32,
	.write =	write_s32,
	.open =		simple_open,
	.llseek =	noop_llseek,
};

int8_t make_gamepad_value(struct mmp2_gamepad_state *st, int8_t in_value,
	uint8_t index)
{
	int8_t raw_value = in_value, deadzone = 0;
	int32_t min, max, center, val;
	unsigned long flags;

	spin_lock_irqsave(&st->lock, flags);
	if ((index >= ADC_MAX_INDEX) || st->raw_mode) {
		spin_unlock_irqrestore(&st->lock, flags);
		return raw_value;
	}

	min = st->min[index];
	max = st->max[index];
	center = st->center[index];
	deadzone = st->deadzone[index];

	/* apply center offset */
	raw_value -= center;

	/* update potential new min/max */
	if (raw_value > max) {
		st->max[index] = raw_value;
		max = raw_value;
	} else if (raw_value <= min) {
		st->min[index] = raw_value;
		min = raw_value;
	}
	spin_unlock_irqrestore(&st->lock, flags);

	if (deadzone > 0 &&
	    raw_value > -deadzone &&
	    raw_value <  deadzone) {
		raw_value = 0;
	} else {
		raw_value = raw_value > 0 ? raw_value - deadzone :
				raw_value + deadzone;
	}

	min += deadzone;
	max -= deadzone;

	/* else, scale according to the sign */
	/* input range : [square_min; square_max] output range [-100; 100] */
	if (raw_value == 0) {
		/* easy case, do nothing ! */
	} else if (raw_value > 0) {
		if (raw_value > max) {
			raw_value = AXIS_MAX_VALUE;
		} else {
			/* val = (100 * val) / max */
			val = (AXIS_MAX_VALUE * raw_value) / max;
			raw_value = (int8_t)val;
		}
	} else {
		if (raw_value < min) {
			raw_value = -AXIS_MAX_VALUE;
		} else {
			/* val = (-100 * val) / min */
			val = (-AXIS_MAX_VALUE * raw_value) / min;
			raw_value = (int8_t)val;
		}
	}

	if (raw_value > AXIS_MAX_VALUE)
		raw_value = AXIS_MAX_VALUE;
	else if (raw_value < -AXIS_MAX_VALUE)
		raw_value = -AXIS_MAX_VALUE;

	return raw_value;
}

static void mpp2_report_input_event(struct mmp2_gamepad_state *st)
{
	uint8_t i, j;
	int32_t val;
	static uint32_t const abs_codes[ADC_MAX_INDEX] = {
		ABS_X,
		ABS_Y,
		ABS_Z,
		ABS_RX,
		ABS_RY,
		ABS_RZ
	};
	static uint32_t const keys_codes[2*ADC_MAX_INDEX] = {
		VIRTUAL_LEFT_L,
		VIRTUAL_LEFT_R,
		VIRTUAL_LEFT_D,
		VIRTUAL_LEFT_U,
		VIRTUAL_RIGHT_L,
		VIRTUAL_RIGHT_R,
		VIRTUAL_RIGHT_D,
		VIRTUAL_RIGHT_U,
		VIRTUAL_ROTL_L,
		VIRTUAL_ROTL_R,
		VIRTUAL_ROTR_L,
		VIRTUAL_ROTR_R
	};
	static uint32_t const inverted[ADC_MAX_INDEX] = {
		1,
		1,
		1,
		1,
		0,
		0
	};

	for_each_set_bit(i, &st->channels_map, ADC_MAX_INDEX) {
		val = 0;
		/* mean */
		for (j = 0; j < st->max_samples; j++)
			val += st->samples[i][j];
		val = val / st->max_samples;

		if (inverted[i])
			val = 127 - val;
		else
			val -= 128;

		/* manage dead zone */
		val = make_gamepad_value(st, (int8_t) val, i);

		input_report_abs(st->input, abs_codes[i], val);

		if (st->threshold[i] < AXIS_MAX_VALUE) {
			j = i << 1;

			if (val > st->threshold[i]) {
				input_report_key(st->input, keys_codes[j], 0);
				input_report_key(st->input,
						keys_codes[j + 1], 1);
			} else if (val < -(st->threshold[i])) {
				input_report_key(st->input, keys_codes[j], 1);
				input_report_key(st->input,
						keys_codes[j + 1], 0);
			} else {
				input_report_key(st->input, keys_codes[j], 0);
				input_report_key(st->input,
						keys_codes[j + 1], 0);
			}
		}

		/* Synchronise event */
		input_sync(st->input);
	}

	st->nr_samples = 0;
}

static void ads7959_complete(void *context)
{
	uint16_t i = 0, j = 0;
	struct mmp2_gamepad_state *st = context;

	if ((atomic_read(&st->retry) > MAX_RETRIES) &&
		(st->msg.status != 0)) {
		dev_err(&st->spi->dev, "SPI communication failed\n");
		return;
	} else if (st->msg.status != 0) {
		/* spi failure, there was less than MAX_RETRIES */
		dev_dbg(&st->spi->dev, "SPI failure (retry %d)\n",
			atomic_read(&st->retry));
		return;
	} else
		atomic_set(&st->retry, 0);

	for_each_set_bit(i, &st->channels_map, ADC_MAX_INDEX) {
		st->samples[i][st->nr_samples] = st->spi_data[j] >> 4;
		st->samples[i][st->nr_samples] &= 0xff;
		j++;
	}

	st->nr_samples++;

	if (st->nr_samples >= st->max_samples)
		mpp2_report_input_event(st);
}

/* Manual mode selected, 2.5v range */
#define ADS7959_CHAN(_channel)	((_channel << 7) | (1 << 11) | (1 << 12))
static int ads7959_init(struct mmp2_gamepad_state *st,
			uint8_t *channels, size_t size)
{
	int ret = -EINVAL;
	uint16_t i, j = 0, tmp;

	spi_message_init(&st->msg);

	for_each_set_bit(i, &st->channels_map, ADC_MAX_INDEX) {
		st->xfer[j].tx_buf = &st->spi_chan[j];
		st->xfer[j].len = 2;
		st->xfer[j].cs_change = 1;

		st->xfer[j + 2].rx_buf = &st->spi_data[j];
		st->xfer[j + 2].len = 2;

		tmp = channels[i];
		st->spi_chan[j] = ADS7959_CHAN(tmp);

		spi_message_add_tail(&st->xfer[j], &st->msg);
		j++;
	}

	if (!j)
		goto ads7959_failure;

	st->xfer[j].tx_buf = &st->spi_chan[j - 1];
	st->xfer[j].len = 2;
	st->xfer[j].cs_change = 1;

	spi_message_add_tail(&st->xfer[j], &st->msg);
	spi_message_add_tail(&st->xfer[j + 1], &st->msg);

	st->msg.context = st;
	st->msg.complete = ads7959_complete;

	ret = 0;

ads7959_failure:
	return ret;
}

static void adc088s022_complete(void *context)
{
	struct mmp2_gamepad_state *st = context;
	uint16_t i, j = 0;

	if ((atomic_read(&st->retry) > MAX_RETRIES) &&
		(st->msg.status != 0)) {
		dev_err(&st->spi->dev, "SPI communication failed\n");
		return;
	} else if (st->msg.status != 0) {
		/* spi failure, there was less than MAX_RETRIES */
		dev_dbg(&st->spi->dev, "SPI failure (retry %d)\n",
			atomic_read(&st->retry));
		return;
	} else
		atomic_set(&st->retry, 0);

	for_each_set_bit(i, &st->channels_map, ADC_MAX_INDEX) {
		st->samples[i][st->nr_samples] = st->spi_data[j] >> 4;
		j++;
	}

	st->nr_samples++;

	if (st->nr_samples >= st->max_samples)
		mpp2_report_input_event(st);
}

static int adc088s022_init(struct mmp2_gamepad_state *st,
			uint8_t *channels, size_t size)
{
	int ret = -EINVAL;
	uint16_t i, j = 0;

	spi_message_init(&st->msg);

	for_each_set_bit(i, &st->channels_map, ADC_MAX_INDEX) {
		st->spi_chan[j] = (u16)((channels[i]) << 11);
		st->xfer[j].tx_buf = &st->spi_chan[j];
		/* Result is on next message */
		if(j)
			st->xfer[j].rx_buf = &st->spi_data[j - 1];
		st->xfer[j].len = 2;
		st->xfer[j].cs_change = 0;
		spi_message_add_tail(&st->xfer[j], &st->msg);
		j++;
	}

	if (!j)
		goto adc088s022_failure;

	/* Get last data */
	st->xfer[j].tx_buf = NULL;
	st->xfer[j].rx_buf = &st->spi_data[j - 1];
	st->xfer[j].len = 2;
	st->xfer[j].cs_change = 0;
	spi_message_add_tail(&st->xfer[j], &st->msg);

	st->msg.context = st;
	st->msg.complete = adc088s022_complete;

	ret = 0;
adc088s022_failure:
	return ret;
}

static void mpp2_adc_read_routine(unsigned long data)
{
	struct mmp2_gamepad_state *st = (struct mmp2_gamepad_state *) data;
	struct spi_device *spi = st->spi;

	/* Debug */
	if (unlikely(atomic_read(&st->debug_active))) {
		dev_dbg(&spi->dev, "Debug mode activated\n");

		if ((st->abs_code >= ABS_X)
			&& (st->abs_code <= ABS_RY)
			&& (st->abs_value >= -AXIS_MAX_VALUE)
			&& (st->abs_value <= AXIS_MAX_VALUE)) {

			input_report_abs(st->input,
				st->abs_code, st->abs_value);
			/* Synchronise event */
			input_sync(st->input);

			st->abs_value = -1;
			st->abs_code = -1;
		}

		if ((((st->key_code >= VIRTUAL_LEFT_R)
			&& (st->key_code <= VIRTUAL_LEFT_D))
			|| ((st->key_code >= VIRTUAL_RIGHT_R)
			&& (st->key_code <= VIRTUAL_ROTL_L)))
			&& ((st->key_value == 0) || (st->key_value == 1))) {

			input_report_key(st->input,
						st->key_code, st->key_value);
			/* Synchronise event */
			input_sync(st->input);

			st->key_code = -1;
			st->key_value = -1;
		}

		goto adc_read_exit;
	}

	if (atomic_inc_return(&st->retry) > MAX_RETRIES) {
		dev_err(&spi->dev, "SPI communication failed\n");
		return;
	}

	if (spi_async(spi, &st->msg))
		dev_err(&spi->dev,
			"SPI: busy line or invalid parameters\n");

adc_read_exit:
	mod_timer(&st->adc_rdr, jiffies + st->expire);
}

int mpp2_input_open(struct input_dev *input)
{
	struct mmp2_gamepad_state *st = input_get_drvdata(input);
	mod_timer(&st->adc_rdr, jiffies + st->expire);
	return 0;
}

void mpp2_input_close(struct input_dev *input)
{
	struct mmp2_gamepad_state *st = input_get_drvdata(input);
	del_timer_sync(&st->adc_rdr);
}

struct adc_chip_property {
	int (*adc_init)(struct mmp2_gamepad_state *st,
			uint8_t *channels, size_t size);
	uint32_t max_speed_hz;
	char *name;
	uint8_t sz_name;
	uint8_t mode;
	uint8_t bits_per_word;
	uint8_t max_adc_channels;
};

#define ADC_CHIP_PROP(_name, _mode, _speed, \
			_bit_per_word, _max_adc_channels, \
			_adc_init) \
	{	.name = _name, \
		.sz_name = sizeof(_name)-1, \
		.mode = _mode, \
		.max_speed_hz = _speed, \
		.bits_per_word = _bit_per_word, \
		.max_adc_channels = _max_adc_channels, \
		.adc_init = _adc_init, }

struct gamepad_chan_names {
	char *name;
	uint8_t sz_name;
};

#define GAMEPAD_ADC_CHAN(_name) \
	{	.name = _name, \
		.sz_name = sizeof(_name)-1, }

int mmp2_gamepad_probe(struct spi_device *spi)
{
	struct mmp2_gamepad_state *st;
	struct input_dev *input;
	int len, i, j, nr_params, ret = -ENOMEM;
	const __be32 *prop;
	struct device_node *np = spi->dev.of_node;
	const char *property;
	const char *adc_chip = NULL;
	const __be32 *list;
	uint32_t *adc_channels = NULL;
	uint32_t max_adc_channels;
	uint8_t adc_chan_nr[ADC_MAX_INDEX];

	struct gamepad_chan_names gchan_names[] = {
		GAMEPAD_ADC_CHAN("joystick-left-x"),
		GAMEPAD_ADC_CHAN("joystick-left-y"),
		GAMEPAD_ADC_CHAN("joystick-right-x"),
		GAMEPAD_ADC_CHAN("joystick-right-y"),
		GAMEPAD_ADC_CHAN("rotary-left"),
		GAMEPAD_ADC_CHAN("rotary-right")};

	struct adc_chip_property adc_chip_props[] = {
		ADC_CHIP_PROP("adc088s022", SPI_MODE_3, 3200000,
			16, 8, adc088s022_init),
		ADC_CHIP_PROP("ads7959", SPI_MODE_0, 3200000,
			16, 8, ads7959_init),
	};

	const struct attribute **attr = NULL;
	int (*adc_init)(struct mmp2_gamepad_state *st,
			uint8_t *channels, size_t size);

	st = devm_kzalloc(&spi->dev, sizeof(*st), GFP_KERNEL);
	input = devm_input_allocate_device(&spi->dev);
	if (!st || !input) {
		dev_err(&spi->dev, "Unable to allocate memory\n");
		goto exit_failure;
	}

	st->channels_map = 0;
	for (i = 0; i < ADC_MAX_INDEX; i++) {
		st->max[i] = AXIS_MAX_VALUE;
		st->min[i] = -AXIS_MAX_VALUE;
		st->threshold[i] = AXIS_MAX_THRESHOLD;
		st->deadzone[i] = (i >= ADC_ROTL_INDEX) ?
			JOYSTICK_DEADZONE_ABS:SLIDER_DEADZONE_ABS;
	}

	st->spi = spi;
	st->input = input;

	if (!np) {
		dev_err(&spi->dev, "Missing property in device tree\n");
		ret = -EINVAL;
		goto exit_failure;
	}

	ret = of_property_read_string(np, "adc-chip", &adc_chip);
	if (ret < 0) {
		dev_err(&spi->dev, "missing 'adc-chip'\n");
		goto exit_failure;
	}

	for (i = 0; i < ARRAY_SIZE(adc_chip_props); i++) {

		if (!strncmp(adc_chip_props[i].name,
			adc_chip, adc_chip_props[i].sz_name)) {

			adc_init = adc_chip_props[i].adc_init;
			nr_params = max_adc_channels =
					adc_chip_props[i].max_adc_channels;

			spi->mode = adc_chip_props[i].mode;
			spi->max_speed_hz = adc_chip_props[i].max_speed_hz;
			spi->bits_per_word = adc_chip_props[i].bits_per_word;
			goto adc_chip_found;
		}
	}

	/* No adc chip has been founda meaning
	   if (i >= ARRAY_SIZE(adc_chip_props)) */
	dev_err(&spi->dev, "Invalid parameter for 'adc-chip'\n");
	ret = -EINVAL;
	goto exit_failure;

adc_chip_found:
	dev_dbg(&spi->dev, "ADC chip : %s\n", adc_chip);

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "Unable to setup spi\n");
		goto exit_failure;
	}

	prop = of_get_property(np,
			"max-samples", &len);
	if (!prop || (len < sizeof(*prop))) {
		dev_warn(&spi->dev,
			"%s has no 'max-samples' property\n",
			np->full_name);
		st->max_samples = 1;
	} else {
		st->max_samples = be32_to_cpup(prop);

		if ((!st->max_samples) ||
		(st->max_samples >= MAX_AVERAGE_SAMPLES)) {
			st->max_samples = MAX_AVERAGE_SAMPLES;
			dev_warn(&spi->dev,
			"'max-samples' set to default value\n");
		}
	}

	st->expire = msecs_to_jiffies(DEFAULT_REFRESH / st->max_samples);

	len = of_property_count_strings(np, "adc-channel-names");
	if (len < 0) {
		dev_err(&spi->dev, "missing 'adc-channel-names'\n");
		ret = -EINVAL;
		goto exit_failure;
	}

	if (len > nr_params)
		dev_warn(&spi->dev,
			"Too many parameters for 'adc-channel-names'\n");
	else
		nr_params = len;

	list = of_get_property(np, "adc-channels", &len);
	if (!list) {
		dev_err(&spi->dev, "missing 'adc-channels'\n");
		ret = -EINVAL;
		goto exit_failure;
	}
	len /= sizeof(*list);

	if (nr_params != len) {
		dev_warn(&spi->dev,
			"Incoherent number of parameters"
			" for 'adc-channel-names' (%d)"
			" and 'adc-channels' (%d)\n",
			nr_params, len);
		nr_params = min_t(int, nr_params, len);
	}

	adc_channels = kzalloc(nr_params * sizeof(*adc_channels),
				GFP_KERNEL);
	if (!adc_channels) {
		dev_err(&spi->dev, "Unable to allocate resource\n");
		ret = -ENOMEM;
		goto exit_failure;
	}

	ret = of_property_read_u32_array(np, "adc-channels",
			adc_channels, nr_params);
	if (ret && (ret != -EINVAL)) {
		dev_err(&spi->dev, "Unable to get 'adc-channels' data\n");
		goto adc_channel_failure;
	}

	for (i = 0; i < nr_params; i++) {

		ret = of_property_read_string_index(np,
				"adc-channel-names", i, &property);
		if (ret) {
			dev_dbg(&spi->dev,
				"Unable to get 'adc-channel-names'\n");
			goto adc_channel_failure;
		}

		if (adc_channels[i] > (max_adc_channels - 1)) {
			dev_dbg(&spi->dev,
				"Bad channel number(%d) for"
				" 'adc-channel-names' '%s'\n",
				adc_channels[i],
				property);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(gchan_names) ; j++) {
			if (!strncmp(gchan_names[j].name,
				property,
				gchan_names[j].sz_name)) {

				if (test_and_set_bit(j, &st->channels_map)) {
					dev_warn(&spi->dev,
					"Channel '%s' already set\n",
					gchan_names[j].name);
					continue;
				}

				adc_chan_nr[j] = (uint8_t) adc_channels[i];

				dev_dbg(&spi->dev, "%s : in%d\n",
					property, adc_channels[i]);
				break;
			}
		}

		if (j >= ARRAY_SIZE(gchan_names))
			dev_err(&spi->dev,
				"Unknow 'adc-channel-names': '%s'\n",
				property);
	}

	dev_dbg(&spi->dev, "Channel maps: 0x%lx\n", st->channels_map);

	i = find_first_bit(&st->channels_map, sizeof(st->channels_map));

	if (i >= sizeof(st->channels_map)) {
		dev_err(&spi->dev, "Bad parameters for 'adc-channel-names'\n");
		ret = -EINVAL;
		goto adc_channel_failure;
	}

	ret = adc_init(st, adc_chan_nr, ARRAY_SIZE(adc_chan_nr));
	if (ret < 0) {
		dev_err(&spi->dev, "Unable to init adc\n");
		goto exit_failure;
	}

	input->name = MPP2_DRIVER_NAME;
	input->phys = "parrot/mpp2";
	input->id.bustype = BUS_SPI;
	input->dev.parent = &spi->dev;

	input->open = mpp2_input_open;
	input->close = mpp2_input_close;

	input->evbit[0] = BIT(EV_ABS) | BIT(EV_KEY);
	__set_bit(VIRTUAL_LEFT_R, input->keybit);
	__set_bit(VIRTUAL_LEFT_L, input->keybit);
	__set_bit(VIRTUAL_LEFT_U, input->keybit);
	__set_bit(VIRTUAL_LEFT_D, input->keybit);
	__set_bit(VIRTUAL_RIGHT_R, input->keybit);
	__set_bit(VIRTUAL_RIGHT_L, input->keybit);
	__set_bit(VIRTUAL_RIGHT_U, input->keybit);
	__set_bit(VIRTUAL_RIGHT_D, input->keybit);
	__set_bit(VIRTUAL_ROTL_R, input->keybit);
	__set_bit(VIRTUAL_ROTL_L, input->keybit);
	__set_bit(VIRTUAL_ROTR_R, input->keybit);
	__set_bit(VIRTUAL_ROTR_L, input->keybit);

	__set_bit(ABS_X, input->absbit);
	input_set_abs_params(input, ABS_X, -100, 100, 0, 0);

	__set_bit(ABS_Y, input->absbit);
	input_set_abs_params(input, ABS_Y, -100, 100, 0, 0);

	__set_bit(ABS_Z, input->absbit);
	input_set_abs_params(input, ABS_Z, -100, 100, 0, 0);

	__set_bit(ABS_RX, input->absbit);
	input_set_abs_params(input, ABS_RX, -100, 100, 0, 0);

	__set_bit(ABS_RY, input->absbit);
	input_set_abs_params(input, ABS_RY, -100, 100, 0, 0);

	__set_bit(ABS_RZ, input->absbit);
	input_set_abs_params(input, ABS_RZ, -100, 100, 0, 0);

	setup_timer(&st->adc_rdr, mpp2_adc_read_routine, (unsigned long) st);

	ret = sysfs_create_file(&spi->dev.kobj,
			&dev_attr_raw_mode.attr);
	if (ret < 0) {
		i = 0;
		dev_err(&spi->dev, "Unable to create sysfs attribute\n");
		goto adc_channel_failure;
	}

	for_each_set_bit(i, &st->channels_map, ADC_MAX_INDEX) {
		dev_dbg(&spi->dev, "bit %d\n", i);

		attr = mpp2_gamepad_attrib[i].attr;

		for (j = 0; j < mpp2_gamepad_attrib[i].nr_attr; j++) {

			ret = sysfs_create_file(&spi->dev.kobj, attr[j]);

			if (ret < 0) {
				dev_err(&spi->dev,
					"Unable to create sysfs"
					" attribute\n");
				goto sysfs_failure;
			}
		}
	}

	spin_lock_init(&st->lock);
	spi_set_drvdata(spi, st);
	input_set_drvdata(input, st);

	dev_info(&spi->dev, "Registering mpp gamepad\n");
	ret = input_register_device(input);
	if (ret < 0) {
		dev_err(&spi->dev, "Registering input device failed\n");
		goto sysfs_failure;
	}

	/* debugfs interface to test events and keys */
	st->debug_dir = debugfs_create_dir(MPP2_DRIVER_NAME, NULL);
	if (st->debug_dir) {

		st->abs_value = -1;
		st->abs_code = -1;
		st->key_code = -1;
		st->key_value = -1;

		debugfs_create_atomic_t("debug_active", S_IWUSR | S_IRUGO,
				 st->debug_dir, &st->debug_active);

		debugfs_create_file("abs_value", S_IWUSR | S_IRUGO,
				   st->debug_dir, &st->abs_value,
				   &fops_s32);

		debugfs_create_file("abs_code", S_IWUSR | S_IRUGO,
				   st->debug_dir, &st->abs_code,
				   &fops_s32);

		debugfs_create_file("key_code", S_IWUSR | S_IRUGO,
				   st->debug_dir, &st->key_code,
				   &fops_s32);

		debugfs_create_file("key_value", S_IWUSR | S_IRUGO,
				   st->debug_dir, &st->key_value,
				   &fops_s32);
	}
	return ret;

sysfs_failure:
	if (!j)
		i--;

	while (i > 0) {

		if (test_bit(i, &st->channels_map)) {
			j = mpp2_gamepad_attrib[i].nr_attr;
			attr = mpp2_gamepad_attrib[i].attr;

			while (j > 0) {
				sysfs_remove_file(&spi->dev.kobj, attr[j - 1]);
				j--;
			}
		}
		i--;
	}

	sysfs_remove_file(&spi->dev.kobj,
		&dev_attr_raw_mode.attr);
adc_channel_failure:
	kfree(adc_channels);
exit_failure:
	return ret;
}

int mmp2_gamepad_remove(struct spi_device *spi)
{
	struct mmp2_gamepad_state *st = spi_get_drvdata(spi);
	const struct attribute **attr = NULL;
	int i, j;

	del_timer_sync(&st->adc_rdr);
	debugfs_remove_recursive(st->debug_dir);

	for_each_set_bit(i, &st->channels_map, ADC_MAX_INDEX) {
		attr = mpp2_gamepad_attrib[i].attr;

		for (j = 0; j < mpp2_gamepad_attrib[i].nr_attr; j++)
			sysfs_remove_file(&spi->dev.kobj, attr[j]);
	}

	sysfs_remove_file(&spi->dev.kobj, &dev_attr_raw_mode.attr);
	spi_set_drvdata(spi, NULL);
	return 0;
}

static const struct spi_device_id mmp2_gamepad_id[] = {
	{MPP2_DRIVER_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(spi, mmp2_gamepad_id);

static const struct of_device_id mmp2_gamepad_of_match[] = {
	{ .compatible = "parrot, "MPP2_DRIVER_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, mmp2_gamepad_of_match);


struct spi_driver mmp2_gamepad_driver = {
	.driver = {
		.name = MPP2_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(mmp2_gamepad_of_match),
	},

	.id_table = mmp2_gamepad_id,
	.probe = mmp2_gamepad_probe,
	.remove = mmp2_gamepad_remove,
};

module_spi_driver(mmp2_gamepad_driver);

MODULE_AUTHOR("Parrot");
MODULE_DESCRIPTION("MPP2 gamepad");
MODULE_LICENSE("GPL v2");
