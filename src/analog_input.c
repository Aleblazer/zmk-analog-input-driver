/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_analog_input

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zmk/keymap.h>
#include <stdlib.h> //for abs()
#include <zephyr/sys/util.h> // for CLAMP

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ANALOG_INPUT, CONFIG_ANALOG_INPUT_LOG_LEVEL);

#include <zmk/drivers/analog_input.h>

/* Plain magnitude deadband: report when the value differs from the last reported
 * one by at least the threshold; threshold 0 = report on any change. */
static inline bool analog_input_changed(int32_t dv, int32_t pv, uint16_t threshold) {
    int32_t d = dv - pv;
    if (d < 0) d = -d;
    return threshold ? (d >= (int32_t)threshold) : (dv != pv);
}

/* Whether channel <c> should emit, given current value <dv> and last reported
 * <pv>. In settle mode (threshold + settle-samples) the idle gating already
 * forces dv==pv when quiet, so reporting is full-resolution "any change";
 * otherwise fall back to the plain magnitude deadband. */
static inline bool analog_input_should_report(const struct analog_input_io_channel *c,
                                              int32_t dv, int32_t pv) {
    if (c->report_threshold && c->report_settle_samples) return dv != pv;
    return analog_input_changed(dv, pv, c->report_threshold);
}

/*
 * Take one ADC sequence, gating the sensor's excitation rail around it.
 *
 * With power-gpios set (a MOSFET switching the top leg of a resistive divider)
 * the divider only burns current while we are actually sampling: assert the
 * gate, wait power-settle-us for the wiper's RC to settle, convert, release.
 * A 10k slide pot across 3V3 draws ~330uA continuously; at 10 Hz with a ~3 ms
 * on-time that becomes ~10uA average, which is what makes it viable on a
 * wireless half. No-op when power-gpios is absent (always-on divider).
 *
 * The rail is released on every path out, including read errors.
 */
static int analog_input_acquire(const struct device *dev) {
    struct analog_input_data *data = dev->data;
    const struct analog_input_config *config = dev->config;
    const struct device *adc = config->io_channels[0].adc_channel.dev;
    int err;

    if (data->power_ready) {
        err = gpio_pin_set_dt(&config->power_gpio, 1);
        if (err < 0) {
            LOG_ERR("excitation gate on returned %d", err);
            return err;
        }
        if (config->power_settle_us) {
            k_sleep(K_USEC(config->power_settle_us));
        }
    }

#ifdef CONFIG_ADC_ASYNC
    err = adc_read_async(adc, &data->as, &data->async_sig);
    if (err == 0) {
        err = k_poll(&data->async_evt, 1, K_FOREVER);
        if (err == 0) {
            if (data->async_evt.signal->signaled) {
                data->async_evt.signal->signaled = 0;
                data->async_evt.state = K_POLL_STATE_NOT_READY;
            } else {
                err = -EAGAIN;
            }
        } else {
            LOG_ERR("k_poll returned %d", err);
        }
    } else {
        LOG_ERR("read_async returned %d", err);
    }
#else
    err = adc_read(adc, &data->as);
    if (err < 0) {
        LOG_ERR("read returned %d", err);
    }
#endif

    if (data->power_ready) {
        int perr = gpio_pin_set_dt(&config->power_gpio, 0);
        if (perr < 0) {
            LOG_ERR("excitation gate off returned %d", perr);
        }
    }

    return err;
}

/*
 * Retime the sampling timer. Used to drop to idle-sampling-hz once every
 * settle-mode channel has gone quiet, and to snap back to the full rate the
 * moment anything moves. The sample rate is the dominant battery cost when
 * the excitation rail is gated, but it is also what the fader's responsiveness
 * on first touch depends on.
 */
static void analog_input_apply_rate(const struct device *dev, uint32_t hz) {
    struct analog_input_data *data = dev->data;

    if (!hz || !data->enabled || data->cur_hz == hz) {
        return;
    }
    data->cur_hz = hz;
    uint32_t usec = 1000000UL / hz;
    k_timer_start(&data->sampling_timer, K_USEC(usec), K_USEC(usec));
}

static int analog_input_report_data(const struct device *dev) {
    struct analog_input_data *data = dev->data;
    const struct analog_input_config *config = dev->config;

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
    static int64_t last_smp_time = 0;
    static int64_t last_rpt_time = 0;
    int64_t now = k_uptime_get();
#endif

    struct adc_sequence* as = &data->as;

    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        struct analog_input_io_channel ch_cfg = (struct analog_input_io_channel)config->io_channels[i];
        const struct device* adc = ch_cfg.adc_channel.dev;

        if (i == 0) {
            int err = analog_input_acquire(dev);
            if (err == -EAGAIN) {
                return 0;
            }
            if (err < 0) {
                return err;
            }
        }

        /*
         * Single-ended readings sit just above 0 V at the bottom of travel, so
         * a noisy sample can come back slightly negative. as_buff is unsigned,
         * which would turn -1 into 65535 and flip the fader to full scale, so
         * reinterpret as signed and floor at zero. (Only sound while the
         * resolution leaves the sign bit free, which every supported width
         * here does.)
         */
        int32_t raw = (int16_t)data->as_buff[i];
        if (raw < 0) {
            raw = 0;
        }
        int32_t mv = raw;
        /*
         * Scale with the channel's own gain rather than a fixed one: an
         * off-chip ADC (e.g. ADS1115, ref 2048 mV at gain 1/2) has a completely
         * different full scale from the nRF SAADC (600 mV at gain 1/6), and
         * hardcoding the latter misreads every other controller. Falls back to
         * the historical constant when channel config isn't taken from DT.
         */
#if IS_ENABLED(CONFIG_ANALOG_INPUT_USE_DTS_ADC_CH_CFG)
        enum adc_gain gain = ch_cfg.adc_channel.channel_cfg.gain;
#else
        enum adc_gain gain = ADC_GAIN_1_6;
#endif
        adc_raw_to_millivolts(adc_ref_internal(adc), gain, as->resolution, &mv);
#if IS_ENABLED(CONFIG_ANALOG_INPUT_LOG_DBG_RAW)
        LOG_DBG("AIN%u raw: %d mv: %d", ch_cfg.adc_channel.channel_id, raw, mv);
#endif

        int16_t v = mv - ch_cfg.mv_mid;
        int16_t dz = ch_cfg.mv_deadzone;
        if (dz) {
            if (v > 0) {
                if (v < dz) v = 0; else v -= dz;
            }
            if (v < 0) {
                if (v > -dz) v = 0; else v += dz;
            }
        }
        uint16_t mm = ch_cfg.mv_min_max;
        if (mm) {
            if (v > 0 && v > mm) v = mm;
            if (v < 0 && v < -mm) v = -mm;
        }

        if (ch_cfg.invert) v *= -1;
        v = (int16_t)((v * ch_cfg.scale_multiplier) / ch_cfg.scale_divisor);

        if (ch_cfg.report_on_change_only) {
            if (ch_cfg.report_threshold && ch_cfg.report_settle_samples) {
                // Settle mode: report every change while moving (full resolution
                // everywhere, including compressed ends), but go quiet once the
                // value has stayed within report_threshold of its anchor for
                // report_settle_samples in a row, so a noisy source can deep-sleep.
                // Any move beyond the band re-anchors and wakes it.
                int32_t dev = v - data->anchor[i];
                if (dev < 0) dev = -dev;
                if (dev >= (int32_t)ch_cfg.report_threshold) {
                    data->anchor[i] = v;
                    data->still[i] = 0;
                    data->idle_ch[i] = false;
                } else if (!data->idle_ch[i] && ++data->still[i] >= ch_cfg.report_settle_samples) {
                    data->idle_ch[i] = true;
                }
                // While idle, hold delta == prev so nothing is emitted.
                data->delta[i] = data->idle_ch[i] ? data->prev[i] : v;
            } else {
                // track value to compare until next report interval
                data->delta[i] = v;
            }
        }
        else {
            // accumulate delta until report in next iteration
            int32_t delta = data->delta[i];
            int32_t dv = delta + v;
            data->delta[i] = dv;
        }
    }

    // First read is setup as calibration
    as->calibrate = false;

    /*
     * Once every settle-mode channel has gone quiet there is nothing to track
     * until the user touches the control again, so sample far more slowly. The
     * cost of the slow rate is first-touch latency (one idle period before the
     * move is noticed), which is why this is separate from report-settle-samples.
     */
    if (config->idle_sampling_hz) {
        bool any_settle_ch = false;
        bool all_idle = true;
        for (uint8_t i = 0; i < config->io_channels_len; i++) {
            const struct analog_input_io_channel *c = &config->io_channels[i];
            if (!c->report_on_change_only || !c->report_threshold || !c->report_settle_samples) {
                continue;
            }
            any_settle_ch = true;
            if (!data->idle_ch[i]) {
                all_idle = false;
                break;
            }
        }
        if (any_settle_ch) {
            analog_input_apply_rate(dev, all_idle ? config->idle_sampling_hz : data->sampling_hz);
        }
    }

#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
    // purge accumulated delta, if last sampled had not been reported on last report tick
    if (now - last_smp_time >= CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN) {
        for (uint8_t i = 0; i < config->io_channels_len; i++) {
            data->delta[i] = 0;
            data->prev[i] = 0;
        }
    }
    last_smp_time = now;
#endif

#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
    // strict to report inerval
    if (now - last_rpt_time < CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN) {
        return 0;
    }
#endif

    if (!data->actived) {
        return 0;
    }

    int8_t idx_to_sync = -1;
    for (int i = config->io_channels_len - 1; i >= 0; i--) {
        struct analog_input_io_channel ch_cfg = (struct analog_input_io_channel)config->io_channels[i];
        if (analog_input_should_report(&ch_cfg, data->delta[i], data->prev[i])) {
            idx_to_sync = i;
            break;
        }
    }

    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        struct analog_input_io_channel ch_cfg = (struct analog_input_io_channel)config->io_channels[i];
        // LOG_DBG("AIN%u get delta AGAIN", i);
        int32_t dv = data->delta[i];
        int32_t pv = data->prev[i];
        if (analog_input_should_report(&ch_cfg, dv, pv)) {
#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
            last_rpt_time = now;
#endif
            data->delta[i] = 0;
            if (ch_cfg.report_on_change_only) {
                data->prev[i] = dv;
            }

#if IS_ENABLED(CONFIG_ANALOG_INPUT_LOG_DBG_REPORT)
            LOG_DBG("input_report %u rv: %d  e:%d  c:%d", i, dv, ch_cfg.evt_type, ch_cfg.input_code);
#endif
            input_report(dev, ch_cfg.evt_type, ch_cfg.input_code, dv, i == idx_to_sync, K_NO_WAIT);
        }
    }
    return 0;
}

K_THREAD_STACK_DEFINE(analog_input_q_stack, CONFIG_ANALOG_INPUT_WORKQUEUE_STACK_SIZE);

static struct k_work_q analog_input_work_q;

static void sampling_work_handler(struct k_work *work) {
    struct analog_input_data *data = CONTAINER_OF(work, struct analog_input_data, sampling_work);
    // LOG_DBG("sampling work triggered");
    analog_input_report_data(data->dev);
}

static void sampling_timer_handler(struct k_timer *timer) {
    struct analog_input_data *data = CONTAINER_OF(timer, struct analog_input_data, sampling_timer);
    // LOG_DBG("sampling timer triggered");
    /*
     * Private queue only. This used to also submit to the system workqueue,
     * which was merely redundant when a sample was a few microseconds of SAADC;
     * with an off-chip ADC a sample blocks for milliseconds (settle delay plus
     * I2C conversion), and stalling ZMK's system workqueue that long would hold
     * up BLE and HID work.
     */
    k_work_submit_to_queue(&analog_input_work_q, &data->sampling_work);
}

static int active_set_value(const struct device *dev, bool active) {
    struct analog_input_data *data = dev->data;
    if (data->actived == active) return 0;
    LOG_DBG("%d", active ? 1 : 0);
    data->actived = active;
    return 0;
}

static int sample_hz_set_value(const struct device *dev, uint32_t hz) {
    struct analog_input_data *data = dev->data;

    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    if (data->enabled) {
        LOG_DBG("Device is busy, would not update sampleing rate in enable state.");
        return -EBUSY;
    }

    LOG_DBG("%d", hz);
    data->sampling_hz = hz;
    return 0;
}

static int enable_set_value(const struct device *dev, bool enable) {
    struct analog_input_data *data = dev->data;
    // const struct tb6612fng_config *config = dev->config;

    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    if (data->enabled == enable) {
        return 0;
    }
    
    LOG_DBG("%d", enable ? 1 : 0);
    if (enable) {
        if (data->sampling_hz != 0) {
            uint32_t usec = 1000000UL / data->sampling_hz;
            k_timer_start(&data->sampling_timer, K_USEC(usec), K_USEC(usec));
        } else {
            k_timer_start(&data->sampling_timer, K_NO_WAIT, K_NO_WAIT);
        }
        data->cur_hz = data->sampling_hz;
        data->enabled = true;
    }
    else {
        k_timer_stop(&data->sampling_timer);
        data->cur_hz = 0;
        data->enabled = false;
    }

    return 0;
}

static void analog_input_async_init(struct k_work *work) {
    struct k_work_delayable *work_delayable = (struct k_work_delayable *)work;
    struct analog_input_data *data = CONTAINER_OF(work_delayable, 
                                                  struct analog_input_data, init_work);
    const struct device *dev = data->dev;
    const struct analog_input_config *config = dev->config;

    // LOG_DBG("ANALOG_INPUT async init");
    uint32_t ch_mask = 0;

    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        struct analog_input_io_channel ch_cfg = (struct analog_input_io_channel)config->io_channels[i];
        const struct device* adc = ch_cfg.adc_channel.dev;
        uint8_t channel_id = ch_cfg.adc_channel.channel_id;
        
#if CONFIG_ANALOG_INPUT_USE_DTS_ADC_CH_CFG
        struct adc_channel_cfg channel_cfg = ch_cfg.adc_channel.channel_cfg;
#else
        struct adc_channel_cfg channel_cfg = {
            .gain = ADC_GAIN_1_6,
            .reference = ADC_REF_INTERNAL,
            .acquisition_time = ADC_ACQ_TIME_DEFAULT,
            .channel_id = channel_id,
            #ifdef CONFIG_ADC_CONFIGURABLE_INPUTS
                #ifdef CONFIG_ADC_NRFX_SAADC
                    .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + channel_id,
                #else /* CONFIG_ADC_NRFX_SAADC */
                    .input_positive = channel_id,
                #endif /* CONFIG_ADC_NRFX_SAADC */
            #endif /* CONFIG_ADC_CONFIGURABLE_INPUTS */
        };
#endif

        ch_mask |= BIT(channel_id);

        if (!device_is_ready(adc)) {
            LOG_ERR("AIN%u device is not ready %s", i, adc->name);
            continue;
        }

        int err = adc_channel_setup(adc, &channel_cfg);
        if (err < 0) {
            LOG_ERR("AIN%u setup returned %d", i, err);
        }
    }

    /*
     * Excitation gate, if the board has one. Configured inactive so the divider
     * stays unpowered between samples; the active level (a P-FET high-side
     * switch is active low) comes from the GPIO flags in devicetree.
     */
    if (config->power_gpio.port) {
        if (!gpio_is_ready_dt(&config->power_gpio)) {
            LOG_ERR("excitation gate %s is not ready", config->power_gpio.port->name);
        } else {
            int err = gpio_pin_configure_dt(&config->power_gpio, GPIO_OUTPUT_INACTIVE);
            if (err < 0) {
                LOG_ERR("excitation gate configure returned %d", err);
            } else {
                data->power_ready = true;
            }
        }
    }

    uint16_t delta_size = config->io_channels_len * sizeof(int32_t);
    data->delta = malloc(delta_size);
    memset(data->delta, 0, delta_size);

    uint16_t prev_size = config->io_channels_len * sizeof(int32_t);
    data->prev = malloc(prev_size);
    memset(data->prev, 0, prev_size);

    data->anchor = malloc(config->io_channels_len * sizeof(int32_t));
    memset(data->anchor, 0, config->io_channels_len * sizeof(int32_t));
    data->still = malloc(config->io_channels_len * sizeof(uint16_t));
    memset(data->still, 0, config->io_channels_len * sizeof(uint16_t));
    data->idle_ch = malloc(config->io_channels_len * sizeof(bool));
    memset(data->idle_ch, 0, config->io_channels_len * sizeof(bool));

#if CONFIG_ANALOG_INPUT_ADC_RES > 16
    uint16_t buff_size = config->io_channels_len * sizeof(uint32_t);
#else
    uint16_t buff_size = config->io_channels_len * sizeof(uint16_t);
#endif
    data->as_buff = malloc(buff_size);
    memset(data->as_buff, 0, buff_size);

    data->as = (struct adc_sequence){
        .channels = ch_mask,
        .buffer = data->as_buff,
        .buffer_size = buff_size,
        .oversampling = 0,
        .resolution = CONFIG_ANALOG_INPUT_ADC_RES,
        .calibrate = true,
    };

#ifdef CONFIG_ADC_ASYNC
    k_poll_signal_init(&data->async_sig);
    struct k_poll_event async_evt = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                                             K_POLL_MODE_NOTIFY_ONLY,
                                                             &data->async_sig);
    data->async_evt = async_evt;
#endif

    data->ready = true;

    k_work_init(&data->sampling_work, sampling_work_handler);
    k_work_queue_start(&analog_input_work_q,
                        analog_input_q_stack, K_THREAD_STACK_SIZEOF(analog_input_q_stack),
                        CONFIG_ANALOG_INPUT_WORKQUEUE_PRIORITY, NULL);

    k_timer_init(&data->sampling_timer, sampling_timer_handler, NULL);

    sample_hz_set_value(dev, config->sampling_hz);
    active_set_value(dev, true);
    if (data->sampling_hz) {
        enable_set_value(dev, true);
    }

}

static int analog_input_init(const struct device *dev) {
    struct analog_input_data *data = dev->data;
    // const struct analog_input_config *config = dev->config;
    int err = 0;

    data->dev = dev;
    k_work_init_delayable(&data->init_work, analog_input_async_init);
    k_work_schedule(&data->init_work, K_MSEC(1));

    return err;
}

static int analog_input_attr_set(const struct device *dev, enum sensor_channel chan,
                            enum sensor_attribute attr, const struct sensor_value *val) {
    struct analog_input_data *data = dev->data;
    // const struct analog_input_config *config = dev->config;
    int err;

    if (chan != SENSOR_CHAN_ALL) {
        LOG_DBG("Selected channel is not supported: %d.", chan);
        return -ENOTSUP;
    }
    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    switch ((uint32_t)attr) {
    case ANALOG_INPUT_ATTR_SAMPLING_HZ:
        err = sample_hz_set_value(dev, ANALOG_INPUT_SVALUE_TO_SAMPLING_HZ(*val));
        break;

    case ANALOG_INPUT_ATTR_ENABLE:
        err = enable_set_value(dev, ANALOG_INPUT_SVALUE_TO_ENABLE(*val));
        break;

    case ANALOG_INPUT_ATTR_ACTIVE:
        err = active_set_value(dev, ANALOG_INPUT_SVALUE_TO_ACTIVE(*val));
        break;

    default:
        LOG_ERR("Unknown attribute");
        err = -ENOTSUP;
    }

    return err;
}

static int analog_input_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct analog_input_data *data = dev->data;
    // const struct analog_input_config *config = dev->config;

    if (chan != SENSOR_CHAN_ALL) {
        LOG_DBG("Selected channel is not supported: %d.", chan);
        return -ENOTSUP;
    }
    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    int err = analog_input_report_data(data->dev);
    if (err < 0) {
        LOG_ERR("analog_input_report_data returned %d", err);
        return err;
    }

    return 0;
}

static int analog_input_channel_get(const struct device *dev, enum sensor_channel chan,
                                    struct sensor_value *val) {
    struct analog_input_data *data = dev->data;
    const struct analog_input_config *config = dev->config;

    if (unlikely(chan != SENSOR_CHAN_ALL)) {
        LOG_DBG("Selected channel is not supported: %d.", chan);
        return -ENOTSUP;
    }
    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        struct analog_input_io_channel ch_cfg = (struct analog_input_io_channel)config->io_channels[i];
        if (!ch_cfg.report_on_change_only) {
            continue;
        }
        if (i == 0)      val->val1 = data->delta[i];
        else if (i == 1) val->val2 = data->delta[i];
    }

    return 0;
}

static const struct sensor_driver_api analog_input_driver_api = {
    .attr_set = analog_input_attr_set,
    .sample_fetch = analog_input_sample_fetch,
    .channel_get = analog_input_channel_get,
};

#define TRANSFORMED_IO_CHANNEL_ENTRY(node_id)                                                      \
    {                                                                                              \
        .adc_channel = ADC_DT_SPEC_GET_BY_IDX(node_id, 0),                                         \
        .mv_mid = DT_PROP(node_id, mv_mid),                                                        \
        .mv_min_max = DT_PROP(node_id, mv_min_max),                                                \
        .mv_deadzone = DT_PROP(node_id, mv_deadzone),                                              \
        .invert = DT_PROP(node_id, invert),                                                        \
        .report_on_change_only = DT_PROP(node_id, report_on_change_only),                          \
        .report_threshold = DT_PROP(node_id, report_threshold),                                    \
        .report_settle_samples = DT_PROP(node_id, report_settle_samples),                          \
        .scale_multiplier = DT_PROP(node_id, scale_multiplier),                                    \
        .scale_divisor = DT_PROP(node_id, scale_divisor),                                          \
        .evt_type = DT_PROP(node_id, evt_type),                                                    \
        .input_code = DT_PROP(node_id, input_code),                                                \
    }

#define ANIN_IOC_CHILD_LEN_PLUS_ONE(node) 1 +

#define ANALOG_INPUT_DEFINE(n)                                                                     \
    static struct analog_input_data data##n = {                                                    \
    };                                                                                             \
    static const struct analog_input_config config##n = {                                          \
        .sampling_hz = DT_PROP(DT_DRV_INST(n), sampling_hz),                                       \
        .idle_sampling_hz = DT_PROP(DT_DRV_INST(n), idle_sampling_hz),                             \
        .power_gpio = GPIO_DT_SPEC_INST_GET_OR(n, power_gpios, {0}),                               \
        .power_settle_us = DT_PROP(DT_DRV_INST(n), power_settle_us),                               \
        .io_channels_len = (DT_FOREACH_CHILD(DT_DRV_INST(n), ANIN_IOC_CHILD_LEN_PLUS_ONE) 0),      \
        .io_channels = { DT_INST_FOREACH_CHILD_SEP(n, TRANSFORMED_IO_CHANNEL_ENTRY, (, )) },       \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, analog_input_init, NULL, &data##n, &config##n, POST_KERNEL,           \
                          CONFIG_SENSOR_INIT_PRIORITY, &analog_input_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ANALOG_INPUT_DEFINE)
