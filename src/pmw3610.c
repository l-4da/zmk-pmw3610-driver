/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610

// 12-bit two's complement value to int16_t
// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/device.h>
#include <zephyr/sys/dlist.h>
#include <drivers/behavior.h>
#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/keys.h>
#include <zmk/behavior_queue.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include "pmw3610.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_INPUT_LOG_LEVEL);

//////// Sensor initialization steps definition //////////
enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,
    ASYNC_INIT_STEP_CLEAR_OB1,
    ASYNC_INIT_STEP_CHECK_OB1,
    ASYNC_INIT_STEP_CONFIGURE,
    ASYNC_INIT_STEP_COUNT
};

/* Timings (in ms) needed in between steps to allow each step finishes succussfully. */
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10,
    [ASYNC_INIT_STEP_CLEAR_OB1] = 200,
    [ASYNC_INIT_STEP_CHECK_OB1] = 50,
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

//////// Function definitions //////////

static int spi_cs_ctrl(const struct device *dev, bool enable) {
    const struct pixart_config *config = dev->config;
    int err;

    if (!enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    err = gpio_pin_set_dt(&config->cs_gpio, (int)enable);
    if (err) {
        LOG_ERR("SPI CS ctrl failed");
    }

    if (enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    return err;
}

static int reg_read(const struct device *dev, uint8_t reg, uint8_t *buf) {
    int err;
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) return err;

    const struct spi_buf tx_buf = {.buf = &reg, .len = 1};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg read failed on SPI write");
        return err;
    }

    k_busy_wait(T_SRAD);

    struct spi_buf rx_buf = {.buf = buf, .len = 1};
    const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Reg read failed on SPI read");
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) return err;

    k_busy_wait(T_SRX);

    return 0;
}

static int _reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) return err;

    uint8_t buf[] = {SPI_WRITE_BIT | reg, val};
    const struct spi_buf tx_buf = {.buf = buf, .len = ARRAY_SIZE(buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg write failed on SPI write");
        return err;
    }

    k_busy_wait(T_SCLK_NCS_WR);

    err = spi_cs_ctrl(dev, false);
    if (err) return err;

    k_busy_wait(T_SWX);

    return 0;
}

static int reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;

    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (unlikely(err != 0)) return err;

    err = _reg_write(dev, reg, val);
    if (unlikely(err != 0)) return err;

    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    if (unlikely(err != 0)) return err;

    return 0;
}

static int motion_burst_read(const struct device *dev, uint8_t *buf, size_t burst_size) {
    int err;
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG(burst_size <= PMW3610_MAX_BURST_SIZE);

    err = spi_cs_ctrl(dev, true);
    if (err) return err;

    uint8_t reg_buf[] = {PMW3610_REG_MOTION_BURST};
    const struct spi_buf tx_buf = {.buf = reg_buf, .len = ARRAY_SIZE(reg_buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI write");
        return err;
    }

    k_busy_wait(T_SRAD_MOTBR);

    const struct spi_buf rx_buf = {.buf = buf, .len = burst_size};
    const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI read");
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) return err;

    k_busy_wait(T_BEXIT);

    return 0;
}

static int burst_write(const struct device *dev, const uint8_t *addr, const uint8_t *buf, size_t size) {
    int err;

    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (unlikely(err != 0)) return err;

    for (size_t i = 0; i < size; i++) {
        err = _reg_write(dev, addr[i], buf[i]);
        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            return err;
        }
    }

    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    if (unlikely(err != 0)) return err;

    return 0;
}

static int check_product_id(const struct device *dev) {
    uint8_t product_id = 0x01;
    int err = reg_read(dev, PMW3610_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }

    if (product_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, PMW3610_PRODUCT_ID);
        return -EIO;
    }

    return 0;
}

static int set_cpi(const struct device *dev, uint32_t cpi) {
    if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    uint8_t value = (cpi / 200);
    LOG_INF("Setting CPI to %u (reg value 0x%x)", cpi, value);

    uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
    uint8_t data[] = {0xFF, value, 0x00};
    int err = burst_write(dev, addr, data, 3);
    if (err) {
        LOG_ERR("Failed to set CPI");
        return err;
    }

    struct pixart_data *dev_data = dev->data;
    dev_data->curr_cpi = cpi;

    return 0;
}

static int set_cpi_if_needed(const struct device *dev, uint32_t cpi) {
    struct pixart_data *data = dev->data;
    if (cpi != data->curr_cpi) return set_cpi(dev, cpi);
    return 0;
}

/* Set sample time, downshift time, init steps etc. */
// ... [省略、先ほどのコードと同じ部分は省略してもOK]

static int pmw3610_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    uint8_t buf[PMW3610_BURST_SIZE];

    if (unlikely(!data->ready)) return -EBUSY;

    int32_t dividor;
    enum pixart_input_mode input_mode = get_input_mode_for_current_layer(dev);
    bool input_mode_changed = data->curr_mode != input_mode;
    switch (input_mode) {
    case MOVE:
        set_cpi_if_needed(dev, CONFIG_PMW3610_CPI);
        dividor = CONFIG_PMW3610_CPI_DIVIDOR;
        break;
    case SCROLL:
        set_cpi_if_needed(dev, CONFIG_PMW3610_CPI);
        if (input_mode_changed) {
            data->scroll_delta_x = 0;
            data->scroll_delta_y = 0;
        }
        dividor = 1;
        break;
    case SNIPE:
        set_cpi_if_needed(dev, CONFIG_PMW3610_SNIPE_CPI);
        dividor = CONFIG_PMW3610_SNIPE_CPI_DIVIDOR;
        break;
    case BALL_ACTION:
        set_cpi_if_needed(dev, CONFIG_PMW3610_CPI);
        if (input_mode_changed) {
            data->ball_action_delta_x = 0;
            data->ball_action_delta_y = 0;
        }
        dividor = 1;
        break;
    default:
        return -ENOTSUP;
    }

    data->curr_mode = input_mode;

    int16_t x = 0;
    int16_t y = 0;

    int err = motion_burst_read(dev, buf, sizeof(buf));
    if (err) return err;

    int16_t raw_x = TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12) / dividor;
    int16_t raw_y = TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12) / dividor;

    // Orientation handling
    if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_0)) { x = -raw_x; y = raw_y; }
    else if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_90)) { x = raw_y; y = -raw_x; }
    else if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_180)) { x = raw_x; y = -raw_y; }
    else if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_270)) { x = -raw_y; y = raw_x; }

    if (IS_ENABLED(CONFIG_PMW3610_INVERT_X)) x = -x;
    if (IS_ENABLED(CONFIG_PMW3610_INVERT_Y)) y = -y;

    // Mouse input reporting
    if (x != 0 || y != 0) {
        if (input_mode == MOVE || input_mode == SNIPE) {
            input_report_rel(dev, INPUT_REL_X, x, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
        } else if (input_mode == SCROLL) {
            data->scroll_delta_x += x;
            data->scroll_delta_y += y;
            if (abs(data->scroll_delta_y) > CONFIG_PMW3610_SCROLL_TICK) {
                input_report_rel(dev, INPUT_REL_WHEEL,
                                 data->scroll_delta_y > 0 ? PMW3610_SCROLL_Y_NEGATIVE : PMW3610_SCROLL_Y_POSITIVE,
                                 true, K_FOREVER);
                data->scroll_delta_x = 0;
                data->scroll_delta_y = 0;
            } else if (abs(data->scroll_delta_x) > CONFIG_PMW3610_SCROLL_TICK) {
                input_report_rel(dev, INPUT_REL_HWHEEL,
                                 data->scroll_delta_x > 0 ? PMW3610_SCROLL_X_NEGATIVE : PMW3610_SCROLL_X_POSITIVE,
                                 true, K_FOREVER);
                data->scroll_delta_x = 0;
                data->scroll_delta_y = 0;
            }
        } else if (input_mode == BALL_ACTION) {
            // Ball action handling...
        }
    }

    return err;
}

// GPIO callback and work handler
static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb, uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;
    set_interrupt(dev, false);
    k_work_submit(&data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;
    pmw3610_report_data(dev);
    set_interrupt(dev, true);
}

// Initialization routine
static int pmw3610_init(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    data->dev = dev;
    data->sw_smart_flag = false;

    k_work_init(&data->trigger_work, pmw3610_work_callback);

    if (!device_is_ready(config->cs_gpio.port)) return -ENODEV;
    err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) return err;

    err = pmw3610_init_irq(dev);
    if (err) return err;

    k_work_init_delayable(&data->init_work, pmw3610_async_init);
    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

// Device definition macro
#define PMW3610_DEFINE(n) \
    static struct pixart_data data##n; \
    static const struct pixart_config config##n = { \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios), \
        .bus = { .bus = DEVICE_DT_GET(DT_INST_BUS(n)), .config = { .frequency = DT_INST_PROP(n, spi_max_frequency), .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA, .slave = DT_INST_REG_ADDR(n) } }, \
        .cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)), \
    }; \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL, &data##n, &config##n, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)
