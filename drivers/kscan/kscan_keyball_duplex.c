/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: MIT
 *
 * ZMK kscan driver implementing the Keyball "Japanese duplex matrix".
 *
 * This is a straight port of the scanning logic from QMK's Keyball
 * matrix.c (duplex_scan_raw): the same physical row/col pins are
 * scanned twice, in opposite directions, which doubles the number of
 * logical columns without needing extra GPIOs.
 *
 *   Phase A (row -> col): each row pin is driven active, each col pin
 *                          is read -> logical columns [0 .. N_COLS-1]
 *   Phase B (col -> row): each col pin is driven active, each row pin
 *                          is read -> logical columns [N_COLS .. 2*N_COLS-1]
 */

#define DT_DRV_COMPAT zmk_kscan_keyball_duplex

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(kscan_keyball_duplex, CONFIG_KSCAN_LOG_LEVEL);

#define INST_ROWS(n) DT_INST_PROP_LEN(n, row_gpios)
#define INST_COLS_PHYS(n) DT_INST_PROP_LEN(n, col_gpios)
#define INST_COLS_LOGICAL(n) (INST_COLS_PHYS(n) * 2)

struct kscan_keyball_duplex_config {
    const struct gpio_dt_spec *rows;
    const struct gpio_dt_spec *cols;
    uint8_t n_rows;
    uint8_t n_cols_phys;
    uint8_t n_cols_logical;
    uint32_t debounce_press_ms;
    uint32_t debounce_release_ms;
    uint32_t poll_period_ms;
    uint32_t settle_time_us;
};

/* Per-cell debounce counter. Positive values count consecutive scans
 * that agree with the *new* candidate state; once the relevant
 * threshold is reached, the stable state flips. */
struct cell_state {
    bool stable;      /* debounced, published state */
    bool candidate;   /* last raw reading */
    uint16_t counter; /* consecutive scans matching `candidate` since it changed */
};

struct kscan_keyball_duplex_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_work_delayable work;
    bool enabled;
    /* flattened [row][logical_col] */
    struct cell_state *cells;
};

static inline struct cell_state *cell_at(const struct device *dev, uint8_t row, uint8_t col) {
    const struct kscan_keyball_duplex_config *cfg = dev->config;
    struct kscan_keyball_duplex_data *data = dev->data;
    return &data->cells[(row * cfg->n_cols_logical) + col];
}

static void pin_drive_active(const struct gpio_dt_spec *spec) {
    gpio_pin_configure_dt(spec, GPIO_OUTPUT_ACTIVE);
}

static void pin_release_to_input(const struct gpio_dt_spec *spec) {
    gpio_pin_configure_dt(spec, GPIO_INPUT);
}

/* Apply one raw reading to the debounce state machine for a single
 * cell. Returns true if the *stable* state changed as a result. */
static bool debounce_apply(const struct kscan_keyball_duplex_config *cfg, struct cell_state *cell,
                            bool pressed_now) {
    uint32_t threshold_ms = pressed_now ? cfg->debounce_press_ms : cfg->debounce_release_ms;
    uint32_t threshold_scans = threshold_ms == 0
                                    ? 0
                                    : (threshold_ms / (cfg->poll_period_ms ? cfg->poll_period_ms : 1));

    if (pressed_now != cell->candidate) {
        /* Reading flipped: restart the debounce window. */
        cell->candidate = pressed_now;
        cell->counter = 0;
    }

    if (cell->candidate == cell->stable) {
        return false;
    }

    if (cell->counter < UINT16_MAX) {
        cell->counter++;
    }

    if (cell->counter > threshold_scans) {
        cell->stable = cell->candidate;
        return true;
    }

    return false;
}

static void kscan_keyball_duplex_scan(const struct device *dev) {
    const struct kscan_keyball_duplex_config *cfg = dev->config;
    struct kscan_keyball_duplex_data *data = dev->data;

    /* --- Phase A: row -> col --- */
    for (uint8_t row = 0; row < cfg->n_rows; row++) {
        pin_drive_active(&cfg->rows[row]);
        if (cfg->settle_time_us) {
            k_busy_wait(cfg->settle_time_us);
        }

        for (uint8_t col = 0; col < cfg->n_cols_phys; col++) {
            bool pressed = gpio_pin_get_dt(&cfg->cols[col]) > 0;
            struct cell_state *cell = cell_at(dev, row, col);

            if (debounce_apply(cfg, cell, pressed) && data->callback) {
                data->callback(dev, row, col, cell->stable);
            }
        }

        pin_release_to_input(&cfg->rows[row]);
    }

    /* --- Phase B: col -> row (fills the "extra" logical columns) --- */
    for (uint8_t col = 0; col < cfg->n_cols_phys; col++) {
        pin_drive_active(&cfg->cols[col]);
        if (cfg->settle_time_us) {
            k_busy_wait(cfg->settle_time_us);
        }

        uint8_t logical_col = col + cfg->n_cols_phys;
        for (uint8_t row = 0; row < cfg->n_rows; row++) {
            bool pressed = gpio_pin_get_dt(&cfg->rows[row]) > 0;
            struct cell_state *cell = cell_at(dev, row, logical_col);

            if (debounce_apply(cfg, cell, pressed) && data->callback) {
                data->callback(dev, row, logical_col, cell->stable);
            }
        }

        pin_release_to_input(&cfg->cols[col]);
    }
}

static void kscan_keyball_duplex_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct kscan_keyball_duplex_data *data =
        CONTAINER_OF(dwork, struct kscan_keyball_duplex_data, work);
    const struct kscan_keyball_duplex_config *cfg = data->dev->config;

    kscan_keyball_duplex_scan(data->dev);

    if (data->enabled) {
        k_work_reschedule(&data->work, K_MSEC(cfg->poll_period_ms));
    }
}

static int kscan_keyball_duplex_configure(const struct device *dev, kscan_callback_t callback) {
    struct kscan_keyball_duplex_data *data = dev->data;
    data->callback = callback;
    return 0;
}

static int kscan_keyball_duplex_enable_callback(const struct device *dev) {
    struct kscan_keyball_duplex_data *data = dev->data;
    const struct kscan_keyball_duplex_config *cfg = dev->config;

    data->enabled = true;
    k_work_reschedule(&data->work, K_MSEC(cfg->poll_period_ms));
    return 0;
}

static int kscan_keyball_duplex_disable_callback(const struct device *dev) {
    struct kscan_keyball_duplex_data *data = dev->data;
    data->enabled = false;
    k_work_cancel_delayable(&data->work);
    return 0;
}

static const struct kscan_driver_api kscan_keyball_duplex_api = {
    .config = kscan_keyball_duplex_configure,
    .enable_callback = kscan_keyball_duplex_enable_callback,
    .disable_callback = kscan_keyball_duplex_disable_callback,
};

static int kscan_keyball_duplex_init(const struct device *dev) {
    const struct kscan_keyball_duplex_config *cfg = dev->config;
    struct kscan_keyball_duplex_data *data = dev->data;

    data->dev = dev;

    for (uint8_t i = 0; i < cfg->n_rows; i++) {
        if (!gpio_is_ready_dt(&cfg->rows[i])) {
            LOG_ERR("Row GPIO %d not ready", i);
            return -ENODEV;
        }
        pin_release_to_input(&cfg->rows[i]);
    }

    for (uint8_t i = 0; i < cfg->n_cols_phys; i++) {
        if (!gpio_is_ready_dt(&cfg->cols[i])) {
            LOG_ERR("Col GPIO %d not ready", i);
            return -ENODEV;
        }
        pin_release_to_input(&cfg->cols[i]);
    }

    k_work_init_delayable(&data->work, kscan_keyball_duplex_work_handler);

    return 0;
}

#define KSCAN_KEYBALL_DUPLEX_INIT(n)                                                             \
    static const struct gpio_dt_spec kscan_keyball_duplex_rows_##n[] = {                         \
        DT_INST_FOREACH_PROP_ELEM_SEP(n, row_gpios, GPIO_DT_SPEC_GET_BY_IDX, (,))};               \
    static const struct gpio_dt_spec kscan_keyball_duplex_cols_##n[] = {                         \
        DT_INST_FOREACH_PROP_ELEM_SEP(n, col_gpios, GPIO_DT_SPEC_GET_BY_IDX, (,))};               \
    static struct cell_state                                                                     \
        kscan_keyball_duplex_cells_##n[INST_ROWS(n) * INST_COLS_LOGICAL(n)];                     \
    static const struct kscan_keyball_duplex_config kscan_keyball_duplex_config_##n = {          \
        .rows = kscan_keyball_duplex_rows_##n,                                                   \
        .cols = kscan_keyball_duplex_cols_##n,                                                   \
        .n_rows = INST_ROWS(n),                                                                  \
        .n_cols_phys = INST_COLS_PHYS(n),                                                        \
        .n_cols_logical = INST_COLS_LOGICAL(n),                                                  \
        .debounce_press_ms = DT_INST_PROP(n, debounce_press_ms),                                 \
        .debounce_release_ms = DT_INST_PROP(n, debounce_release_ms),                             \
        .poll_period_ms = DT_INST_PROP(n, poll_period_ms),                                       \
        .settle_time_us = DT_INST_PROP(n, settle_time_us),                                       \
    };                                                                                            \
    static struct kscan_keyball_duplex_data kscan_keyball_duplex_data_##n = {                    \
        .cells = kscan_keyball_duplex_cells_##n,                                                 \
    };                                                                                            \
    DEVICE_DT_INST_DEFINE(n, kscan_keyball_duplex_init, NULL, &kscan_keyball_duplex_data_##n,     \
                          &kscan_keyball_duplex_config_##n, POST_KERNEL,                          \
                          CONFIG_KSCAN_INIT_PRIORITY, &kscan_keyball_duplex_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_KEYBALL_DUPLEX_INIT)
