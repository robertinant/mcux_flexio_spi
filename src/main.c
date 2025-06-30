/*
 * Copyright (c) 2025 Fluke Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

#define LED0_NODE DT_ALIAS(led0)

#define MY_SPI_MASTER DT_NODELABEL(my_spi_master)

static const struct gpio_dt_spec spi_cs_gpio =
    GPIO_DT_SPEC_GET_BY_IDX(MY_SPI_MASTER, cs_gpios, 0);

const struct device *spi_dev;
static struct k_poll_signal spi_done_sig =
    K_POLL_SIGNAL_INITIALIZER(spi_done_sig);

static void spi_init(void) {
    spi_dev = DEVICE_DT_GET(MY_SPI_MASTER);
    if (!device_is_ready(spi_dev)) {
        printk("SPI master device not ready!\n");
    }

    if (!device_is_ready(spi_cs_gpio.port)) {
        printk("SPI master chip select device not ready!\n");
    } else {
        printk("SPI CS GPIO device ready\n");
    }
}

static const struct spi_config spi_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .frequency = 4000000,
    .slave = 0,
    .cs =
        {
            .gpio = spi_cs_gpio,
            .delay = 0,
        },
};

static int spi_write_test_msg(void) {
    static uint8_t counter = 0;
    static uint8_t tx_buffer[2];
    static uint8_t rx_buffer[2];

    const struct spi_buf tx_buf = {.buf = tx_buffer, .len = sizeof(tx_buffer)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = sizeof(rx_buffer),
    };
    const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    tx_buffer[0] = counter++;
    printk("SPI TX: 0x%.2x, 0x%.2x\n", tx_buffer[0], tx_buffer[1]);

    k_poll_signal_reset(&spi_done_sig);

    int error =
        spi_transceive_signal(spi_dev, &spi_cfg, &tx, &rx, &spi_done_sig);
    if (error != 0) {
        printk("SPI transceive error: %i\n", error);
        return error;
    }

    int spi_signaled, spi_result;
    do {
        k_poll_signal_check(&spi_done_sig, &spi_signaled, &spi_result);
    } while (spi_signaled == 0);
    printk("SPI RX: 0x%.2x, 0x%.2x\n", rx_buffer[0], rx_buffer[1]);
    return 0;
}

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void) {
    int ret;

    if (!device_is_ready(led.port)) {
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

    spi_init();

    printk("SPI master/slave example started\n");

    while (1) {
        spi_write_test_msg();
        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            return 0;
        }
        k_msleep(SLEEP_TIME_MS);
    }

    return 0;
}
