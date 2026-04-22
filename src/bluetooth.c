/*
 * bluetooth.c — BLE NUS library for nRF54L15
 *
 * Обеспечивает:
 *   - инициализацию BT-стека
 *   - Nordic UART Service (NUS) для приёма AT-команд по BLE
 *   - авто-рестарт рекламы после разрыва соединения
 *
 * Не использует UART-мост и dk_buttons_and_leds —
 * GPIO управляются в main.c.
 */

#include "bluetooth.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <string.h>

#define LOG_MODULE_NAME ble_service
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* ------------------------------------------------------------------
 * Внутренние переменные
 * ------------------------------------------------------------------ */

static struct bt_conn *current_conn;
static struct k_work   adv_work;
static ble_data_received_cb_t user_rx_cb;

static struct k_work_delayable flush_work;

static void flush_work_handler(struct k_work *work)
{
    if (user_rx_cb != NULL) {
        const uint8_t newline = '\n';
        user_rx_cb(&newline, 1);
    }
}

/* ------------------------------------------------------------------
 * BLE advertising data
 * ------------------------------------------------------------------ */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

/* ------------------------------------------------------------------
 * Реклама (advertising)
 * ------------------------------------------------------------------ */

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2,
				  ad, ARRAY_SIZE(ad),
				  sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}
	LOG_INF("BLE advertising started");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

/* ------------------------------------------------------------------
 * Connection callbacks
 * ------------------------------------------------------------------ */

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err 0x%02x)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected: %s", addr);

	current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected: %s, reason 0x%02x", addr, reason);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

/* Вызывается когда объект соединения освобождён — можно рекламироваться снова */
static void recycled_cb(void)
{
	LOG_INF("Connection recycled, restarting advertising");
	advertising_start();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
	.recycled     = recycled_cb,
};

/* ------------------------------------------------------------------
 * NUS receive callback
 * ------------------------------------------------------------------ */

static void bt_receive_cb(struct bt_conn *conn,
                           const uint8_t *const data,
                           uint16_t len)
{
    printk("BLE RX: %d bytes: %.*s\n", len, len, data);

    if (user_rx_cb == NULL) {
        return;
    }

    user_rx_cb(data, len);

    if (data[len - 1] == '\r' || data[len - 1] == '\n') {
        /* \n уже есть — отменить таймер */
        k_work_cancel_delayable(&flush_work);
    } else {
        /* Ждём ещё пакеты. Если 100мс тишина — завершить команду */
        k_work_reschedule(&flush_work, K_MSEC(100));
    }
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

/* ------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------ */

int bt_service_init(ble_data_received_cb_t cb)
{
    int err;

    if (cb == NULL) {
        LOG_ERR("rx callback must not be NULL");
        return -EINVAL;
    }

    user_rx_cb = cb;

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth enable failed (err %d)", err);
        return err;
    }
    LOG_INF("Bluetooth initialized");

    /* settings_load() убран — требует BT_SETTINGS */

    err = bt_nus_init(&nus_cb);
    if (err) {
        LOG_ERR("NUS init failed (err %d)", err);
        return err;
    }

    k_work_init_delayable(&flush_work, flush_work_handler);
    k_work_init(&adv_work, adv_work_handler);
    advertising_start();

    return 0;
}