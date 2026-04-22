#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "bluetooth.h"   /* bt_service_init(), ble_data_received_cb_t */

/* ------------------------------------------------------------------
 * Параметры
 * ------------------------------------------------------------------ */
#define RECEIVE_BUFF_SIZE 100
#define STACKSIZE         1024
#define THREAD_PRIORITY   6

/* ------------------------------------------------------------------
 * Кнопка
 * ------------------------------------------------------------------ */
#define SW0_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);

/* ------------------------------------------------------------------
 * Светодиоды
 * LED0–LED2 — пользовательские, LED3 — индикатор статуса
 * ------------------------------------------------------------------ */
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
static const struct gpio_dt_spec *user_leds[] = {&led0, &led1, &led2};
#define USER_LED_COUNT 3

/* ------------------------------------------------------------------
 * Буфер накопления команды (заполняется из BLE-колбэка)
 * ------------------------------------------------------------------ */
static uint16_t cmd_index;
static char     cmd_buffer[RECEIVE_BUFF_SIZE];

/* ------------------------------------------------------------------
 * Типы данных команды
 * ------------------------------------------------------------------ */
#define MAX_SEQ_STEPS 5

typedef struct {
	uint8_t  leds;
	uint32_t period;
	uint32_t duration;
} seq_step_t;

typedef struct {
	uint8_t    step_count;
	seq_step_t steps[MAX_SEQ_STEPS];
} led_command_t;

/* ------------------------------------------------------------------
 * Состояние программы
 * ------------------------------------------------------------------ */
enum program_state {
	STATE_IDLE,
	STATE_RUNNING,
	STATE_PAUSED
};

static enum program_state current_state = STATE_IDLE;
static led_command_t       current_cmd;
static int                 current_step;
static volatile bool       stop_requested  = false;
static bool                pause_requested = false;

/* ------------------------------------------------------------------
 * Очередь событий
 * ------------------------------------------------------------------ */
typedef enum {
	EVT_CMD,
	EVT_BUTTON,
	EVT_STOP,
} event_type_t;

typedef struct {
	event_type_t type;
	union {
		led_command_t cmd;
	} data;
} app_event_t;

K_MSGQ_DEFINE(event_msgq, sizeof(app_event_t), 10, 4);

/* ==================================================================
 * Вспомогательные функции для светодиодов
 * ================================================================== */

static void turn_off_user_leds(void)
{
	for (uint8_t i = 0; i < USER_LED_COUNT; i++) {
		gpio_pin_set_dt(user_leds[i], false);
	}
}

static void set_user_leds(uint8_t led_mask, bool state)
{
	for (int i = 0; i < USER_LED_COUNT; i++) {
		if (led_mask & (1 << i)) {
			gpio_pin_set_dt(user_leds[i], state);
		}
	}
}

/* ==================================================================
 * Обработчик событий
 * ================================================================== */

static void process_event(const app_event_t *ev)
{
	switch (ev->type) {

	case EVT_CMD:
		stop_requested = true;
		k_sleep(K_MSEC(10));
		turn_off_user_leds();
		current_cmd     = ev->data.cmd;
		current_step    = 0;
		pause_requested = false;
		stop_requested  = false;
		current_state   = STATE_RUNNING;
		printk("New command started\n");
		break;

	case EVT_STOP:
		stop_requested = true;
		k_sleep(K_MSEC(10));
		turn_off_user_leds();
		current_state   = STATE_IDLE;
		pause_requested = false;
		stop_requested  = false;
		printk("Stop command\n");
		break;

	case EVT_BUTTON:
		if (current_state == STATE_RUNNING) {
			pause_requested = true;
			printk("Pause requested after current step\n");
		} else if (current_state == STATE_PAUSED) {
			current_state = STATE_RUNNING;
			printk("Resuming\n");
		} else {
			printk("Button ignored in state %d\n", current_state);
		}
		break;

	default:
		break;
	}
}

/* ==================================================================
 * Мигание одного шага с периодической проверкой очереди событий
 * ================================================================== */

static bool wait_with_events(uint32_t duration_ms)
{
	uint32_t start = k_uptime_get_32();

	while ((k_uptime_get_32() - start) < duration_ms && !stop_requested) {
		app_event_t ev;
		while (k_msgq_get(&event_msgq, &ev, K_NO_WAIT) == 0) {
			process_event(&ev);
			if (stop_requested) {
				return false;
			}
		}
		k_msleep(10);
	}
	return !stop_requested;
}

static void blink_step(const seq_step_t *step)
{
	uint8_t  mask     = step->leds;
	uint32_t period   = step->period;
	uint32_t duration = step->duration;

	printk("Step: mask=%u, period=%u, duration=%u\n", mask, period, duration);

	if (period == 0) {
		/* Постоянное включение на duration мс */
		set_user_leds(mask, true);
		wait_with_events(duration);
		set_user_leds(mask, false);
		return;
	}

	uint32_t end  = k_uptime_get_32() + duration;
	uint32_t half = period / 2;

	while (k_uptime_get_32() < end && !stop_requested) {
		set_user_leds(mask, true);
		if (!wait_with_events(half) || k_uptime_get_32() >= end) {
			break;
		}
		set_user_leds(mask, false);
		if (!wait_with_events(half) || k_uptime_get_32() >= end) {
			break;
		}
	}

	set_user_leds(mask, false);
}

/* ==================================================================
 * Парсинг команды AT+START
 *
 * Формат: AT+START L<n> <period> <duration> [L<n> <period> <duration> ...]
 *   L0  → LED0 (маска 0x01)
 *   L1  → LED1 (маска 0x02)
 *   L2  → LED2 (маска 0x04)
 *   L12 → LED1+LED2 (маска 0x06)
 * ================================================================== */

static int parse_sequence(const char *cmd, seq_step_t *steps, int max_steps)
{
	const char *ptr = cmd + 8; /* пропустить "AT+START" */
	int step_idx = 0;

	while (*ptr != '\0' && step_idx < max_steps) {
		while (*ptr == ' ') ptr++;
		if (*ptr == '\0') break;
		if (*ptr != 'L') return -1;
		ptr++;

		uint8_t leds = 0;
		if (strncmp(ptr, "12", 2) == 0) {
			leds = 0x06; ptr += 2;
		} else if (*ptr == '0') {
			leds = 0x01; ptr++;
		} else if (*ptr == '1') {
			leds = 0x02; ptr++;
		} else if (*ptr == '2') {
			leds = 0x04; ptr++;
		} else {
			return -1;
		}

		while (*ptr == ' ') ptr++;
		if (*ptr == '\0') return -1;
		char *end;
		long period = strtol(ptr, &end, 10);
		if (end == ptr || period < 0) return -1;
		ptr = end;

		while (*ptr == ' ') ptr++;
		if (*ptr == '\0') return -1;
		long duration = strtol(ptr, &end, 10);
		if (end == ptr || duration <= 0) return -1;
		ptr = end;

		steps[step_idx].leds     = (uint8_t)leds;
		steps[step_idx].period   = (uint32_t)period;
		steps[step_idx].duration = (uint32_t)duration;
		step_idx++;
	}
	return step_idx;
}

/* ==================================================================
 * BLE receive callback
 *
 * Вызывается из bluetooth.c при каждом входящем BLE-пакете.
 * Накапливает байты в cmd_buffer, при \r или \n парсит команду
 * и кладёт событие в очередь event_msgq.
 * ================================================================== */

static void ble_receive_cb(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        char ch = (char)data[i];

        if (ch == '\r' || ch == '\n') {
            if (cmd_index == 0) {
                continue;
            }

            cmd_buffer[cmd_index] = '\0';
            cmd_index = 0;


            app_event_t ev = {0};

            if (strncmp(cmd_buffer, "AT+STOP", 7) == 0) {
                ev.type = EVT_STOP;
                k_msgq_put(&event_msgq, &ev, K_NO_WAIT);

            } else if (strncmp(cmd_buffer, "AT+START", 8) == 0) {
                led_command_t cmd = {0};
                int cnt = parse_sequence(cmd_buffer, cmd.steps, MAX_SEQ_STEPS);
                if (cnt > 0) {
                    cmd.step_count = (uint8_t)cnt;
                    ev.type        = EVT_CMD;
                    ev.data.cmd    = cmd;
                    int q = k_msgq_put(&event_msgq, &ev, K_NO_WAIT);
                    printk("EVT_CMD queued, result=%d\n", q); /* <-- 4 */
                } else {
                    printk("Invalid START command: '%s'\r\n", cmd_buffer);
                }
            } else {
                printk("Unknown command: '%s'\r\n", cmd_buffer);
            }

        } else if (cmd_index < (uint16_t)(sizeof(cmd_buffer) - 1)) {
            cmd_buffer[cmd_index++] = ch;
        }
    }
}

/* ==================================================================
 * Обработчик кнопки (прерывание GPIO)
 * ================================================================== */

static void button_cb(const struct device *dev,
		      struct gpio_callback *cb,
		      uint32_t pins)
{
	printk("Button pressed!\n");
	app_event_t ev = { .type = EVT_BUTTON };
	k_msgq_put(&event_msgq, &ev, K_NO_WAIT);
}

static struct gpio_callback button_cb_data;

/* ==================================================================
 * Поток управления светодиодами
 * ================================================================== */

void control_thread(void *arg1, void *arg2, void *arg3)
{
	app_event_t ev;

	while (1) {
		/* Ждём первое событие (блокирующий вызов) */
		k_msgq_get(&event_msgq, &ev, K_FOREVER);
		process_event(&ev);

		/* Основной цикл воспроизведения шагов */
		while (current_state == STATE_RUNNING &&
		       current_step < current_cmd.step_count) {

			stop_requested = false;
			blink_step(&current_cmd.steps[current_step]);

			if (stop_requested) {
				/* Получена команда STOP или новая START */
				current_state = STATE_IDLE;
				turn_off_user_leds();
				break;
			}

			current_step++;

			/* Переходим в PAUSED между шагами */
			if (current_step < current_cmd.step_count && pause_requested) {
				current_state   = STATE_PAUSED;
				pause_requested = false;
				turn_off_user_leds();
				break;
			}
		}

		/* Все шаги выполнены — вернуться в IDLE */
		if (current_state == STATE_RUNNING &&
		    current_step >= current_cmd.step_count) {
			current_state = STATE_IDLE;
			turn_off_user_leds();
		}
	}
}

K_THREAD_DEFINE(control_thread_id, STACKSIZE,
		control_thread, NULL, NULL, NULL,
		THREAD_PRIORITY, 0, 0);

/* ==================================================================
 * Поток индикации статуса (LED3)
 *
 *   IDLE    → быстрое мигание (100 мс вкл / 100 мс выкл)
 *   RUNNING → медленное мигание (500 мс / 500 мс)
 *   PAUSED  → постоянно горит
 * ================================================================== */

void status_led_thread(void *arg1, void *arg2, void *arg3)
{
	while (1) {
		switch (current_state) {
		case STATE_IDLE:
			gpio_pin_set_dt(&led3, 1);
			k_msleep(100);
			gpio_pin_set_dt(&led3, 0);
			k_msleep(100);
			break;

		case STATE_RUNNING:
			gpio_pin_set_dt(&led3, 1);
			k_msleep(500);
			gpio_pin_set_dt(&led3, 0);
			k_msleep(500);
			break;

		case STATE_PAUSED:
			gpio_pin_set_dt(&led3, 1);
			k_msleep(100);
			break;
		}
	}
}

K_THREAD_DEFINE(status_led_thread_id, STACKSIZE,
		status_led_thread, NULL, NULL, NULL,
		THREAD_PRIORITY, 0, 0);

/* ==================================================================
 * Main
 * ================================================================== */

int main(void)
{
	int err;

	/* --- Инициализация светодиодов --- */
	const struct gpio_dt_spec *all_leds[] = {&led0, &led1, &led2, &led3};

	for (int i = 0; i < 4; i++) {
		if (!device_is_ready(all_leds[i]->port)) {
			printk("LED%d not ready\n", i);
			return 1;
		}
		gpio_pin_configure_dt(all_leds[i], GPIO_OUTPUT_ACTIVE);
		gpio_pin_set_dt(all_leds[i], 0); /* выключить при старте */
	}

	/* --- Инициализация кнопки --- */
	if (device_is_ready(button.port)) {
		gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
		gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_FALLING);
		gpio_init_callback(&button_cb_data, button_cb, BIT(button.pin));
		gpio_add_callback(button.port, &button_cb_data);
		printk("Button initialized\n");
	} else {
		printk("Button not ready\n");
	}

	/* --- Инициализация BLE (вместо UART) --- */
	err = bt_service_init(ble_receive_cb);
	if (err) {
		printk("BT service init failed (err %d)\n", err);
		return err;
	}

	printk("LED controller ready. Waiting for BLE commands...\n");

	/*
	 * main() возвращает 0 — потоки control_thread и status_led_thread
	 * продолжают работать независимо (Zephyr kernel не завершается).
	 */
	return 0;
}
