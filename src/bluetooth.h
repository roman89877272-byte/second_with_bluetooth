#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>

/**
 * @file bluetooth.h
 * @brief BLE NUS (Nordic UART Service) library interface.
 *
 * Инициализирует стек Bluetooth, регистрирует NUS-сервис и
 * запускает BLE-рекламу. Полученные по BLE данные передаются
 * в пользовательский колбэк.
 */

/**
 * @brief Тип колбэка для приёма данных по BLE.
 *
 * Вызывается из контекста BT-потока при каждом входящем пакете NUS.
 *
 * @param data  Указатель на принятые байты (не нуль-терминирован).
 * @param len   Количество принятых байт.
 */
typedef void (*ble_data_received_cb_t)(const uint8_t *data, uint16_t len);

/**
 * @brief Инициализировать BLE-стек и запустить NUS-сервис.
 *
 * Функция должна быть вызвана один раз из main() до начала
 * основного цикла приложения.
 *
 * @param cb  Колбэк, вызываемый при каждом приёме данных по BLE.
 *            Не может быть NULL.
 * @return 0 при успехе, отрицательный код ошибки при сбое.
 */
int bt_service_init(ble_data_received_cb_t cb);

#endif /* BLUETOOTH_H */