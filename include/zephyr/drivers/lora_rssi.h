#ifndef LORA_RSSI_H_
#define LORA_RSSI_H_

#include <zephyr/drivers/lora.h>
#include <zephyr/device.h>

/* Noise floor measurement constants */
#define NOISE_FLOOR_DEFAULT_VALUE  -30   /* Default to "noisy" when measurement fails */

/**
 * @brief Function pointer type for noise floor measurement implementations
 */
typedef int16_t (*lora_api_get_noise_floor)(const struct device *dev);

/**
 * @brief Extended API functions for LoRa drivers
 */
struct lora_api_extensions {
    lora_api_get_noise_floor get_noise_floor;  /* Noise floor measurement function */
};

/**
 * @brief Get the RF noise floor measurement at the current frequency
 *
 * @param dev LoRa device
 * @return int16_t Noise floor in dBm
 */
int16_t lora_get_noise_floor(const struct device *dev);

/**
 * @brief Register noise floor measurement function with device
 *
 * @param dev_ptr Pointer to the device
 * @param func_ptr Pointer to the get_noise_floor implementation
 */
#define LORA_REGISTER_GET_NOISE_FLOOR(dev_ptr, func_ptr) \
    do { \
        struct sx127x_data *data = dev_ptr->data; \
        if (data) { \
            data->ext_api.get_noise_floor = func_ptr; \
        } \
    } while (0)

#endif /* LORA_RSSI_H_ */
