/*
 * udp_app.h
 *
 *  Created on: Sep 28, 2025
 *      Author: mohsen
 */

#pragma once
#include "lwip/udp.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void udp_app_init(void);

/* Returns number of bytes copied (0 if no new packet since last call) */
uint16_t udp_app_try_get_latest(uint8_t *out, uint16_t max);

/* Sends a small reply to the last sender (optional) */
void udp_app_send_reply(const void *data, uint16_t len);

#ifdef __cplusplus
}
#endif
