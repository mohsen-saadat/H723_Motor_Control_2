/*
 * udp_app.c
 *
 *  Created on: Sep 28, 2025
 *      Author: mohsen
 */
#include "udp_app.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include <string.h>

#define RX_MAX 64

static struct {
    volatile uint32_t seq;
    uint16_t len;
    uint8_t  data[RX_MAX];
    ip_addr_t last_addr;
    u16_t     last_port;
} g_rx;

static struct udp_pcb *g_pcb = NULL;

static void rx_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                  const ip_addr_t *addr, u16_t port)
{
    if (!p) return;

    const u16_t n = (p->tot_len > RX_MAX) ? RX_MAX : p->tot_len;
    pbuf_copy_partial(p, g_rx.data, n, 0);
    g_rx.len = n;
    ip_addr_copy(g_rx.last_addr, *addr);
    g_rx.last_port = port;
    __DMB();               // publish safely before bumping seq
    g_rx.seq++;

    pbuf_free(p);
}

void udp_app_init(void)
{
    g_pcb = udp_new();
    udp_bind(g_pcb, IP_ADDR_ANY, 11000);  // listen on 11000, accept any sender
    udp_recv(g_pcb, rx_cb, NULL);
    // NOTE: no global TX pbuf reuse; we allocate per send now.
}

uint16_t udp_app_try_get_latest(uint8_t *out, uint16_t max)
{
    static uint32_t last = 0;
    if (g_rx.seq == last) return 0;
    __DMB();

    const uint16_t n = (g_rx.len > max) ? max : g_rx.len;
    memcpy(out, g_rx.data, n);
    last = g_rx.seq;
    return n;
}

void udp_app_send_reply(const void *data, uint16_t len)
{
    if (!g_pcb || !data || len == 0) return;

    // Allocate a right-sized pbuf for this reply
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (!p) return;

    memcpy(p->payload, data, len);

    // Send back to the last sender (captured in rx_cb)
    udp_sendto(g_pcb, p, &g_rx.last_addr, g_rx.last_port);

    // Free the temporary pbuf
    pbuf_free(p);
}
