#pragma once

#include <stdint.h>
#include "spi.h"
#include "gpio.h"
#include "enc28j60_register_map.h"

// configuration
#define SPI_CONTROLLER hspi2
#define CS_PORT GPIOB
#define CS_PIN GPIO_PIN_12
#define RESET_PORT GPIOC
#define RESET_PIN GPIO_PIN_6
#define SPI_TIMEOUT 1000


/**
 * Initialize ENC28J60
 * @brief require pree SPI and GPIO init
 * Default configuration:
 * - SPI mode
 *   -- CRC disabled
 *   -- FullDuplex
 *   -- Master
 *   -- 8B data size
 *   -- CPOL low
 *   -- CPHA 1 Edge
 *   -- MSB first
 *   -- NSS soft
 */
void enc28j60_init(uint8_t *macadr);

// Snd/Rcv packets
void enc28j60_send_packet(uint8_t *data, uint16_t len);
uint16_t enc28j60_recv_packet(uint8_t *buf, uint16_t buflen);

// R/W control registers
uint8_t enc28j60_rcr(uint8_t adr);
void enc28j60_wcr(uint8_t adr, uint8_t arg);
uint16_t enc28j60_rcr16(uint8_t adr);
void enc28j60_wcr16(uint8_t adr, uint16_t arg);
void enc28j60_bfc(uint8_t adr, uint8_t mask); // Clr bits (reg &= ~mask)
void enc28j60_bfs(uint8_t adr, uint8_t mask); // Set bits (reg |= mask)

// R/W Rx/Tx buffer
void enc28j60_read_buffer(uint8_t *buf, uint16_t len);
void enc28j60_write_buffer(uint8_t *buf, uint16_t len);

// R/W PHY reg
uint16_t enc28j60_read_phy(uint8_t adr);
void enc28j60_write_phy(uint8_t adr, uint16_t data);




