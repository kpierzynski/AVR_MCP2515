#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "spi.h"
#include "uart.h"

typedef enum MCP2515_result
{
    MCP2515_RESULT_SUCCESS = 0,
    MCP2515_RESULT_ERROR = 1,
    MCP2515_RESULT_INVALID_PARAMETER = 2
} MCP2515_result_t;

#define REG_BFPCTRL 0x0c
#define REG_TXRTSCTRL 0x0d

#define REG_CANCTRL 0x0f

#define REG_CNF3 0x28
#define REG_CNF2 0x29
#define REG_CNF1 0x2a

#define REG_CANINTE 0x2b
#define REG_CANINTF 0x2c

#define FLAG_RXnIE(n) (0x01 << n)
#define FLAG_RXnIF(n) (0x01 << n)
#define FLAG_TXnIF(n) (0x04 << n)

#define REG_RXFnSIDH(n) (0x00 + (n * 4))
#define REG_RXFnSIDL(n) (0x01 + (n * 4))
#define REG_RXFnEID8(n) (0x02 + (n * 4))
#define REG_RXFnEID0(n) (0x03 + (n * 4))

#define REG_RXMnSIDH(n) (0x20 + (n * 0x04))
#define REG_RXMnSIDL(n) (0x21 + (n * 0x04))
#define REG_RXMnEID8(n) (0x22 + (n * 0x04))
#define REG_RXMnEID0(n) (0x23 + (n * 0x04))

#define REG_TXBnCTRL(n) (0x30 + (n * 0x10))
#define REG_TXBnSIDH(n) (0x31 + (n * 0x10))
#define REG_TXBnSIDL(n) (0x32 + (n * 0x10))
#define REG_TXBnEID8(n) (0x33 + (n * 0x10))
#define REG_TXBnEID0(n) (0x34 + (n * 0x10))
#define REG_TXBnDLC(n) (0x35 + (n * 0x10))
#define REG_TXBnD0(n) (0x36 + (n * 0x10))

#define REG_RXBnCTRL(n) (0x60 + (n * 0x10))
#define REG_RXBnSIDH(n) (0x61 + (n * 0x10))
#define REG_RXBnSIDL(n) (0x62 + (n * 0x10))
#define REG_RXBnEID8(n) (0x63 + (n * 0x10))
#define REG_RXBnEID0(n) (0x64 + (n * 0x10))
#define REG_RXBnDLC(n) (0x65 + (n * 0x10))
#define REG_RXBnD0(n) (0x66 + (n * 0x10))

#define FLAG_IDE 0x08
#define FLAG_SRR 0x10
#define FLAG_RTR 0x40
#define FLAG_EXIDE 0x08

#define FLAG_RXM0 0x20
#define FLAG_RXM1 0x40

#define MCP2515_CS_DDR DDRB
#define MCP2515_CS_PORT PORTB
#define MCP2515_CS_PIN PB2
#define MCP2515_CS_LOW MCP2515_CS_PORT &= ~(1 << MCP2515_CS_PIN)
#define MCP2515_CS_HIGH MCP2515_CS_PORT |= (1 << MCP2515_CS_PIN)

#define MCP2515_INT_DDR DDRD
#define MCP2515_INT_PORT PORTD
#define MCP2515_INT_PIN PD2

#define MCP2515_RESET_DDR DDRD
#define MCP2515_RESET_PORT PORTD
#define MCP2515_RESET_PIN PD3
#define MCP2515_RESET_LOW MCP2515_RESET_PORT &= ~(1 << MCP2515_RESET_PIN)
#define MCP2515_RESET_HIGH MCP2515_RESET_PORT |= (1 << MCP2515_RESET_PIN)

void mcp2515_write_reg(uint8_t address, uint8_t value)
{
    MCP2515_CS_LOW;
    spi_transmit(0x02);
    spi_transmit(address);
    spi_transmit(value);
    MCP2515_CS_HIGH;
}

uint8_t mcp2515_read_reg(uint8_t address)
{
    MCP2515_CS_LOW;
    spi_transmit(0x03);
    spi_transmit(address);
    uint8_t value = spi_transmit(0x00);
    MCP2515_CS_HIGH;

    return value;
}

void mcp2515_soft_reset()
{
    MCP2515_CS_LOW;
    spi_transmit(0xC0);
    MCP2515_CS_HIGH;
}

MCP2515_result_t mcp2515_enter_config()
{
    mcp2515_write_reg(REG_CANCTRL, 0x80);
    if (mcp2515_read_reg(REG_CANCTRL) != 0x80)
        return MCP2515_RESULT_ERROR;

    return MCP2515_RESULT_SUCCESS;
}

MCP2515_result_t mcp2515_init()
{
    mcp2515_soft_reset();

    if (mcp2515_enter_config() != MCP2515_RESULT_SUCCESS)
        return MCP2515_RESULT_ERROR;

    // for 8MHz crystal and 500E3 baud rate:
    // (long)8E6,  (long)500E3, { 0x00, 0x90, 0x02 }

    mcp2515_write_reg(REG_CNF1, 0x00);
    mcp2515_write_reg(REG_CNF2, 0x90);
    mcp2515_write_reg(REG_CNF3, 0x02);

    mcp2515_write_reg(REG_CANINTE, FLAG_RXnIE(1) | FLAG_RXnIE(0));
    mcp2515_write_reg(REG_BFPCTRL, 0x00);
    mcp2515_write_reg(REG_TXRTSCTRL, 0x00);
    mcp2515_write_reg(REG_RXBnCTRL(0), FLAG_RXM1 | FLAG_RXM0);
    mcp2515_write_reg(REG_RXBnCTRL(1), FLAG_RXM1 | FLAG_RXM0);

    mcp2515_write_reg(REG_CANCTRL, 0x00);
    if (mcp2515_read_reg(REG_CANCTRL) != 0x00)
        return MCP2515_RESULT_ERROR;

    return MCP2515_RESULT_SUCCESS;
}

MCP2515_result_t mcp2515_filer(uint16_t id, uint16_t mask)
{
    mask &= 0x7FF;
    id &= 0x7FF;

    // if (id > 0x7FF || mask > 0x7FF) return MCP2515_RESULT_INVALID_PARAMETER;

    if (mcp2515_enter_config() != MCP2515_RESULT_SUCCESS)
        return MCP2515_RESULT_ERROR;

    for (uint8_t n = 0; n < 2; n++)
    {
        mcp2515_write_reg(REG_RXBnCTRL(n), FLAG_RXM0);
        mcp2515_write_reg(REG_RXBnCTRL(n), FLAG_RXM0);

        mcp2515_write_reg(REG_RXMnSIDH(n), mask >> 3);
        mcp2515_write_reg(REG_RXMnSIDL(n), mask << 5);

        mcp2515_write_reg(REG_RXMnEID8(n), 0x00);
        mcp2515_write_reg(REG_RXMnEID0(n), 0x00);
    }

    for (uint8_t n = 0; n < 6; n++)
    {
        mcp2515_write_reg(REG_RXFnSIDH(n), id >> 3);
        mcp2515_write_reg(REG_RXFnSIDL(n), id << 5);
        mcp2515_write_reg(REG_RXFnEID8(n), 0x00);
        mcp2515_write_reg(REG_RXFnEID0(n), 0x00);
    }

    mcp2515_write_reg(REG_CANCTRL, 0x00);
    if (mcp2515_read_reg(REG_CANCTRL) != 0x00)
        return MCP2515_RESULT_ERROR;

    return MCP2515_RESULT_SUCCESS;
}

typedef struct
{
    uint16_t tx_id;
    uint8_t tx_len;
    uint8_t tx_data[8];
    int8_t tx_dlc;
    int8_t tx_rtr;

} packet_t;

MCP2515_result_t mcp2515_begin(packet_t *packet, uint16_t id, int8_t dlc, int8_t rtr)
{
    if (id > 0x7FF || dlc > 8)
        return MCP2515_RESULT_INVALID_PARAMETER;

    packet->tx_id = id;
    packet->tx_len = 0;
    packet->tx_dlc = dlc;
    packet->tx_rtr = rtr;

    for (uint8_t i = 0; i < 8; i++)
    {
        packet->tx_data[i] = 0x00;
    }

    return MCP2515_RESULT_SUCCESS;
}

void mcp2515_write(packet_t *packet, uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        packet->tx_data[i] = data[i];
    }

    packet->tx_len = len;
}

void mcp2515_modify_reg(uint8_t address, uint8_t mask, uint8_t data)
{
    MCP2515_CS_LOW;
    spi_transmit(0x05);
    spi_transmit(address);
    spi_transmit(mask);
    spi_transmit(data);
    MCP2515_CS_HIGH;
}

MCP2515_result_t mcp2515_send(packet_t *packet)
{
    if (packet->tx_dlc >= 0)
        packet->tx_len = packet->tx_dlc;

    mcp2515_write_reg(REG_TXBnSIDH(0), packet->tx_id >> 3);
    mcp2515_write_reg(REG_TXBnSIDL(0), packet->tx_id << 5);
    mcp2515_write_reg(REG_TXBnEID8(0), 0x00);
    mcp2515_write_reg(REG_TXBnEID0(0), 0x00);

    if (packet->tx_rtr)
    {
        mcp2515_write_reg(REG_TXBnDLC(0), 0x40 | packet->tx_len);
    }
    else
    {
        mcp2515_write_reg(REG_TXBnDLC(0), packet->tx_len);

        for (uint8_t i = 0; i < packet->tx_len; i++)
        {
            mcp2515_write_reg(REG_TXBnD0(0) + i, packet->tx_data[i]);
        }
    }

    mcp2515_write_reg(REG_TXBnCTRL(0), 0x08);

    uint8_t status = 0;
    while (mcp2515_read_reg(REG_TXBnCTRL(0)) & 0x08)
    {
        uart_putc('|');
        _delay_ms(100);
        if ((status = (mcp2515_read_reg(REG_RXBnCTRL(0) & 0x10))))
        {
            mcp2515_modify_reg(REG_CANCTRL, 0x10, 0x10);
        }
    }

    if (status)
        mcp2515_modify_reg(REG_CANCTRL, 0x10, 0x00);

    mcp2515_modify_reg(REG_CANINTF, FLAG_TXnIF(0), 0x00);

    return (mcp2515_read_reg(REG_TXBnCTRL(0)) & 0x70) ? MCP2515_RESULT_ERROR : MCP2515_RESULT_SUCCESS;
}

MCP2515_result_t mcp2515_parse(uint8_t *rx_data)
{
    uint8_t status = mcp2515_read_reg(REG_CANINTF);

    if (!(status & FLAG_RXnIF(0)))
        return MCP2515_RESULT_ERROR;

    // uint32_t idA = ((mcp2515_read_reg(REG_RXBnSIDH(0)) << 3) & 0x07F8) | ((mcp2515_read_reg(REG_RXBnSIDL(0)) >> 5) & 0x07);
    uint8_t rx_rtr = (mcp2515_read_reg(REG_RXBnSIDL(0)) & FLAG_SRR) ? 1 : 0;
    uint8_t rx_dlc = mcp2515_read_reg(REG_RXBnDLC(0)) & 0x0F;

    uint8_t rx_len = 0;

    if (rx_rtr)
        rx_len = 0;
    else
    {
        rx_len = rx_dlc;
        for (uint8_t i = 0; i < rx_len; i++)
        {
            rx_data[i] = mcp2515_read_reg(REG_RXBnD0(0) + i);
        }
    }

    mcp2515_modify_reg(REG_CANINTF, FLAG_RXnIF(0), 0x00);

    return MCP2515_RESULT_SUCCESS;
}

int main()
{
    _delay_ms(2000);

    spi_init();
    uart_init();
    sei();

    uart_puts_P(PSTR("Hello, World!\r\n"));

    if (mcp2515_init() != MCP2515_RESULT_SUCCESS)
    {
        uart_puts_P(PSTR("MCP2515 init failed.\r\n"));
        while (1)
            ;
    }

    while (1)
    {
        uint8_t recv_buf[8];
        for (uint8_t i = 0; i < 8; i++)
            recv_buf[i] = 0x00;

        packet_t packet;
        uint8_t buf[] = {0x02, 0x01, 0x0c};

        if (mcp2515_begin(&packet, 0x7DF, 8, 0) != MCP2515_RESULT_SUCCESS)
        {
            uart_puts("Begin failed.\r\n");
            while (1)
                ;
        }
        mcp2515_write(&packet, buf, 3);
        if (mcp2515_send(&packet) != MCP2515_RESULT_SUCCESS)
        {
            uart_puts("Send failed.\r\n");
            while (1)
                ;
        }

        while (MCP2515_RESULT_SUCCESS != mcp2515_parse(recv_buf))
        {
            uart_putc('.');
            _delay_ms(100);
        }

        _delay_ms(500);
        uart_putbuf(recv_buf, 8, "MCP2515");

        uint32_t value = (((uint32_t)recv_buf[3] * 256) + recv_buf[4]) / 4;

        uart_puts("Value: ");
        uart_putd16(value);
    }

    return 0;
}