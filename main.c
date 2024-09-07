#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "spi.h"
#include "mcp2515.h"
#include "uart.h"
#include "obd_pid.h"

void obd_request()
{
    uint8_t buf[] = {0x02, 0x01, OBD2_PID_ENGINE_RPM};
    if (mcp2515_put(0x7DF, 8, 0, buf, 3) != MCP2515_RESULT_SUCCESS)
        uart_puts("Put failed.\r\n");
}

void rx_obd(uint16_t id, uint8_t *buf, uint8_t len)
{
    uart_puts("Received: ");
    uart_putd16(id);
    uart_puts(" - ");
    uart_putbuf(buf, len, "Data");

    uint16_t value = OBD2_PID_ENGINE_RPM_CONV(buf[3], buf[4]);

    uart_puts("Value: ");
    uart_putd16(value);

    // ------------

    obd_request();
}

int main()
{
    _delay_ms(2000);

    spi_init();
    uart_init();
    sei();

    uart_puts_P(PSTR("Hello, World!\r\n"));

    register_mcp2515_rx_event_callback(rx_obd);

    if (mcp2515_init() != MCP2515_RESULT_SUCCESS)
    {
        uart_puts_P(PSTR("MCP2515 init failed.\r\n"));
        while (1)
            ;
    }

    uint8_t recv_buf[8];
    for (uint8_t i = 0; i < 8; i++)
        recv_buf[i] = 0x00;

    obd_request();

    while (1)
    {
        mcp2515_get(recv_buf);
    }

    return 0;
}