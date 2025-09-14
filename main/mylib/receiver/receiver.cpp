#include "receiver.hpp"


mylib::Receiver::Receiver(uart_port_t uart_port, int gpio_rx)
{
    this->uart_port = uart_port;
    this->gpio_rx = gpio_rx;
}

void mylib::Receiver::uart_begin(void)
{
    uart_config_t uart_cfg = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_BITS,
        .parity = UART_PARITY,
        .stop_bits = UART_STOP_BITS,
    };

    uart_param_config(uart_port, &uart_cfg);
    uart_set_pin(uart_port, UART_PIN_NO_CHANGE, gpio_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_port, UART_BUFFER_SIZE, 0, 0, NULL, 0);

    return;
}

void mylib::Receiver::uart_end(void)
{
    uart_driver_delete(uart_port);
    return;
}

bool mylib::Receiver::update(void)
{
    bool data_updated = false;

    size_t data_len;
    uart_get_buffered_data_len(uart_port, &data_len);
    uart_read_bytes(uart_port, queue_buff, data_len, 0);

    if (data_len > PACKET_SIZE * 2)
    {
        for (int i = 0; i < PACKET_SIZE * 2; i++)
        {
            queue_buff[i] = queue_buff[data_len - PACKET_SIZE * 2 + i];
        }
    }

    for (int i = 0; i < data_len; i++)
    {
        new_data_buff[buff_ptr] = queue_buff[i];
        buff_ptr = (buff_ptr + 1) % PACKET_SIZE;
        
        if (new_data_buff[buff_ptr] == HEADER_H && new_data_buff[(buff_ptr + 1) % PACKET_SIZE] == HEADER_L)
        {
            // compute checksum and validate packet
            uint16_t computed_checksum = 0xFFFF;
            for (int j = 0; j < PACKET_SIZE - 2; j++)
            {
                computed_checksum -= new_data_buff[(buff_ptr + j) % PACKET_SIZE];
            }
            uint16_t received_checksum = (new_data_buff[(buff_ptr + PACKET_SIZE - 1) % PACKET_SIZE] << 8) |
                                          new_data_buff[(buff_ptr + PACKET_SIZE - 2) % PACKET_SIZE]; // little endian
            
            if (computed_checksum == received_checksum) {
                for (int j = 0; j < PACKET_SIZE; j++)
                {
                    last_data[j] = new_data_buff[(buff_ptr + j) % PACKET_SIZE];
                }
                data_updated = true;
            }
        }
    }

    return data_updated;
}

mylib::Receiver::receiver_data_t mylib::Receiver::read_data(void)
{
    receiver_data_t data;
    
    for (int i = 0; i < CHANNEL_NUM; i++)
    {
        data.aux[i] = (last_data[i * 2 + 3] << 8) | last_data[i * 2 + 2]; // little endian
    }
    return data;
}
