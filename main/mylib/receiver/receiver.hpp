#ifndef MYLIB_RECEIVER_HPP
#define MYLIB_RECEIVER_HPP

#include "driver/uart.h"

namespace mylib
{
    class Receiver
    {
    public:
        static constexpr int UART_BAUD_RATE = 115200;
        static constexpr uart_word_length_t UART_DATA_BITS = UART_DATA_8_BITS;
        static constexpr uart_parity_t UART_PARITY = UART_PARITY_DISABLE;
        static constexpr uart_stop_bits_t UART_STOP_BITS = UART_STOP_BITS_1;
        static constexpr size_t UART_BUFFER_SIZE = 1024;
        
        static constexpr size_t PACKET_SIZE = 32;
        static constexpr size_t CHANNEL_NUM = 14;

        static constexpr uint8_t HEADER_H = 0x20;
        static constexpr uint8_t HEADER_L = 0x40;

        struct receiver_data_t
        {
            uint16_t aux[CHANNEL_NUM];
        };

    private:
        uart_port_t uart_port;
        int gpio_rx;
        
        uint8_t last_data[PACKET_SIZE] = {0}, new_data_buff[PACKET_SIZE], queue_buff[UART_BUFFER_SIZE];
        size_t buff_ptr = 0;

    public:
        Receiver(uart_port_t uart_port, int gpio_rx);

        void uart_begin(void);
        void uart_end(void);
        bool update(void);
        receiver_data_t read_data(void);
    };
}


#endif
