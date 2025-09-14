#ifndef MYLIB_MPU9250_HPP
#define MYLIB_MPU9250_HPP

#include "driver/spi_master.h"

namespace mylib
{
    class MPU9250
    {
    public:
        static constexpr uint8_t SMPLRT_DIV = 0x19;
        static constexpr uint8_t CONFIG = 0x1A;
        static constexpr uint8_t GYRO_CONFIG = 0x1B;
        static constexpr uint8_t ACCEL_CONFIG = 0x1C;
        static constexpr uint8_t ACCEL_CONFIG2 = 0x1D;

        static constexpr uint8_t ACCEL_XOUT_H = 0x3B;
        static constexpr uint8_t ACCEL_XOUT_L = 0x3C;
        static constexpr uint8_t ACCEL_YOUT_H = 0x3D;
        static constexpr uint8_t ACCEL_YOUT_L = 0x3E;
        static constexpr uint8_t ACCEL_ZOUT_H = 0x3F;
        static constexpr uint8_t ACCEL_ZOUT_L = 0x40;
        static constexpr uint8_t TEMP_OUT_H = 0x41;
        static constexpr uint8_t TEMP_OUT_L = 0x42;
        static constexpr uint8_t GYRO_XOUT_H = 0x43;
        static constexpr uint8_t GYRO_XOUT_L = 0x44;
        static constexpr uint8_t GYRO_YOUT_H = 0x45;
        static constexpr uint8_t GYRO_YOUT_L = 0x46;
        static constexpr uint8_t GYRO_ZOUT_H = 0x47;
        static constexpr uint8_t GYRO_ZOUT_L = 0x48;

        static constexpr uint8_t PWR_MGMT_1 = 0x6B;

        static constexpr size_t ADDR_SIZE = 8;

        struct gyro_dlpf_cfg_t
        {
            uint8_t dlpf_cfg, fchoice_b;
        };
        struct gyro_scale_cfg_t
        {
            uint8_t gyro_fs_sel;
            int dps;
        };
        struct accel_dlpf_cfg_t
        {
            uint8_t dlpf_cfg, fchoice_b;
        };
        struct accel_scale_cfg_t
        {
            uint8_t accel_fs_sel;
            int g;
        };

        static constexpr gyro_dlpf_cfg_t GYRO_DLPF_CFG[10] = {
            {0, 0b11},
            {0, 0b10},
            {0, 0b00},
            {1, 0b00},
            {2, 0b00},
            {3, 0b00},
            {4, 0b00},
            {5, 0b00},
            {6, 0b00},
            {7, 0b00},
        };
        static constexpr gyro_scale_cfg_t GYRO_SCALE_CFG[4] = {
            {0b00, 250},
            {0b01, 500},
            {0b10, 1000},
            {0b11, 2000},
        };
        static constexpr accel_dlpf_cfg_t ACCEL_DLPF_CFG[9] = {
            {0, 0b1},
            {0, 0b0},
            {1, 0b0},
            {2, 0b0},
            {3, 0b0},
            {4, 0b0},
            {5, 0b0},
            {6, 0b0},
            {7, 0b0},
        };
        static constexpr accel_scale_cfg_t ACCEL_SCALE_CFG[4] = {
            {0b00, 2},
            {0b01, 4},
            {0b10, 8},
            {0b11, 16},
        };

        struct sensor_data_t
        {
            int16_t r_accel[3], r_temp, r_gyro[3];
            double accel[3], temp, gyro[3];
        };

        static constexpr double ROOM_TEMP_OFFSET = 0.0;
        static constexpr double TEMP_SENSITIVITY = 333.87;
        static constexpr double TEMP_OFFSET = 21.0;

    private:
        spi_device_handle_t spi_device_handle;
        spi_host_device_t spi_port;
        int gpio_mosi, gpio_miso, gpio_sclk, gpio_cs;

        esp_err_t spi_writebytes(uint8_t addr, uint8_t *data, size_t len);
        esp_err_t spi_writebyte(uint8_t addr, uint8_t data);
        esp_err_t spi_readbytes(uint8_t addr, uint8_t *data, size_t len);
        esp_err_t spi_readbyte(uint8_t addr, uint8_t data);

    public:
        gyro_dlpf_cfg_t gyro_dlpf_cfg;
        gyro_scale_cfg_t gyro_scale_cfg;
        accel_dlpf_cfg_t accel_dlpf_cfg;
        accel_scale_cfg_t accel_scale_cfg;

        MPU9250(spi_host_device_t spi_port, int mosi, int miso, int sclk, int cs);

        void spi_begin(void);
        void spi_end(void);
        void set_config(
            gyro_dlpf_cfg_t gyro_dlpf_cfg, gyro_scale_cfg_t gyro_scale_cfg,
            accel_dlpf_cfg_t accel_dlpf_cfg, accel_scale_cfg_t accel_scale_cfg);
        sensor_data_t read_data(void);
    };
}

#endif
