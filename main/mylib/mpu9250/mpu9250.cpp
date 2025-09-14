#include "mpu9250.hpp"


esp_err_t mylib::MPU9250::spi_writebytes(uint8_t addr, uint8_t *data, size_t len) {
  spi_transaction_t transaction = {
    .flags = 0,
    .addr = (uint64_t)(addr & 0b01111111), // write mode
    .length = mylib::MPU9250::ADDR_SIZE + (size_t)(8 * len),
    .tx_buffer = data,
  };
  return spi_device_transmit(spi_device_handle, &transaction);;
}

esp_err_t mylib::MPU9250::spi_writebyte(uint8_t addr, uint8_t data) {
  return spi_writebytes(addr, &data, 1);
}

esp_err_t mylib::MPU9250::spi_readbytes(uint8_t addr, uint8_t *data, size_t len) {
  spi_transaction_t transaction = {
    .flags = 0,
    .addr = (uint64_t)(addr | 0b10000000), // read mode
    .length = mylib::MPU9250::ADDR_SIZE + 8 * len,
    .rxlength = (size_t)(8 * len),
    .rx_buffer = data,
  };
  return spi_device_transmit(spi_device_handle, &transaction);
}

esp_err_t mylib::MPU9250::spi_readbyte(uint8_t addr, uint8_t data) {
  return spi_readbytes(addr, &data, 1);
}


mylib::MPU9250::MPU9250(spi_host_device_t spi_port, int mosi, int miso, int sclk, int cs) {
  this->spi_port = spi_port;
  this->gpio_mosi = mosi;
  this->gpio_miso = miso;
  this->gpio_sclk = sclk;
  this->gpio_cs = cs;
}


void mylib::MPU9250::spi_begin(void) {
  spi_bus_config_t bus_cfg = {
    .mosi_io_num = gpio_mosi,
    .miso_io_num = gpio_miso,
    .sclk_io_num = gpio_sclk,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4092, // TODO
    .flags = SPICOMMON_BUSFLAG_MASTER, // TODO
  };
  spi_bus_initialize(spi_port, &bus_cfg, SPI_DMA_CH_AUTO);

  spi_device_interface_config_t device_cfg = {
    .command_bits = 0,
    .address_bits = 8,
    .dummy_bits = 0,
    .mode = 0,
    .duty_cycle_pos = 128,
    .cs_ena_pretrans = 0,
    .cs_ena_posttrans = 0,
    .clock_speed_hz = 1000000, // 1Mhz
    .input_delay_ns = 0,
    .spics_io_num = gpio_cs,
    .flags = 0, // TODO
    .queue_size = 1, // TODO
    .pre_cb = NULL,
    .post_cb = NULL,
  };
  spi_bus_add_device(spi_port, &device_cfg, &spi_device_handle);

  return;
}

void mylib::MPU9250::spi_end(void) {
  spi_bus_remove_device(spi_device_handle);
  spi_bus_free(spi_port);
  return;
}

void mylib::MPU9250::set_config(
  gyro_dlpf_cfg_t gyro_dlpf_cfg, gyro_scale_cfg_t gyro_scale_cfg,
  accel_dlpf_cfg_t accel_dlpf_cfg, accel_scale_cfg_t accel_scale_cfg
  ) {

  this->gyro_dlpf_cfg = gyro_dlpf_cfg;
  this->gyro_scale_cfg = gyro_scale_cfg;
  this->accel_dlpf_cfg = accel_dlpf_cfg;
  this->accel_scale_cfg = accel_scale_cfg;

  // reset
  spi_writebyte(PWR_MGMT_1, 0b10000000);
  vTaskDelay(pdMS_TO_TICKS(100)); // wait 100ms

  spi_writebyte(SMPLRT_DIV, 0x00);

  spi_writebyte(CONFIG, gyro_dlpf_cfg.dlpf_cfg);

  spi_writebyte(GYRO_CONFIG, (gyro_scale_cfg.gyro_fs_sel << 3) | gyro_dlpf_cfg.fchoice_b);

  spi_writebyte(ACCEL_CONFIG, (accel_scale_cfg.accel_fs_sel << 3));

  spi_writebyte(ACCEL_CONFIG2, (accel_dlpf_cfg.fchoice_b << 3) | accel_dlpf_cfg.dlpf_cfg);

  return;
}

mylib::MPU9250::sensor_data_t mylib::MPU9250::read_data(void) {
  uint8_t *data = (uint8_t*)heap_caps_malloc(14, MALLOC_CAP_DMA);
  spi_readbytes(ACCEL_XOUT_H, data, 14);

  sensor_data_t sensor_data;
  sensor_data.r_accel[0] = (int16_t)((data[0] << 8) | data[1]);
  sensor_data.r_accel[1] = (int16_t)((data[2] << 8) | data[3]);
  sensor_data.r_accel[2] = (int16_t)((data[4] << 8) | data[5]);
  sensor_data.r_temp = (int16_t)((data[6] << 8) | data[7]);
  sensor_data.r_gyro[0] = (int16_t)((data[ 8] << 8) | data[ 9]);
  sensor_data.r_gyro[1] = (int16_t)((data[10] << 8) | data[11]);
  sensor_data.r_gyro[2] = (int16_t)((data[12] << 8) | data[13]);

  heap_caps_free(data);

  sensor_data.temp = ((double)sensor_data.r_temp - ROOM_TEMP_OFFSET) / TEMP_SENSITIVITY + TEMP_OFFSET;
  for (int i = 0; i < 3; i++) {
    sensor_data.accel[i] = (double)sensor_data.r_accel[i] / ((double)(1 << 15) / accel_scale_cfg.g);
    sensor_data.gyro[i] = (double)sensor_data.r_gyro[i] / ((double)(1 << 15) / gyro_scale_cfg.dps);
  }
  return sensor_data;
}
