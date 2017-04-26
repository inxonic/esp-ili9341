void ili_cmd(spi_device_handle_t spi, const uint8_t cmd);
void ili_data(spi_device_handle_t spi, const uint8_t *data, int len);
void ili_spi_pre_transfer_callback(spi_transaction_t *t);
void ili_init(spi_device_handle_t spi);
void ili_draw_bitmap(spi_device_handle_t spi, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t * bitmap, int free_buffer);
void send_line_finish(spi_device_handle_t spi);
