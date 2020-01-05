# Driver for HTU21D temperature and humidity sensor for NRF5 SDK

Driver was tested with nrf52840 SoC. One of the twi interfaces (`twi`, `twim`) must be enabled. You can use following preprocessor flags for NRF SDK build: `-DNRFX_TWI_ENABLED=1 -DNRFX_TWI0_ENABLED=1`.
