# Retain the ECG sources on disk; select only one app_main for this image.
# REQUIRES is deliberately independent of CONFIG_*: ESP-IDF discovers component
# dependencies before sdkconfig is loaded. Sources may be conditional; dependencies
# may not. The standalone image simply does not initialize wireless.
if(CONFIG_APP_MODE_BMI270_USB)
    set(app_sources "bmi270_test.c" "bmi270_driver.c"
                    "bmi270_sensorapi/bmi2.c" "bmi270_sensorapi/bmi270.c")
    set(app_requires esp_driver_i2c esp_driver_gpio esp_timer)
else()
    set(app_sources "main.c" "power_shutdown.c" "ads_control.c" "ads1292r_driver.c" "ble_link.c" "battery_adc.c"
                    "bmi270_driver.c" "bmi270_sensorapi/bmi2.c" "bmi270_sensorapi/bmi270.c")
    set(app_requires esp_driver_i2c esp_driver_spi esp_driver_gpio esp_timer bt nvs_flash esp_adc esp_hw_support)
endif()

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "." "bmi270_sensorapi"
    REQUIRES ${app_requires}
)
