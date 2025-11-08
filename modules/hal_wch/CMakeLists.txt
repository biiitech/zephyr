if(CONFIG_SOC_FAMILY_CH32V OR CONFIG_SOC_FAMILY_CH5XX)
  zephyr_include_directories(${ZEPHYR_HAL_WCH_MODULE_DIR}/ch32fun .)
endif()
