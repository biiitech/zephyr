if(CONFIG_BOARD_CH572DK)
  # Add board-specific source files
  zephyr_library()
  zephyr_library_sources(peripheral_init.c)
endif()