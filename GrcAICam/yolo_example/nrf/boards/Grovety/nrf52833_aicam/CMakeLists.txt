# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BOARD_NRF52833_AICAM)
  zephyr_compile_definitions(DEVELOP_IN_NRF52833)
  zephyr_compile_definitions(NRFX_COREDEP_DELAY_US_LOOP_CYCLES=3)
endif()
