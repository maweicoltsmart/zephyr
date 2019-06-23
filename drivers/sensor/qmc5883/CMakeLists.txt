# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_QMC5883 qmc5883.c)
zephyr_library_sources_ifdef(CONFIG_QMC5883_TRIGGER qmc5883_trigger.c)
