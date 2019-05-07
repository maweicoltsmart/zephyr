# SPDX-License-Identifier: Apache-2.0

# See root CMakeLists.txt for description and expectations of these macros

macro(toolchain_ld_base)

  # TOOLCHAIN_LD_FLAGS comes from compiler/gcc/target.cmake
  # LINKERFLAGPREFIX comes from linker/ld/target.cmake
  zephyr_ld_options(
    ${TOOLCHAIN_LD_FLAGS}
    ${LINKERFLAGPREFIX},--gc-sections
    ${LINKERFLAGPREFIX},--build-id=none
  )

  # Sort the common symbols and each input section by alignment
  # in descending order to minimize padding between these symbols.
  zephyr_ld_option_ifdef(
    CONFIG_LINKER_SORT_BY_ALIGNMENT
    ${LINKERFLAGPREFIX},--sort-common=descending
    ${LINKERFLAGPREFIX},--sort-section=alignment
  )

endmacro()
