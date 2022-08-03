if("${BOARD}" STREQUAL "lpcxpresso54114_m4")
  set(BOARD_REMOTE "lpcxpresso54114_m0")
elseif("${BOARD}" STREQUAL "lpcxpresso55s69_cpu0")
  set(BOARD_REMOTE "lpcxpresso55s69_cpu1")
elseif("${BOARD}" STREQUAL "mps2_an521")
  set(QEMU_EXTRA_FLAGS "-device;loader,file=${REMOTE_ZEPHYR_DIR}/zephyr.elf")
  set(BOARD_REMOTE "mps2_an521_remote")
elseif("${BOARD}" STREQUAL "v2m_musca_b1")
  set(BOARD_REMOTE "v2m_musca_b1_ns")
elseif("${BOARD}" STREQUAL "mimxrt1170_evk_cm7")
  set(BOARD_REMOTE "mimxrt1170_evk_cm4")
else()
  message(FATAL_ERROR "${BOARD} was not supported for this sample")
endif()
message(STATUS "${BOARD} compile as Master in this sample")

ExternalZephyrProject_Add(
  APPLICATION openamp_remote
  SOURCE_DIR ${APP_DIR}/remote
  BOARD ${BOARD_REMOTE}
)

# Add a dependency to ensure that the openamp remote image is built first
add_dependencies(openamp openamp_remote)

# Place the remote openamp image first in the image list, so it will be flashed
# first
set(IMAGES "openamp_remote" ${IMAGES})
