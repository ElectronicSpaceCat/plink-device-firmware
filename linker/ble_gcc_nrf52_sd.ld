/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

/* 
 * SD  = Softdevice
 * 1kB = 1024 bytes
 * 
 * SoC: nRF52832-CIAA
 * ---------------------------------
 * TOTAL_FLASH_SIZE = 512kB (0x80000)
 * TOTAL_RAM_SIZE = 64kB (0x10000)
 * 
 * MBR uses  4 kB (0x1000  bytes) flash according to MBR_SIZE      defined in ${SDK_ROOT}\components\softdevice\s132\headers\nrf52\nrf_mbr.h
 * SD  uses 96 kB (0x18000 bytes) flash according to SD_FLASH_SIZE defined in ${SDK_ROOT}\components\softdevice\s132\headers\nrf_sdm.h
 * Therefore the SD uses (96 kB (0x18000 bytes) + 4 kB (0x1000  bytes)) = 100kB (0x19000 bytes)
 *
 * The SD flash size can also be referenced in:
 * ${SDK_ROOT}components/softdevice/s132/doc/s112_nrf52_7.2.0_release-notes.pdf under SD Properties
 * 
 * According to the release-notes:
 * "RAM: 3.7 kB (0x0EB8 bytes). This is the minimum required memory."
 * "The worst-case stack usage for the SD is 1.75 kB (0x0700 bytes)."
 * 
 * According to the release-notes:
 * "The Firmware ID of this SoftDevice is 0x0103."
 * 
 * Our app flash size if determined by (BOOTLOADER_FLASH_ORIGIN - APP_ORIGIN)
 * so if the bootloader starts at 0x72000
 * then our app size would be (0x72000 - 0x19000) = 0x59000
 * 
 * FLASH layout for above example:
 * 0x7F000 - 0x80000  (4   kB) Bootloader settings
 * 0x7E000 - 0x7F000  (4   kB) MBR parameter storage
 * 0x72000 - 0x7E000  (48  kB) Bootloader area
 * 0x19000 - 0x72000  (356 kB) Application area
 * 0x01000 - 0x19000  (96  kB) SoftDevice
 * 0x00000 - 0x01000  (4   kB) Master Boot Record (MBR)
 *
 */

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00019000, LENGTH = 0x59000 /* 0x58000 if using bootloader_debug */
  RAM (rwx) :  ORIGIN = 0x20001A70, LENGTH = 0xE590  /* The debugger will tell how much to use if this is wrong */
  uicr_bootloader_start_address (r) : ORIGIN = 0x10001014, LENGTH = 0x4 /* Must match address defined in bootloader linker */
}

SECTIONS
{
  . = ALIGN(4);
  .uicr_bootloader_start_address :
  {
    PROVIDE(__start_uicr_bootloader_start_address = .);
    KEEP(*(SORT(.uicr_bootloader_start_address*)))
    PROVIDE(__stop_uicr_bootloader_start_address = .);
  } > uicr_bootloader_start_address
}

SECTIONS
{
  . = ALIGN(4);
  .mem_section_dummy_ram :
  {
  }
  .log_dynamic_data :
  {
    PROVIDE(__start_log_dynamic_data = .);
    KEEP(*(SORT(.log_dynamic_data*)))
    PROVIDE(__stop_log_dynamic_data = .);
  } > RAM
  .log_filter_data :
  {
    PROVIDE(__start_log_filter_data = .);
    KEEP(*(SORT(.log_filter_data*)))
    PROVIDE(__stop_log_filter_data = .);
  } > RAM
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM

} INSERT AFTER .data;

SECTIONS
{
  .mem_section_dummy_rom :
  {
  }
  .sdh_ble_observers :
  {
    PROVIDE(__start_sdh_ble_observers = .);
    KEEP(*(SORT(.sdh_ble_observers*)))
    PROVIDE(__stop_sdh_ble_observers = .);
  } > FLASH
  .sdh_soc_observers :
  {
    PROVIDE(__start_sdh_soc_observers = .);
    KEEP(*(SORT(.sdh_soc_observers*)))
    PROVIDE(__stop_sdh_soc_observers = .);
  } > FLASH
  .pwr_mgmt_data :
  {
    PROVIDE(__start_pwr_mgmt_data = .);
    KEEP(*(SORT(.pwr_mgmt_data*)))
    PROVIDE(__stop_pwr_mgmt_data = .);
  } > FLASH
  .log_const_data :
  {
    PROVIDE(__start_log_const_data = .);
    KEEP(*(SORT(.log_const_data*)))
    PROVIDE(__stop_log_const_data = .);
  } > FLASH
  .sdh_req_observers :
  {
    PROVIDE(__start_sdh_req_observers = .);
    KEEP(*(SORT(.sdh_req_observers*)))
    PROVIDE(__stop_sdh_req_observers = .);
  } > FLASH
  .sdh_state_observers :
  {
    PROVIDE(__start_sdh_state_observers = .);
    KEEP(*(SORT(.sdh_state_observers*)))
    PROVIDE(__stop_sdh_state_observers = .);
  } > FLASH
  .sdh_stack_observers :
  {
    PROVIDE(__start_sdh_stack_observers = .);
    KEEP(*(SORT(.sdh_stack_observers*)))
    PROVIDE(__stop_sdh_stack_observers = .);
  } > FLASH
  .log_backends :
  {
    PROVIDE(__start_log_backends = .);
    KEEP(*(SORT(.log_backends*)))
    PROVIDE(__stop_log_backends = .);
  } > FLASH
    .nrf_balloc :
  {
    PROVIDE(__start_nrf_balloc = .);
    KEEP(*(.nrf_balloc))
    PROVIDE(__stop_nrf_balloc = .);
  } > FLASH

} INSERT AFTER .text


INCLUDE "nrf_common.ld"
