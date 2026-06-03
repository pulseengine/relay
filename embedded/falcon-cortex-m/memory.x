/* STM32H743 (Cortex-M7, the FMU-class MCU from the falcon roadmap). */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM   : ORIGIN = 0x24000000, LENGTH = 512K   /* AXI SRAM */
}
