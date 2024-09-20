VA108xx Bootloader Application
=======

This is the Rust version of the bootloader supplied by Vorago.

## Memory Map

The bootloader uses the following memory map:

| Address | Notes | Size |
| ------ | ---- |  ---- |
| 0x0 | Bootloader start | code up to 0x3FFC bytes |
| 0x2FFE | Bootloader CRC | word |
| 0x3000 | App image A start | code up to 0xE7F8 (~58K) bytes |
| 0x117F8 | App image A CRC check length | word |
| 0x117FC | App image A CRC check value | word |
| 0x11800 | App image B start | code up to 0xE7F8 (~58K) bytes |
| 0x1FFF8 | App image B CRC check length | word |
| 0x1FFFC | App image B CRC check value | word |
| 0x20000 | End of NVM | end  |

## Additional Information

This bootloader was specifically written for the REB1 board, so it assumes a M95M01 ST EEPROM
is used to load the application code. The bootloader will also delay for a configurable amount
of time before booting. This allows to catch the RTT printout, but should probably be disabled
for production firmware.

This bootloader does not provide tools to flash the NVM memory by itself. Instead, you can use
the [flashloader](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/flashloader)
application to perform this task using a CCSDS interface via a UART.

The bootloader performs the following steps:

1. The application will calculate the checksum of itself if the bootloader CRC is blank (all zeroes
   or all ones). If the CRC is not blank and the checksum check fails, it will immediately boot
   application image A. Otherwise, it proceeds to the next step.
2. Check the checksum of App A. If that checksum is valid, it will boot App A. If not, it will
   proceed to the next step.
3. Check the checksum of App B. If that checksum is valid, it will boot App B. If not, it will
   boot App A as the fallback image.

You could adapt and combine this bootloader with a non-volatile memory to select a prefered app
image, which would be a first step towards an updatable flight software.

Please note that you *MUST* compile the application at slot A and slot B with an appropriate
`memory.x` file where the base address of the `FLASH` was adapted according to the base address
shown in the memory map above. The memory files to do this were provided in the `scripts` folder.
