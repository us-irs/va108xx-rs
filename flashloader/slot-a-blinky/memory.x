/* Special linker script for application slot A with an offset at address 0x3000 */
MEMORY
{
	FLASH : ORIGIN = 0x00003000, LENGTH = 0xE7FC
	RAM : ORIGIN = 0x10000000, LENGTH = 0x08000 /* 32K */
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* NOTE Do NOT modify `_stack_start` unless you know what you are doing */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
