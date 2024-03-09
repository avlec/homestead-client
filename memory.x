MEMORY {
    BOOT2  : ORIGIN = 0x10000000, LENGTH = 0x100
    /* application flash section remainder of the flash storage */
    /* after the bootloader and the wifi firmware */
    FLASH  : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100 - 256K
    /* wifi binary blob sections 256K in total length */
    /* two different blobs that are stored seperately */
    CYWFW  : ORIGIN = 0x101C0000, LENGTH = 0x38400
    CYWCLM : ORIGIN = 0x101f8400, LENGTH = 0x07c00
    RAM    : ORIGIN = 0x20000000, LENGTH = 256K
}
