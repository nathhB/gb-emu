package gb_emu

import "core:fmt"

// https://gbdev.io/pandocs/nombc.html

mbc0_init :: proc(mem: ^GB_Memory, rom: []u8) {
    mem.rom = rom
    mem.write = mbc0_write
    mem.read = mbc0_read
    mem.get_ptr = mbc0_get_ptr 
}

mbc0_write :: proc(mem: ^GB_Memory, addr: u16, byte: u8) {
    if addr >= 0x8000 {
        mem.data[addr] = byte

        return
    }

    if !(addr >= 2000 && addr <= 0x3FFF) {
        panic(fmt.aprintf("Attempted to write into ROM at addr %x (value: %x)", addr, byte))
    }
}

mbc0_read :: proc(mem: ^GB_Memory, addr: u16) -> u8 {
    return mem.data[addr]
}

mbc0_get_ptr :: proc(mem: ^GB_Memory, addr: u16) -> ^u8 {
    if addr < 8000 {
        panic("Tried to get a pointer to ROM")
    }

    return &mem.data[addr]
}
