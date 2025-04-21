package gb_emu

import "core:fmt"

// https://gbdev.io/pandocs/MBC1.html

mbc1_init :: proc(mem: ^GB_Memory, rom: []u8) {
    mem.rom = rom
    mem.write = mbc1_write
    mem.read = mbc1_read
    mem.get_ptr = mbc1_get_ptr
    mem.rom_bank = 1 // ROM bank for the 0x4000 - 0x7FFF region
}

mbc1_write :: proc(mem: ^GB_Memory, addr: u16, byte: u8) {
    if addr >= 0x8000 {
        // write to RAM
        mem.data[addr] = byte

        return
    }

    if addr >= 2000 && addr <= 0x3FFF {
        // writing to ROM bank number register
        mem.rom_bank = get_rom_bank_number(byte)
    } else {
        panic(fmt.aprintf("Attempted to write into ROM at addr %x (value: %x)", addr, byte))
    }
}

mbc1_read :: proc(mem: ^GB_Memory, addr: u16) -> u8 {
    if addr <= 0x3FFF {
        return mem.data[addr]
    }

    bank_addr := u16(mem.rom_bank * 0x4000)
    bank_offset := addr - 0x4000

    return mem.data[bank_addr + bank_offset]
}

mbc1_get_ptr :: proc(mem: ^GB_Memory, addr: u16) -> ^u8 {
    if addr < 8000 {
        panic("Tried to get a pointer to ROM")
    }

    return &mem.data[addr]
}

get_rom_bank_number :: proc(byte: u8) -> int {
    if byte == 0 {
        return 1
    }

    // only consider the 5 lower bits
    return int(byte & 0b00011111)
}
