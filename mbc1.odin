package gb_emu

// https://gbdev.io/pandocs/MBC1.html

mbc1_init :: proc(mem: ^GB_Memory, rom: []u8) {
    mem.rom = rom
    mem.write_proc = mbc1_write
    mem.read_proc = mbc1_read
    mem.rom_bank = 1 // ROM bank for the 0x4000 - 0x7FFF region
}

mbc1_write :: proc(mem: ^GB_Memory, addr: u16, byte: u8) {
    if addr > 0x7FFF {
        // write to RAM
        mem.data[addr] = byte
    }

    if addr >= 2000 && addr <= 0x3FFF {
        // writing to ROM bank number register
        mem.rom_bank = get_rom_bank_number(byte)
    } else {
        panic("attempted to write to RO memory")
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

get_rom_bank_number :: proc(byte: u8) -> int {
    if byte == 0 {
        return 1
    }

    // only consider the 5 lower bits
    return int(byte & 0b00011111)
}
