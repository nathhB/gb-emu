package gb_emu

// https://gbdev.io/pandocs/nombc.html

mbc0_init :: proc(mem: ^GB_Memory, rom: []u8) {
    mem.rom = rom
    mem.write_proc = mbc0_write
    mem.read_proc = mbc0_read

    // mapped the full rom to 0 - 0x7FFF

    copy(mem.data[:], rom)
}

mbc0_write :: proc(mem: ^GB_Memory, addr: u16, byte: u8) {
    mem.data[addr] = byte
}

mbc0_read :: proc(mem: ^GB_Memory, addr: u16) -> u8 {
    return mem.data[addr]
}
