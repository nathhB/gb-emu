package gb_emu

GB_Memory :: struct {
    data: [0xFFFF+1]u8,
    rom: []u8,
    rom_bank: int,
    read_proc: proc(mem: ^GB_Memory, addr: u16) -> u8,
    write_proc: proc(mem: ^GB_Memory, addr: u16, byte: u8)
}

mem_write :: proc(mem: ^GB_Memory, addr: u16, byte: u8) {
    mem.write_proc(mem, addr, byte)
}

mem_read :: proc(mem: ^GB_Memory, addr: u16) -> u8 {
    return mem.read_proc(mem, addr)
}
