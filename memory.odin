package gb_emu

GB_Memory :: struct {
    data: [0xFFFF+1]u8,
    rom: []u8,
    rom_bank: int,
    read: proc(mem: ^GB_Memory, addr: u16) -> u8,
    write: proc(mem: ^GB_Memory, addr: u16, byte: u8),
    get_ptr: proc(mem: ^GB_Memory, addr: u16) -> ^u8
}

mem_write :: proc(mem: ^GB_Memory, addr: u16, byte: u8) {
    mem.write(mem, addr, byte)
}

mem_read :: proc(mem: ^GB_Memory, addr: u16) -> u8 {
    return mem.read(mem, addr)
}

mem_get_ptr :: proc(mem: ^GB_Memory, addr: u16) -> ^u8 {
    return mem.get_ptr(mem, addr)
}
