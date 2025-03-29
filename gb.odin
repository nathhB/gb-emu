package gb_emu

GB :: struct {
    af: u16,
    bc: u16,
    de: u16,
    hl: u16,
    sp: u16,
    pc: u16,
    mem: [0xFFFF]u8
}

gb_init :: proc(gb: ^GB) {
    gb.pc = 0x100
    gb.sp = 0xFFFE
}
