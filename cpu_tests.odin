package gb_emu

import "core:testing"
import "core:log"

@(test)
write_read_registers :: proc(t: ^testing.T) {
    gb := GB{}

    write_register_low(&gb.bc, u8(42))
    write_register_high(&gb.bc, u8(24))
    write_register_high(&gb.de, u8(255))
    write_register_high(&gb.hl, u8(128))
    write_register_low(&gb.hl, u8(55))

    testing.expect_value(t, read_register_low(gb.bc), u8(42))
    testing.expect_value(t, read_register_high(gb.bc), u8(24))
    testing.expect_value(t, read_register_high(gb.de), u8(255))
    testing.expect_value(t, read_register_high(gb.hl), u8(128))
    testing.expect_value(t, read_register_low(gb.hl), u8(55))
}

@(test)
write_read_flags :: proc(t: ^testing.T) {
    gb := GB{}

    write_flag(&gb, Flags.C, true)
    write_flag(&gb, Flags.Z, true)

    testing.expect(t, read_flag(&gb, Flags.C))
    testing.expect(t, read_flag(&gb, Flags.Z))
    testing.expect(t, !read_flag(&gb, Flags.N))
    testing.expect(t, !read_flag(&gb, Flags.H))

    write_flag(&gb, Flags.Z, false)
    write_flag(&gb, Flags.H, true)

    testing.expect(t, read_flag(&gb, Flags.C))
    testing.expect(t, !read_flag(&gb, Flags.Z))
    testing.expect(t, !read_flag(&gb, Flags.N))
    testing.expect(t, read_flag(&gb, Flags.H))
}

@(test)
inc_registers :: proc(t: ^testing.T) {
    gb := GB{}

    write_register_high(&gb.hl, 42)
    inc_register_high(&gb, &gb.hl) // no HC increment

    testing.expect_value(t, read_register_high(gb.hl), u8(43))

    testing.expect(t, !read_flag(&gb, Flags.C))
    testing.expect(t, !read_flag(&gb, Flags.Z))
    testing.expect(t, !read_flag(&gb, Flags.N))
    testing.expect(t, !read_flag(&gb, Flags.H))

    write_register_low(&gb.de, 15)
    inc_register_low(&gb, &gb.de) // HC increment

    testing.expect_value(t, read_register_low(gb.de), u8(16))

    testing.expect(t, !read_flag(&gb, Flags.C))
    testing.expect(t, !read_flag(&gb, Flags.Z))
    testing.expect(t, !read_flag(&gb, Flags.N))
    testing.expect(t, read_flag(&gb, Flags.H))
}

@(test)
add_to_registers :: proc(t: ^testing.T) {
    gb := GB{}

    gb.bc = 42

    add_to_register(&gb, &gb.bc, 100) // no carry or half carry

    testing.expect_value(t, gb.bc, 142)

    testing.expect(t, !read_flag(&gb, Flags.C))
    testing.expect(t, !read_flag(&gb, Flags.Z))
    testing.expect(t, !read_flag(&gb, Flags.N))
    testing.expect(t, !read_flag(&gb, Flags.H))

    gb.de = 0xFFF0

    add_to_register(&gb, &gb.de, 0x1A)

    testing.expect_value(t, gb.de, 0xA) // wrapped around, carry and half carry

    testing.expect(t, read_flag(&gb, Flags.C))
    testing.expect(t, !read_flag(&gb, Flags.Z))
    testing.expect(t, !read_flag(&gb, Flags.N))
    testing.expect(t, read_flag(&gb, Flags.H))

    add_to_register_high(&gb, &gb.hl, 0xF)
    add_to_register_low(&gb, &gb.hl, 0xFA)

    testing.expect_value(t, read_register_high(gb.hl), 0xF) 
    testing.expect_value(t, read_register_low(gb.hl), 0xFA) 

    testing.expect(t, !read_flag(&gb, Flags.C))
    testing.expect(t, !read_flag(&gb, Flags.Z))
    testing.expect(t, !read_flag(&gb, Flags.N))
    testing.expect(t, !read_flag(&gb, Flags.H))

    add_to_register_high(&gb, &gb.hl, 0x3)

    testing.expect_value(t, read_register_high(gb.hl), 0x12) 
    testing.expect_value(t, read_register_low(gb.hl), 0xFA)

    testing.expect(t, !read_flag(&gb, Flags.C))
    testing.expect(t, !read_flag(&gb, Flags.Z))
    testing.expect(t, !read_flag(&gb, Flags.N))
    testing.expect(t, read_flag(&gb, Flags.H))

    add_to_register_low(&gb, &gb.hl, 0x8)

    testing.expect_value(t, read_register_low(gb.hl), 0x2)

    testing.expect(t, read_flag(&gb, Flags.C))
    testing.expect(t, !read_flag(&gb, Flags.Z))
    testing.expect(t, !read_flag(&gb, Flags.N))
    testing.expect(t, read_flag(&gb, Flags.H))
}

@(test)
and_registers :: proc(t: ^testing.T) {
    gb := GB{}

    gb.de = 0x0FFF

    and_register_low(&gb, &gb.de, 0xF)

    testing.expect_value(t, read_register_low(gb.de), 0xF)

    testing.expect(t, !read_flag(&gb, Flags.C))
    testing.expect(t, !read_flag(&gb, Flags.Z))
    testing.expect(t, !read_flag(&gb, Flags.N))
    testing.expect(t, read_flag(&gb, Flags.H))

    and_register_high(&gb, &gb.de, 0xF0)

    testing.expect_value(t, read_register_high(gb.de), 0)

    testing.expect(t, !read_flag(&gb, Flags.C))
    testing.expect(t, read_flag(&gb, Flags.Z))
    testing.expect(t, !read_flag(&gb, Flags.N))
    testing.expect(t, read_flag(&gb, Flags.H))
}

@(test)
data_to_ram :: proc(t: ^testing.T) {
    gb := GB{}

    copy_to_ram(&gb, 0xABCD, u8(42))
    copy_to_ram(&gb, 0xDEF0, u16(0x4567))
    copy_to_high_ram(&gb, u8(0x12), u8(128))

    testing.expect_value(t, u8(42), gb.mem[0xABCD])
    testing.expect_value(t, u8(0x67), gb.mem[0xDEF0])
    testing.expect_value(t, u8(0x45), gb.mem[0xDEF0 + 1])
    testing.expect_value(t, u8(128), gb.mem[0xFF12])
}

@(test)
rlca :: proc(t: ^testing.T) {
    gb := GB{}

    write_register_high(&gb.af, 0x80)
    RLCA_0x07(&gb, 0)

    testing.expect_value(t, u8(0), read_register_high(gb.af))
    testing.expect(t, read_flag(&gb, Flags.C))

    write_register_high(&gb.af, 0x2A)
    RLCA_0x07(&gb, 0)

    testing.expect_value(t, u8(0x2A << 1), read_register_high(gb.af))
    testing.expect(t, !read_flag(&gb, Flags.C))
}

@(test)
rrca :: proc(t: ^testing.T) {
    gb := GB{}

    write_register_high(&gb.af, 0x80)
    RRCA_0x0f(&gb, 0)

    testing.expect_value(t, u8(0x80 >> 1), read_register_high(gb.af))
    testing.expect(t, !read_flag(&gb, Flags.C))

    write_register_high(&gb.af, 0x2B)
    RRCA_0x0f(&gb, 0)

    testing.expect_value(t, u8(0x2B >> 1), read_register_high(gb.af))
    testing.expect(t, read_flag(&gb, Flags.C))
}

@(test)
rla :: proc(t: ^testing.T) {
    gb := GB{}

    write_flag(&gb, Flags.C, true)
    write_register_high(&gb.af, 0x80)

    RLA_0x17(&gb, 0)

    testing.expect_value(t, u8(1), read_register_high(gb.af))
    testing.expect(t, read_flag(&gb, Flags.C))
}

@(test)
rra :: proc(t: ^testing.T) {
    gb := GB{}

    write_flag(&gb, Flags.C, true)
    write_register_high(&gb.af, 0x2A)

    RRA_0x1f(&gb, 0)

    testing.expect_value(t, u8(0x95), read_register_high(gb.af))
    testing.expect(t, !read_flag(&gb, Flags.C))

    RRA_0x1f(&gb, 0)

    testing.expect_value(t, u8(0x4A), read_register_high(gb.af))
    testing.expect(t, read_flag(&gb, Flags.C))
}
