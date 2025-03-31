package gb_emu

import "core:testing"
import "core:log"

@(test)
write_read_registers :: proc(t: ^testing.T) {
    cpu := CPU{}

    write_register_low(&cpu.bc, u8(42))
    write_register_high(&cpu.bc, u8(24))
    write_register_high(&cpu.de, u8(255))
    write_register_high(&cpu.hl, u8(128))
    write_register_low(&cpu.hl, u8(55))

    testing.expect_value(t, read_register_low(cpu.bc), u8(42))
    testing.expect_value(t, read_register_high(cpu.bc), u8(24))
    testing.expect_value(t, read_register_high(cpu.de), u8(255))
    testing.expect_value(t, read_register_high(cpu.hl), u8(128))
    testing.expect_value(t, read_register_low(cpu.hl), u8(55))
}

@(test)
write_read_flags :: proc(t: ^testing.T) {
    cpu := CPU{}

    write_flag(&cpu, Flags.C, true)
    write_flag(&cpu, Flags.Z, true)

    testing.expect(t, read_flag(&cpu, Flags.C))
    testing.expect(t, read_flag(&cpu, Flags.Z))
    testing.expect(t, !read_flag(&cpu, Flags.N))
    testing.expect(t, !read_flag(&cpu, Flags.H))

    write_flag(&cpu, Flags.Z, false)
    write_flag(&cpu, Flags.H, true)

    testing.expect(t, read_flag(&cpu, Flags.C))
    testing.expect(t, !read_flag(&cpu, Flags.Z))
    testing.expect(t, !read_flag(&cpu, Flags.N))
    testing.expect(t, read_flag(&cpu, Flags.H))
}

@(test)
inc_registers :: proc(t: ^testing.T) {
    cpu := CPU{}

    write_register_high(&cpu.hl, 42)
    inc_register_high(&cpu, &cpu.hl) // no HC increment

    testing.expect_value(t, read_register_high(cpu.hl), u8(43))

    testing.expect(t, !read_flag(&cpu, Flags.C))
    testing.expect(t, !read_flag(&cpu, Flags.Z))
    testing.expect(t, !read_flag(&cpu, Flags.N))
    testing.expect(t, !read_flag(&cpu, Flags.H))

    write_register_low(&cpu.de, 15)
    inc_register_low(&cpu, &cpu.de) // HC increment

    testing.expect_value(t, read_register_low(cpu.de), u8(16))

    testing.expect(t, !read_flag(&cpu, Flags.C))
    testing.expect(t, !read_flag(&cpu, Flags.Z))
    testing.expect(t, !read_flag(&cpu, Flags.N))
    testing.expect(t, read_flag(&cpu, Flags.H))
}

@(test)
add_to_registers :: proc(t: ^testing.T) {
    cpu := CPU{}

    cpu.bc = 42

    add_to_register(&cpu, &cpu.bc, 100) // no carry or half carry

    testing.expect_value(t, cpu.bc, 142)

    testing.expect(t, !read_flag(&cpu, Flags.C))
    testing.expect(t, !read_flag(&cpu, Flags.Z))
    testing.expect(t, !read_flag(&cpu, Flags.N))
    testing.expect(t, !read_flag(&cpu, Flags.H))

    cpu.de = 0xFFF0

    add_to_register(&cpu, &cpu.de, 0x1A)

    testing.expect_value(t, cpu.de, 0xA) // wrapped around, carry and half carry

    testing.expect(t, read_flag(&cpu, Flags.C))
    testing.expect(t, !read_flag(&cpu, Flags.Z))
    testing.expect(t, !read_flag(&cpu, Flags.N))
    testing.expect(t, read_flag(&cpu, Flags.H))

    add_to_register_high(&cpu, &cpu.hl, 0xF)
    add_to_register_low(&cpu, &cpu.hl, 0xFA)

    testing.expect_value(t, read_register_high(cpu.hl), 0xF) 
    testing.expect_value(t, read_register_low(cpu.hl), 0xFA) 

    testing.expect(t, !read_flag(&cpu, Flags.C))
    testing.expect(t, !read_flag(&cpu, Flags.Z))
    testing.expect(t, !read_flag(&cpu, Flags.N))
    testing.expect(t, !read_flag(&cpu, Flags.H))

    add_to_register_high(&cpu, &cpu.hl, 0x3)

    testing.expect_value(t, read_register_high(cpu.hl), 0x12) 
    testing.expect_value(t, read_register_low(cpu.hl), 0xFA)

    testing.expect(t, !read_flag(&cpu, Flags.C))
    testing.expect(t, !read_flag(&cpu, Flags.Z))
    testing.expect(t, !read_flag(&cpu, Flags.N))
    testing.expect(t, read_flag(&cpu, Flags.H))

    add_to_register_low(&cpu, &cpu.hl, 0x8)

    testing.expect_value(t, read_register_low(cpu.hl), 0x2)

    testing.expect(t, read_flag(&cpu, Flags.C))
    testing.expect(t, !read_flag(&cpu, Flags.Z))
    testing.expect(t, !read_flag(&cpu, Flags.N))
    testing.expect(t, read_flag(&cpu, Flags.H))
}

@(test)
and_registers :: proc(t: ^testing.T) {
    cpu := CPU{}

    cpu.de = 0x0FFF

    and_register_low(&cpu, &cpu.de, 0xF)

    testing.expect_value(t, read_register_low(cpu.de), 0xF)

    testing.expect(t, !read_flag(&cpu, Flags.C))
    testing.expect(t, !read_flag(&cpu, Flags.Z))
    testing.expect(t, !read_flag(&cpu, Flags.N))
    testing.expect(t, read_flag(&cpu, Flags.H))

    and_register_high(&cpu, &cpu.de, 0xF0)

    testing.expect_value(t, read_register_high(cpu.de), 0)

    testing.expect(t, !read_flag(&cpu, Flags.C))
    testing.expect(t, read_flag(&cpu, Flags.Z))
    testing.expect(t, !read_flag(&cpu, Flags.N))
    testing.expect(t, read_flag(&cpu, Flags.H))
}

@(test)
data_to_ram :: proc(t: ^testing.T) {
    cpu := CPU{}
    mem: [0xFFFF]u8

    copy_to_ram(mem[:], 0xABCD, u8(42))
    copy_to_ram(mem[:], 0xDEF0, u16(0x4567))
    copy_to_high_ram(mem[:], u8(0x12), u8(128))

    testing.expect_value(t, u8(42), mem[0xABCD])
    testing.expect_value(t, u8(0x67), mem[0xDEF0])
    testing.expect_value(t, u8(0x45), mem[0xDEF0 + 1])
    testing.expect_value(t, u8(128), mem[0xFF12])
}

@(test)
rlca :: proc(t: ^testing.T) {
    cpu := CPU{}
    mem: [0xFFFF]u8

    write_register_high(&cpu.af, 0x80)
    RLCA_0x07(&cpu, mem[:], 0)

    testing.expect_value(t, u8(0), read_register_high(cpu.af))
    testing.expect(t, read_flag(&cpu, Flags.C))

    write_register_high(&cpu.af, 0x2A)
    RLCA_0x07(&cpu, mem[:], 0)

    testing.expect_value(t, u8(0x2A << 1), read_register_high(cpu.af))
    testing.expect(t, !read_flag(&cpu, Flags.C))
}

@(test)
rrca :: proc(t: ^testing.T) {
    cpu := CPU{}
    mem: [0xFFFF]u8

    write_register_high(&cpu.af, 0x80)
    RRCA_0x0f(&cpu, mem[:], 0)

    testing.expect_value(t, u8(0x80 >> 1), read_register_high(cpu.af))
    testing.expect(t, !read_flag(&cpu, Flags.C))

    write_register_high(&cpu.af, 0x2B)
    RRCA_0x0f(&cpu, mem[:], 0)

    testing.expect_value(t, u8(0x2B >> 1), read_register_high(cpu.af))
    testing.expect(t, read_flag(&cpu, Flags.C))
}

@(test)
rla :: proc(t: ^testing.T) {
    cpu := CPU{}
    mem: [0xFFFF]u8

    write_flag(&cpu, Flags.C, true)
    write_register_high(&cpu.af, 0x80)

    RLA_0x17(&cpu, mem[:], 0)

    testing.expect_value(t, u8(1), read_register_high(cpu.af))
    testing.expect(t, read_flag(&cpu, Flags.C))
}

@(test)
rra :: proc(t: ^testing.T) {
    cpu := CPU{}
    mem: [0xFFFF]u8

    write_flag(&cpu, Flags.C, true)
    write_register_high(&cpu.af, 0x2A)

    RRA_0x1f(&cpu, mem[:], 0)

    testing.expect_value(t, u8(0x95), read_register_high(cpu.af))
    testing.expect(t, !read_flag(&cpu, Flags.C))

    RRA_0x1f(&cpu, mem[:], 0)

    testing.expect_value(t, u8(0x4A), read_register_high(cpu.af))
    testing.expect(t, read_flag(&cpu, Flags.C))
}

@(test)
stack :: proc(t: ^testing.T) {
    cpu := CPU{}
    mem: [0xFFFF]u8

    cpu_init(&cpu)

    cpu.de = 0x42
    cpu.bc = 0x24

    push_register(&cpu, mem[:], cpu.de)
    push_register(&cpu, mem[:], cpu.bc)

    pop_register(&cpu, mem[:], &cpu.hl)
    testing.expect_value(t, cpu.hl, 0x24)
    pop_register(&cpu, mem[:], &cpu.hl)
    testing.expect_value(t, cpu.hl, 0x42)

    cpu.pc = 0x1010

    call(&cpu, mem[:], 0x2020)
    testing.expect_value(t, cpu.pc, 0x2020)
    call(&cpu, mem[:], 0x2030)
    testing.expect_value(t, cpu.pc, 0x2030)
    call(&cpu, mem[:], 0x2040)
    testing.expect_value(t, cpu.pc, 0x2040)
    ret(&cpu, mem[:])
    testing.expect_value(t, cpu.pc, 0x2030)
    ret(&cpu, mem[:])
    testing.expect_value(t, cpu.pc, 0x2020)
    ret(&cpu, mem[:])
    testing.expect_value(t, cpu.pc, 0x1010)
}
