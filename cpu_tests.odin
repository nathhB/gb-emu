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
    mem: [0xFFFF+1]u8

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
    mem: [0xFFFF+1]u8

    write_register_high(&cpu.af, 0x80)
    RLCA_0x07(&cpu, mem[:], 0)

    testing.expect_value(t, u8(1), read_register_high(cpu.af))
    testing.expect(t, read_flag(&cpu, Flags.C))

    write_register_high(&cpu.af, 0x2A)
    RLCA_0x07(&cpu, mem[:], 0)

    testing.expect_value(t, u8(0x2A * 2), read_register_high(cpu.af))
    testing.expect(t, !read_flag(&cpu, Flags.C))
}

@(test)
rrca :: proc(t: ^testing.T) {
    cpu := CPU{}
    mem: [0xFFFF+1]u8

    write_register_high(&cpu.af, 0x43)
    RRCA_0x0f(&cpu, mem[:], 0)

    testing.expect_value(t, u8(0xA1), read_register_high(cpu.af))
    testing.expect(t, read_flag(&cpu, Flags.C))

    write_register_high(&cpu.af, 0x42)
    RRCA_0x0f(&cpu, mem[:], 0)

    testing.expect_value(t, u8(0x21), read_register_high(cpu.af))
    testing.expect(t, !read_flag(&cpu, Flags.C))
}

@(test)
rla :: proc(t: ^testing.T) {
    cpu := CPU{}
    mem: [0xFFFF+1]u8

    write_flag(&cpu, Flags.C, true)
    write_register_high(&cpu.af, 0x80)

    RLA_0x17(&cpu, mem[:], 0)

    testing.expect_value(t, u8(1), read_register_high(cpu.af))
    testing.expect(t, read_flag(&cpu, Flags.C))
}

@(test)
rra :: proc(t: ^testing.T) {
    cpu := CPU{}
    mem: [0xFFFF+1]u8

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
sra_srl :: proc(t: ^testing.T) {
    cpu := CPU{}
    mem: [0xFFFF+1]u8

    v: u8 = 0xA4

    sra(&cpu, &v)
    testing.expect_value(t, v, 0xD2)

    v = 0xA4

    srl(&cpu, &v)
    testing.expect_value(t, v, 0x52)
}

@(test)
swap_bytes :: proc(t: ^testing.T) {
    cpu := CPU{}
    v: u8 = 0x42

    swap(&cpu, &v)

    testing.expect_value(t, v, u8(0x24))
    testing.expect(t, !read_flag(&cpu, Flags.Z))
    testing.expect(t, !read_flag(&cpu, Flags.N))
    testing.expect(t, !read_flag(&cpu, Flags.H))
    testing.expect(t, !read_flag(&cpu, Flags.C))
}

@(test)
bit_is_set :: proc(t: ^testing.T) {
    cpu := CPU{}
    v: u8 = 0x42

    bit(&cpu, &v, 0)
    testing.expect(t, !read_flag(&cpu, Flags.Z))

    bit(&cpu, &v, 1)
    testing.expect(t, read_flag(&cpu, Flags.Z))

    bit(&cpu, &v, 6)
    testing.expect(t, read_flag(&cpu, Flags.Z))

    bit(&cpu, &v, 7)
    testing.expect(t, !read_flag(&cpu, Flags.Z))
}

@(test)
reset_bit :: proc(t: ^testing.T) {
    cpu := CPU{}
    v: u8 = 0x42

    res(&cpu, &v, 1)
    res(&cpu, &v, 6)
    res(&cpu, &v, 7)

    bit(&cpu, &v, 1)
    testing.expect(t, !read_flag(&cpu, Flags.Z))
    bit(&cpu, &v, 6)
    testing.expect(t, !read_flag(&cpu, Flags.Z))
    bit(&cpu, &v, 7)
    testing.expect(t, !read_flag(&cpu, Flags.Z))
}

@(test)
set_bit :: proc(t: ^testing.T) {
    cpu := CPU{}
    v: u8 = 0

    set(&cpu, &v, 1)
    set(&cpu, &v, 6)
    set(&cpu, &v, 7)

    bit(&cpu, &v, 1)
    testing.expect(t, read_flag(&cpu, Flags.Z))
    bit(&cpu, &v, 6)
    testing.expect(t, read_flag(&cpu, Flags.Z))
    bit(&cpu, &v, 7)
    testing.expect(t, read_flag(&cpu, Flags.Z))
}

@(test)
stack :: proc(t: ^testing.T) {
    cpu := CPU{}
    mem: [0xFFFF+1]u8

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
@(test)
exec_instructions :: proc(t: ^testing.T) {
    cpu := CPU{}
    mem: [0xFFFF+1]u8

    cpu_init(&cpu)

    // write 0x01 instruction in memory
    mem[cpu.pc] = 0x01
    mem[cpu.pc + 1] = 0x10
    mem[cpu.pc + 2] = 0x42
    // write 0x03 instruction in memory
    mem[cpu.pc + 3] = 0x03

    cpu_tick(&cpu, mem[:])

    testing.expect_value(t, cpu.exec_cyles, 11)
    testing.expect_value(t, cpu.bc, 0x4210)

    // 0x03 instruction should only be executed after the 12 cyles of instruction 0x01 are done

    run_cpu(&cpu, mem[:], 5)

    testing.expect_value(t, cpu.exec_cyles, 6)
    testing.expect_value(t, cpu.bc, 0x4210)

    run_cpu(&cpu, mem[:], 6)

    // 0x01 is done at this point, 0x03 should be executed next tick
    testing.expect_value(t, cpu.exec_cyles, 0)
    testing.expect_value(t, cpu.bc, 0x4210)

    cpu_tick(&cpu, mem[:])

    testing.expect_value(t, cpu.exec_cyles, 7)
    testing.expect_value(t, cpu.bc, 0x4211) // incremented
}

@(test)
ei :: proc(t: ^testing.T) {
    cpu := CPU{}
    mem: [0xFFFF+1]u8

    cpu_init(&cpu)

    testing.expect_value(t, cpu.enable_interrupts, false)

    mem[cpu.pc] = 0xFB // EI
    mem[cpu.pc + 1] = 0x00 // NOP
    mem[cpu.pc + 2] = 0x00 // NOP

    cpu_tick(&cpu, mem[:])

    // EI instruction takes 4 t cycles
    testing.expect_value(t, cpu.exec_cyles, 3)
    testing.expect_value(t, cpu.enable_interrupts, false)

    run_cpu(&cpu, mem[:], 3)

    testing.expect_value(t, cpu.enable_interrupts, true)
    // IME should still be false at this point, it should only be set after
    // the next instruction
    testing.expect_value(t, cpu.ime, false)

    // execute NOP right after EI
    run_cpu(&cpu, mem[:], 4)

    testing.expect_value(t, cpu.ime, true)
    testing.expect_value(t, cpu.enable_interrupts, false)
}

@(test)
interrupts :: proc(t: ^testing.T) {
    cpu := CPU{}
    mem: [0xFFFF+1]u8

    cpu_init(&cpu)

    cpu.sp = 0xFFFE
    mem[0xFFFF] = 0xFF // IE = 0xFF, allow all interrupts

    // bunch of NOPs
    mem[cpu.pc] = 0x00
    mem[cpu.pc + 1] = 0x00
    mem[cpu.pc + 2] = 0x00
    mem[cpu.pc + 3] = 0x00
    mem[0x40] = 0xd9 // RETI in VBlank handler

    cpu_request_interrupt(&cpu, mem[:], Interrupt.VBlank)
    cpu_request_interrupt(&cpu, mem[:], Interrupt.Joypad)

    testing.expect(t, mem[0xFF0F] & (1 << u8(Interrupt.VBlank)) > 0)
    testing.expect(t, mem[0xFF0F] & (1 << u8(Interrupt.Joypad)) > 0)

    cpu_tick(&cpu, mem[:]) // not executing the interrupt because IME is not set

    testing.expect_value(t, cpu.state, CpuState.ExecuteInstruction)
    testing.expect_value(t, cpu.exec_cyles, 3)

    cpu.ime = true // set IME, the interrupt should be executed after the current NOP
    run_cpu(&cpu, mem[:], 3)

    pc_before_interreupt := cpu.pc

    testing.expect_value(t, cpu.state, CpuState.Fetch)

    cpu_tick(&cpu, mem[:])

    testing.expect_value(t, cpu.state, CpuState.ExecuteInterrupt)
    testing.expect_value(t, cpu.ime, false)
    testing.expect(t, mem[0xFF0F] & (1 << u8(Interrupt.VBlank)) == 0)
    testing.expect(t, mem[0xFF0F] & (1 << u8(Interrupt.Joypad)) > 0)
    testing.expect_value(t, cpu.exec_cyles, 20)

    run_cpu(&cpu, mem[:], 20)

    // done executing interrupt
    testing.expect_value(t, cpu.state, CpuState.Fetch)
    testing.expect_value(t, cpu.ime, false)
    testing.expect_value(t, cpu.pc, 0x40)

    // execute instruction at handler's address (RETI)
    // RETI executes a RET and reenables interrupts

    run_cpu(&cpu, mem[:], 16)

    testing.expect_value(t, cpu.state, CpuState.Fetch)
    testing.expect_value(t, cpu.pc, pc_before_interreupt)
    testing.expect_value(t, cpu.ime, true)
    testing.expect(t, mem[0xFF0F] & (1 << u8(Interrupt.VBlank)) == 0)
    testing.expect(t, mem[0xFF0F] & (1 << u8(Interrupt.Joypad)) > 0)

    run_cpu(&cpu, mem[:], 21)

    testing.expect_value(t, cpu.state, CpuState.Fetch)
    testing.expect_value(t, cpu.pc, 0x60) // Joypad interrupt handler
}

@(private="file")
run_cpu :: proc(cpu: ^CPU, mem: []u8, ticks: int) {
    for i := 0; i < ticks; i += 1 {
        cpu_tick(cpu, mem[:])
    }
}
