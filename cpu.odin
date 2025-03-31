package gb_emu

import "core:log"

Flags :: enum { Z, N, H, C }

CPU :: struct {
    af: u16,
    bc: u16,
    de: u16,
    hl: u16,
    sp: u16,
    pc: u16,
    instructions: [0xFF]Instruction,
    exec_cyles: int
}

Instruction :: struct {
    len: int,
    cycles: int,
    func: proc(cpu: ^CPU, mem: []u8, data: u16)
}

cpu_init :: proc(cpu: ^CPU) {
    cpu.instructions[0x00] = Instruction{1, 4, NOP_0x00}
    cpu.instructions[0x01] = Instruction{3, 12, LD_0x01}
    cpu.instructions[0x02] = Instruction{1, 8, LD_0x02}
    cpu.instructions[0x03] = Instruction{1, 8, INC_0x03}

    cpu.pc = 0x100
    cpu.sp = 0xFFFE
}

cpu_tick :: proc(cpu: ^CPU, mem: []u8) {
    if cpu.exec_cyles > 0 {
        cpu.exec_cyles -= 1

        return
    }

    op := mem[cpu.pc]
    instruction := cpu.instructions[op]

    cpu.pc += 1

    data: u16 = 0
    len := instruction.len - 1

    if len > 0 {
        data = fetch(cpu, mem, len)

        cpu.pc += u16(len)
    }

    instruction.func(cpu, mem, data)
    cpu.exec_cyles = instruction.cycles - 1
}

fetch :: proc(cpu: ^CPU, mem: []u8, len: int) -> u16 {
    assert(len == 1 || len == 2)

    data := u16(mem[cpu.pc])

    if len == 2 {
        data |= (u16(mem[cpu.pc + 1]) << 8)
    }

    return data
}

write_register_high :: proc(reg: ^u16, value: u8) {
    reg^ = (reg^ & 0x00FF) | (u16(value) << 8)
}

write_register_low :: proc(reg: ^u16, value: u8) {
    reg^ = (reg^ & 0xFF00) | u16(value)
}

read_register_high :: proc(reg: u16) -> u8 {
    return u8(reg >> 8)
}

read_register_low :: proc(reg: u16) -> u8 {
    return u8(reg & 0xFF)
}

write_flag :: proc(cpu: ^CPU, flag: Flags, value: bool) {
    flags := read_register_low(cpu.af)
    mask := u8(1 << (7 - uint(flag)))

    if value {
        write_register_low(&cpu.af, flags | mask)
    } else {
        write_register_low(&cpu.af, flags & ~mask)
    }
}

read_flag :: proc(cpu: ^CPU, flag: Flags) -> bool {
    flags := read_register_low(cpu.af)
    mask := u8(1 << (7 - uint(flag)))

    return (flags & mask) > 0
}

inc_register_high :: proc(cpu: ^CPU, reg: ^u16) {
    reg8 := read_register_high(reg^)

    inc_byte(cpu, &reg8)
    write_register_high(reg, reg8)
}

inc_register_low :: proc(cpu: ^CPU, reg: ^u16) {
    reg8 := read_register_low(reg^)

    inc_byte(cpu, &reg8)
    write_register_low(reg, reg8)
}

inc_byte :: proc(cpu: ^CPU, byte: ^u8) {
    hc := (((byte^ & 0xF) + 1) & 0x10) == 0x10

    byte^ += 1

    write_flag(cpu, Flags.Z, byte^ == 0)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, hc)
}

dec_register_high :: proc(cpu: ^CPU, reg: ^u16) {
    reg8 := read_register_high(reg^)

    dec_byte(cpu, &reg8)
    write_register_high(reg, reg8)
}

dec_register_low :: proc(cpu: ^CPU, reg: ^u16) {
    reg8 := read_register_low(reg^)

    dec_byte(cpu, &reg8)
    write_register_low(reg, reg8)
}

dec_byte :: proc(cpu: ^CPU, byte: ^u8) {
    hc := (((byte^ & 0xF) - 1) & 0x10) == 0x10

    byte^ -= 1

    write_flag(cpu, Flags.Z, byte^ == 0)
    write_flag(cpu, Flags.N, true)
    write_flag(cpu, Flags.H, hc)
}

add_to_register :: proc(cpu: ^CPU, reg: ^u16, value: u16) {
    c := u32(reg^) + u32(value) > 0xFFFF
    hc := (((reg^ & 0xFFF) + (value & 0xFFF)) & 0x1000) == 0x1000

    reg^ += value

    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.C, c)
    write_flag(cpu, Flags.H, hc)
}

add_to_register_high :: proc(cpu: ^CPU, reg: ^u16, value: u8, add_carry: bool = false) {
    reg8 := read_register_high(reg^) 

    add_to_byte(cpu, &reg8, value, add_carry)
    write_register_high(reg, reg8) 
}

add_to_register_low :: proc(cpu: ^CPU, reg: ^u16, value: u8, add_carry: bool = false) {
    reg8 := read_register_low(reg^) 

    add_to_byte(cpu, &reg8, value, add_carry)
    write_register_low(reg, reg8) 
}

add_to_byte :: proc(cpu: ^CPU, byte: ^u8, value: u8, add_carry: bool = false) {
    add := int(value)

    if add_carry && read_flag(cpu, Flags.C) {
        add += 1
    }

    c := int(byte^) + add > 0xFF
    hc := ((int(byte^ & 0xF) + (add & 0xF)) & 0x10) == 0x10

    byte^ += u8(add)
    
    write_flag(cpu, Flags.Z, byte^ == 0)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.C, c)
    write_flag(cpu, Flags.H, hc)
}

sub_from_register_high :: proc(cpu: ^CPU, reg: ^u16, value: u8, sub_carry: bool = false) {
    reg8 := read_register_high(reg^) 

    sub_from_byte(cpu, &reg8, value, sub_carry)
    write_register_high(reg, reg8) 
}

sub_from_register_low :: proc(cpu: ^CPU, reg: ^u16, value: u8, sub_carry: bool = false) {
    reg8 := read_register_low(reg^) 

    sub_from_byte(cpu, &reg8, value, sub_carry)
    write_register_low(reg, reg8) 
}

sub_from_byte :: proc(cpu: ^CPU, byte: ^u8, value: u8, sub_carry: bool = false) {
    sub := int(value)

    if sub_carry && read_flag(cpu, Flags.C) {
        sub += 1
    }

    c := int(byte^) - sub < 0
    hc := int(byte^ & 0xF) - int(sub & 0xF) < 0

    byte^ -= u8(sub)

    write_flag(cpu, Flags.Z, byte^ == 0)
    write_flag(cpu, Flags.N, true)
    write_flag(cpu, Flags.C, c)
    write_flag(cpu, Flags.H, hc)
}

and_register_high :: proc(cpu: ^CPU, reg: ^u16, value: u8) {
    reg8 := read_register_high(reg^)

    and_byte(cpu, &reg8, value)
    write_register_high(reg, reg8)
}

and_register_low :: proc(cpu: ^CPU, reg: ^u16, value: u8) {
    reg8 := read_register_low(reg^)

    and_byte(cpu, &reg8, value)
    write_register_low(reg, reg8)
}

and_byte :: proc(cpu: ^CPU, byte: ^u8, value: u8) {
    byte^ &= value

    write_flag(cpu, Flags.Z, byte^ == 0)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.C, false)
    write_flag(cpu, Flags.H, true)
}

xor_register_high :: proc(cpu: ^CPU, reg: ^u16, value: u8) {
    reg8 := read_register_high(reg^)

    xor_byte(cpu, &reg8, value)
    write_register_high(reg, reg8)
}

xor_register_low :: proc(cpu: ^CPU, reg: ^u16, value: u8) {
    reg8 := read_register_low(reg^)

    xor_byte(cpu, &reg8, value)
    write_register_low(reg, reg8)
}

xor_byte :: proc(cpu: ^CPU, byte: ^u8, value: u8) {
    byte^ ~= value

    write_flag(cpu, Flags.Z, byte^ == 0)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.C, false)
    write_flag(cpu, Flags.H, true)
}

or_register_high :: proc(cpu: ^CPU, reg: ^u16, value: u8) {
    reg8 := read_register_high(reg^)

    or_byte(cpu, &reg8, value)
    write_register_high(reg, reg8)
}

or_register_low :: proc(cpu: ^CPU, reg: ^u16, value: u8) {
    reg8 := read_register_low(reg^)

    or_byte(cpu, &reg8, value)
    write_register_low(reg, reg8)
}

or_byte :: proc(cpu: ^CPU, byte: ^u8, value: u8) {
    byte^ |= value

    write_flag(cpu, Flags.Z, byte^ == 0)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.C, false)
    write_flag(cpu, Flags.H, false)
}

cp_bytes :: proc(cpu: ^CPU, byteA: u8, byteB: u8) {
    res := int(byteA) - int(byteB)
    hc := int(byteA & 0xF) - int(byteB & 0xF) < 0

    write_flag(cpu, Flags.Z, res == 0)
    write_flag(cpu, Flags.N, true)
    write_flag(cpu, Flags.C, res < 0)
    write_flag(cpu, Flags.H, hc)
}

copy_to_ram :: proc { copy_to_ram_n8, copy_to_ram_n16 }

copy_to_ram_n8 :: proc(mem: []u8, addr: u16, value: u8) {
    mem[addr] = value
}

copy_to_ram_n16 :: proc(mem: []u8, addr: u16, value: u16) {
    mem[addr] = u8(value & 0xFF)
    mem[addr + 1] = u8((value & 0xFF00) >> 8)
}

copy_to_high_ram :: proc(mem: []u8, addr: u8, value: u8) {
    mem[0xFF00 + u16(addr)] = value
}

jr :: proc(cpu: ^CPU, offset: i8) {
    cpu.pc = u16(int(cpu.pc) + int(offset))
}

push_register :: proc(cpu: ^CPU, mem: []u8, reg: u16) {
    cpu.sp -= 1
    mem[cpu.sp] = read_register_high(reg)
    cpu.sp -= 1
    mem[cpu.sp] = read_register_low(reg)
}

pop_register :: proc(cpu: ^CPU, mem: []u8, reg: ^u16) {
    write_register_low(reg, mem[cpu.sp])
    cpu.sp += 1
    write_register_high(reg, mem[cpu.sp])
    cpu.sp += 1
}

call :: proc(cpu: ^CPU, mem: []u8, addr: u16) {
    push_register(cpu, mem, cpu.pc)
    cpu.pc = addr
}

ret :: proc(cpu: ^CPU, mem: []u8) {
    pop_register(cpu, mem, &cpu.pc)
}

// UNPREFIXED INSTRUCTIONS

// GROUP: control/misc

/*
    0x00 - NOP
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
NOP_0x00 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
}

/*
    0x10 - STOP
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
STOP_0x10 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
}

/*
    0x76 - HALT
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
HALT_0x76 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
}

/*
    0xcb - PREFIX
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
PREFIX_0xcb :: proc(cpu: ^CPU, mem: []u8, data: u16) {
}

/*
    0xf3 - DI
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
DI_0xf3 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
}

/*
    0xfb - EI
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
EI_0xfb :: proc(cpu: ^CPU, mem: []u8, data: u16) {
}

// GROUP: x16/lsm

/*
    0x01 - LD
    Length: 3
    Cycles: 12
    Flags: - - - -
*/
LD_0x01 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cpu.bc = data
}

/*
    0x08 - LD
    Length: 3
    Cycles: 20
    Flags: - - - -
*/
LD_0x08 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    copy_to_ram(mem, data, cpu.sp)
}

/*
    0x11 - LD
    Length: 3
    Cycles: 12
    Flags: - - - -
*/
LD_0x11 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cpu.de = data
}

/*
    0x21 - LD
    Length: 3
    Cycles: 12
    Flags: - - - -
*/
LD_0x21 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cpu.de = data
}

/*
    0x31 - LD
    Length: 3
    Cycles: 12
    Flags: - - - -
*/
LD_0x31 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cpu.sp = data
}

/*
    0xc1 - POP
    Length: 1
    Cycles: 12
    Flags: - - - -
*/
POP_0xc1 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    pop_register(cpu, mem, &cpu.bc)
}

/*
    0xc5 - PUSH
    Length: 1
    Cycles: 16
    Flags: - - - -
*/
PUSH_0xc5 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    push_register(cpu, mem, cpu.bc)
}

/*
    0xd1 - POP
    Length: 1
    Cycles: 12
    Flags: - - - -
*/
POP_0xd1 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    pop_register(cpu, mem, &cpu.de)
}

/*
    0xd5 - PUSH
    Length: 1
    Cycles: 16
    Flags: - - - -
*/
PUSH_0xd5 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    push_register(cpu, mem, cpu.de)
}

/*
    0xe1 - POP
    Length: 1
    Cycles: 12
    Flags: - - - -
*/
POP_0xe1 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    pop_register(cpu, mem, &cpu.hl)
}

/*
    0xe5 - PUSH
    Length: 1
    Cycles: 16
    Flags: - - - -
*/
PUSH_0xe5 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    push_register(cpu, mem, cpu.hl)
}

/*
    0xf1 - POP
    Length: 1
    Cycles: 12
    Flags: Z N H C
*/
POP_0xf1 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    pop_register(cpu, mem, &cpu.af)
}

/*
    0xf5 - PUSH
    Length: 1
    Cycles: 16
    Flags: - - - -
*/
PUSH_0xf5 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    push_register(cpu, mem, cpu.af)
}

/*
    0xf8 - LD
    Length: 2
    Cycles: 12
    Flags: 0 0 H C
*/
LD_0xf8 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    c := int(cpu.sp) + int(data) > 0xFFFF
    hc := (((cpu.sp & 0xFFF) + (data & 0xFFF)) & 0x1000) == 0x1000

    cpu.hl = cpu.sp + data

    write_flag(cpu, Flags.Z, false)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, hc)
    write_flag(cpu, Flags.C, c)
}

/*
    0xf9 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0xf9 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cpu.sp = cpu.hl
}

// GROUP: x8/lsm

/*
    0x02 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x02 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    mem[cpu.bc] = read_register_high(cpu.af)
}

/*
    0x06 - LD
    Length: 2
    Cycles: 8
    Flags: - - - -
*/
LD_0x06 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.bc, u8(data))
}

/*
    0x0a - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x0a :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.af, mem[cpu.bc])
}

/*
    0x0e - LD
    Length: 2
    Cycles: 8
    Flags: - - - -
*/
LD_0x0e :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.bc, u8(data))
}

/*
    0x12 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x12 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    mem[cpu.de] = read_register_high(cpu.af)
}

/*
    0x16 - LD
    Length: 2
    Cycles: 8
    Flags: - - - -
*/
LD_0x16 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.de, u8(data))
}

/*
    0x1a - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x1a :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.af, mem[cpu.de])
}

/*
    0x1e - LD
    Length: 2
    Cycles: 8
    Flags: - - - -
*/
LD_0x1e :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.de, u8(data))
}

/*
    0x22 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x22 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    mem[cpu.hl] = read_register_high(cpu.af)
    cpu.hl += 1
}

/*
    0x26 - LD
    Length: 2
    Cycles: 8
    Flags: - - - -
*/
LD_0x26 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.hl, u8(data))
}

/*
    0x2a - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x2a :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.af, mem[cpu.hl])
    cpu.hl += 1
}

/*
    0x2e - LD
    Length: 2
    Cycles: 8
    Flags: - - - -
*/
LD_0x2e :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.hl, u8(data))
}

/*
    0x32 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x32 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    mem[cpu.hl] = read_register_high(cpu.af)
    cpu.hl -= 1
}

/*
    0x36 - LD
    Length: 2
    Cycles: 12
    Flags: - - - -
*/
LD_0x36 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    mem[cpu.hl] = u8(data)
}

/*
    0x3a - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x3a :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.af, mem[cpu.hl])
    cpu.hl -= 1
}

/*
    0x3e - LD
    Length: 2
    Cycles: 8
    Flags: - - - -
*/
LD_0x3e :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.af, u8(data))
}

/*
    0x40 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x40 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.bc, read_register_high(cpu.bc))
}

/*
    0x41 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x41 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.bc, read_register_low(cpu.bc))
}

/*
    0x42 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x42 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.bc, read_register_high(cpu.de))
}

/*
    0x43 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x43 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.bc, read_register_low(cpu.de))
}

/*
    0x44 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x44 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.bc, read_register_high(cpu.hl))
}

/*
    0x45 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x45 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.bc, read_register_low(cpu.hl))
}

/*
    0x46 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x46 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.bc, mem[cpu.hl])
}

/*
    0x47 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x47 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.bc, read_register_high(cpu.af))
}

/*
    0x48 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x48 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.bc, read_register_high(cpu.bc))
}

/*
    0x49 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x49 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.bc, read_register_low(cpu.bc))
}

/*
    0x4a - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x4a :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.bc, read_register_high(cpu.de))
}

/*
    0x4b - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x4b :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.bc, read_register_low(cpu.de))
}

/*
    0x4c - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x4c :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.bc, read_register_high(cpu.hl))
}

/*
    0x4d - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x4d :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.bc, read_register_low(cpu.hl))
}

/*
    0x4e - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x4e :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.bc, mem[cpu.hl])
}

/*
    0x4f - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x4f :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.bc, read_register_high(cpu.af))
}

/*
    0x50 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x50 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.de, read_register_high(cpu.bc))
}

/*
    0x51 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x51 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.de, read_register_low(cpu.bc))
}

/*
    0x52 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x52 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.de, read_register_high(cpu.de))
}

/*
    0x53 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x53 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.de, read_register_low(cpu.de))
}

/*
    0x54 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x54 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.de, read_register_high(cpu.hl))
}

/*
    0x55 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x55 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.de, read_register_low(cpu.hl))
}

/*
    0x56 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x56 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.de, mem[cpu.hl])
}

/*
    0x57 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x57 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.de, read_register_high(cpu.af))
}

/*
    0x58 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x58 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.de, read_register_high(cpu.bc))
}

/*
    0x59 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x59 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.de, read_register_low(cpu.bc))
}

/*
    0x5a - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x5a :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.de, read_register_high(cpu.de))
}

/*
    0x5b - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x5b :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.de, read_register_low(cpu.de))
}

/*
    0x5c - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x5c :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.de, read_register_high(cpu.hl))
}

/*
    0x5d - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x5d :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.de, read_register_low(cpu.hl))
}

/*
    0x5e - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x5e :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.de, mem[cpu.hl])
}

/*
    0x5f - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x5f :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.de, read_register_high(cpu.af))
}

/*
    0x60 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x60 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.hl, read_register_high(cpu.bc))
}

/*
    0x61 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x61 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.hl, read_register_low(cpu.bc))
}

/*
    0x62 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x62 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.hl, read_register_high(cpu.de))
}

/*
    0x63 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x63 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.hl, read_register_low(cpu.de))
}

/*
    0x64 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x64 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.hl, read_register_high(cpu.hl))
}

/*
    0x65 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x65 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.hl, read_register_low(cpu.hl))
}

/*
    0x66 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x66 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.hl, mem[cpu.hl])
}

/*
    0x67 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x67 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.hl, read_register_high(cpu.af))
}

/*
    0x68 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x68 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.hl, read_register_high(cpu.bc))
}

/*
    0x69 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x69 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.hl, read_register_low(cpu.bc))
}

/*
    0x6a - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x6a :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.hl, read_register_high(cpu.de))
}

/*
    0x6b - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x6b :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.hl, read_register_low(cpu.de))
}

/*
    0x6c - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x6c :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.hl, read_register_high(cpu.hl))
}

/*
    0x6d - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x6d :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.hl, read_register_low(cpu.hl))
}

/*
    0x6e - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x6e :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.hl, mem[cpu.hl])
}

/*
    0x6f - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x6f :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_low(&cpu.hl, read_register_high(cpu.af))
}

/*
    0x70 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x70 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    mem[cpu.hl] = read_register_high(cpu.bc)
}

/*
    0x71 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x71 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    mem[cpu.hl] = read_register_low(cpu.bc)
}

/*
    0x72 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x72 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    mem[cpu.hl] = read_register_high(cpu.de)
}

/*
    0x73 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x73 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    mem[cpu.hl] = read_register_low(cpu.de)
}

/*
    0x74 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x74 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    mem[cpu.hl] = read_register_high(cpu.hl)
}

/*
    0x75 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x75 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    mem[cpu.hl] = read_register_low(cpu.hl)
}

/*
    0x77 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x77 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    mem[cpu.hl] = read_register_high(cpu.af)
}

/*
    0x78 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x78 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.af, read_register_high(cpu.bc))
}

/*
    0x79 - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x79 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.af, read_register_low(cpu.bc))
}

/*
    0x7a - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x7a :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.af, read_register_high(cpu.de))
}

/*
    0x7b - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x7b :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.af, read_register_low(cpu.de))
}

/*
    0x7c - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x7c :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.af, read_register_high(cpu.hl))
}

/*
    0x7d - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x7d :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.af, read_register_low(cpu.hl))
}

/*
    0x7e - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0x7e :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.af, mem[cpu.hl])
}

/*
    0x7f - LD
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
LD_0x7f :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.af, read_register_high(cpu.af))
}

/*
    0xe0 - LDH
    Length: 2
    Cycles: 12
    Flags: - - - -
*/
LDH_0xe0 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    copy_to_high_ram(mem, u8(data), read_register_high(cpu.af))
}

/*
    0xe2 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0xe2 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    copy_to_high_ram(mem, read_register_low(cpu.bc), read_register_high(cpu.af))
}

/*
    0xea - LD
    Length: 3
    Cycles: 16
    Flags: - - - -
*/
LD_0xea :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    copy_to_ram(mem, data, read_register_high(cpu.af))
}

/*
    0xf0 - LDH
    Length: 2
    Cycles: 12
    Flags: - - - -
*/
LDH_0xf0 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.af, mem[0xFF00 + data])
}

/*
    0xf2 - LD
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
LD_0xf2 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    addr := 0xFF00 + u16(read_register_low(cpu.bc))

    write_register_high(&cpu.af, mem[addr])
}

/*
    0xfa - LD
    Length: 3
    Cycles: 16
    Flags: - - - -
*/
LD_0xfa :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_register_high(&cpu.af, mem[data])
}

// GROUP: x16/alu

/*
    0x03 - INC
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
INC_0x03 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cpu.bc += 1
}

/*
    0x09 - ADD
    Length: 1
    Cycles: 8
    Flags: - 0 H C
*/
ADD_0x09 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register(cpu, &cpu.hl, cpu.bc)
}

/*
    0x0b - DEC
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
DEC_0x0b :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cpu.bc -= 1
}

/*
    0x13 - INC
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
INC_0x13 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cpu.de += 1
}

/*
    0x19 - ADD
    Length: 1
    Cycles: 8
    Flags: - 0 H C
*/
ADD_0x19 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register(cpu, &cpu.hl, cpu.de)
}

/*
    0x1b - DEC
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
DEC_0x1b :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cpu.de -= 1
}

/*
    0x23 - INC
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
INC_0x23 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cpu.de += 1
}

/*
    0x29 - ADD
    Length: 1
    Cycles: 8
    Flags: - 0 H C
*/
ADD_0x29 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register(cpu, &cpu.hl, cpu.hl)
}

/*
    0x2b - DEC
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
DEC_0x2b :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cpu.hl -= 1
}

/*
    0x33 - INC
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
INC_0x33 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cpu.sp += 1
}

/*
    0x39 - ADD
    Length: 1
    Cycles: 8
    Flags: - 0 H C
*/
ADD_0x39 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register(cpu, &cpu.hl, cpu.sp)
}

/*
    0x3b - DEC
    Length: 1
    Cycles: 8
    Flags: - - - -
*/
DEC_0x3b :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cpu.sp -= 1
}

/*
    0xe8 - ADD
    Length: 2
    Cycles: 16
    Flags: 0 0 H C
*/
ADD_0xe8 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
}

// GROUP: x8/alu

/*
    0x04 - INC
    Length: 1
    Cycles: 4
    Flags: Z 0 H -
*/
INC_0x04 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    inc_register_high(cpu, &cpu.bc)
}

/*
    0x05 - DEC
    Length: 1
    Cycles: 4
    Flags: Z 1 H -
*/
DEC_0x05 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    dec_register_high(cpu, &cpu.bc)
}

/*
    0x0c - INC
    Length: 1
    Cycles: 4
    Flags: Z 0 H -
*/
INC_0x0c :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    inc_register_low(cpu, &cpu.bc)
}

/*
    0x0d - DEC
    Length: 1
    Cycles: 4
    Flags: Z 1 H -
*/
DEC_0x0d :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    dec_register_low(cpu, &cpu.bc)
}

/*
    0x14 - INC
    Length: 1
    Cycles: 4
    Flags: Z 0 H -
*/
INC_0x14 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    inc_register_high(cpu, &cpu.de)
}

/*
    0x15 - DEC
    Length: 1
    Cycles: 4
    Flags: Z 1 H -
*/
DEC_0x15 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    dec_register_high(cpu, &cpu.de)
}

/*
    0x1c - INC
    Length: 1
    Cycles: 4
    Flags: Z 0 H -
*/
INC_0x1c :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    inc_register_low(cpu, &cpu.de)
}

/*
    0x1d - DEC
    Length: 1
    Cycles: 4
    Flags: Z 1 H -
*/
DEC_0x1d :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    dec_register_low(cpu, &cpu.de)
}

/*
    0x24 - INC
    Length: 1
    Cycles: 4
    Flags: Z 0 H -
*/
INC_0x24 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    inc_register_high(cpu, &cpu.de)
}

/*
    0x25 - DEC
    Length: 1
    Cycles: 4
    Flags: Z 1 H -
*/
DEC_0x25 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    dec_register_high(cpu, &cpu.de)
}

/*
    0x27 - DAA
    Length: 1
    Cycles: 4
    Flags: Z - 0 C
*/
DAA_0x27 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    // TODO
}

/*
    0x2c - INC
    Length: 1
    Cycles: 4
    Flags: Z 0 H -
*/
INC_0x2c :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    inc_register_low(cpu, &cpu.hl)
}

/*
    0x2d - DEC
    Length: 1
    Cycles: 4
    Flags: Z 1 H -
*/
DEC_0x2d :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    dec_register_low(cpu, &cpu.hl)
}

/*
    0x2f - CPL
    Length: 1
    Cycles: 4
    Flags: - 1 1 -
*/
CPL_0x2f :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    acc := read_register_high(cpu.af)

    write_register_high(&cpu.af, ~acc)
    write_flag(cpu, Flags.N, true)
    write_flag(cpu, Flags.H, true)
}

/*
    0x34 - INC
    Length: 1
    Cycles: 12
    Flags: Z 0 H -
*/
INC_0x34 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    inc_byte(cpu, &mem[cpu.hl])
}

/*
    0x35 - DEC
    Length: 1
    Cycles: 12
    Flags: Z 1 H -
*/
DEC_0x35 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    dec_byte(cpu, &mem[cpu.hl])
}

/*
    0x37 - SCF
    Length: 1
    Cycles: 4
    Flags: - 0 0 1
*/
SCF_0x37 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, false)
    write_flag(cpu, Flags.C, true)
}

/*
    0x3c - INC
    Length: 1
    Cycles: 4
    Flags: Z 0 H -
*/
INC_0x3c :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    inc_register_high(cpu, &cpu.af)
}

/*
    0x3d - DEC
    Length: 1
    Cycles: 4
    Flags: Z 1 H -
*/
DEC_0x3d :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    dec_register_high(cpu, &cpu.af)
}

/*
    0x3f - CCF
    Length: 1
    Cycles: 4
    Flags: - 0 0 C
*/
CCF_0x3f :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, false)
    write_flag(cpu, Flags.C, !read_flag(cpu, Flags.C))
}

/*
    0x80 - ADD
    Length: 1
    Cycles: 4
    Flags: Z 0 H C
*/
ADD_0x80 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.bc))
}

/*
    0x81 - ADD
    Length: 1
    Cycles: 4
    Flags: Z 0 H C
*/
ADD_0x81 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_low(cpu.bc))
}

/*
    0x82 - ADD
    Length: 1
    Cycles: 4
    Flags: Z 0 H C
*/
ADD_0x82 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.de))
}

/*
    0x83 - ADD
    Length: 1
    Cycles: 4
    Flags: Z 0 H C
*/
ADD_0x83 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_low(cpu.de))
}

/*
    0x84 - ADD
    Length: 1
    Cycles: 4
    Flags: Z 0 H C
*/
ADD_0x84 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.hl))
}

/*
    0x85 - ADD
    Length: 1
    Cycles: 4
    Flags: Z 0 H C
*/
ADD_0x85 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_low(cpu.hl))
}

/*
    0x86 - ADD
    Length: 1
    Cycles: 8
    Flags: Z 0 H C
*/
ADD_0x86 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, mem[cpu.hl])
}

/*
    0x87 - ADD
    Length: 1
    Cycles: 4
    Flags: Z 0 H C
*/
ADD_0x87 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.af))
}

/*
    0x88 - ADC
    Length: 1
    Cycles: 4
    Flags: Z 0 H C
*/
ADC_0x88 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.bc), add_carry = true)
}

/*
    0x89 - ADC
    Length: 1
    Cycles: 4
    Flags: Z 0 H C
*/
ADC_0x89 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_low(cpu.bc), add_carry = true)
}

/*
    0x8a - ADC
    Length: 1
    Cycles: 4
    Flags: Z 0 H C
*/
ADC_0x8a :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.de), add_carry = true)
}

/*
    0x8b - ADC
    Length: 1
    Cycles: 4
    Flags: Z 0 H C
*/
ADC_0x8b :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_low(cpu.de), add_carry = true)
}

/*
    0x8c - ADC
    Length: 1
    Cycles: 4
    Flags: Z 0 H C
*/
ADC_0x8c :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.hl), add_carry = true)
}

/*
    0x8d - ADC
    Length: 1
    Cycles: 4
    Flags: Z 0 H C
*/
ADC_0x8d :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_low(cpu.hl), add_carry = true)
}

/*
    0x8e - ADC
    Length: 1
    Cycles: 8
    Flags: Z 0 H C
*/
ADC_0x8e :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, mem[cpu.hl], add_carry = true)
}

/*
    0x8f - ADC
    Length: 1
    Cycles: 4
    Flags: Z 0 H C
*/
ADC_0x8f :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.af), add_carry = true)
}

/*
    0x90 - SUB
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
SUB_0x90 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.bc))
}

/*
    0x91 - SUB
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
SUB_0x91 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_low(cpu.bc))
}

/*
    0x92 - SUB
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
SUB_0x92 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.de))
}

/*
    0x93 - SUB
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
SUB_0x93 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_low(cpu.de))
}

/*
    0x94 - SUB
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
SUB_0x94 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.hl))
}

/*
    0x95 - SUB
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
SUB_0x95 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_low(cpu.hl))
}

/*
    0x96 - SUB
    Length: 1
    Cycles: 8
    Flags: Z 1 H C
*/
SUB_0x96 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, mem[cpu.hl])
}

/*
    0x97 - SUB
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
SUB_0x97 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.af))
}

/*
    0x98 - SBC
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
SBC_0x98 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.bc), sub_carry = true)
}

/*
    0x99 - SBC
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
SBC_0x99 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_low(cpu.bc), sub_carry = true)
}

/*
    0x9a - SBC
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
SBC_0x9a :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.de), sub_carry = true)
}

/*
    0x9b - SBC
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
SBC_0x9b :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_low(cpu.de), sub_carry = true)
}

/*
    0x9c - SBC
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
SBC_0x9c :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.hl), sub_carry = true)
}

/*
    0x9d - SBC
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
SBC_0x9d :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_low(cpu.hl), sub_carry = true)
}

/*
    0x9e - SBC
    Length: 1
    Cycles: 8
    Flags: Z 1 H C
*/
SBC_0x9e :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, mem[cpu.hl], sub_carry = true)
}

/*
    0x9f - SBC
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
SBC_0x9f :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.af), sub_carry = true)
}

/*
    0xa0 - AND
    Length: 1
    Cycles: 4
    Flags: Z 0 1 0
*/
AND_0xa0 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    and_register_high(cpu, &cpu.af, read_register_high(cpu.bc))
}

/*
    0xa1 - AND
    Length: 1
    Cycles: 4
    Flags: Z 0 1 0
*/
AND_0xa1 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    and_register_high(cpu, &cpu.af, read_register_low(cpu.bc))
}

/*
    0xa2 - AND
    Length: 1
    Cycles: 4
    Flags: Z 0 1 0
*/
AND_0xa2 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    and_register_high(cpu, &cpu.af, read_register_high(cpu.de))
}

/*
    0xa3 - AND
    Length: 1
    Cycles: 4
    Flags: Z 0 1 0
*/
AND_0xa3 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    and_register_high(cpu, &cpu.af, read_register_low(cpu.de))
}

/*
    0xa4 - AND
    Length: 1
    Cycles: 4
    Flags: Z 0 1 0
*/
AND_0xa4 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    and_register_high(cpu, &cpu.af, read_register_high(cpu.hl))
}

/*
    0xa5 - AND
    Length: 1
    Cycles: 4
    Flags: Z 0 1 0
*/
AND_0xa5 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    and_register_high(cpu, &cpu.af, read_register_low(cpu.hl))
}

/*
    0xa6 - AND
    Length: 1
    Cycles: 8
    Flags: Z 0 1 0
*/
AND_0xa6 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    and_register_high(cpu, &cpu.af, mem[cpu.hl])
}

/*
    0xa7 - AND
    Length: 1
    Cycles: 4
    Flags: Z 0 1 0
*/
AND_0xa7 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    and_register_high(cpu, &cpu.af, read_register_high(cpu.af))
}

/*
    0xa8 - XOR
    Length: 1
    Cycles: 4
    Flags: Z 0 0 0
*/
XOR_0xa8 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    xor_register_high(cpu, &cpu.af, read_register_high(cpu.bc))
}

/*
    0xa9 - XOR
    Length: 1
    Cycles: 4
    Flags: Z 0 0 0
*/
XOR_0xa9 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    xor_register_high(cpu, &cpu.af, read_register_low(cpu.bc))
}

/*
    0xaa - XOR
    Length: 1
    Cycles: 4
    Flags: Z 0 0 0
*/
XOR_0xaa :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    xor_register_high(cpu, &cpu.af, read_register_high(cpu.de))
}

/*
    0xab - XOR
    Length: 1
    Cycles: 4
    Flags: Z 0 0 0
*/
XOR_0xab :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    xor_register_high(cpu, &cpu.af, read_register_low(cpu.de))
}

/*
    0xac - XOR
    Length: 1
    Cycles: 4
    Flags: Z 0 0 0
*/
XOR_0xac :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    xor_register_high(cpu, &cpu.af, read_register_high(cpu.hl))
}

/*
    0xad - XOR
    Length: 1
    Cycles: 4
    Flags: Z 0 0 0
*/
XOR_0xad :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    xor_register_high(cpu, &cpu.af, read_register_low(cpu.hl))
}

/*
    0xae - XOR
    Length: 1
    Cycles: 8
    Flags: Z 0 0 0
*/
XOR_0xae :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    xor_register_high(cpu, &cpu.af, mem[cpu.hl])
}

/*
    0xaf - XOR
    Length: 1
    Cycles: 4
    Flags: Z 0 0 0
*/
XOR_0xaf :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    xor_register_high(cpu, &cpu.af, read_register_high(cpu.af))
}

/*
    0xb0 - OR
    Length: 1
    Cycles: 4
    Flags: Z 0 0 0
*/
OR_0xb0 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    or_register_high(cpu, &cpu.af, read_register_high(cpu.bc))
}

/*
    0xb1 - OR
    Length: 1
    Cycles: 4
    Flags: Z 0 0 0
*/
OR_0xb1 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    or_register_high(cpu, &cpu.af, read_register_low(cpu.bc))
}

/*
    0xb2 - OR
    Length: 1
    Cycles: 4
    Flags: Z 0 0 0
*/
OR_0xb2 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    or_register_high(cpu, &cpu.af, read_register_high(cpu.de))
}

/*
    0xb3 - OR
    Length: 1
    Cycles: 4
    Flags: Z 0 0 0
*/
OR_0xb3 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    or_register_high(cpu, &cpu.af, read_register_low(cpu.de))
}

/*
    0xb4 - OR
    Length: 1
    Cycles: 4
    Flags: Z 0 0 0
*/
OR_0xb4 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    or_register_high(cpu, &cpu.af, read_register_high(cpu.hl))
}

/*
    0xb5 - OR
    Length: 1
    Cycles: 4
    Flags: Z 0 0 0
*/
OR_0xb5 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    or_register_high(cpu, &cpu.af, read_register_low(cpu.hl))
}

/*
    0xb6 - OR
    Length: 1
    Cycles: 8
    Flags: Z 0 0 0
*/
OR_0xb6 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    or_register_high(cpu, &cpu.af, mem[cpu.hl])
}

/*
    0xb7 - OR
    Length: 1
    Cycles: 4
    Flags: Z 0 0 0
*/
OR_0xb7 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    or_register_high(cpu, &cpu.af, read_register_high(cpu.af))
}

/*
    0xb8 - CP
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
CP_0xb8 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), read_register_high(cpu.bc))
}

/*
    0xb9 - CP
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
CP_0xb9 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), read_register_low(cpu.bc))
}

/*
    0xba - CP
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
CP_0xba :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), read_register_high(cpu.de))
}

/*
    0xbb - CP
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
CP_0xbb :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), read_register_low(cpu.de))
}

/*
    0xbc - CP
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
CP_0xbc :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), read_register_high(cpu.hl))
}

/*
    0xbd - CP
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
CP_0xbd :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), read_register_low(cpu.hl))
}

/*
    0xbe - CP
    Length: 1
    Cycles: 8
    Flags: Z 1 H C
*/
CP_0xbe :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), mem[cpu.hl])
}

/*
    0xbf - CP
    Length: 1
    Cycles: 4
    Flags: Z 1 H C
*/
CP_0xbf :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), read_register_high(cpu.af))
}

/*
    0xc6 - ADD
    Length: 2
    Cycles: 8
    Flags: Z 0 H C
*/
ADD_0xc6 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, u8(data))
}

/*
    0xce - ADC
    Length: 2
    Cycles: 8
    Flags: Z 0 H C
*/
ADC_0xce :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    add_to_register_high(cpu, &cpu.af, u8(data), add_carry = true)
}

/*
    0xd6 - SUB
    Length: 2
    Cycles: 8
    Flags: Z 1 H C
*/
SUB_0xd6 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, u8(data))
}

/*
    0xde - SBC
    Length: 2
    Cycles: 8
    Flags: Z 1 H C
*/
SBC_0xde :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    sub_from_register_high(cpu, &cpu.af, u8(data), sub_carry = true)
}

/*
    0xe6 - AND
    Length: 2
    Cycles: 8
    Flags: Z 0 1 0
*/
AND_0xe6 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    and_register_high(cpu, &cpu.af, u8(data))
}

/*
    0xee - XOR
    Length: 2
    Cycles: 8
    Flags: Z 0 0 0
*/
XOR_0xee :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    xor_register_high(cpu, &cpu.af, u8(data))
}

/*
    0xf6 - OR
    Length: 2
    Cycles: 8
    Flags: Z 0 0 0
*/
OR_0xf6 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    or_register_high(cpu, &cpu.af, u8(data))
}

/*
    0xfe - CP
    Length: 2
    Cycles: 8
    Flags: Z 1 H C
*/
CP_0xfe :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), u8(data))
}

// GROUP: x8/rsb

/*
    0x07 - RLCA
    Length: 1
    Cycles: 4
    Flags: 0 0 0 C
*/
RLCA_0x07 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    acc := read_register_high(cpu.af)
    c := acc & 0x80 > 0

    acc <<= 1

    write_register_high(&cpu.af, acc)

    write_flag(cpu, Flags.Z, false)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, false)
    write_flag(cpu, Flags.C, c)
}

/*
    0x0f - RRCA
    Length: 1
    Cycles: 4
    Flags: 0 0 0 C
*/
RRCA_0x0f :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    acc := read_register_high(cpu.af)
    c := acc & 1 > 0

    acc >>= 1

    write_register_high(&cpu.af, acc)

    write_flag(cpu, Flags.Z, false)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, false)
    write_flag(cpu, Flags.C, c)
}

/*
    0x17 - RLA
    Length: 1
    Cycles: 4
    Flags: 0 0 0 C
*/
RLA_0x17 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    acc := read_register_high(cpu.af)
    c := acc & 0x80 > 0

    acc <<= 1
    acc |= u8(read_flag(cpu, Flags.C))

    write_register_high(&cpu.af, acc)

    write_flag(cpu, Flags.Z, false)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, false)
    write_flag(cpu, Flags.C, c)
}

/*
    0x1f - RRA
    Length: 1
    Cycles: 4
    Flags: 0 0 0 C
*/
RRA_0x1f :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    acc := read_register_high(cpu.af)
    c := acc & 1 > 0

    acc >>= 1
    acc |= (u8(read_flag(cpu, Flags.C)) << 7)

    write_register_high(&cpu.af, acc)

    write_flag(cpu, Flags.Z, false)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, false)
    write_flag(cpu, Flags.C, c)
}

// GROUP: control/br

/*
    0x18 - JR
    Length: 2
    Cycles: 12
    Flags: - - - -
*/
JR_0x18 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    jr(cpu, i8(data))
}

/*
    0x20 - JR
    Length: 2
    Cycles: 12
    Flags: - - - -
*/
JR_0x20 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.N) && read_flag(cpu, Flags.Z) {
        jr(cpu, i8(data))
    }
}

/*
    0x28 - JR
    Length: 2
    Cycles: 12
    Flags: - - - -
*/
JR_0x28 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.Z) {
        jr(cpu, i8(data))
    }
}

/*
    0x30 - JR
    Length: 2
    Cycles: 12
    Flags: - - - -
*/
JR_0x30 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.N) && read_flag(cpu, Flags.C) {
        jr(cpu, i8(data))
    }
}

/*
    0x38 - JR
    Length: 2
    Cycles: 12
    Flags: - - - -
*/
JR_0x38 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.C) {
        jr(cpu, i8(data))
    }
}

/*
    0xc0 - RET
    Length: 1
    Cycles: 20
    Flags: - - - -
*/
RET_0xc0 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.N) && read_flag(cpu, Flags.Z) {
        ret(cpu, mem)
    }
}

/*
    0xc2 - JP
    Length: 3
    Cycles: 16
    Flags: - - - -
*/
JP_0xc2 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.N) && read_flag(cpu, Flags.Z) {
        cpu.pc = data
    }
}

/*
    0xc3 - JP
    Length: 3
    Cycles: 16
    Flags: - - - -
*/
JP_0xc3 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cpu.pc = data
}

/*
    0xc4 - CALL
    Length: 3
    Cycles: 24
    Flags: - - - -
*/
CALL_0xc4 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.N) && read_flag(cpu, Flags.Z) {
        call(cpu, mem, data)
    }
}

/*
    0xc7 - RST
    Length: 1
    Cycles: 16
    Flags: - - - -
*/
RST_0xc7 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    call(cpu, mem, 0)
}

/*
    0xc8 - RET
    Length: 1
    Cycles: 20
    Flags: - - - -
*/
RET_0xc8 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.Z) {
        ret(cpu, mem)
    }
}

/*
    0xc9 - RET
    Length: 1
    Cycles: 16
    Flags: - - - -
*/
RET_0xc9 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    ret(cpu, mem)
}

/*
    0xca - JP
    Length: 3
    Cycles: 16
    Flags: - - - -
*/
JP_0xca :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.Z) {
        cpu.pc = data
    }
}

/*
    0xcc - CALL
    Length: 3
    Cycles: 24
    Flags: - - - -
*/
CALL_0xcc :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.Z) {
        call(cpu, mem, data)
    }
}

/*
    0xcd - CALL
    Length: 3
    Cycles: 24
    Flags: - - - -
*/
CALL_0xcd :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    call(cpu, mem, data)
}

/*
    0xcf - RST
    Length: 1
    Cycles: 16
    Flags: - - - -
*/
RST_0xcf :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    call(cpu, mem, 0x08)
}

/*
    0xd0 - RET
    Length: 1
    Cycles: 20
    Flags: - - - -
*/
RET_0xd0 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.N) && read_flag(cpu, Flags.C) {
        ret(cpu, mem)
    }
}

/*
    0xd2 - JP
    Length: 3
    Cycles: 16
    Flags: - - - -
*/
JP_0xd2 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.N) && read_flag(cpu, Flags.C) {
        cpu.pc = data
    }
}

/*
    0xd4 - CALL
    Length: 3
    Cycles: 24
    Flags: - - - -
*/
CALL_0xd4 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.N) && read_flag(cpu, Flags.C) {
        call(cpu, mem, data)
    }
}

/*
    0xd7 - RST
    Length: 1
    Cycles: 16
    Flags: - - - -
*/
RST_0xd7 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    call(cpu, mem, 0x10)
}

/*
    0xd8 - RET
    Length: 1
    Cycles: 20
    Flags: - - - -
*/
RET_0xd8 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.C) {
        ret(cpu, mem)
    }
}

/*
    0xd9 - RETI
    Length: 1
    Cycles: 16
    Flags: - - - -
*/
RETI_0xd9 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    ret(cpu, mem)
    // TODO: enable interrupts
}

/*
    0xda - JP
    Length: 3
    Cycles: 16
    Flags: - - - -
*/
JP_0xda :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.C) {
        cpu.pc = data
    }
}

/*
    0xdc - CALL
    Length: 3
    Cycles: 24
    Flags: - - - -
*/
CALL_0xdc :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    if read_flag(cpu, Flags.C) {
        call(cpu, mem, data)
    }
}

/*
    0xdf - RST
    Length: 1
    Cycles: 16
    Flags: - - - -
*/
RST_0xdf :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    call(cpu, mem, 0x18)
}

/*
    0xe7 - RST
    Length: 1
    Cycles: 16
    Flags: - - - -
*/
RST_0xe7 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    call(cpu, mem, 0x20)
}

/*
    0xe9 - JP
    Length: 1
    Cycles: 4
    Flags: - - - -
*/
JP_0xe9 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    cpu.pc = cpu.hl
}

/*
    0xef - RST
    Length: 1
    Cycles: 16
    Flags: - - - -
*/
RST_0xef :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    call(cpu, mem, 0x28)
}

/*
    0xf7 - RST
    Length: 1
    Cycles: 16
    Flags: - - - -
*/
RST_0xf7 :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    call(cpu, mem, 0x30)
}

/*
    0xff - RST
    Length: 1
    Cycles: 16
    Flags: - - - -
*/
RST_0xff :: proc(cpu: ^CPU, mem: []u8, data: u16) {
    call(cpu, mem, 0x38)
}

