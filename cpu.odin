package gb_emu

import "core:log"

Flags :: enum { Z, N, H, C }

// enum order matters: from highest to lowest priority
Interrupt :: enum { VBlank, LCD, Timer, Serial, Joypad }

CPU_State :: enum { Fetch, ExecuteInstruction, ExecuteInterrupt }

Breakpoint_Proc :: proc(cpu: ^CPU, mem: ^GB_Memory)

CPU :: struct {
    af: u16,
    bc: u16,
    de: u16,
    hl: u16,
    sp: u16,
    pc: u16,
    ime: bool,
    instructions: [0xFF+1]Instruction,
    prefixed_instructions: [0xFF+1]Instruction,
    interrupt_handlers: [5]u16,
    state: CPU_State,
    exec_op: u8,
    exec_cyles: int,
    prefix: bool,
    enable_interrupts: bool,
    breakpoints: map[u16]Breakpoint_Proc,
    breakpoint: u16

}

Instruction :: struct {
    len: int,
    cycles: int,
    mnemonic: string,
    func: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16)
}

cpu_init :: proc(cpu: ^CPU) {
    create_prefixed_instructions(cpu)
    create_unprefixed_instructions(cpu)

    cpu.interrupt_handlers[Interrupt.VBlank] = 0x40
    cpu.interrupt_handlers[Interrupt.LCD] = 0x48
    cpu.interrupt_handlers[Interrupt.Timer] = 0x50
    cpu.interrupt_handlers[Interrupt.Serial] = 0x58
    cpu.interrupt_handlers[Interrupt.Joypad] = 0x60

    cpu.state = CPU_State.Fetch
    cpu.pc = 0
    cpu.sp = 0
    cpu.ime = false
}

cpu_tick :: proc(cpu: ^CPU, mem: ^GB_Memory) {
    bp, bp_proc := check_breakpoints(cpu)

    if bp {
        log.debugf("Hit breakpoint at %x", cpu.pc)
        print_cpu(cpu)
        bp_proc(cpu, mem)
        cpu.breakpoint = cpu.pc

        return
    }

    switch cpu.state {
    case CPU_State.Fetch:
        do_fetch_state(cpu, mem)
    case CPU_State.ExecuteInstruction:
        do_execute_instruction_state(cpu)
    case CPU_State.ExecuteInterrupt:
        do_execute_interrupt_state(cpu)
    }
}

cpu_request_interrupt :: proc(cpu: ^CPU, mem: ^GB_Memory, interrupt: Interrupt) {
    i_f := mem_read(mem, u16(GB_HardRegister.IF))

    i_f |= (1 << u8(interrupt))
    mem_write(mem, u16(GB_HardRegister.IF), i_f)
}

cpu_add_breakpoint :: proc(cpu: ^CPU, addr: u16, bp_proc: Breakpoint_Proc) {
    cpu.breakpoints[addr] = bp_proc
}

check_breakpoints :: proc(cpu: ^CPU) -> (bool, Breakpoint_Proc) {
    bp_proc, exist := cpu.breakpoints[cpu.pc]

    if exist {
        return true, bp_proc
    }

    return false, nil
}

do_fetch_state :: proc(cpu: ^CPU, mem: ^GB_Memory) {
    if check_interrupts(cpu, mem) {
        return
    }

    op := mem_read(mem, cpu.pc)
    op_pc := cpu.pc
    instructions := cpu.prefix ? cpu.prefixed_instructions[:] : cpu.instructions[:]
    instruction := instructions[op]

    cpu.exec_op = op
    cpu.pc += 1
    cpu.prefix = false

    data: u16 = 0
    len := instruction.len - 1

    if len > 0 {
        data = fetch(cpu, mem, len)

        cpu.pc += u16(len)
    }

    // print_cpu(cpu)
    // log.debugf("0x%x - %s (0x%x) 0x%x (SP: %d)", op_pc, instruction.mnemonic, op, data, cpu.sp)
    instruction.func(cpu, mem, data)
    cpu.exec_cyles = instruction.cycles - 1
    cpu.state = CPU_State.ExecuteInstruction
}

do_execute_instruction_state :: proc(cpu: ^CPU) {
    cpu.exec_cyles -= 1

    if (cpu.exec_cyles > 0) {
        return
    }

    // instruction is done at this point

    if (cpu.enable_interrupts) {
        cpu.ime = true
        cpu.enable_interrupts = false
    }

    if (cpu.exec_op == 0xFB) {
        // enable interrupts after the next instruction if the last executed
        // operation is EI
        cpu.enable_interrupts = true
    }

    cpu.state = CPU_State.Fetch
}

do_execute_interrupt_state :: proc(cpu: ^CPU) {
    cpu.exec_cyles -= 1

    if (cpu.exec_cyles > 0) {
        return
    }

    cpu.state = CPU_State.Fetch
}

fetch :: proc(cpu: ^CPU, mem: ^GB_Memory, len: int) -> u16 {
    assert(len == 1 || len == 2)

    data := u16(mem_read(mem, cpu.pc))

    if len == 2 {
        data |= (u16(mem_read(mem, cpu.pc + 1)) << 8)
    }

    return data
}

check_interrupts :: proc(cpu: ^CPU, mem: ^GB_Memory) -> bool {
    i_e := mem_read(mem, u16(GB_HardRegister.IE))
    i_f := mem_read(mem, u16(GB_HardRegister.IF))

    if (!cpu.ime) {
        return false
    }

    for interrupt in Interrupt {
        if check_interrupt(cpu, mem, interrupt) {
            execute_interrupt(cpu, mem, interrupt)

            return true
        }
    }

    return false
}

check_interrupt :: proc(cpu: ^CPU, mem: ^GB_Memory, interrupt: Interrupt) -> bool {
    i_e := mem_read(mem, u16(GB_HardRegister.IE))
    i_f := mem_read(mem, u16(GB_HardRegister.IF))
    interrupt_mask := u8(1) << u8(interrupt)

    return (i_e & interrupt_mask) > 0 && (i_f & interrupt_mask) > 0
}

execute_interrupt :: proc(cpu: ^CPU, mem: ^GB_Memory, interrupt: Interrupt) {
    i_f := mem_read(mem, u16(GB_HardRegister.IF))
    i_f &= ~(1 << u8(interrupt))

    mem_write(mem, u16(GB_HardRegister.IF), i_f)
    call(cpu, mem, cpu.interrupt_handlers[interrupt])
    cpu.ime = false
    cpu.state = CPU_State.ExecuteInterrupt
    cpu.exec_cyles = 5 * 4 // interrupt handle takes 5 M-cycles
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
    return read_flag_from_byte(read_register_low(cpu.af), flag)
}

read_flag_from_byte :: proc(byte: u8, flag: Flags) -> bool {
    mask := u8(1 << (7 - uint(flag)))

    return (byte & mask) > 0
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

write_to_ram :: proc { write_to_ram_n8, write_to_ram_n16 }

write_to_ram_n8 :: proc(mem: ^GB_Memory, addr: u16, value: u8) {
    mem_write(mem, addr, value)
}

write_to_ram_n16 :: proc(mem: ^GB_Memory, addr: u16, value: u16) {
    low_byte := u8(value & 0xFF)
    high_byte := u8((value & 0xFF00) >> 8)

    mem_write(mem, addr, low_byte)
    mem_write(mem, addr + 1, high_byte)
}

write_to_high_ram :: proc(mem: ^GB_Memory, addr: u8, value: u8) {
    mem_write(mem, 0xFF00 + u16(addr), value)
}

jr :: proc(cpu: ^CPU, offset: i8) {
    cpu.pc = u16(int(cpu.pc) + int(offset))
}

push_register :: proc(cpu: ^CPU, mem: ^GB_Memory, reg: u16) {
    cpu.sp -= 1
    mem_write(mem, cpu.sp, read_register_high(reg))
    cpu.sp -= 1
    mem_write(mem, cpu.sp, read_register_low(reg))
}

pop_register :: proc(cpu: ^CPU, mem: ^GB_Memory, reg: ^u16) {
    write_register_low(reg, mem_read(mem, cpu.sp))
    cpu.sp += 1
    write_register_high(reg, mem_read(mem, cpu.sp))
    cpu.sp += 1
}

call :: proc(cpu: ^CPU, mem: ^GB_Memory, addr: u16) {
    push_register(cpu, mem, cpu.pc)
    cpu.pc = addr
}

ret :: proc(cpu: ^CPU, mem: ^GB_Memory) {
    pop_register(cpu, mem, &cpu.pc)
}

rlc_register_high :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_high(reg^)

    rlc(cpu, &byte)
    write_register_high(reg, byte)
}

rlc_register_low :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_low(reg^)

    rlc(cpu, &byte)
    write_register_low(reg, byte)
}

rrc_register_high :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_high(reg^)

    rrc(cpu, &byte)
    write_register_high(reg, byte)
}

rrc_register_low :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_low(reg^)

    rrc(cpu, &byte)
    write_register_low(reg, byte)
}

rl_register_high :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_high(reg^)

    rl(cpu, &byte)
    write_register_high(reg, byte)
}

rl_register_low :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_low(reg^)

    rl(cpu, &byte)
    write_register_low(reg, byte)
}

rr_register_high :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_high(reg^)

    rr(cpu, &byte)
    write_register_high(reg, byte)
}

rr_register_low :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_low(reg^)

    rr(cpu, &byte)
    write_register_low(reg, byte)
}

sla_register_high :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_high(reg^)

    sla(cpu, &byte)
    write_register_high(reg, byte)
}

sla_register_low :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_low(reg^)

    sla(cpu, &byte)
    write_register_low(reg, byte)
}

sra_register_high :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_high(reg^)

    sra(cpu, &byte)
    write_register_high(reg, byte)
}

sra_register_low :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_low(reg^)

    sra(cpu, &byte)
    write_register_low(reg, byte)
}

srl_register_high :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_high(reg^)

    srl(cpu, &byte)
    write_register_high(reg, byte)
}

srl_register_low :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_low(reg^)

    srl(cpu, &byte)
    write_register_low(reg, byte)
}

swap_register_high :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_high(reg^)

    swap(cpu, &byte)
    write_register_high(reg, byte)
}

swap_register_low :: proc(cpu: ^CPU, reg: ^u16) {
    byte := read_register_low(reg^)

    swap(cpu, &byte)
    write_register_low(reg, byte)
}

bit_register_high :: proc(cpu: ^CPU, reg: ^u16, b: u8) {
    byte := read_register_high(reg^)

    bit(cpu, &byte, b)
    write_register_high(reg, byte)
}

bit_register_low :: proc(cpu: ^CPU, reg: ^u16, b: u8) {
    byte := read_register_low(reg^)

    bit(cpu, &byte, b)
    write_register_low(reg, byte)
}

res_register_high :: proc(cpu: ^CPU, reg: ^u16, b: u8) {
    byte := read_register_high(reg^)

    res(cpu, &byte, b)
    write_register_high(reg, byte)
}

res_register_low :: proc(cpu: ^CPU, reg: ^u16, b: u8) {
    byte := read_register_low(reg^)

    res(cpu, &byte, b)
    write_register_low(reg, byte)
}

set_register_high :: proc(cpu: ^CPU, reg: ^u16, b: u8) {
    byte := read_register_high(reg^)

    set(cpu, &byte, b)
    write_register_high(reg, byte)
}

set_register_low :: proc(cpu: ^CPU, reg: ^u16, b: u8) {
    byte := read_register_low(reg^)

    set(cpu, &byte, b)
    write_register_low(reg, byte)
}

rlc :: proc(cpu: ^CPU, byte: ^u8) {
    c := byte^ & 0x80 > 0
    byte^ = (byte^ << 1) | u8(c)

    write_flag(cpu, Flags.Z, byte^ == 0)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, false)
    write_flag(cpu, Flags.C, c)
}

rrc :: proc(cpu: ^CPU, byte: ^u8) {
    c := byte^ & 1 > 0
    byte^ = (byte^ >> 1) | (u8(c) << 7)

    write_flag(cpu, Flags.Z, byte^ == 0)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, false)
    write_flag(cpu, Flags.C, c)
}

rl :: proc(cpu: ^CPU, byte: ^u8) {
    c := byte^ & 0x80 > 0
    byte^ = (byte^ << 1) | u8(read_flag(cpu, Flags.C))

    write_flag(cpu, Flags.Z, byte^ == 0)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, false)
    write_flag(cpu, Flags.C, c)
}

rr :: proc(cpu: ^CPU, byte: ^u8) {
    c := byte^ & 1 > 0

    byte^ = (byte^ >> 1) | (u8(read_flag(cpu, Flags.C)) << 7)

    write_flag(cpu, Flags.Z, byte^ == 0)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, false)
    write_flag(cpu, Flags.C, c)
}

sla :: proc(cpu: ^CPU, byte: ^u8) {
    c := byte^ & 0x80 > 0
    byte^ <<= 1

    write_flag(cpu, Flags.Z, byte^ == 0)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, false)
    write_flag(cpu, Flags.C, c)
}

sra :: proc(cpu: ^CPU, byte: ^u8) {
    c := byte^ & 1 > 0
    byte^ = (byte^ >> 1) | (byte^ & 0x80)

    write_flag(cpu, Flags.Z, byte^ == 0)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, false)
    write_flag(cpu, Flags.C, c)
}

srl :: proc(cpu: ^CPU, byte: ^u8) {
    c := byte^ & 1 > 0
    byte^ >>= 1

    write_flag(cpu, Flags.Z, byte^ == 0)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, false)
    write_flag(cpu, Flags.C, c)
}

swap :: proc(cpu: ^CPU, byte: ^u8) {
    byte^ = (byte^ << 4) | ((byte^ & 0xF0) >> 4)

    write_flag(cpu, Flags.Z, byte^ == 0)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, false)
    write_flag(cpu, Flags.C, false)
}

bit :: proc(cpu: ^CPU, byte: ^u8, b: u8) {
    write_flag(cpu, Flags.Z, byte^ & (1 << b) > 0)
    write_flag(cpu, Flags.N, false)
    write_flag(cpu, Flags.H, true)
}

res :: proc(cpu: ^CPU, byte: ^u8, b: u8) {
    byte^ &= ~(1 << b)
}

set :: proc(cpu: ^CPU, byte: ^u8, b: u8) {
    byte^ |= (1 << b)
}

// UNPREFIXED INSTRUCTIONS

// GROUP: control/misc

/*
   0x00 - NOP
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
NOP_0x00 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
}

/*
   0x10 - STOP
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
STOP_0x10 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    // TODO
}

/*
   0x76 - HALT
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
HALT_0x76 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    // TODO
}

/*
   0xcb - PREFIX
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
PREFIX_0xcb :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.prefix = true
}

/*
   0xf3 - DI
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
DI_0xf3 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.ime = false
}

/*
   0xfb - EI
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
EI_0xfb :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    // see the 'do_execute_instruction_state' function
}

// GROUP: x16/lsm

/*
   0x01 - LD
   Length: 3
   Cycles: 12
   Flags: - - - -
 */
LD_0x01 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.bc = data
}

/*
   0x08 - LD
   Length: 3
   Cycles: 20
   Flags: - - - -
 */
LD_0x08 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_to_ram(mem, data, cpu.sp)
}

/*
   0x11 - LD
   Length: 3
   Cycles: 12
   Flags: - - - -
 */
LD_0x11 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.de = data
}

/*
   0x21 - LD
   Length: 3
   Cycles: 12
   Flags: - - - -
 */
LD_0x21 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.hl = data
}

/*
   0x31 - LD
   Length: 3
   Cycles: 12
   Flags: - - - -
 */
LD_0x31 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.sp = data
}

/*
   0xc1 - POP
   Length: 1
   Cycles: 12
   Flags: - - - -
 */
POP_0xc1 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    pop_register(cpu, mem, &cpu.bc)
}

/*
   0xc5 - PUSH
   Length: 1
   Cycles: 16
   Flags: - - - -
 */
PUSH_0xc5 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    push_register(cpu, mem, cpu.bc)
}

/*
   0xd1 - POP
   Length: 1
   Cycles: 12
   Flags: - - - -
 */
POP_0xd1 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    pop_register(cpu, mem, &cpu.de)
}

/*
   0xd5 - PUSH
   Length: 1
   Cycles: 16
   Flags: - - - -
 */
PUSH_0xd5 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    push_register(cpu, mem, cpu.de)
}

/*
   0xe1 - POP
   Length: 1
   Cycles: 12
   Flags: - - - -
 */
POP_0xe1 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    pop_register(cpu, mem, &cpu.hl)
}

/*
   0xe5 - PUSH
   Length: 1
   Cycles: 16
   Flags: - - - -
 */
PUSH_0xe5 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    push_register(cpu, mem, cpu.hl)
}

/*
   0xf1 - POP
   Length: 1
   Cycles: 12
   Flags: Z N H C
 */
POP_0xf1 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    pop_register(cpu, mem, &cpu.af)
}

/*
   0xf5 - PUSH
   Length: 1
   Cycles: 16
   Flags: - - - -
 */
PUSH_0xf5 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    push_register(cpu, mem, cpu.af)
}

/*
   0xf8 - LD
   Length: 2
   Cycles: 12
   Flags: 0 0 H C
 */
LD_0xf8 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
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
LD_0xf9 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.sp = cpu.hl
}

// GROUP: x8/lsm

/*
   0x02 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x02 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    mem_write(mem, cpu.bc, read_register_high(cpu.af))
}

/*
   0x06 - LD
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
LD_0x06 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.bc, u8(data))
}

/*
   0x0a - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x0a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.af, mem_read(mem, cpu.bc))
}

/*
   0x0e - LD
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
LD_0x0e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.bc, u8(data))
}

/*
   0x12 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x12 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    mem_write(mem, cpu.de, read_register_high(cpu.af))
}

/*
   0x16 - LD
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
LD_0x16 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.de, u8(data))
}

/*
   0x1a - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x1a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.af, mem_read(mem, cpu.de))
}

/*
   0x1e - LD
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
LD_0x1e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.de, u8(data))
}

/*
   0x22 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x22 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    mem_write(mem, cpu.hl, read_register_high(cpu.af))
    cpu.hl += 1
}

/*
   0x26 - LD
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
LD_0x26 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.hl, u8(data))
}

/*
   0x2a - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x2a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.af, mem_read(mem, cpu.hl))
    cpu.hl += 1
}

/*
   0x2e - LD
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
LD_0x2e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.hl, u8(data))
}

/*
   0x32 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x32 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    mem_write(mem, cpu.hl, read_register_high(cpu.af))
    cpu.hl -= 1
}

/*
   0x36 - LD
   Length: 2
   Cycles: 12
   Flags: - - - -
 */
LD_0x36 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    mem_write(mem, cpu.hl, u8(data))
}

/*
   0x3a - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x3a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.af, mem_read(mem, cpu.hl))
    cpu.hl -= 1
}

/*
   0x3e - LD
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
LD_0x3e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.af, u8(data))
}

/*
   0x40 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x40 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.bc, read_register_high(cpu.bc))
}

/*
   0x41 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x41 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.bc, read_register_low(cpu.bc))
}

/*
   0x42 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x42 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.bc, read_register_high(cpu.de))
}

/*
   0x43 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x43 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.bc, read_register_low(cpu.de))
}

/*
   0x44 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x44 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.bc, read_register_high(cpu.hl))
}

/*
   0x45 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x45 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.bc, read_register_low(cpu.hl))
}

/*
   0x46 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x46 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.bc, mem_read(mem, cpu.hl))
}

/*
   0x47 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x47 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.bc, read_register_high(cpu.af))
}

/*
   0x48 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x48 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.bc, read_register_high(cpu.bc))
}

/*
   0x49 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x49 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.bc, read_register_low(cpu.bc))
}

/*
   0x4a - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x4a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.bc, read_register_high(cpu.de))
}

/*
   0x4b - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x4b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.bc, read_register_low(cpu.de))
}

/*
   0x4c - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x4c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.bc, read_register_high(cpu.hl))
}

/*
   0x4d - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x4d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.bc, read_register_low(cpu.hl))
}

/*
   0x4e - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x4e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.bc, mem_read(mem, cpu.hl))
}

/*
   0x4f - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x4f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.bc, read_register_high(cpu.af))
}

/*
   0x50 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x50 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.de, read_register_high(cpu.bc))
}

/*
   0x51 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x51 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.de, read_register_low(cpu.bc))
}

/*
   0x52 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x52 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.de, read_register_high(cpu.de))
}

/*
   0x53 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x53 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.de, read_register_low(cpu.de))
}

/*
   0x54 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x54 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.de, read_register_high(cpu.hl))
}

/*
   0x55 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x55 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.de, read_register_low(cpu.hl))
}

/*
   0x56 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x56 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.de, mem_read(mem, cpu.hl))
}

/*
   0x57 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x57 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.de, read_register_high(cpu.af))
}

/*
   0x58 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x58 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.de, read_register_high(cpu.bc))
}

/*
   0x59 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x59 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.de, read_register_low(cpu.bc))
}

/*
   0x5a - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x5a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.de, read_register_high(cpu.de))
}

/*
   0x5b - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x5b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.de, read_register_low(cpu.de))
}

/*
   0x5c - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x5c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.de, read_register_high(cpu.hl))
}

/*
   0x5d - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x5d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.de, read_register_low(cpu.hl))
}

/*
   0x5e - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x5e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.de, mem_read(mem, cpu.hl))
}

/*
   0x5f - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x5f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.de, read_register_high(cpu.af))
}

/*
   0x60 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x60 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.hl, read_register_high(cpu.bc))
}

/*
   0x61 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x61 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.hl, read_register_low(cpu.bc))
}

/*
   0x62 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x62 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.hl, read_register_high(cpu.de))
}

/*
   0x63 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x63 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.hl, read_register_low(cpu.de))
}

/*
   0x64 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x64 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.hl, read_register_high(cpu.hl))
}

/*
   0x65 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x65 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.hl, read_register_low(cpu.hl))
}

/*
   0x66 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x66 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.hl, mem_read(mem, cpu.hl))
}

/*
   0x67 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x67 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.hl, read_register_high(cpu.af))
}

/*
   0x68 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x68 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.hl, read_register_high(cpu.bc))
}

/*
   0x69 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x69 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.hl, read_register_low(cpu.bc))
}

/*
   0x6a - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x6a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.hl, read_register_high(cpu.de))
}

/*
   0x6b - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x6b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.hl, read_register_low(cpu.de))
}

/*
   0x6c - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x6c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.hl, read_register_high(cpu.hl))
}

/*
   0x6d - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x6d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.hl, read_register_low(cpu.hl))
}

/*
   0x6e - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x6e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.hl, mem_read(mem, cpu.hl))
}

/*
   0x6f - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x6f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_low(&cpu.hl, read_register_high(cpu.af))
}

/*
   0x70 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x70 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    mem_write(mem, cpu.hl, read_register_high(cpu.bc))
}

/*
   0x71 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x71 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    mem_write(mem, cpu.hl, read_register_low(cpu.bc))
}

/*
   0x72 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x72 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    mem_write(mem, cpu.hl, read_register_high(cpu.de))
}

/*
   0x73 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x73 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    mem_write(mem, cpu.hl, read_register_low(cpu.de))
}

/*
   0x74 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x74 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    mem_write(mem, cpu.hl, read_register_high(cpu.hl))
}

/*
   0x75 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x75 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    mem_write(mem, cpu.hl, read_register_low(cpu.hl))
}

/*
   0x77 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x77 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    mem_write(mem, cpu.hl, read_register_high(cpu.af))
}

/*
   0x78 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x78 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.af, read_register_high(cpu.bc))
}

/*
   0x79 - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x79 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.af, read_register_low(cpu.bc))
}

/*
   0x7a - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x7a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.af, read_register_high(cpu.de))
}

/*
   0x7b - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x7b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.af, read_register_low(cpu.de))
}

/*
   0x7c - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x7c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.af, read_register_high(cpu.hl))
}

/*
   0x7d - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x7d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.af, read_register_low(cpu.hl))
}

/*
   0x7e - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0x7e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.af, mem_read(mem, cpu.hl))
}

/*
   0x7f - LD
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
LD_0x7f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.af, read_register_high(cpu.af))
}

/*
   0xe0 - LDH
   Length: 2
   Cycles: 12
   Flags: - - - -
 */
LDH_0xe0 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_to_high_ram(mem, u8(data), read_register_high(cpu.af))
}

/*
   0xe2 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0xe2 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_to_high_ram(mem, read_register_low(cpu.bc), read_register_high(cpu.af))
}

/*
   0xea - LD
   Length: 3
   Cycles: 16
   Flags: - - - -
 */
LD_0xea :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_to_ram(mem, data, read_register_high(cpu.af))
}

/*
   0xf0 - LDH
   Length: 2
   Cycles: 12
   Flags: - - - -
 */
LDH_0xf0 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.af, mem_read(mem, 0xFF00 + data))
}

/*
   0xf2 - LD
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
LD_0xf2 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    addr := 0xFF00 + u16(read_register_low(cpu.bc))

    write_register_high(&cpu.af, mem_read(mem, addr))
}

/*
   0xfa - LD
   Length: 3
   Cycles: 16
   Flags: - - - -
 */
LD_0xfa :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    write_register_high(&cpu.af, mem_read(mem, data))
}

// GROUP: x16/alu

/*
   0x03 - INC
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
INC_0x03 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.bc += 1
}

/*
   0x09 - ADD
   Length: 1
   Cycles: 8
   Flags: - 0 H C
 */
ADD_0x09 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register(cpu, &cpu.hl, cpu.bc)
}

/*
   0x0b - DEC
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
DEC_0x0b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.bc -= 1
}

/*
   0x13 - INC
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
INC_0x13 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.de += 1
}

/*
   0x19 - ADD
   Length: 1
   Cycles: 8
   Flags: - 0 H C
 */
ADD_0x19 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register(cpu, &cpu.hl, cpu.de)
}

/*
   0x1b - DEC
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
DEC_0x1b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.de -= 1
}

/*
   0x23 - INC
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
INC_0x23 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.hl += 1
}

/*
   0x29 - ADD
   Length: 1
   Cycles: 8
   Flags: - 0 H C
 */
ADD_0x29 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register(cpu, &cpu.hl, cpu.hl)
}

/*
   0x2b - DEC
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
DEC_0x2b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.hl -= 1
}

/*
   0x33 - INC
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
INC_0x33 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.sp += 1
}

/*
   0x39 - ADD
   Length: 1
   Cycles: 8
   Flags: - 0 H C
 */
ADD_0x39 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register(cpu, &cpu.hl, cpu.sp)
}

/*
   0x3b - DEC
   Length: 1
   Cycles: 8
   Flags: - - - -
 */
DEC_0x3b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.sp -= 1
}

/*
   0xe8 - ADD
   Length: 2
   Cycles: 16
   Flags: 0 0 H C
 */
ADD_0xe8 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
}

// GROUP: x8/alu

/*
   0x04 - INC
   Length: 1
   Cycles: 4
   Flags: Z 0 H -
 */
INC_0x04 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    inc_register_high(cpu, &cpu.bc)
}

/*
   0x05 - DEC
   Length: 1
   Cycles: 4
   Flags: Z 1 H -
 */
DEC_0x05 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    dec_register_high(cpu, &cpu.bc)
}

/*
   0x0c - INC
   Length: 1
   Cycles: 4
   Flags: Z 0 H -
 */
INC_0x0c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    inc_register_low(cpu, &cpu.bc)
}

/*
   0x0d - DEC
   Length: 1
   Cycles: 4
   Flags: Z 1 H -
 */
DEC_0x0d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    dec_register_low(cpu, &cpu.bc)
}

/*
   0x14 - INC
   Length: 1
   Cycles: 4
   Flags: Z 0 H -
 */
INC_0x14 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    inc_register_high(cpu, &cpu.de)
}

/*
   0x15 - DEC
   Length: 1
   Cycles: 4
   Flags: Z 1 H -
 */
DEC_0x15 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    dec_register_high(cpu, &cpu.de)
}

/*
   0x1c - INC
   Length: 1
   Cycles: 4
   Flags: Z 0 H -
 */
INC_0x1c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    inc_register_low(cpu, &cpu.de)
}

/*
   0x1d - DEC
   Length: 1
   Cycles: 4
   Flags: Z 1 H -
 */
DEC_0x1d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    dec_register_low(cpu, &cpu.de)
}

/*
   0x24 - INC
   Length: 1
   Cycles: 4
   Flags: Z 0 H -
 */
INC_0x24 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    inc_register_high(cpu, &cpu.hl)
}

/*
   0x25 - DEC
   Length: 1
   Cycles: 4
   Flags: Z 1 H -
 */
DEC_0x25 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    dec_register_high(cpu, &cpu.hl)
}

/*
   0x27 - DAA
   Length: 1
   Cycles: 4
   Flags: Z - 0 C
 */
DAA_0x27 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    // TODO
}

/*
   0x2c - INC
   Length: 1
   Cycles: 4
   Flags: Z 0 H -
 */
INC_0x2c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    inc_register_low(cpu, &cpu.hl)
}

/*
   0x2d - DEC
   Length: 1
   Cycles: 4
   Flags: Z 1 H -
 */
DEC_0x2d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    dec_register_low(cpu, &cpu.hl)
}

/*
   0x2f - CPL
   Length: 1
   Cycles: 4
   Flags: - 1 1 -
 */
CPL_0x2f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
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
INC_0x34 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    inc_byte(cpu, mem_get_ptr(mem, cpu.hl))
}

/*
   0x35 - DEC
   Length: 1
   Cycles: 12
   Flags: Z 1 H -
 */
DEC_0x35 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    dec_byte(cpu, mem_get_ptr(mem, cpu.hl))
}

/*
   0x37 - SCF
   Length: 1
   Cycles: 4
   Flags: - 0 0 1
 */
SCF_0x37 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
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
INC_0x3c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    inc_register_high(cpu, &cpu.af)
}

/*
   0x3d - DEC
   Length: 1
   Cycles: 4
   Flags: Z 1 H -
 */
DEC_0x3d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    dec_register_high(cpu, &cpu.af)
}

/*
   0x3f - CCF
   Length: 1
   Cycles: 4
   Flags: - 0 0 C
 */
CCF_0x3f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
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
ADD_0x80 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.bc))
}

/*
   0x81 - ADD
   Length: 1
   Cycles: 4
   Flags: Z 0 H C
 */
ADD_0x81 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_low(cpu.bc))
}

/*
   0x82 - ADD
   Length: 1
   Cycles: 4
   Flags: Z 0 H C
 */
ADD_0x82 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.de))
}

/*
   0x83 - ADD
   Length: 1
   Cycles: 4
   Flags: Z 0 H C
 */
ADD_0x83 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_low(cpu.de))
}

/*
   0x84 - ADD
   Length: 1
   Cycles: 4
   Flags: Z 0 H C
 */
ADD_0x84 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.hl))
}

/*
   0x85 - ADD
   Length: 1
   Cycles: 4
   Flags: Z 0 H C
 */
ADD_0x85 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_low(cpu.hl))
}

/*
   0x86 - ADD
   Length: 1
   Cycles: 8
   Flags: Z 0 H C
 */
ADD_0x86 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, mem_read(mem, cpu.hl))
}

/*
   0x87 - ADD
   Length: 1
   Cycles: 4
   Flags: Z 0 H C
 */
ADD_0x87 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.af))
}

/*
   0x88 - ADC
   Length: 1
   Cycles: 4
   Flags: Z 0 H C
 */
ADC_0x88 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.bc), add_carry = true)
}

/*
   0x89 - ADC
   Length: 1
   Cycles: 4
   Flags: Z 0 H C
 */
ADC_0x89 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_low(cpu.bc), add_carry = true)
}

/*
   0x8a - ADC
   Length: 1
   Cycles: 4
   Flags: Z 0 H C
 */
ADC_0x8a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.de), add_carry = true)
}

/*
   0x8b - ADC
   Length: 1
   Cycles: 4
   Flags: Z 0 H C
 */
ADC_0x8b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_low(cpu.de), add_carry = true)
}

/*
   0x8c - ADC
   Length: 1
   Cycles: 4
   Flags: Z 0 H C
 */
ADC_0x8c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.hl), add_carry = true)
}

/*
   0x8d - ADC
   Length: 1
   Cycles: 4
   Flags: Z 0 H C
 */
ADC_0x8d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_low(cpu.hl), add_carry = true)
}

/*
   0x8e - ADC
   Length: 1
   Cycles: 8
   Flags: Z 0 H C
 */
ADC_0x8e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, mem_read(mem, cpu.hl), add_carry = true)
}

/*
   0x8f - ADC
   Length: 1
   Cycles: 4
   Flags: Z 0 H C
 */
ADC_0x8f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, read_register_high(cpu.af), add_carry = true)
}

/*
   0x90 - SUB
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
SUB_0x90 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.bc))
}

/*
   0x91 - SUB
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
SUB_0x91 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_low(cpu.bc))
}

/*
   0x92 - SUB
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
SUB_0x92 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.de))
}

/*
   0x93 - SUB
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
SUB_0x93 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_low(cpu.de))
}

/*
   0x94 - SUB
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
SUB_0x94 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.hl))
}

/*
   0x95 - SUB
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
SUB_0x95 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_low(cpu.hl))
}

/*
   0x96 - SUB
   Length: 1
   Cycles: 8
   Flags: Z 1 H C
 */
SUB_0x96 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, mem_read(mem, cpu.hl))
}

/*
   0x97 - SUB
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
SUB_0x97 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.af))
}

/*
   0x98 - SBC
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
SBC_0x98 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.bc), sub_carry = true)
}

/*
   0x99 - SBC
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
SBC_0x99 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_low(cpu.bc), sub_carry = true)
}

/*
   0x9a - SBC
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
SBC_0x9a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.de), sub_carry = true)
}

/*
   0x9b - SBC
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
SBC_0x9b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_low(cpu.de), sub_carry = true)
}

/*
   0x9c - SBC
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
SBC_0x9c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.hl), sub_carry = true)
}

/*
   0x9d - SBC
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
SBC_0x9d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_low(cpu.hl), sub_carry = true)
}

/*
   0x9e - SBC
   Length: 1
   Cycles: 8
   Flags: Z 1 H C
 */
SBC_0x9e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, mem_read(mem, cpu.hl), sub_carry = true)
}

/*
   0x9f - SBC
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
SBC_0x9f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, read_register_high(cpu.af), sub_carry = true)
}

/*
   0xa0 - AND
   Length: 1
   Cycles: 4
   Flags: Z 0 1 0
 */
AND_0xa0 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    and_register_high(cpu, &cpu.af, read_register_high(cpu.bc))
}

/*
   0xa1 - AND
   Length: 1
   Cycles: 4
   Flags: Z 0 1 0
 */
AND_0xa1 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    and_register_high(cpu, &cpu.af, read_register_low(cpu.bc))
}

/*
   0xa2 - AND
   Length: 1
   Cycles: 4
   Flags: Z 0 1 0
 */
AND_0xa2 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    and_register_high(cpu, &cpu.af, read_register_high(cpu.de))
}

/*
   0xa3 - AND
   Length: 1
   Cycles: 4
   Flags: Z 0 1 0
 */
AND_0xa3 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    and_register_high(cpu, &cpu.af, read_register_low(cpu.de))
}

/*
   0xa4 - AND
   Length: 1
   Cycles: 4
   Flags: Z 0 1 0
 */
AND_0xa4 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    and_register_high(cpu, &cpu.af, read_register_high(cpu.hl))
}

/*
   0xa5 - AND
   Length: 1
   Cycles: 4
   Flags: Z 0 1 0
 */
AND_0xa5 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    and_register_high(cpu, &cpu.af, read_register_low(cpu.hl))
}

/*
   0xa6 - AND
   Length: 1
   Cycles: 8
   Flags: Z 0 1 0
 */
AND_0xa6 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    and_register_high(cpu, &cpu.af, mem_read(mem, cpu.hl))
}

/*
   0xa7 - AND
   Length: 1
   Cycles: 4
   Flags: Z 0 1 0
 */
AND_0xa7 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    and_register_high(cpu, &cpu.af, read_register_high(cpu.af))
}

/*
   0xa8 - XOR
   Length: 1
   Cycles: 4
   Flags: Z 0 0 0
 */
XOR_0xa8 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    xor_register_high(cpu, &cpu.af, read_register_high(cpu.bc))
}

/*
   0xa9 - XOR
   Length: 1
   Cycles: 4
   Flags: Z 0 0 0
 */
XOR_0xa9 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    xor_register_high(cpu, &cpu.af, read_register_low(cpu.bc))
}

/*
   0xaa - XOR
   Length: 1
   Cycles: 4
   Flags: Z 0 0 0
 */
XOR_0xaa :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    xor_register_high(cpu, &cpu.af, read_register_high(cpu.de))
}

/*
   0xab - XOR
   Length: 1
   Cycles: 4
   Flags: Z 0 0 0
 */
XOR_0xab :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    xor_register_high(cpu, &cpu.af, read_register_low(cpu.de))
}

/*
   0xac - XOR
   Length: 1
   Cycles: 4
   Flags: Z 0 0 0
 */
XOR_0xac :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    xor_register_high(cpu, &cpu.af, read_register_high(cpu.hl))
}

/*
   0xad - XOR
   Length: 1
   Cycles: 4
   Flags: Z 0 0 0
 */
XOR_0xad :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    xor_register_high(cpu, &cpu.af, read_register_low(cpu.hl))
}

/*
   0xae - XOR
   Length: 1
   Cycles: 8
   Flags: Z 0 0 0
 */
XOR_0xae :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    xor_register_high(cpu, &cpu.af, mem_read(mem, cpu.hl))
}

/*
   0xaf - XOR
   Length: 1
   Cycles: 4
   Flags: Z 0 0 0
 */
XOR_0xaf :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    xor_register_high(cpu, &cpu.af, read_register_high(cpu.af))
}

/*
   0xb0 - OR
   Length: 1
   Cycles: 4
   Flags: Z 0 0 0
 */
OR_0xb0 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    or_register_high(cpu, &cpu.af, read_register_high(cpu.bc))
}

/*
   0xb1 - OR
   Length: 1
   Cycles: 4
   Flags: Z 0 0 0
 */
OR_0xb1 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    or_register_high(cpu, &cpu.af, read_register_low(cpu.bc))
}

/*
   0xb2 - OR
   Length: 1
   Cycles: 4
   Flags: Z 0 0 0
 */
OR_0xb2 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    or_register_high(cpu, &cpu.af, read_register_high(cpu.de))
}

/*
   0xb3 - OR
   Length: 1
   Cycles: 4
   Flags: Z 0 0 0
 */
OR_0xb3 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    or_register_high(cpu, &cpu.af, read_register_low(cpu.de))
}

/*
   0xb4 - OR
   Length: 1
   Cycles: 4
   Flags: Z 0 0 0
 */
OR_0xb4 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    or_register_high(cpu, &cpu.af, read_register_high(cpu.hl))
}

/*
   0xb5 - OR
   Length: 1
   Cycles: 4
   Flags: Z 0 0 0
 */
OR_0xb5 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    or_register_high(cpu, &cpu.af, read_register_low(cpu.hl))
}

/*
   0xb6 - OR
   Length: 1
   Cycles: 8
   Flags: Z 0 0 0
 */
OR_0xb6 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    or_register_high(cpu, &cpu.af, mem_read(mem, cpu.hl))
}

/*
   0xb7 - OR
   Length: 1
   Cycles: 4
   Flags: Z 0 0 0
 */
OR_0xb7 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    or_register_high(cpu, &cpu.af, read_register_high(cpu.af))
}

/*
   0xb8 - CP
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
CP_0xb8 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), read_register_high(cpu.bc))
}

/*
   0xb9 - CP
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
CP_0xb9 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), read_register_low(cpu.bc))
}

/*
   0xba - CP
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
CP_0xba :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), read_register_high(cpu.de))
}

/*
   0xbb - CP
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
CP_0xbb :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), read_register_low(cpu.de))
}

/*
   0xbc - CP
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
CP_0xbc :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), read_register_high(cpu.hl))
}

/*
   0xbd - CP
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
CP_0xbd :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), read_register_low(cpu.hl))
}

/*
   0xbe - CP
   Length: 1
   Cycles: 8
   Flags: Z 1 H C
 */
CP_0xbe :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), mem_read(mem, cpu.hl))
}

/*
   0xbf - CP
   Length: 1
   Cycles: 4
   Flags: Z 1 H C
 */
CP_0xbf :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), read_register_high(cpu.af))
}

/*
   0xc6 - ADD
   Length: 2
   Cycles: 8
   Flags: Z 0 H C
 */
ADD_0xc6 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, u8(data))
}

/*
   0xce - ADC
   Length: 2
   Cycles: 8
   Flags: Z 0 H C
 */
ADC_0xce :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    add_to_register_high(cpu, &cpu.af, u8(data), add_carry = true)
}

/*
   0xd6 - SUB
   Length: 2
   Cycles: 8
   Flags: Z 1 H C
 */
SUB_0xd6 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, u8(data))
}

/*
   0xde - SBC
   Length: 2
   Cycles: 8
   Flags: Z 1 H C
 */
SBC_0xde :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sub_from_register_high(cpu, &cpu.af, u8(data), sub_carry = true)
}

/*
   0xe6 - AND
   Length: 2
   Cycles: 8
   Flags: Z 0 1 0
 */
AND_0xe6 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    and_register_high(cpu, &cpu.af, u8(data))
}

/*
   0xee - XOR
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
XOR_0xee :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    xor_register_high(cpu, &cpu.af, u8(data))
}

/*
   0xf6 - OR
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
OR_0xf6 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    or_register_high(cpu, &cpu.af, u8(data))
}

/*
   0xfe - CP
   Length: 2
   Cycles: 8
   Flags: Z 1 H C
 */
CP_0xfe :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cp_bytes(cpu, read_register_high(cpu.af), u8(data))
}

// GROUP: x8/rsb

/*
   0x07 - RLCA
   Length: 1
   Cycles: 4
   Flags: 0 0 0 C
 */
RLCA_0x07 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    acc := read_register_high(cpu.af)
    c := acc & 0x80 > 0

    write_register_high(&cpu.af, (acc << 1) | u8(c))

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
RRCA_0x0f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    acc := read_register_high(cpu.af)
    c := acc & 1 > 0

    write_register_high(&cpu.af, (acc >> 1) | (u8(c) << 7))

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
RLA_0x17 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    acc := read_register_high(cpu.af)
    c := acc & 0x80 > 0
    acc = (acc << 1) | u8(read_flag(cpu, Flags.C))

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
RRA_0x1f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    acc := read_register_high(cpu.af)
    c := acc & 1 > 0
    acc = (acc >> 1) | (u8(read_flag(cpu, Flags.C)) << 7)

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
JR_0x18 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    jr(cpu, i8(data))
}

/*
   0x20 - JR
   Length: 2
   Cycles: 12
   Flags: - - - -
 */
JR_0x20 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    if !read_flag(cpu, Flags.Z) {
        jr(cpu, i8(data))
    }
}

/*
   0x28 - JR
   Length: 2
   Cycles: 12
   Flags: - - - -
 */
JR_0x28 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
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
JR_0x30 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    if !read_flag(cpu, Flags.C) {
        jr(cpu, i8(data))
    }
}

/*
   0x38 - JR
   Length: 2
   Cycles: 12
   Flags: - - - -
 */
JR_0x38 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
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
RET_0xc0 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    if !read_flag(cpu, Flags.Z) {
        ret(cpu, mem)
    }
}

/*
   0xc2 - JP
   Length: 3
   Cycles: 16
   Flags: - - - -
 */
JP_0xc2 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    if !read_flag(cpu, Flags.Z) {
        cpu.pc = data
    }
}

/*
   0xc3 - JP
   Length: 3
   Cycles: 16
   Flags: - - - -
 */
JP_0xc3 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.pc = data
}

/*
   0xc4 - CALL
   Length: 3
   Cycles: 24
   Flags: - - - -
 */
CALL_0xc4 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    if !read_flag(cpu, Flags.Z) {
        call(cpu, mem, data)
    }
}

/*
   0xc7 - RST
   Length: 1
   Cycles: 16
   Flags: - - - -
 */
RST_0xc7 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    call(cpu, mem, 0)
}

/*
   0xc8 - RET
   Length: 1
   Cycles: 20
   Flags: - - - -
 */
RET_0xc8 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
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
RET_0xc9 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    ret(cpu, mem)
}

/*
   0xca - JP
   Length: 3
   Cycles: 16
   Flags: - - - -
 */
JP_0xca :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
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
CALL_0xcc :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
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
CALL_0xcd :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    call(cpu, mem, data)
}

/*
   0xcf - RST
   Length: 1
   Cycles: 16
   Flags: - - - -
 */
RST_0xcf :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    call(cpu, mem, 0x08)
}

/*
   0xd0 - RET
   Length: 1
   Cycles: 20
   Flags: - - - -
 */
RET_0xd0 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    if !read_flag(cpu, Flags.C) {
        ret(cpu, mem)
    }
}

/*
   0xd2 - JP
   Length: 3
   Cycles: 16
   Flags: - - - -
 */
JP_0xd2 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    if !read_flag(cpu, Flags.C) {
        cpu.pc = data
    }
}

/*
   0xd4 - CALL
   Length: 3
   Cycles: 24
   Flags: - - - -
 */
CALL_0xd4 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    if !read_flag(cpu, Flags.C) {
        call(cpu, mem, data)
    }
}

/*
   0xd7 - RST
   Length: 1
   Cycles: 16
   Flags: - - - -
 */
RST_0xd7 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    call(cpu, mem, 0x10)
}

/*
   0xd8 - RET
   Length: 1
   Cycles: 20
   Flags: - - - -
 */
RET_0xd8 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
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
RETI_0xd9 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    ret(cpu, mem)
    cpu.ime = true
}

/*
   0xda - JP
   Length: 3
   Cycles: 16
   Flags: - - - -
 */
JP_0xda :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
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
CALL_0xdc :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
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
RST_0xdf :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    call(cpu, mem, 0x18)
}

/*
   0xe7 - RST
   Length: 1
   Cycles: 16
   Flags: - - - -
 */
RST_0xe7 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    call(cpu, mem, 0x20)
}

/*
   0xe9 - JP
   Length: 1
   Cycles: 4
   Flags: - - - -
 */
JP_0xe9 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    cpu.pc = cpu.hl
}

/*
   0xef - RST
   Length: 1
   Cycles: 16
   Flags: - - - -
 */
RST_0xef :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    call(cpu, mem, 0x28)
}

/*
   0xf7 - RST
   Length: 1
   Cycles: 16
   Flags: - - - -
 */
RST_0xf7 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    call(cpu, mem, 0x30)
}

/*
   0xff - RST
   Length: 1
   Cycles: 16
   Flags: - - - -
 */
RST_0xff :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    call(cpu, mem, 0x38)
}

// PREFIXED INSTRUCTIONS

// GROUP: x8/rsb

/*
   0x00 - RLC
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RLC_0x00 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rlc_register_high(cpu, &cpu.bc)
}

/*
   0x01 - RLC
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RLC_0x01 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rlc_register_low(cpu, &cpu.bc)
}

/*
   0x02 - RLC
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RLC_0x02 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rlc_register_high(cpu, &cpu.de)
}

/*
   0x03 - RLC
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RLC_0x03 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rlc_register_low(cpu, &cpu.de)
}

/*
   0x04 - RLC
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RLC_0x04 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rlc_register_high(cpu, &cpu.hl)
}

/*
   0x05 - RLC
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RLC_0x05 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rlc_register_low(cpu, &cpu.hl)
}

/*
   0x06 - RLC
   Length: 2
   Cycles: 16
   Flags: Z 0 0 C
 */
RLC_0x06 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rlc(cpu, mem_get_ptr(mem, cpu.hl))
}

/*
   0x07 - RLC
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RLC_0x07 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rlc_register_high(cpu, &cpu.af)
}

/*
   0x08 - RRC
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RRC_0x08 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rrc_register_high(cpu, &cpu.bc)
}

/*
   0x09 - RRC
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RRC_0x09 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rrc_register_low(cpu, &cpu.bc)
}

/*
   0x0a - RRC
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RRC_0x0a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rrc_register_high(cpu, &cpu.de)
}

/*
   0x0b - RRC
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RRC_0x0b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rrc_register_low(cpu, &cpu.de)
}

/*
   0x0c - RRC
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RRC_0x0c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rrc_register_high(cpu, &cpu.hl)
}

/*
   0x0d - RRC
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RRC_0x0d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rrc_register_low(cpu, &cpu.hl)
}

/*
   0x0e - RRC
   Length: 2
   Cycles: 16
   Flags: Z 0 0 C
 */
RRC_0x0e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rrc(cpu, mem_get_ptr(mem, cpu.hl))
}

/*
   0x0f - RRC
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RRC_0x0f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rrc_register_high(cpu, &cpu.af)
}

/*
   0x10 - RL
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RL_0x10 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rl_register_high(cpu, &cpu.bc)
}

/*
   0x11 - RL
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RL_0x11 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rl_register_low(cpu, &cpu.bc)
}

/*
   0x12 - RL
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RL_0x12 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rl_register_high(cpu, &cpu.de)
}

/*
   0x13 - RL
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RL_0x13 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rl_register_low(cpu, &cpu.de)
}

/*
   0x14 - RL
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RL_0x14 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rl_register_high(cpu, &cpu.hl)
}

/*
   0x15 - RL
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RL_0x15 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rl_register_low(cpu, &cpu.hl)
}

/*
   0x16 - RL
   Length: 2
   Cycles: 16
   Flags: Z 0 0 C
 */
RL_0x16 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rl(cpu, mem_get_ptr(mem, cpu.hl))
}

/*
   0x17 - RL
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RL_0x17 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rl_register_high(cpu, &cpu.af)
}

/*
   0x18 - RR
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RR_0x18 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rr_register_high(cpu, &cpu.bc)
}

/*
   0x19 - RR
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RR_0x19 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rr_register_low(cpu, &cpu.bc)
}

/*
   0x1a - RR
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RR_0x1a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rr_register_high(cpu, &cpu.de)
}

/*
   0x1b - RR
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RR_0x1b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rr_register_low(cpu, &cpu.de)
}

/*
   0x1c - RR
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RR_0x1c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rr_register_high(cpu, &cpu.hl)
}

/*
   0x1d - RR
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RR_0x1d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rr_register_low(cpu, &cpu.hl)
}

/*
   0x1e - RR
   Length: 2
   Cycles: 16
   Flags: Z 0 0 C
 */
RR_0x1e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rr(cpu, mem_get_ptr(mem, cpu.hl))
}

/*
   0x1f - RR
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
RR_0x1f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    rr_register_high(cpu, &cpu.af)
}

/*
   0x20 - SLA
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
SLA_0x20 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sla_register_high(cpu, &cpu.bc)
}

/*
   0x21 - SLA
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
SLA_0x21 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sla_register_low(cpu, &cpu.bc)
}

/*
   0x22 - SLA
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
SLA_0x22 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sla_register_high(cpu, &cpu.de)
}

/*
   0x23 - SLA
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
SLA_0x23 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sla_register_low(cpu, &cpu.de)
}

/*
   0x24 - SLA
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
SLA_0x24 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sla_register_high(cpu, &cpu.hl)
}

/*
   0x25 - SLA
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
SLA_0x25 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sla_register_low(cpu, &cpu.hl)
}

/*
   0x26 - SLA
   Length: 2
   Cycles: 16
   Flags: Z 0 0 C
 */
SLA_0x26 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sla(cpu, mem_get_ptr(mem, cpu.hl))
}

/*
   0x27 - SLA
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
SLA_0x27 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sla_register_high(cpu, &cpu.af)
}

/*
   0x28 - SRA
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
SRA_0x28 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sra_register_high(cpu, &cpu.bc)
}

/*
   0x29 - SRA
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
SRA_0x29 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sra_register_low(cpu, &cpu.bc)
}

/*
   0x2a - SRA
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
SRA_0x2a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sra_register_high(cpu, &cpu.de)
}

/*
   0x2b - SRA
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
SRA_0x2b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sra_register_low(cpu, &cpu.de)
}

/*
   0x2c - SRA
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
SRA_0x2c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sra_register_high(cpu, &cpu.hl)
}

/*
   0x2d - SRA
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
SRA_0x2d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sra_register_low(cpu, &cpu.hl)
}

/*
   0x2e - SRA
   Length: 2
   Cycles: 16
   Flags: Z 0 0 0
 */
SRA_0x2e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sra(cpu, mem_get_ptr(mem, cpu.hl))
}

/*
   0x2f - SRA
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
SRA_0x2f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    sra_register_high(cpu, &cpu.af)
}

/*
   0x30 - SWAP
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
SWAP_0x30 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    swap_register_high(cpu, &cpu.bc)
}

/*
   0x31 - SWAP
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
SWAP_0x31 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    swap_register_low(cpu, &cpu.bc)
}

/*
   0x32 - SWAP
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
SWAP_0x32 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    swap_register_high(cpu, &cpu.de)
}

/*
   0x33 - SWAP
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
SWAP_0x33 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    swap_register_low(cpu, &cpu.de)
}

/*
   0x34 - SWAP
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
SWAP_0x34 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    swap_register_high(cpu, &cpu.hl)
}

/*
   0x35 - SWAP
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
SWAP_0x35 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    swap_register_low(cpu, &cpu.hl)
}

/*
   0x36 - SWAP
   Length: 2
   Cycles: 16
   Flags: Z 0 0 0
 */
SWAP_0x36 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    swap(cpu, mem_get_ptr(mem, cpu.hl))
}

/*
   0x37 - SWAP
   Length: 2
   Cycles: 8
   Flags: Z 0 0 0
 */
SWAP_0x37 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    swap_register_high(cpu, &cpu.af)
}

/*
   0x38 - SRL
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
SRL_0x38 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    srl_register_high(cpu, &cpu.bc)
}

/*
   0x39 - SRL
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
SRL_0x39 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    srl_register_low(cpu, &cpu.bc)
}

/*
   0x3a - SRL
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
SRL_0x3a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    srl_register_high(cpu, &cpu.de)
}

/*
   0x3b - SRL
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
SRL_0x3b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    srl_register_low(cpu, &cpu.de)
}

/*
   0x3c - SRL
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
SRL_0x3c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    srl_register_high(cpu, &cpu.hl)
}

/*
   0x3d - SRL
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
SRL_0x3d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    srl_register_low(cpu, &cpu.hl)
}

/*
   0x3e - SRL
   Length: 2
   Cycles: 16
   Flags: Z 0 0 C
 */
SRL_0x3e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    srl(cpu, mem_get_ptr(mem, cpu.hl))
}

/*
   0x3f - SRL
   Length: 2
   Cycles: 8
   Flags: Z 0 0 C
 */
SRL_0x3f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    srl_register_high(cpu, &cpu.af)
}

/*
   0x40 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x40 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.bc, 0)
}

/*
   0x41 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x41 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.bc, 0)
}

/*
   0x42 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x42 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.de, 0)
}

/*
   0x43 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x43 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.de, 0)
}

/*
   0x44 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x44 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.hl, 0)
}

/*
   0x45 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x45 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.hl, 0)
}

/*
   0x46 - BIT
   Length: 2
   Cycles: 16
   Flags: Z 0 1 -
 */
BIT_0x46 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit(cpu, mem_get_ptr(mem, cpu.hl), 0)
}

/*
   0x47 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x47 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.af, 0)
}

/*
   0x48 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x48 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.bc, 1)
}

/*
   0x49 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x49 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.bc, 1)
}

/*
   0x4a - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x4a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.de, 1)
}

/*
   0x4b - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x4b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.de, 1)
}

/*
   0x4c - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x4c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.hl, 1)
}

/*
   0x4d - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x4d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.hl, 1)
}

/*
   0x4e - BIT
   Length: 2
   Cycles: 16
   Flags: Z 0 1 -
 */
BIT_0x4e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit(cpu, mem_get_ptr(mem, cpu.hl), 1)
}

/*
   0x4f - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x4f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.af, 1)
}

/*
   0x50 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x50 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.bc, 2)
}

/*
   0x51 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x51 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.bc, 2)
}

/*
   0x52 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x52 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.de, 2)
}

/*
   0x53 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x53 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.de, 2)
}

/*
   0x54 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x54 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.hl, 2)
}

/*
   0x55 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x55 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.hl, 2)
}

/*
   0x56 - BIT
   Length: 2
   Cycles: 16
   Flags: Z 0 1 -
 */
BIT_0x56 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit(cpu, mem_get_ptr(mem, cpu.hl), 2)
}

/*
   0x57 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x57 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.af, 2)
}

/*
   0x58 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x58 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.bc, 3)
}

/*
   0x59 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x59 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.bc, 3)
}

/*
   0x5a - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x5a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.de, 3)
}

/*
   0x5b - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x5b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.de, 3)
}

/*
   0x5c - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x5c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.hl, 3)
}

/*
   0x5d - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x5d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.hl, 3)
}

/*
   0x5e - BIT
   Length: 2
   Cycles: 16
   Flags: Z 0 1 -
 */
BIT_0x5e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit(cpu, mem_get_ptr(mem, cpu.hl), 3)
}

/*
   0x5f - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x5f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.af, 3)
}

/*
   0x60 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x60 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.bc, 4)
}

/*
   0x61 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x61 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.bc, 4)
}

/*
   0x62 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x62 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.de, 4)
}

/*
   0x63 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x63 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.de, 4)
}

/*
   0x64 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x64 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.hl, 4)
}

/*
   0x65 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x65 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.hl, 4)
}

/*
   0x66 - BIT
   Length: 2
   Cycles: 16
   Flags: Z 0 1 -
 */
BIT_0x66 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit(cpu, mem_get_ptr(mem, cpu.hl), 4)
}

/*
   0x67 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x67 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.af, 4)
}

/*
   0x68 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x68 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.bc, 5)
}

/*
   0x69 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x69 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.bc, 5)
}

/*
   0x6a - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x6a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.de, 5)
}

/*
   0x6b - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x6b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.de, 5)
}

/*
   0x6c - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x6c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.hl, 5)
}

/*
   0x6d - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x6d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.hl, 5)
}

/*
   0x6e - BIT
   Length: 2
   Cycles: 16
   Flags: Z 0 1 -
 */
BIT_0x6e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit(cpu, mem_get_ptr(mem, cpu.hl), 5)
}

/*
   0x6f - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x6f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.af, 5)
}

/*
   0x70 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x70 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.bc, 6)
}

/*
   0x71 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x71 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.bc, 6)
}

/*
   0x72 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x72 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.de, 6)
}

/*
   0x73 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x73 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.de, 6)
}

/*
   0x74 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x74 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.hl, 6)
}

/*
   0x75 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x75 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.hl, 6)
}

/*
   0x76 - BIT
   Length: 2
   Cycles: 16
   Flags: Z 0 1 -
 */
BIT_0x76 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit(cpu, mem_get_ptr(mem, cpu.hl), 6)
}

/*
   0x77 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x77 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.af, 6)
}

/*
   0x78 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x78 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.bc, 7)
}

/*
   0x79 - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x79 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.bc, 7)
}

/*
   0x7a - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x7a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.de, 7)
}

/*
   0x7b - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x7b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.de, 7)
}

/*
   0x7c - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x7c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.hl, 7)
}

/*
   0x7d - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x7d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_low(cpu, &cpu.hl, 7)
}

/*
   0x7e - BIT
   Length: 2
   Cycles: 16
   Flags: Z 0 1 -
 */
BIT_0x7e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit(cpu, mem_get_ptr(mem, cpu.hl), 7)
}

/*
   0x7f - BIT
   Length: 2
   Cycles: 8
   Flags: Z 0 1 -
 */
BIT_0x7f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    bit_register_high(cpu, &cpu.af, 7)
}

/*
   0x80 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x80 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.bc, 0)
}

/*
   0x81 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x81 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.bc, 0)
}

/*
   0x82 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x82 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.de, 0)
}

/*
   0x83 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x83 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.de, 0)
}

/*
   0x84 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x84 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.hl, 0)
}

/*
   0x85 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x85 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.hl, 0)
}

/*
   0x86 - RES
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
RES_0x86 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res(cpu, mem_get_ptr(mem, cpu.hl), 0)
}

/*
   0x87 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x87 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.af, 0)
}

/*
   0x88 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x88 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.bc, 1)
}

/*
   0x89 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x89 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.bc, 1)
}

/*
   0x8a - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x8a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.de, 1)
}

/*
   0x8b - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x8b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.de, 1)
}

/*
   0x8c - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x8c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.hl, 1)
}

/*
   0x8d - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x8d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.hl, 1)
}

/*
   0x8e - RES
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
RES_0x8e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res(cpu, mem_get_ptr(mem, cpu.hl), 1)
}

/*
   0x8f - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x8f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.af, 1)
}

/*
   0x90 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x90 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.bc, 2)
}

/*
   0x91 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x91 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.bc, 2)
}

/*
   0x92 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x92 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.de, 2)
}

/*
   0x93 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x93 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.de, 2)
}

/*
   0x94 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x94 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.hl, 2)
}

/*
   0x95 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x95 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.hl, 2)
}

/*
   0x96 - RES
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
RES_0x96 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res(cpu, mem_get_ptr(mem, cpu.hl), 2)
}

/*
   0x97 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x97 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.af, 2)
}

/*
   0x98 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x98 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.bc, 3)
}

/*
   0x99 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x99 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.bc, 3)
}

/*
   0x9a - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x9a :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.de, 3)
}

/*
   0x9b - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x9b :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.de, 3)
}

/*
   0x9c - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x9c :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.hl, 3)
}

/*
   0x9d - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x9d :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.hl, 3)
}

/*
   0x9e - RES
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
RES_0x9e :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res(cpu, mem_get_ptr(mem, cpu.hl), 3)
}

/*
   0x9f - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0x9f :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.af, 3)
}

/*
   0xa0 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xa0 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.bc, 4)
}

/*
   0xa1 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xa1 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.bc, 4)
}

/*
   0xa2 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xa2 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.de, 4)
}

/*
   0xa3 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xa3 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.de, 4)
}

/*
   0xa4 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xa4 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.hl, 4)
}

/*
   0xa5 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xa5 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.hl, 4)
}

/*
   0xa6 - RES
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
RES_0xa6 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res(cpu, mem_get_ptr(mem, cpu.hl), 4)
}

/*
   0xa7 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xa7 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.af, 4)
}

/*
   0xa8 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xa8 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.bc, 5)
}

/*
   0xa9 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xa9 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.bc, 5)
}

/*
   0xaa - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xaa :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.de, 5)
}

/*
   0xab - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xab :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.de, 5)
}

/*
   0xac - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xac :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.hl, 5)
}

/*
   0xad - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xad :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.hl, 5)
}

/*
   0xae - RES
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
RES_0xae :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res(cpu, mem_get_ptr(mem, cpu.hl), 5)
}

/*
   0xaf - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xaf :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.af, 5)
}

/*
   0xb0 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xb0 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.bc, 6)
}

/*
   0xb1 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xb1 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.bc, 6)
}

/*
   0xb2 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xb2 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.de, 6)
}

/*
   0xb3 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xb3 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.de, 6)
}

/*
   0xb4 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xb4 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.hl, 6)
}

/*
   0xb5 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xb5 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.hl, 6)
}

/*
   0xb6 - RES
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
RES_0xb6 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res(cpu, mem_get_ptr(mem, cpu.hl), 6)
}

/*
   0xb7 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xb7 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.af, 6)
}

/*
   0xb8 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xb8 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.bc, 7)
}

/*
   0xb9 - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xb9 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.bc, 7)
}

/*
   0xba - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xba :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.de, 7)
}

/*
   0xbb - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xbb :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.de, 7)
}

/*
   0xbc - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xbc :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.hl, 7)
}

/*
   0xbd - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xbd :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_low(cpu, &cpu.hl, 7)
}

/*
   0xbe - RES
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
RES_0xbe :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res(cpu, mem_get_ptr(mem, cpu.hl), 7)
}

/*
   0xbf - RES
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
RES_0xbf :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    res_register_high(cpu, &cpu.af, 7)
}

/*
   0xc0 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xc0 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.bc, 0)
}

/*
   0xc1 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xc1 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.bc, 0)
}

/*
   0xc2 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xc2 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.de, 0)
}

/*
   0xc3 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xc3 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.de, 0)
}

/*
   0xc4 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xc4 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.hl, 0)
}

/*
   0xc5 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xc5 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.hl, 0)
}

/*
   0xc6 - SET
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
SET_0xc6 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set(cpu, mem_get_ptr(mem, cpu.hl), 0)
}

/*
   0xc7 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xc7 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.af, 0)
}

/*
   0xc8 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xc8 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.bc, 1)
}

/*
   0xc9 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xc9 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.bc, 1)
}

/*
   0xca - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xca :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.de, 1)
}

/*
   0xcb - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xcb :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.de, 1)
}

/*
   0xcc - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xcc :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.hl, 1)
}

/*
   0xcd - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xcd :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.hl, 1)
}

/*
   0xce - SET
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
SET_0xce :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set(cpu, mem_get_ptr(mem, cpu.hl), 1)
}

/*
   0xcf - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xcf :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.af, 1)
}

/*
   0xd0 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xd0 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.bc, 2)
}

/*
   0xd1 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xd1 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.bc, 2)
}

/*
   0xd2 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xd2 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.de, 2)
}

/*
   0xd3 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xd3 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.de, 2)
}

/*
   0xd4 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xd4 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.hl, 2)
}

/*
   0xd5 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xd5 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.hl, 2)
}

/*
   0xd6 - SET
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
SET_0xd6 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set(cpu, mem_get_ptr(mem, cpu.hl), 2)
}

/*
   0xd7 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xd7 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.af, 2)
}

/*
   0xd8 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xd8 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.bc, 3)
}

/*
   0xd9 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xd9 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.bc, 3)
}

/*
   0xda - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xda :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.de, 3)
}

/*
   0xdb - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xdb :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.de, 3)
}

/*
   0xdc - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xdc :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.hl, 3)
}

/*
   0xdd - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xdd :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.hl, 3)
}

/*
   0xde - SET
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
SET_0xde :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set(cpu, mem_get_ptr(mem, cpu.hl), 3)
}

/*
   0xdf - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xdf :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.af, 3)
}

/*
   0xe0 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xe0 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.bc, 4)
}

/*
   0xe1 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xe1 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.bc, 4)
}

/*
   0xe2 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xe2 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.de, 4)
}

/*
   0xe3 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xe3 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.de, 4)
}

/*
   0xe4 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xe4 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.hl, 4)
}

/*
   0xe5 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xe5 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.hl, 4)
}

/*
   0xe6 - SET
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
SET_0xe6 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set(cpu, mem_get_ptr(mem, cpu.hl), 4)
}

/*
   0xe7 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xe7 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.af, 4)
}

/*
   0xe8 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xe8 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.bc, 5)
}

/*
   0xe9 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xe9 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.bc, 5)
}

/*
   0xea - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xea :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.de, 5)
}

/*
   0xeb - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xeb :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.de, 5)
}

/*
   0xec - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xec :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.hl, 5)
}

/*
   0xed - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xed :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.hl, 5)
}

/*
   0xee - SET
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
SET_0xee :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set(cpu, mem_get_ptr(mem, cpu.hl), 5)
}

/*
   0xef - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xef :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.af, 5)
}

/*
   0xf0 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xf0 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.bc, 6)
}

/*
   0xf1 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xf1 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.bc, 6)
}

/*
   0xf2 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xf2 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.de, 6)
}

/*
   0xf3 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xf3 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.de, 6)
}

/*
   0xf4 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xf4 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.hl, 6)
}

/*
   0xf5 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xf5 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.hl, 6)
}

/*
   0xf6 - SET
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
SET_0xf6 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set(cpu, mem_get_ptr(mem, cpu.hl), 6)
}

/*
   0xf7 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xf7 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.af, 6)
}

/*
   0xf8 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xf8 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.bc, 7)
}

/*
   0xf9 - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xf9 :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.bc, 7)
}

/*
   0xfa - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xfa :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.de, 7)
}

/*
   0xfb - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xfb :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.de, 7)
}

/*
   0xfc - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xfc :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.hl, 7)
}

/*
   0xfd - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xfd :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_low(cpu, &cpu.hl, 7)
}

/*
   0xfe - SET
   Length: 2
   Cycles: 16
   Flags: - - - -
 */
SET_0xfe :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set(cpu, mem_get_ptr(mem, cpu.hl), 7)
}

/*
   0xff - SET
   Length: 2
   Cycles: 8
   Flags: - - - -
 */
SET_0xff :: proc(cpu: ^CPU, mem: ^GB_Memory, data: u16) {
    set_register_high(cpu, &cpu.af, 7)
}

create_unprefixed_instructions :: proc(cpu: ^CPU) { 
    cpu.instructions[0x00] = Instruction{1, 4, "NOP", NOP_0x00}
    cpu.instructions[0x01] = Instruction{3, 12, "LD BC d16", LD_0x01}
    cpu.instructions[0x02] = Instruction{1, 8, "LD (BC) A", LD_0x02}
    cpu.instructions[0x03] = Instruction{1, 8, "INC BC", INC_0x03}
    cpu.instructions[0x04] = Instruction{1, 4, "INC B", INC_0x04}
    cpu.instructions[0x05] = Instruction{1, 4, "DEC B", DEC_0x05}
    cpu.instructions[0x06] = Instruction{2, 8, "LD B d8", LD_0x06}
    cpu.instructions[0x07] = Instruction{1, 4, "RLCA", RLCA_0x07}
    cpu.instructions[0x08] = Instruction{3, 20, "LD (a16) SP", LD_0x08}
    cpu.instructions[0x09] = Instruction{1, 8, "ADD HL BC", ADD_0x09}
    cpu.instructions[0x0a] = Instruction{1, 8, "LD A (BC)", LD_0x0a}
    cpu.instructions[0x0b] = Instruction{1, 8, "DEC BC", DEC_0x0b}
    cpu.instructions[0x0c] = Instruction{1, 4, "INC C", INC_0x0c}
    cpu.instructions[0x0d] = Instruction{1, 4, "DEC C", DEC_0x0d}
    cpu.instructions[0x0e] = Instruction{2, 8, "LD C d8", LD_0x0e}
    cpu.instructions[0x0f] = Instruction{1, 4, "RRCA", RRCA_0x0f}
    cpu.instructions[0x10] = Instruction{1, 4, "STOP 0", STOP_0x10}
    cpu.instructions[0x11] = Instruction{3, 12, "LD DE d16", LD_0x11}
    cpu.instructions[0x12] = Instruction{1, 8, "LD (DE) A", LD_0x12}
    cpu.instructions[0x13] = Instruction{1, 8, "INC DE", INC_0x13}
    cpu.instructions[0x14] = Instruction{1, 4, "INC D", INC_0x14}
    cpu.instructions[0x15] = Instruction{1, 4, "DEC D", DEC_0x15}
    cpu.instructions[0x16] = Instruction{2, 8, "LD D d8", LD_0x16}
    cpu.instructions[0x17] = Instruction{1, 4, "RLA", RLA_0x17}
    cpu.instructions[0x18] = Instruction{2, 12, "JR r8", JR_0x18}
    cpu.instructions[0x19] = Instruction{1, 8, "ADD HL DE", ADD_0x19}
    cpu.instructions[0x1a] = Instruction{1, 8, "LD A (DE)", LD_0x1a}
    cpu.instructions[0x1b] = Instruction{1, 8, "DEC DE", DEC_0x1b}
    cpu.instructions[0x1c] = Instruction{1, 4, "INC E", INC_0x1c}
    cpu.instructions[0x1d] = Instruction{1, 4, "DEC E", DEC_0x1d}
    cpu.instructions[0x1e] = Instruction{2, 8, "LD E d8", LD_0x1e}
    cpu.instructions[0x1f] = Instruction{1, 4, "RRA", RRA_0x1f}
    cpu.instructions[0x20] = Instruction{2, 12, "JR NZ r8", JR_0x20}
    cpu.instructions[0x21] = Instruction{3, 12, "LD HL d16", LD_0x21}
    cpu.instructions[0x22] = Instruction{1, 8, "LD (HL+) A", LD_0x22}
    cpu.instructions[0x23] = Instruction{1, 8, "INC HL", INC_0x23}
    cpu.instructions[0x24] = Instruction{1, 4, "INC H", INC_0x24}
    cpu.instructions[0x25] = Instruction{1, 4, "DEC H", DEC_0x25}
    cpu.instructions[0x26] = Instruction{2, 8, "LD H d8", LD_0x26}
    cpu.instructions[0x27] = Instruction{1, 4, "DAA", DAA_0x27}
    cpu.instructions[0x28] = Instruction{2, 12, "JR Z r8", JR_0x28}
    cpu.instructions[0x29] = Instruction{1, 8, "ADD HL HL", ADD_0x29}
    cpu.instructions[0x2a] = Instruction{1, 8, "LD A (HL+)", LD_0x2a}
    cpu.instructions[0x2b] = Instruction{1, 8, "DEC HL", DEC_0x2b}
    cpu.instructions[0x2c] = Instruction{1, 4, "INC L", INC_0x2c}
    cpu.instructions[0x2d] = Instruction{1, 4, "DEC L", DEC_0x2d}
    cpu.instructions[0x2e] = Instruction{2, 8, "LD L d8", LD_0x2e}
    cpu.instructions[0x2f] = Instruction{1, 4, "CPL", CPL_0x2f}
    cpu.instructions[0x30] = Instruction{2, 12, "JR NC r8", JR_0x30}
    cpu.instructions[0x31] = Instruction{3, 12, "LD SP d16", LD_0x31}
    cpu.instructions[0x32] = Instruction{1, 8, "LD (HL-) A", LD_0x32}
    cpu.instructions[0x33] = Instruction{1, 8, "INC SP", INC_0x33}
    cpu.instructions[0x34] = Instruction{1, 12, "INC (HL)", INC_0x34}
    cpu.instructions[0x35] = Instruction{1, 12, "DEC (HL)", DEC_0x35}
    cpu.instructions[0x36] = Instruction{2, 12, "LD (HL) d8", LD_0x36}
    cpu.instructions[0x37] = Instruction{1, 4, "SCF", SCF_0x37}
    cpu.instructions[0x38] = Instruction{2, 12, "JR C r8", JR_0x38}
    cpu.instructions[0x39] = Instruction{1, 8, "ADD HL SP", ADD_0x39}
    cpu.instructions[0x3a] = Instruction{1, 8, "LD A (HL-)", LD_0x3a}
    cpu.instructions[0x3b] = Instruction{1, 8, "DEC SP", DEC_0x3b}
    cpu.instructions[0x3c] = Instruction{1, 4, "INC A", INC_0x3c}
    cpu.instructions[0x3d] = Instruction{1, 4, "DEC A", DEC_0x3d}
    cpu.instructions[0x3e] = Instruction{2, 8, "LD A d8", LD_0x3e}
    cpu.instructions[0x3f] = Instruction{1, 4, "CCF", CCF_0x3f}
    cpu.instructions[0x40] = Instruction{1, 4, "LD B B", LD_0x40}
    cpu.instructions[0x41] = Instruction{1, 4, "LD B C", LD_0x41}
    cpu.instructions[0x42] = Instruction{1, 4, "LD B D", LD_0x42}
    cpu.instructions[0x43] = Instruction{1, 4, "LD B E", LD_0x43}
    cpu.instructions[0x44] = Instruction{1, 4, "LD B H", LD_0x44}
    cpu.instructions[0x45] = Instruction{1, 4, "LD B L", LD_0x45}
    cpu.instructions[0x46] = Instruction{1, 8, "LD B (HL)", LD_0x46}
    cpu.instructions[0x47] = Instruction{1, 4, "LD B A", LD_0x47}
    cpu.instructions[0x48] = Instruction{1, 4, "LD C B", LD_0x48}
    cpu.instructions[0x49] = Instruction{1, 4, "LD C C", LD_0x49}
    cpu.instructions[0x4a] = Instruction{1, 4, "LD C D", LD_0x4a}
    cpu.instructions[0x4b] = Instruction{1, 4, "LD C E", LD_0x4b}
    cpu.instructions[0x4c] = Instruction{1, 4, "LD C H", LD_0x4c}
    cpu.instructions[0x4d] = Instruction{1, 4, "LD C L", LD_0x4d}
    cpu.instructions[0x4e] = Instruction{1, 8, "LD C (HL)", LD_0x4e}
    cpu.instructions[0x4f] = Instruction{1, 4, "LD C A", LD_0x4f}
    cpu.instructions[0x50] = Instruction{1, 4, "LD D B", LD_0x50}
    cpu.instructions[0x51] = Instruction{1, 4, "LD D C", LD_0x51}
    cpu.instructions[0x52] = Instruction{1, 4, "LD D D", LD_0x52}
    cpu.instructions[0x53] = Instruction{1, 4, "LD D E", LD_0x53}
    cpu.instructions[0x54] = Instruction{1, 4, "LD D H", LD_0x54}
    cpu.instructions[0x55] = Instruction{1, 4, "LD D L", LD_0x55}
    cpu.instructions[0x56] = Instruction{1, 8, "LD D (HL)", LD_0x56}
    cpu.instructions[0x57] = Instruction{1, 4, "LD D A", LD_0x57}
    cpu.instructions[0x58] = Instruction{1, 4, "LD E B", LD_0x58}
    cpu.instructions[0x59] = Instruction{1, 4, "LD E C", LD_0x59}
    cpu.instructions[0x5a] = Instruction{1, 4, "LD E D", LD_0x5a}
    cpu.instructions[0x5b] = Instruction{1, 4, "LD E E", LD_0x5b}
    cpu.instructions[0x5c] = Instruction{1, 4, "LD E H", LD_0x5c}
    cpu.instructions[0x5d] = Instruction{1, 4, "LD E L", LD_0x5d}
    cpu.instructions[0x5e] = Instruction{1, 8, "LD E (HL)", LD_0x5e}
    cpu.instructions[0x5f] = Instruction{1, 4, "LD E A", LD_0x5f}
    cpu.instructions[0x60] = Instruction{1, 4, "LD H B", LD_0x60}
    cpu.instructions[0x61] = Instruction{1, 4, "LD H C", LD_0x61}
    cpu.instructions[0x62] = Instruction{1, 4, "LD H D", LD_0x62}
    cpu.instructions[0x63] = Instruction{1, 4, "LD H E", LD_0x63}
    cpu.instructions[0x64] = Instruction{1, 4, "LD H H", LD_0x64}
    cpu.instructions[0x65] = Instruction{1, 4, "LD H L", LD_0x65}
    cpu.instructions[0x66] = Instruction{1, 8, "LD H (HL)", LD_0x66}
    cpu.instructions[0x67] = Instruction{1, 4, "LD H A", LD_0x67}
    cpu.instructions[0x68] = Instruction{1, 4, "LD L B", LD_0x68}
    cpu.instructions[0x69] = Instruction{1, 4, "LD L C", LD_0x69}
    cpu.instructions[0x6a] = Instruction{1, 4, "LD L D", LD_0x6a}
    cpu.instructions[0x6b] = Instruction{1, 4, "LD L E", LD_0x6b}
    cpu.instructions[0x6c] = Instruction{1, 4, "LD L H", LD_0x6c}
    cpu.instructions[0x6d] = Instruction{1, 4, "LD L L", LD_0x6d}
    cpu.instructions[0x6e] = Instruction{1, 8, "LD L (HL)", LD_0x6e}
    cpu.instructions[0x6f] = Instruction{1, 4, "LD L A", LD_0x6f}
    cpu.instructions[0x70] = Instruction{1, 8, "LD (HL) B", LD_0x70}
    cpu.instructions[0x71] = Instruction{1, 8, "LD (HL) C", LD_0x71}
    cpu.instructions[0x72] = Instruction{1, 8, "LD (HL) D", LD_0x72}
    cpu.instructions[0x73] = Instruction{1, 8, "LD (HL) E", LD_0x73}
    cpu.instructions[0x74] = Instruction{1, 8, "LD (HL) H", LD_0x74}
    cpu.instructions[0x75] = Instruction{1, 8, "LD (HL) L", LD_0x75}
    cpu.instructions[0x76] = Instruction{1, 4, "HALT", HALT_0x76}
    cpu.instructions[0x77] = Instruction{1, 8, "LD (HL) A", LD_0x77}
    cpu.instructions[0x78] = Instruction{1, 4, "LD A B", LD_0x78}
    cpu.instructions[0x79] = Instruction{1, 4, "LD A C", LD_0x79}
    cpu.instructions[0x7a] = Instruction{1, 4, "LD A D", LD_0x7a}
    cpu.instructions[0x7b] = Instruction{1, 4, "LD A E", LD_0x7b}
    cpu.instructions[0x7c] = Instruction{1, 4, "LD A H", LD_0x7c}
    cpu.instructions[0x7d] = Instruction{1, 4, "LD A L", LD_0x7d}
    cpu.instructions[0x7e] = Instruction{1, 8, "LD A (HL)", LD_0x7e}
    cpu.instructions[0x7f] = Instruction{1, 4, "LD A A", LD_0x7f}
    cpu.instructions[0x80] = Instruction{1, 4, "ADD A B", ADD_0x80}
    cpu.instructions[0x81] = Instruction{1, 4, "ADD A C", ADD_0x81}
    cpu.instructions[0x82] = Instruction{1, 4, "ADD A D", ADD_0x82}
    cpu.instructions[0x83] = Instruction{1, 4, "ADD A E", ADD_0x83}
    cpu.instructions[0x84] = Instruction{1, 4, "ADD A H", ADD_0x84}
    cpu.instructions[0x85] = Instruction{1, 4, "ADD A L", ADD_0x85}
    cpu.instructions[0x86] = Instruction{1, 8, "ADD A (HL)", ADD_0x86}
    cpu.instructions[0x87] = Instruction{1, 4, "ADD A A", ADD_0x87}
    cpu.instructions[0x88] = Instruction{1, 4, "ADC A B", ADC_0x88}
    cpu.instructions[0x89] = Instruction{1, 4, "ADC A C", ADC_0x89}
    cpu.instructions[0x8a] = Instruction{1, 4, "ADC A D", ADC_0x8a}
    cpu.instructions[0x8b] = Instruction{1, 4, "ADC A E", ADC_0x8b}
    cpu.instructions[0x8c] = Instruction{1, 4, "ADC A H", ADC_0x8c}
    cpu.instructions[0x8d] = Instruction{1, 4, "ADC A L", ADC_0x8d}
    cpu.instructions[0x8e] = Instruction{1, 8, "ADC A (HL)", ADC_0x8e}
    cpu.instructions[0x8f] = Instruction{1, 4, "ADC A A", ADC_0x8f}
    cpu.instructions[0x90] = Instruction{1, 4, "SUB B", SUB_0x90}
    cpu.instructions[0x91] = Instruction{1, 4, "SUB C", SUB_0x91}
    cpu.instructions[0x92] = Instruction{1, 4, "SUB D", SUB_0x92}
    cpu.instructions[0x93] = Instruction{1, 4, "SUB E", SUB_0x93}
    cpu.instructions[0x94] = Instruction{1, 4, "SUB H", SUB_0x94}
    cpu.instructions[0x95] = Instruction{1, 4, "SUB L", SUB_0x95}
    cpu.instructions[0x96] = Instruction{1, 8, "SUB (HL)", SUB_0x96}
    cpu.instructions[0x97] = Instruction{1, 4, "SUB A", SUB_0x97}
    cpu.instructions[0x98] = Instruction{1, 4, "SBC A B", SBC_0x98}
    cpu.instructions[0x99] = Instruction{1, 4, "SBC A C", SBC_0x99}
    cpu.instructions[0x9a] = Instruction{1, 4, "SBC A D", SBC_0x9a}
    cpu.instructions[0x9b] = Instruction{1, 4, "SBC A E", SBC_0x9b}
    cpu.instructions[0x9c] = Instruction{1, 4, "SBC A H", SBC_0x9c}
    cpu.instructions[0x9d] = Instruction{1, 4, "SBC A L", SBC_0x9d}
    cpu.instructions[0x9e] = Instruction{1, 8, "SBC A (HL)", SBC_0x9e}
    cpu.instructions[0x9f] = Instruction{1, 4, "SBC A A", SBC_0x9f}
    cpu.instructions[0xa0] = Instruction{1, 4, "AND B", AND_0xa0}
    cpu.instructions[0xa1] = Instruction{1, 4, "AND C", AND_0xa1}
    cpu.instructions[0xa2] = Instruction{1, 4, "AND D", AND_0xa2}
    cpu.instructions[0xa3] = Instruction{1, 4, "AND E", AND_0xa3}
    cpu.instructions[0xa4] = Instruction{1, 4, "AND H", AND_0xa4}
    cpu.instructions[0xa5] = Instruction{1, 4, "AND L", AND_0xa5}
    cpu.instructions[0xa6] = Instruction{1, 8, "AND (HL)", AND_0xa6}
    cpu.instructions[0xa7] = Instruction{1, 4, "AND A", AND_0xa7}
    cpu.instructions[0xa8] = Instruction{1, 4, "XOR B", XOR_0xa8}
    cpu.instructions[0xa9] = Instruction{1, 4, "XOR C", XOR_0xa9}
    cpu.instructions[0xaa] = Instruction{1, 4, "XOR D", XOR_0xaa}
    cpu.instructions[0xab] = Instruction{1, 4, "XOR E", XOR_0xab}
    cpu.instructions[0xac] = Instruction{1, 4, "XOR H", XOR_0xac}
    cpu.instructions[0xad] = Instruction{1, 4, "XOR L", XOR_0xad}
    cpu.instructions[0xae] = Instruction{1, 8, "XOR (HL)", XOR_0xae}
    cpu.instructions[0xaf] = Instruction{1, 4, "XOR A", XOR_0xaf}
    cpu.instructions[0xb0] = Instruction{1, 4, "OR B", OR_0xb0}
    cpu.instructions[0xb1] = Instruction{1, 4, "OR C", OR_0xb1}
    cpu.instructions[0xb2] = Instruction{1, 4, "OR D", OR_0xb2}
    cpu.instructions[0xb3] = Instruction{1, 4, "OR E", OR_0xb3}
    cpu.instructions[0xb4] = Instruction{1, 4, "OR H", OR_0xb4}
    cpu.instructions[0xb5] = Instruction{1, 4, "OR L", OR_0xb5}
    cpu.instructions[0xb6] = Instruction{1, 8, "OR (HL)", OR_0xb6}
    cpu.instructions[0xb7] = Instruction{1, 4, "OR A", OR_0xb7}
    cpu.instructions[0xb8] = Instruction{1, 4, "CP B", CP_0xb8}
    cpu.instructions[0xb9] = Instruction{1, 4, "CP C", CP_0xb9}
    cpu.instructions[0xba] = Instruction{1, 4, "CP D", CP_0xba}
    cpu.instructions[0xbb] = Instruction{1, 4, "CP E", CP_0xbb}
    cpu.instructions[0xbc] = Instruction{1, 4, "CP H", CP_0xbc}
    cpu.instructions[0xbd] = Instruction{1, 4, "CP L", CP_0xbd}
    cpu.instructions[0xbe] = Instruction{1, 8, "CP (HL)", CP_0xbe}
    cpu.instructions[0xbf] = Instruction{1, 4, "CP A", CP_0xbf}
    cpu.instructions[0xc0] = Instruction{1, 20, "RET NZ", RET_0xc0}
    cpu.instructions[0xc1] = Instruction{1, 12, "POP BC", POP_0xc1}
    cpu.instructions[0xc2] = Instruction{3, 16, "JP NZ a16", JP_0xc2}
    cpu.instructions[0xc3] = Instruction{3, 16, "JP a16", JP_0xc3}
    cpu.instructions[0xc4] = Instruction{3, 24, "CALL NZ a16", CALL_0xc4}
    cpu.instructions[0xc5] = Instruction{1, 16, "PUSH BC", PUSH_0xc5}
    cpu.instructions[0xc6] = Instruction{2, 8, "ADD A d8", ADD_0xc6}
    cpu.instructions[0xc7] = Instruction{1, 16, "RST 00H", RST_0xc7}
    cpu.instructions[0xc8] = Instruction{1, 20, "RET Z", RET_0xc8}
    cpu.instructions[0xc9] = Instruction{1, 16, "RET", RET_0xc9}
    cpu.instructions[0xca] = Instruction{3, 16, "JP Z a16", JP_0xca}
    cpu.instructions[0xcb] = Instruction{1, 4, "PREFIX CB", PREFIX_0xcb}
    cpu.instructions[0xcc] = Instruction{3, 24, "CALL Z a16", CALL_0xcc}
    cpu.instructions[0xcd] = Instruction{3, 24, "CALL a16", CALL_0xcd}
    cpu.instructions[0xce] = Instruction{2, 8, "ADC A d8", ADC_0xce}
    cpu.instructions[0xcf] = Instruction{1, 16, "RST 08H", RST_0xcf}
    cpu.instructions[0xd0] = Instruction{1, 20, "RET NC", RET_0xd0}
    cpu.instructions[0xd1] = Instruction{1, 12, "POP DE", POP_0xd1}
    cpu.instructions[0xd2] = Instruction{3, 16, "JP NC a16", JP_0xd2}
    cpu.instructions[0xd4] = Instruction{3, 24, "CALL NC a16", CALL_0xd4}
    cpu.instructions[0xd5] = Instruction{1, 16, "PUSH DE", PUSH_0xd5}
    cpu.instructions[0xd6] = Instruction{2, 8, "SUB d8", SUB_0xd6}
    cpu.instructions[0xd7] = Instruction{1, 16, "RST 10H", RST_0xd7}
    cpu.instructions[0xd8] = Instruction{1, 20, "RET C", RET_0xd8}
    cpu.instructions[0xd9] = Instruction{1, 16, "RETI", RETI_0xd9}
    cpu.instructions[0xda] = Instruction{3, 16, "JP C a16", JP_0xda}
    cpu.instructions[0xdc] = Instruction{3, 24, "CALL C a16", CALL_0xdc}
    cpu.instructions[0xde] = Instruction{2, 8, "SBC A d8", SBC_0xde}
    cpu.instructions[0xdf] = Instruction{1, 16, "RST 18H", RST_0xdf}
    cpu.instructions[0xe0] = Instruction{2, 12, "LDH (a8) A", LDH_0xe0}
    cpu.instructions[0xe1] = Instruction{1, 12, "POP HL", POP_0xe1}
    cpu.instructions[0xe2] = Instruction{1, 8, "LD (C) A", LD_0xe2}
    cpu.instructions[0xe5] = Instruction{1, 16, "PUSH HL", PUSH_0xe5}
    cpu.instructions[0xe6] = Instruction{2, 8, "AND d8", AND_0xe6}
    cpu.instructions[0xe7] = Instruction{1, 16, "RST 20H", RST_0xe7}
    cpu.instructions[0xe8] = Instruction{2, 16, "ADD SP r8", ADD_0xe8}
    cpu.instructions[0xe9] = Instruction{1, 4, "JP (HL)", JP_0xe9}
    cpu.instructions[0xea] = Instruction{3, 16, "LD (a16) A", LD_0xea}
    cpu.instructions[0xee] = Instruction{2, 8, "XOR d8", XOR_0xee}
    cpu.instructions[0xef] = Instruction{1, 16, "RST 28H", RST_0xef}
    cpu.instructions[0xf0] = Instruction{2, 12, "LDH A (a8)", LDH_0xf0}
    cpu.instructions[0xf1] = Instruction{1, 12, "POP AF", POP_0xf1}
    cpu.instructions[0xf2] = Instruction{1, 8, "LD A (C)", LD_0xf2}
    cpu.instructions[0xf3] = Instruction{1, 4, "DI", DI_0xf3}
    cpu.instructions[0xf5] = Instruction{1, 16, "PUSH AF", PUSH_0xf5}
    cpu.instructions[0xf6] = Instruction{2, 8, "OR d8", OR_0xf6}
    cpu.instructions[0xf7] = Instruction{1, 16, "RST 30H", RST_0xf7}
    cpu.instructions[0xf8] = Instruction{2, 12, "LD HL SP+r8", LD_0xf8}
    cpu.instructions[0xf9] = Instruction{1, 8, "LD SP HL", LD_0xf9}
    cpu.instructions[0xfa] = Instruction{3, 16, "LD A (a16)", LD_0xfa}
    cpu.instructions[0xfb] = Instruction{1, 4, "EI", EI_0xfb}
    cpu.instructions[0xfe] = Instruction{2, 8, "CP d8", CP_0xfe}
    cpu.instructions[0xff] = Instruction{1, 16, "RST 38H", RST_0xff}
}

create_prefixed_instructions :: proc(cpu: ^CPU) {
    cpu.prefixed_instructions[0x00] = Instruction{1, 8, "RLC B", RLC_0x00}
    cpu.prefixed_instructions[0x01] = Instruction{1, 8, "RLC C", RLC_0x01}
    cpu.prefixed_instructions[0x02] = Instruction{1, 8, "RLC D", RLC_0x02}
    cpu.prefixed_instructions[0x03] = Instruction{1, 8, "RLC E", RLC_0x03}
    cpu.prefixed_instructions[0x04] = Instruction{1, 8, "RLC H", RLC_0x04}
    cpu.prefixed_instructions[0x05] = Instruction{1, 8, "RLC L", RLC_0x05}
    cpu.prefixed_instructions[0x06] = Instruction{1, 16, "RLC (HL)", RLC_0x06}
    cpu.prefixed_instructions[0x07] = Instruction{1, 8, "RLC A", RLC_0x07}
    cpu.prefixed_instructions[0x08] = Instruction{1, 8, "RRC B", RRC_0x08}
    cpu.prefixed_instructions[0x09] = Instruction{1, 8, "RRC C", RRC_0x09}
    cpu.prefixed_instructions[0x0a] = Instruction{1, 8, "RRC D", RRC_0x0a}
    cpu.prefixed_instructions[0x0b] = Instruction{1, 8, "RRC E", RRC_0x0b}
    cpu.prefixed_instructions[0x0c] = Instruction{1, 8, "RRC H", RRC_0x0c}
    cpu.prefixed_instructions[0x0d] = Instruction{1, 8, "RRC L", RRC_0x0d}
    cpu.prefixed_instructions[0x0e] = Instruction{1, 16, "RRC (HL)", RRC_0x0e}
    cpu.prefixed_instructions[0x0f] = Instruction{1, 8, "RRC A", RRC_0x0f}
    cpu.prefixed_instructions[0x10] = Instruction{1, 8, "RL B", RL_0x10}
    cpu.prefixed_instructions[0x11] = Instruction{1, 8, "RL C", RL_0x11}
    cpu.prefixed_instructions[0x12] = Instruction{1, 8, "RL D", RL_0x12}
    cpu.prefixed_instructions[0x13] = Instruction{1, 8, "RL E", RL_0x13}
    cpu.prefixed_instructions[0x14] = Instruction{1, 8, "RL H", RL_0x14}
    cpu.prefixed_instructions[0x15] = Instruction{1, 8, "RL L", RL_0x15}
    cpu.prefixed_instructions[0x16] = Instruction{1, 16, "RL (HL)", RL_0x16}
    cpu.prefixed_instructions[0x17] = Instruction{1, 8, "RL A", RL_0x17}
    cpu.prefixed_instructions[0x18] = Instruction{1, 8, "RR B", RR_0x18}
    cpu.prefixed_instructions[0x19] = Instruction{1, 8, "RR C", RR_0x19}
    cpu.prefixed_instructions[0x1a] = Instruction{1, 8, "RR D", RR_0x1a}
    cpu.prefixed_instructions[0x1b] = Instruction{1, 8, "RR E", RR_0x1b}
    cpu.prefixed_instructions[0x1c] = Instruction{1, 8, "RR H", RR_0x1c}
    cpu.prefixed_instructions[0x1d] = Instruction{1, 8, "RR L", RR_0x1d}
    cpu.prefixed_instructions[0x1e] = Instruction{1, 16, "RR (HL)", RR_0x1e}
    cpu.prefixed_instructions[0x1f] = Instruction{1, 8, "RR A", RR_0x1f}
    cpu.prefixed_instructions[0x20] = Instruction{1, 8, "SLA B", SLA_0x20}
    cpu.prefixed_instructions[0x21] = Instruction{1, 8, "SLA C", SLA_0x21}
    cpu.prefixed_instructions[0x22] = Instruction{1, 8, "SLA D", SLA_0x22}
    cpu.prefixed_instructions[0x23] = Instruction{1, 8, "SLA E", SLA_0x23}
    cpu.prefixed_instructions[0x24] = Instruction{1, 8, "SLA H", SLA_0x24}
    cpu.prefixed_instructions[0x25] = Instruction{1, 8, "SLA L", SLA_0x25}
    cpu.prefixed_instructions[0x26] = Instruction{1, 16, "SLA (HL)", SLA_0x26}
    cpu.prefixed_instructions[0x27] = Instruction{1, 8, "SLA A", SLA_0x27}
    cpu.prefixed_instructions[0x28] = Instruction{1, 8, "SRA B", SRA_0x28}
    cpu.prefixed_instructions[0x29] = Instruction{1, 8, "SRA C", SRA_0x29}
    cpu.prefixed_instructions[0x2a] = Instruction{1, 8, "SRA D", SRA_0x2a}
    cpu.prefixed_instructions[0x2b] = Instruction{1, 8, "SRA E", SRA_0x2b}
    cpu.prefixed_instructions[0x2c] = Instruction{1, 8, "SRA H", SRA_0x2c}
    cpu.prefixed_instructions[0x2d] = Instruction{1, 8, "SRA L", SRA_0x2d}
    cpu.prefixed_instructions[0x2e] = Instruction{1, 16, "SRA (HL)", SRA_0x2e}
    cpu.prefixed_instructions[0x2f] = Instruction{1, 8, "SRA A", SRA_0x2f}
    cpu.prefixed_instructions[0x30] = Instruction{1, 8, "SWAP B", SWAP_0x30}
    cpu.prefixed_instructions[0x31] = Instruction{1, 8, "SWAP C", SWAP_0x31}
    cpu.prefixed_instructions[0x32] = Instruction{1, 8, "SWAP D", SWAP_0x32}
    cpu.prefixed_instructions[0x33] = Instruction{1, 8, "SWAP E", SWAP_0x33}
    cpu.prefixed_instructions[0x34] = Instruction{1, 8, "SWAP H", SWAP_0x34}
    cpu.prefixed_instructions[0x35] = Instruction{1, 8, "SWAP L", SWAP_0x35}
    cpu.prefixed_instructions[0x36] = Instruction{1, 16, "SWAP (HL)", SWAP_0x36}
    cpu.prefixed_instructions[0x37] = Instruction{1, 8, "SWAP A", SWAP_0x37}
    cpu.prefixed_instructions[0x38] = Instruction{1, 8, "SRL B", SRL_0x38}
    cpu.prefixed_instructions[0x39] = Instruction{1, 8, "SRL C", SRL_0x39}
    cpu.prefixed_instructions[0x3a] = Instruction{1, 8, "SRL D", SRL_0x3a}
    cpu.prefixed_instructions[0x3b] = Instruction{1, 8, "SRL E", SRL_0x3b}
    cpu.prefixed_instructions[0x3c] = Instruction{1, 8, "SRL H", SRL_0x3c}
    cpu.prefixed_instructions[0x3d] = Instruction{1, 8, "SRL L", SRL_0x3d}
    cpu.prefixed_instructions[0x3e] = Instruction{1, 16, "SRL (HL)", SRL_0x3e}
    cpu.prefixed_instructions[0x3f] = Instruction{1, 8, "SRL A", SRL_0x3f}
    cpu.prefixed_instructions[0x40] = Instruction{1, 8, "BIT 0 B", BIT_0x40}
    cpu.prefixed_instructions[0x41] = Instruction{1, 8, "BIT 0 C", BIT_0x41}
    cpu.prefixed_instructions[0x42] = Instruction{1, 8, "BIT 0 D", BIT_0x42}
    cpu.prefixed_instructions[0x43] = Instruction{1, 8, "BIT 0 E", BIT_0x43}
    cpu.prefixed_instructions[0x44] = Instruction{1, 8, "BIT 0 H", BIT_0x44}
    cpu.prefixed_instructions[0x45] = Instruction{1, 8, "BIT 0 L", BIT_0x45}
    cpu.prefixed_instructions[0x46] = Instruction{1, 16, "BIT 0 (HL)", BIT_0x46}
    cpu.prefixed_instructions[0x47] = Instruction{1, 8, "BIT 0 A", BIT_0x47}
    cpu.prefixed_instructions[0x48] = Instruction{1, 8, "BIT 1 B", BIT_0x48}
    cpu.prefixed_instructions[0x49] = Instruction{1, 8, "BIT 1 C", BIT_0x49}
    cpu.prefixed_instructions[0x4a] = Instruction{1, 8, "BIT 1 D", BIT_0x4a}
    cpu.prefixed_instructions[0x4b] = Instruction{1, 8, "BIT 1 E", BIT_0x4b}
    cpu.prefixed_instructions[0x4c] = Instruction{1, 8, "BIT 1 H", BIT_0x4c}
    cpu.prefixed_instructions[0x4d] = Instruction{1, 8, "BIT 1 L", BIT_0x4d}
    cpu.prefixed_instructions[0x4e] = Instruction{1, 16, "BIT 1 (HL)", BIT_0x4e}
    cpu.prefixed_instructions[0x4f] = Instruction{1, 8, "BIT 1 A", BIT_0x4f}
    cpu.prefixed_instructions[0x50] = Instruction{1, 8, "BIT 2 B", BIT_0x50}
    cpu.prefixed_instructions[0x51] = Instruction{1, 8, "BIT 2 C", BIT_0x51}
    cpu.prefixed_instructions[0x52] = Instruction{1, 8, "BIT 2 D", BIT_0x52}
    cpu.prefixed_instructions[0x53] = Instruction{1, 8, "BIT 2 E", BIT_0x53}
    cpu.prefixed_instructions[0x54] = Instruction{1, 8, "BIT 2 H", BIT_0x54}
    cpu.prefixed_instructions[0x55] = Instruction{1, 8, "BIT 2 L", BIT_0x55}
    cpu.prefixed_instructions[0x56] = Instruction{1, 16, "BIT 2 (HL)", BIT_0x56}
    cpu.prefixed_instructions[0x57] = Instruction{1, 8, "BIT 2 A", BIT_0x57}
    cpu.prefixed_instructions[0x58] = Instruction{1, 8, "BIT 3 B", BIT_0x58}
    cpu.prefixed_instructions[0x59] = Instruction{1, 8, "BIT 3 C", BIT_0x59}
    cpu.prefixed_instructions[0x5a] = Instruction{1, 8, "BIT 3 D", BIT_0x5a}
    cpu.prefixed_instructions[0x5b] = Instruction{1, 8, "BIT 3 E", BIT_0x5b}
    cpu.prefixed_instructions[0x5c] = Instruction{1, 8, "BIT 3 H", BIT_0x5c}
    cpu.prefixed_instructions[0x5d] = Instruction{1, 8, "BIT 3 L", BIT_0x5d}
    cpu.prefixed_instructions[0x5e] = Instruction{1, 16, "BIT 3 (HL)", BIT_0x5e}
    cpu.prefixed_instructions[0x5f] = Instruction{1, 8, "BIT 3 A", BIT_0x5f}
    cpu.prefixed_instructions[0x60] = Instruction{1, 8, "BIT 4 B", BIT_0x60}
    cpu.prefixed_instructions[0x61] = Instruction{1, 8, "BIT 4 C", BIT_0x61}
    cpu.prefixed_instructions[0x62] = Instruction{1, 8, "BIT 4 D", BIT_0x62}
    cpu.prefixed_instructions[0x63] = Instruction{1, 8, "BIT 4 E", BIT_0x63}
    cpu.prefixed_instructions[0x64] = Instruction{1, 8, "BIT 4 H", BIT_0x64}
    cpu.prefixed_instructions[0x65] = Instruction{1, 8, "BIT 4 L", BIT_0x65}
    cpu.prefixed_instructions[0x66] = Instruction{1, 16, "BIT 4 (HL)", BIT_0x66}
    cpu.prefixed_instructions[0x67] = Instruction{1, 8, "BIT 4 A", BIT_0x67}
    cpu.prefixed_instructions[0x68] = Instruction{1, 8, "BIT 5 B", BIT_0x68}
    cpu.prefixed_instructions[0x69] = Instruction{1, 8, "BIT 5 C", BIT_0x69}
    cpu.prefixed_instructions[0x6a] = Instruction{1, 8, "BIT 5 D", BIT_0x6a}
    cpu.prefixed_instructions[0x6b] = Instruction{1, 8, "BIT 5 E", BIT_0x6b}
    cpu.prefixed_instructions[0x6c] = Instruction{1, 8, "BIT 5 H", BIT_0x6c}
    cpu.prefixed_instructions[0x6d] = Instruction{1, 8, "BIT 5 L", BIT_0x6d}
    cpu.prefixed_instructions[0x6e] = Instruction{1, 16, "BIT 5 (HL)", BIT_0x6e}
    cpu.prefixed_instructions[0x6f] = Instruction{1, 8, "BIT 5 A", BIT_0x6f}
    cpu.prefixed_instructions[0x70] = Instruction{1, 8, "BIT 6 B", BIT_0x70}
    cpu.prefixed_instructions[0x71] = Instruction{1, 8, "BIT 6 C", BIT_0x71}
    cpu.prefixed_instructions[0x72] = Instruction{1, 8, "BIT 6 D", BIT_0x72}
    cpu.prefixed_instructions[0x73] = Instruction{1, 8, "BIT 6 E", BIT_0x73}
    cpu.prefixed_instructions[0x74] = Instruction{1, 8, "BIT 6 H", BIT_0x74}
    cpu.prefixed_instructions[0x75] = Instruction{1, 8, "BIT 6 L", BIT_0x75}
    cpu.prefixed_instructions[0x76] = Instruction{1, 16, "BIT 6 (HL)", BIT_0x76}
    cpu.prefixed_instructions[0x77] = Instruction{1, 8, "BIT 6 A", BIT_0x77}
    cpu.prefixed_instructions[0x78] = Instruction{1, 8, "BIT 7 B", BIT_0x78}
    cpu.prefixed_instructions[0x79] = Instruction{1, 8, "BIT 7 C", BIT_0x79}
    cpu.prefixed_instructions[0x7a] = Instruction{1, 8, "BIT 7 D", BIT_0x7a}
    cpu.prefixed_instructions[0x7b] = Instruction{1, 8, "BIT 7 E", BIT_0x7b}
    cpu.prefixed_instructions[0x7c] = Instruction{1, 8, "BIT 7 H", BIT_0x7c}
    cpu.prefixed_instructions[0x7d] = Instruction{1, 8, "BIT 7 L", BIT_0x7d}
    cpu.prefixed_instructions[0x7e] = Instruction{1, 16, "BIT 7 (HL)", BIT_0x7e}
    cpu.prefixed_instructions[0x7f] = Instruction{1, 8, "BIT 7 A", BIT_0x7f}
    cpu.prefixed_instructions[0x80] = Instruction{1, 8, "RES 0 B", RES_0x80}
    cpu.prefixed_instructions[0x81] = Instruction{1, 8, "RES 0 C", RES_0x81}
    cpu.prefixed_instructions[0x82] = Instruction{1, 8, "RES 0 D", RES_0x82}
    cpu.prefixed_instructions[0x83] = Instruction{1, 8, "RES 0 E", RES_0x83}
    cpu.prefixed_instructions[0x84] = Instruction{1, 8, "RES 0 H", RES_0x84}
    cpu.prefixed_instructions[0x85] = Instruction{1, 8, "RES 0 L", RES_0x85}
    cpu.prefixed_instructions[0x86] = Instruction{1, 16, "RES 0 (HL)", RES_0x86}
    cpu.prefixed_instructions[0x87] = Instruction{1, 8, "RES 0 A", RES_0x87}
    cpu.prefixed_instructions[0x88] = Instruction{1, 8, "RES 1 B", RES_0x88}
    cpu.prefixed_instructions[0x89] = Instruction{1, 8, "RES 1 C", RES_0x89}
    cpu.prefixed_instructions[0x8a] = Instruction{1, 8, "RES 1 D", RES_0x8a}
    cpu.prefixed_instructions[0x8b] = Instruction{1, 8, "RES 1 E", RES_0x8b}
    cpu.prefixed_instructions[0x8c] = Instruction{1, 8, "RES 1 H", RES_0x8c}
    cpu.prefixed_instructions[0x8d] = Instruction{1, 8, "RES 1 L", RES_0x8d}
    cpu.prefixed_instructions[0x8e] = Instruction{1, 16, "RES 1 (HL)", RES_0x8e}
    cpu.prefixed_instructions[0x8f] = Instruction{1, 8, "RES 1 A", RES_0x8f}
    cpu.prefixed_instructions[0x90] = Instruction{1, 8, "RES 2 B", RES_0x90}
    cpu.prefixed_instructions[0x91] = Instruction{1, 8, "RES 2 C", RES_0x91}
    cpu.prefixed_instructions[0x92] = Instruction{1, 8, "RES 2 D", RES_0x92}
    cpu.prefixed_instructions[0x93] = Instruction{1, 8, "RES 2 E", RES_0x93}
    cpu.prefixed_instructions[0x94] = Instruction{1, 8, "RES 2 H", RES_0x94}
    cpu.prefixed_instructions[0x95] = Instruction{1, 8, "RES 2 L", RES_0x95}
    cpu.prefixed_instructions[0x96] = Instruction{1, 16, "RES 2 (HL)", RES_0x96}
    cpu.prefixed_instructions[0x97] = Instruction{1, 8, "RES 2 A", RES_0x97}
    cpu.prefixed_instructions[0x98] = Instruction{1, 8, "RES 3 B", RES_0x98}
    cpu.prefixed_instructions[0x99] = Instruction{1, 8, "RES 3 C", RES_0x99}
    cpu.prefixed_instructions[0x9a] = Instruction{1, 8, "RES 3 D", RES_0x9a}
    cpu.prefixed_instructions[0x9b] = Instruction{1, 8, "RES 3 E", RES_0x9b}
    cpu.prefixed_instructions[0x9c] = Instruction{1, 8, "RES 3 H", RES_0x9c}
    cpu.prefixed_instructions[0x9d] = Instruction{1, 8, "RES 3 L", RES_0x9d}
    cpu.prefixed_instructions[0x9e] = Instruction{1, 16, "RES 3 (HL)", RES_0x9e}
    cpu.prefixed_instructions[0x9f] = Instruction{1, 8, "RES 3 A", RES_0x9f}
    cpu.prefixed_instructions[0xa0] = Instruction{1, 8, "RES 4 B", RES_0xa0}
    cpu.prefixed_instructions[0xa1] = Instruction{1, 8, "RES 4 C", RES_0xa1}
    cpu.prefixed_instructions[0xa2] = Instruction{1, 8, "RES 4 D", RES_0xa2}
    cpu.prefixed_instructions[0xa3] = Instruction{1, 8, "RES 4 E", RES_0xa3}
    cpu.prefixed_instructions[0xa4] = Instruction{1, 8, "RES 4 H", RES_0xa4}
    cpu.prefixed_instructions[0xa5] = Instruction{1, 8, "RES 4 L", RES_0xa5}
    cpu.prefixed_instructions[0xa6] = Instruction{1, 16, "RES 4 (HL)", RES_0xa6}
    cpu.prefixed_instructions[0xa7] = Instruction{1, 8, "RES 4 A", RES_0xa7}
    cpu.prefixed_instructions[0xa8] = Instruction{1, 8, "RES 5 B", RES_0xa8}
    cpu.prefixed_instructions[0xa9] = Instruction{1, 8, "RES 5 C", RES_0xa9}
    cpu.prefixed_instructions[0xaa] = Instruction{1, 8, "RES 5 D", RES_0xaa}
    cpu.prefixed_instructions[0xab] = Instruction{1, 8, "RES 5 E", RES_0xab}
    cpu.prefixed_instructions[0xac] = Instruction{1, 8, "RES 5 H", RES_0xac}
    cpu.prefixed_instructions[0xad] = Instruction{1, 8, "RES 5 L", RES_0xad}
    cpu.prefixed_instructions[0xae] = Instruction{1, 16, "RES 5 (HL)", RES_0xae}
    cpu.prefixed_instructions[0xaf] = Instruction{1, 8, "RES 5 A", RES_0xaf}
    cpu.prefixed_instructions[0xb0] = Instruction{1, 8, "RES 6 B", RES_0xb0}
    cpu.prefixed_instructions[0xb1] = Instruction{1, 8, "RES 6 C", RES_0xb1}
    cpu.prefixed_instructions[0xb2] = Instruction{1, 8, "RES 6 D", RES_0xb2}
    cpu.prefixed_instructions[0xb3] = Instruction{1, 8, "RES 6 E", RES_0xb3}
    cpu.prefixed_instructions[0xb4] = Instruction{1, 8, "RES 6 H", RES_0xb4}
    cpu.prefixed_instructions[0xb5] = Instruction{1, 8, "RES 6 L", RES_0xb5}
    cpu.prefixed_instructions[0xb6] = Instruction{1, 16, "RES 6 (HL)", RES_0xb6}
    cpu.prefixed_instructions[0xb7] = Instruction{1, 8, "RES 6 A", RES_0xb7}
    cpu.prefixed_instructions[0xb8] = Instruction{1, 8, "RES 7 B", RES_0xb8}
    cpu.prefixed_instructions[0xb9] = Instruction{1, 8, "RES 7 C", RES_0xb9}
    cpu.prefixed_instructions[0xba] = Instruction{1, 8, "RES 7 D", RES_0xba}
    cpu.prefixed_instructions[0xbb] = Instruction{1, 8, "RES 7 E", RES_0xbb}
    cpu.prefixed_instructions[0xbc] = Instruction{1, 8, "RES 7 H", RES_0xbc}
    cpu.prefixed_instructions[0xbd] = Instruction{1, 8, "RES 7 L", RES_0xbd}
    cpu.prefixed_instructions[0xbe] = Instruction{1, 16, "RES 7 (HL)", RES_0xbe}
    cpu.prefixed_instructions[0xbf] = Instruction{1, 8, "RES 7 A", RES_0xbf}
    cpu.prefixed_instructions[0xc0] = Instruction{1, 8, "SET 0 B", SET_0xc0}
    cpu.prefixed_instructions[0xc1] = Instruction{1, 8, "SET 0 C", SET_0xc1}
    cpu.prefixed_instructions[0xc2] = Instruction{1, 8, "SET 0 D", SET_0xc2}
    cpu.prefixed_instructions[0xc3] = Instruction{1, 8, "SET 0 E", SET_0xc3}
    cpu.prefixed_instructions[0xc4] = Instruction{1, 8, "SET 0 H", SET_0xc4}
    cpu.prefixed_instructions[0xc5] = Instruction{1, 8, "SET 0 L", SET_0xc5}
    cpu.prefixed_instructions[0xc6] = Instruction{1, 16, "SET 0 (HL)", SET_0xc6}
    cpu.prefixed_instructions[0xc7] = Instruction{1, 8, "SET 0 A", SET_0xc7}
    cpu.prefixed_instructions[0xc8] = Instruction{1, 8, "SET 1 B", SET_0xc8}
    cpu.prefixed_instructions[0xc9] = Instruction{1, 8, "SET 1 C", SET_0xc9}
    cpu.prefixed_instructions[0xca] = Instruction{1, 8, "SET 1 D", SET_0xca}
    cpu.prefixed_instructions[0xcb] = Instruction{1, 8, "SET 1 E", SET_0xcb}
    cpu.prefixed_instructions[0xcc] = Instruction{1, 8, "SET 1 H", SET_0xcc}
    cpu.prefixed_instructions[0xcd] = Instruction{1, 8, "SET 1 L", SET_0xcd}
    cpu.prefixed_instructions[0xce] = Instruction{1, 16, "SET 1 (HL)", SET_0xce}
    cpu.prefixed_instructions[0xcf] = Instruction{1, 8, "SET 1 A", SET_0xcf}
    cpu.prefixed_instructions[0xd0] = Instruction{1, 8, "SET 2 B", SET_0xd0}
    cpu.prefixed_instructions[0xd1] = Instruction{1, 8, "SET 2 C", SET_0xd1}
    cpu.prefixed_instructions[0xd2] = Instruction{1, 8, "SET 2 D", SET_0xd2}
    cpu.prefixed_instructions[0xd3] = Instruction{1, 8, "SET 2 E", SET_0xd3}
    cpu.prefixed_instructions[0xd4] = Instruction{1, 8, "SET 2 H", SET_0xd4}
    cpu.prefixed_instructions[0xd5] = Instruction{1, 8, "SET 2 L", SET_0xd5}
    cpu.prefixed_instructions[0xd6] = Instruction{1, 16, "SET 2 (HL)", SET_0xd6}
    cpu.prefixed_instructions[0xd7] = Instruction{1, 8, "SET 2 A", SET_0xd7}
    cpu.prefixed_instructions[0xd8] = Instruction{1, 8, "SET 3 B", SET_0xd8}
    cpu.prefixed_instructions[0xd9] = Instruction{1, 8, "SET 3 C", SET_0xd9}
    cpu.prefixed_instructions[0xda] = Instruction{1, 8, "SET 3 D", SET_0xda}
    cpu.prefixed_instructions[0xdb] = Instruction{1, 8, "SET 3 E", SET_0xdb}
    cpu.prefixed_instructions[0xdc] = Instruction{1, 8, "SET 3 H", SET_0xdc}
    cpu.prefixed_instructions[0xdd] = Instruction{1, 8, "SET 3 L", SET_0xdd}
    cpu.prefixed_instructions[0xde] = Instruction{1, 16, "SET 3 (HL)", SET_0xde}
    cpu.prefixed_instructions[0xdf] = Instruction{1, 8, "SET 3 A", SET_0xdf}
    cpu.prefixed_instructions[0xe0] = Instruction{1, 8, "SET 4 B", SET_0xe0}
    cpu.prefixed_instructions[0xe1] = Instruction{1, 8, "SET 4 C", SET_0xe1}
    cpu.prefixed_instructions[0xe2] = Instruction{1, 8, "SET 4 D", SET_0xe2}
    cpu.prefixed_instructions[0xe3] = Instruction{1, 8, "SET 4 E", SET_0xe3}
    cpu.prefixed_instructions[0xe4] = Instruction{1, 8, "SET 4 H", SET_0xe4}
    cpu.prefixed_instructions[0xe5] = Instruction{1, 8, "SET 4 L", SET_0xe5}
    cpu.prefixed_instructions[0xe6] = Instruction{1, 16, "SET 4 (HL)", SET_0xe6}
    cpu.prefixed_instructions[0xe7] = Instruction{1, 8, "SET 4 A", SET_0xe7}
    cpu.prefixed_instructions[0xe8] = Instruction{1, 8, "SET 5 B", SET_0xe8}
    cpu.prefixed_instructions[0xe9] = Instruction{1, 8, "SET 5 C", SET_0xe9}
    cpu.prefixed_instructions[0xea] = Instruction{1, 8, "SET 5 D", SET_0xea}
    cpu.prefixed_instructions[0xeb] = Instruction{1, 8, "SET 5 E", SET_0xeb}
    cpu.prefixed_instructions[0xec] = Instruction{1, 8, "SET 5 H", SET_0xec}
    cpu.prefixed_instructions[0xed] = Instruction{1, 8, "SET 5 L", SET_0xed}
    cpu.prefixed_instructions[0xee] = Instruction{1, 16, "SET 5 (HL)", SET_0xee}
    cpu.prefixed_instructions[0xef] = Instruction{1, 8, "SET 5 A", SET_0xef}
    cpu.prefixed_instructions[0xf0] = Instruction{1, 8, "SET 6 B", SET_0xf0}
    cpu.prefixed_instructions[0xf1] = Instruction{1, 8, "SET 6 C", SET_0xf1}
    cpu.prefixed_instructions[0xf2] = Instruction{1, 8, "SET 6 D", SET_0xf2}
    cpu.prefixed_instructions[0xf3] = Instruction{1, 8, "SET 6 E", SET_0xf3}
    cpu.prefixed_instructions[0xf4] = Instruction{1, 8, "SET 6 H", SET_0xf4}
    cpu.prefixed_instructions[0xf5] = Instruction{1, 8, "SET 6 L", SET_0xf5}
    cpu.prefixed_instructions[0xf6] = Instruction{1, 16, "SET 6 (HL)", SET_0xf6}
    cpu.prefixed_instructions[0xf7] = Instruction{1, 8, "SET 6 A", SET_0xf7}
    cpu.prefixed_instructions[0xf8] = Instruction{1, 8, "SET 7 B", SET_0xf8}
    cpu.prefixed_instructions[0xf9] = Instruction{1, 8, "SET 7 C", SET_0xf9}
    cpu.prefixed_instructions[0xfa] = Instruction{1, 8, "SET 7 D", SET_0xfa}
    cpu.prefixed_instructions[0xfb] = Instruction{1, 8, "SET 7 E", SET_0xfb}
    cpu.prefixed_instructions[0xfc] = Instruction{1, 8, "SET 7 H", SET_0xfc}
    cpu.prefixed_instructions[0xfd] = Instruction{1, 8, "SET 7 L", SET_0xfd}
    cpu.prefixed_instructions[0xfe] = Instruction{1, 16, "SET 7 (HL)", SET_0xfe}
    cpu.prefixed_instructions[0xff] = Instruction{1, 8, "SET 7 A", SET_0xff}
}
