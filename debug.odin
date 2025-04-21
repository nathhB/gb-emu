package gb_emu

import "core:log"
import "core:fmt"
import "core:strings"

print_vram :: proc(vram: []u8) {
    for i := 0; i < 384; i += 1 {
        tile_addr := 0x8000 + (i * 16)

        log.debugf("Tile %d: ");

        for j := 0; j < 16; j += 1 {
            log.debugf("%x", vram[tile_addr + j])
        }
    }
}

print_cpu :: proc(cpu: ^CPU) {
    z := int(read_flag(cpu, Flags.Z))
    h := int(read_flag(cpu, Flags.H))
    n := int(read_flag(cpu, Flags.N))
    c := int(read_flag(cpu, Flags.C))
    ime := int(cpu.ime)

    log.debugf("AF: %x, BC: %x, DE: %x, HL: %x | Z: %d, H: %d, N: %d, C: %d | IME: %d", cpu.af, cpu.bc, cpu.de, cpu.hl, z, h, n, c, ime)
}

dump_vram :: proc(cpu: ^CPU, mem: ^GB_Memory) {
    dump_memory(cpu, mem, 0x8000, 0x9FFF)
}

dump_memory :: proc(cpu: ^CPU, mem: ^GB_Memory, start_addr: u16, end_addr: u16) {
    addr := start_addr
    sb := strings.Builder{}

    strings.builder_init(&sb)
    defer strings.builder_destroy(&sb)

    for addr < end_addr {
        strings.write_string(&sb, fmt.aprintf("0x%x\t", addr))

        for i: u16 = 0; i < 16; i += 1 {
            byte := mem_read(mem, addr + i)

            strings.write_string(&sb, fmt.aprintf("%2x ", byte))
        }

        strings.write_string(&sb, fmt.aprintln(" "))
        addr += 16
    }

    mem_str := strings.to_string(sb)

    fmt.println(mem_str)
}
