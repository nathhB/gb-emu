package gb_emu

import "core:time"
import "core:log"
import "core:os"
import "core:mem"
import "vendor:raylib"

DotNs: i64 = 238 // 1 dot = 2^22 Hz, 1 dot = 1 T-Cycle, 1 M-Cycle = 4 T-Cycle
ScanelineDots :: 456
FrameScanlines :: 154
FrameDrawScanlines :: 144
FrameDots: i64 = ScanelineDots * FrameScanlines
FrameNs: i64 = FrameDots * DotNs
BootRomPath :: "ROMS/dmg_boot.gb"

GB :: struct {
    mem: [0xFFFF+1]u8,
    cpu: CPU,
    ppu: PPU,
    booted: bool
}

GB_Error :: enum u32 {
    None,
    FailedToReadROM,
    ROMTooBig
}

ROM_Header :: struct {
    rom_size: int,
    rom_type: u8,
    ram_size: int
}

GB_HardRegister :: enum u16 {
    LCDC = 0xFF40,
    STAT = 0xFF41,
    SCY = 0xFF42,
    SCX = 0xFF43,
    LY = 0xFF44,
    LYC = 0xFF45,
}

gb_init :: proc(gb: ^GB) {
    cpu_init(&gb.cpu)
    ppu_init(&gb.ppu, gb.mem[:]) 
}

gb_run :: proc(gb: ^GB) {
    for !raylib.WindowShouldClose() {
        if !gb.booted && gb.mem[0xFF50] != 0 {
            // TODO: unload boot rom?
            gb.booted = true
            log.debug("BOOTED: %x", gb.cpu.sp)
        }

        do_frame(gb)

        raylib.BeginDrawing()
        raylib.ClearBackground(raylib.LIGHTGRAY)
        raylib.EndDrawing()
    }
}

gb_load_boot_rom :: proc(gb: ^GB) -> GB_Error {
    data, success := os.read_entire_file(BootRomPath)

    if !success {
        return GB_Error.FailedToReadROM
    }

    if (len(data) > len(gb.mem)) {
        return GB_Error.ROMTooBig
    }

    copy(gb.mem[:], data)

    return GB_Error.None
}

gb_load_rom :: proc(gb: ^GB, path: string) -> GB_Error {
    data, success := os.read_entire_file(path)

    if !success {
        return GB_Error.FailedToReadROM
    }

    if (len(data) > len(gb.mem)) {
        return GB_Error.ROMTooBig
    }

    header := read_rom_header(data)

    if header.rom_type == 0 {
        copy(gb.mem[:], data)
    } else {
        panic("unsupported ROM type")
    }

    log.infof("Successfully loaded ROM %s (size: %d)", path, len(data))
    print_rom_info(header)

    return GB_Error.None
}

do_frame :: proc(gb: ^GB) {
    for t: i64 = 0; t < FrameDots; t += 1 {
        cpu_tick(&gb.cpu, gb.mem[:])
        ppu_tick(&gb.ppu, gb.mem[:], t)
    }
}

read_rom_header :: proc(rom: []u8) -> ROM_Header {
    rom_size := int(32 * (1 << rom[0x148])) * 1024
    rom_type := rom[0x147]
    ram_size_type := rom[0x149]
    ram_size := 0

    if ram_size_type == 0 {
        ram_size = 0
    } else if ram_size == 1 {
        ram_size = 2 * 1024
    } else if ram_size == 2 {
        ram_size = 8 * 1024
    } else if ram_size == 3 {
        ram_size = 32 * 1024
    }

    return (ROM_Header){rom_size, rom_type, ram_size}
}

print_rom_info :: proc(header: ROM_Header) {
    log.infof("ROM Type: %d", header.rom_type)
    log.infof("ROM Size (Kb): %d", header.rom_size)
    log.infof("RAM Size (Kb): %d", header.ram_size)
}
