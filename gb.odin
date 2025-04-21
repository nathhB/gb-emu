package gb_emu

import "core:time"
import "core:log"
import "core:os"
import "core:mem"
import rl "vendor:raylib"

DotNs: i64 = 238 // 1 dot = 2^22 Hz, 1 dot = 1 T-Cycle, 1 M-Cycle = 4 T-Cycle
ScanelineDots :: 456
FrameScanlines :: 154
FrameDrawScanlines :: 144
FrameDots: i64 = ScanelineDots * FrameScanlines
FrameNs: i64 = FrameDots * DotNs
ScreenWidth :: 160
ScreenHeight :: 144
BootRomPath :: "ROMS/dmg_boot.gb"

GB :: struct {
    mem: GB_Memory,
    cpu: CPU,
    ppu: PPU,
    booted: bool
}

GB_Error :: enum u32 {
    None,
    ROM_FileError,
    ROM_TooBig,
    ROM_Unsupported,
    ROM_FailedToLoadBoot
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
    screen_texture := rl.LoadRenderTexture(ScreenWidth, ScreenHeight)
    screen_rect := rl.Rectangle{0, 0, ScreenWidth, ScreenHeight}
    target_rect := rl.Rectangle{0, 0, f32(rl.GetScreenWidth()), f32(rl.GetScreenHeight())}

    for !rl.WindowShouldClose() {
        if !gb.booted && mem_read(&gb.mem, 0xFF50) != 0 {
            unload_boot_rom(gb)
            gb.booted = true
            log.debug("BOOTED")
        }

        do_frame(gb)

        rl.UpdateTexture(screen_texture.texture, raw_data(gb.ppu.framebuffer[:]))

        rl.BeginDrawing()
        {
            rl.ClearBackground(rl.LIGHTGRAY)

            rl.DrawTexturePro(screen_texture.texture, screen_rect, target_rect, rl.Vector2{0, 0}, 0, rl.WHITE)
            rl.DrawFPS(0, 0) 
        }
        rl.EndDrawing()
    }

    rl.UnloadTexture(screen_texture.texture)
}

gb_load_rom :: proc(gb: ^GB, path: string) -> GB_Error {
    data, success := os.read_entire_file(path)

    if !success {
        return GB_Error.ROM_FileError
    } 

    header := read_rom_header(data)
    rom := make([]u8, header.rom_size)

    switch header.rom_type {
    case 0:
        mbc0_init(&gb.mem, rom)
    case 1:
        mbc1_init(&gb.mem, rom)
    case:
        return GB_Error.ROM_Unsupported
    }

    if load_boot_rom(gb) != GB_Error.None {
        return GB_Error.ROM_FailedToLoadBoot
    }

    copy(gb.mem.data[0x100:], data[0x100:0x4000]) // always load the first 16kb bank 

    if (len(data) > header.rom_size) {
        return GB_Error.ROM_TooBig
    }

    copy(gb.mem.rom, data)

    log.infof("Successfully loaded ROM %s (size: %d)", path, len(data))
    print_rom_info(header)

    return GB_Error.None
}

load_boot_rom :: proc(gb: ^GB) -> GB_Error {
    data, success := os.read_entire_file(BootRomPath)

    if !success {
        return GB_Error.FailedToReadROM
    }

    if (len(data) > 0x100) {
        return GB_Error.ROMTooBig
    }

    copy(gb.mem.data[:], data)

    return GB_Error.None
}

unload_boot_rom :: proc(gb: ^GB) {
    copy(gb.mem.data[0:0x100], gb.mem.rom[0:0x100])
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
