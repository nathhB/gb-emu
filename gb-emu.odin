package gb_emu

import "core:os"
import "core:log"
import rl "vendor:raylib"

main :: proc() {
    context.logger = log.create_console_logger()
    gb := GB{}

    gb_init(&gb)
    // cpu_add_breakpoint(&gb.cpu, 0x02c4, dump_vram)
    // cpu_add_breakpoint(&gb.cpu, 0x02cd, dump_vram)
    // cpu_add_breakpoint(&gb.cpu, 0x40, dump_vram)
    // cpu_add_breakpoint(&gb.cpu, 0x17e, dump_vram) // VBlank handler
    // cpu_add_breakpoint(&gb.cpu, 0x02f8, dump_vram)
    // cpu_add_breakpoint(&gb.cpu, 0x393, dump_vram)

    rom_path := "ROMS/tetris.gb"

    err := gb_load_rom(&gb, rom_path)

    if err != GB_Error.None {
        log.error("Failed to load boot ROM")

        os.exit(1)
    }

    rl.SetTargetFPS(60)
    rl.SetConfigFlags({rl.ConfigFlag.WINDOW_HIGHDPI, rl.ConfigFlag.WINDOW_RESIZABLE})
    rl.InitWindow(640, 480, "GB Emulator")

    gb_run(&gb)
}
