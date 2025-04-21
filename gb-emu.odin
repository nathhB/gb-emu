package gb_emu

import "core:os"
import "core:log"
import rl "vendor:raylib"

main :: proc() {
    context.logger = log.create_console_logger()
    gb := GB{}

    gb_init(&gb)

    rom_path := "ROMS/tetris.gb" 

    rom, err := gb_load_rom(&gb, rom_path)

    if err != GB_Error.None {
        log.error("Failed to load boot ROM")

        os.exit(1)
    }

    rl.SetTargetFPS(60)
    rl.SetConfigFlags({rl.ConfigFlag.WINDOW_HIGHDPI, rl.ConfigFlag.WINDOW_RESIZABLE})
    rl.InitWindow(640, 480, "GB Emulator")

    gb_run(&gb)
}
