package gb_emu

import "core:log"
import "core:os"
import "vendor:raylib"

main :: proc() {
    context.logger = log.create_console_logger()
    gb := GB{}

    gb_init(&gb)

    rom_path := "ROMS/tetris.gb"

    if gb_load_rom(&gb, rom_path) != GB_Error.None {
        log.errorf("Failed to load ROM: %s", rom_path)

        os.exit(1)
    }

    if gb_load_boot_rom(&gb) != GB_Error.None {
        log.error("Failed to load boot ROM")

        os.exit(1)
    }

    raylib.SetTargetFPS(60)
    raylib.InitWindow(640, 480, "GB Emulator")

    gb_run(&gb)
}
