package gb_emu

import "core:os"
import "core:log"
import rl "vendor:raylib"

main :: proc() {
    /*fd, err := os.open("log", os.O_RDWR | os.O_CREATE)

    if err != os.General_Error.None {
        os.exit(1)
    }

    context.logger = log.create_file_logger(fd)
    */
    context.logger = log.create_console_logger()
    gb := GB{}

    gb_init(&gb)

    rom_path := "ROMS/cpu_instrs.gb"

    if gb_load_rom(&gb, rom_path) != GB_Error.None {
        log.errorf("Failed to load ROM: %s", rom_path)

        os.exit(1)
    }

    if gb_load_boot_rom(&gb) != GB_Error.None {
        log.error("Failed to load boot ROM")

        os.exit(1)
    }

    rl.SetTargetFPS(60)
    rl.SetConfigFlags({rl.ConfigFlag.WINDOW_HIGHDPI, rl.ConfigFlag.WINDOW_RESIZABLE})
    rl.InitWindow(640, 480, "GB Emulator")

    gb_run(&gb)
}
