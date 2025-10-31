package gb_emu

import "core:log"
import "core:os"
import rl "vendor:raylib"

main :: proc() {
	context.logger = log.create_console_logger()
	gb := GB{}

	gb_init(&gb)
	defer gb_deinit(&gb)

	rom_path := "ROMS/tetris.gb"

	err := gb_load_rom(&gb, rom_path)

	if err != GB_Error.None {
		log.errorf("Failed to load boot ROM: %w", err)

		os.exit(1)
	}

	rl.SetTargetFPS(60)
	rl.SetConfigFlags({rl.ConfigFlag.WINDOW_HIGHDPI, rl.ConfigFlag.WINDOW_RESIZABLE})
	rl.InitWindow(640, 480, "GB Emulator")

	gb_run(&gb)
}
