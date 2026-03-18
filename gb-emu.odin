package gb_emu

import "core:flags"
import "core:log"
import "core:os"
import rl "vendor:raylib"

Command_Line_Options :: struct {
	rom:   string,
	boot:  string,
	color: bool,
}

main :: proc() {
	context.logger = log.create_console_logger()

	opts := Command_Line_Options{}
	opts_err := flags.parse(&opts, os.args[1:])

	if opts_err != nil {
		log.debugf("Invalid command line arguments: %s", opts_err.(flags.Parse_Error).message)
		return
	}

	gb := GB{}

	gb_init(&gb, opts.color)
	defer gb_deinit(&gb)

	err: GB_Error

	err = gb_load_rom(&gb, opts.rom)

	if err != .None {
		log.errorf("Failed to load ROM %w: %w", opts.rom, err)

		os.exit(1)
	}

	if opts.boot != "" {
		err = gb_boot(&gb, opts.boot)
	} else {
		err = gb_init_mbc(&gb)

		if err != .None {
			log.errorf("Failed to run ROM: %w", err)

			os.exit(1)
		}

		gb_map_rom(&gb)

		if gb.color {
			gb_init_cgb_registers(&gb)
		} else {
			gb_init_dmg_registers(&gb)
		}
	}

	if err != .None {
		log.errorf("Failed to run boot ROM: %w", err)

		os.exit(1)
	}

	rl.SetTargetFPS(60)
	rl.SetConfigFlags({rl.ConfigFlag.WINDOW_HIGHDPI, rl.ConfigFlag.WINDOW_RESIZABLE})
	rl.InitWindow(640, 480, "GB Emulator")

	bp := proc(gb: ^GB) {

	}
	cpu_add_breakpoint(&gb.cpu, 0x455f, bp)

	gb_run(&gb)
}
