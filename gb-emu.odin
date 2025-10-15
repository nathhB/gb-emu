package gb_emu

import "core:log"
import "core:os"
import rl "vendor:raylib"

main :: proc() {
	context.logger = log.create_console_logger()
	gb := GB{}

	gb_init(&gb)

	// test failed = 0xc1b9

	// cpu_add_breakpoint(&gb.cpu, 0xc1b9, proc(cpu: ^CPU, mem: ^GB_Memory) {
	// 	print_cpu(cpu)
	// })

	// cpu_add_breakpoint(&gb.cpu, 0xc325, proc(cpu: ^CPU, mem: ^GB_Memory) {
	// 	print_cpu(cpu)
	// 	log.debugf("Show stack: %x %x", mem.data[cpu.sp], mem.data[cpu.sp + 1])
	// })

	rom_path := "ROMS/zelda.gb"

	err := gb_load_rom(&gb, rom_path)

	// dump_memory(&gb.cpu, &gb.mem, 0x4000, 0x7FFF)

	if err != GB_Error.None {
		log.errorf("Failed to load boot ROM: %w", err)

		os.exit(1)
	}

	rl.SetTargetFPS(60)
	rl.SetConfigFlags({rl.ConfigFlag.WINDOW_HIGHDPI, rl.ConfigFlag.WINDOW_RESIZABLE})
	rl.InitWindow(640, 480, "GB Emulator")

	gb_run(&gb)
}
