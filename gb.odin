package gb_emu

import "core:log"
import "core:mem"
import "core:os"
import "core:time"
import rl "vendor:raylib"

DotNs: u64 = 238 // 1 dot = 2^22 Hz, 1 dot = 1 T-Cycle, 1 M-Cycle = 4 T-Cycle
FrameDots: u64 = ScanelineDots * FrameScanlines
FrameNs: u64 = FrameDots * DotNs
ScreenWidth :: 160
ScreenHeight :: 144
BootRomPath :: "ROMS/dmg_boot.gb"

GB :: struct {
	mem:           GB_Memory,
	cpu:           CPU,
	ppu:           PPU,
	booted:        bool,
	div_acc:       u64,
	timer_acc:     u64,
	timer_enabled: bool,
	timer_dots:    u64,
	speed:         int,
}

GB_Debug_Context :: struct {
	tileset_texture:      rl.RenderTexture2D,
	show_tileset_texture: bool,
}

GB_Error :: enum u32 {
	None,
	ROM_FileError,
	ROM_TooBig,
	ROM_InvalidSize,
	ROM_Unsupported,
	ROM_FailedToLoadBoot,
}

ROM_Header :: struct {
	rom_type: u8,
	rom_size: int,
	ram_size: int,
}

GB_HardRegister :: enum u16 {
	LCDC   = 0xFF40,
	STAT   = 0xFF41,
	SCY    = 0xFF42,
	SCX    = 0xFF43,
	LY     = 0xFF44,
	LYC    = 0xFF45,
	IF     = 0xFF0F,
	IE     = 0xFFFF,
	JOYPAD = 0xFF00,
	DIV    = 0xFF04,
	TIMA   = 0xFF05,
	TMA    = 0xFF06,
	TAC    = 0xFF07,
	WY     = 0xFF4A,
	WX     = 0xFF4B,
	DMA    = 0xFF46,

	// Audio registers
	NR10   = 0xFF10, // Channel 1 sweep
	NR11   = 0xFF11, // Channel 1 length timer & duty cycle
	NR12   = 0xFF12, // Channel 1 volume & enveloppe
	NR13   = 0xFF13, // Channel 1 period low
	NR14   = 0xFF14, // Channel 1 period high & control
	NR21   = 0xFF16, // Channel 2 length timer & duty cycle
	NR22   = 0xFF17, // Channel 2 volume & enveloppe
	NR23   = 0xFF18, // Channel 2 period low
	NR24   = 0xFF19, // Channel 2 period high & control
	NR30   = 0xFF1A, // Channel 3 DAC enable
	NR31   = 0xFF1B, // Channel 3 length timer [write-only]
	NR32   = 0xFF1C, // Channel 3 output level
	NR33   = 0xFF1D, // Channel 3 period low [write-only]
	NR34   = 0xFF1E, // Channel 3 period high & control
	NR50   = 0xFF24, // Master volume & VIN panning
	NR51   = 0xFF25, // Sound panning
	NR52   = 0xFF26, // Audio master control
}

gb_init :: proc(gb: ^GB) {
	cpu_init(&gb.cpu)
	ppu_init(&gb.ppu, &gb.mem)

	gb.mem.data[GB_HardRegister.JOYPAD] = 0xFF
	gb.speed = 4 // start at x4 to go through the booting phase quicker
}

gb_run :: proc(gb: ^GB) {
	screen_texture := rl.LoadRenderTexture(ScreenWidth, ScreenHeight)
	defer rl.UnloadRenderTexture(screen_texture)
	screen_rect := rl.Rectangle{0, 0, ScreenWidth, ScreenHeight}
	debug_ctx := GB_Debug_Context{}
	debug_ctx.tileset_texture = create_debug_tileset_texture()
	defer rl.UnloadRenderTexture(debug_ctx.tileset_texture)

	screen_texture.texture.format = rl.PixelFormat.UNCOMPRESSED_R8G8B8A8

	for !rl.WindowShouldClose() {
		if !gb.booted && mem_read(&gb.mem, 0xFF50) != 0 {
			unload_boot_rom(gb)
			gb.booted = true
			gb.speed = 1
		}

		if gb.cpu.breakpoint == 0 {
			do_frame(gb)
			process_debug_inputs(&gb.cpu, &gb.mem, &gb.ppu, &debug_ctx)
		} else {
			process_debugger_inputs(&gb.cpu)
		}

		rl.UpdateTexture(screen_texture.texture, raw_data(gb.ppu.framebuffer[:]))

		rl.BeginDrawing()
		{
			rl.ClearBackground(rl.LIGHTGRAY)

			if debug_ctx.show_tileset_texture {
				tileset_texture := debug_ctx.tileset_texture.texture
				aspect_ratio := f32(tileset_texture.width) / f32(tileset_texture.height)
				target_rect := rl.Rectangle {
					0,
					0,
					f32(rl.GetScreenHeight()) * aspect_ratio,
					f32(rl.GetScreenHeight()),
				}
				src_rect := rl.Rectangle {
					0,
					0,
					f32(tileset_texture.width),
					f32(tileset_texture.height),
				}

				rl.DrawTexturePro(
					tileset_texture,
					src_rect,
					target_rect,
					rl.Vector2{0, 0},
					0,
					rl.WHITE,
				)
			} else {
				target_rect := rl.Rectangle {
					0,
					0,
					f32(rl.GetScreenWidth()),
					f32(rl.GetScreenHeight()),
				}

				rl.DrawTexturePro(
					screen_texture.texture,
					screen_rect,
					target_rect,
					rl.Vector2{0, 0},
					0,
					rl.WHITE,
				)
			}

			rl.DrawFPS(0, 0)
			// draw_debugger_info(gb);
		}
		rl.EndDrawing()
	}
}

gb_load_rom :: proc(gb: ^GB, path: string) -> GB_Error {
	rom, success := os.read_entire_file(path)

	if !success {
		return GB_Error.ROM_FileError
	}

	header := read_rom_header(rom)

	if (len(rom) != header.rom_size) {
		return GB_Error.ROM_InvalidSize
	}

	if load_boot_rom(gb) != GB_Error.None {
		return GB_Error.ROM_FailedToLoadBoot
	}

	print_rom_info(header)

	switch header.rom_type {
	case 0:
		mbc0_init(&gb.mem, rom)
		copy(gb.mem.data[0x100:], rom[0x100:0x8000]) // map the full rom to 0 - 0x7FFF
		log.info("No memory controller")
	case 0x1:
		mbc1_init(&gb.mem, rom, header.ram_size)
		copy(gb.mem.data[0x100:], rom[0x100:0x4000]) // load the first 16kb bank 
		log.info("Using MBC1 memory controller")
	case 0x3:
		mbc1_init(&gb.mem, rom, header.ram_size)
		copy(gb.mem.data[0x100:], rom[0x100:0x4000]) // load the first 16kb bank 
		log.info("Using MBC1 memory controller")
	case:
		return GB_Error.ROM_Unsupported
	}

	log.infof("Successfully loaded ROM %s (size: %d)", path, len(rom))

	return GB_Error.None
}

load_boot_rom :: proc(gb: ^GB) -> GB_Error {
	data, success := os.read_entire_file(BootRomPath)

	if !success {
		return GB_Error.ROM_FileError
	}

	if (len(data) > 0x100) {
		return GB_Error.ROM_TooBig
	}

	copy(gb.mem.data[:], data)

	return GB_Error.None
}

unload_boot_rom :: proc(gb: ^GB) {
	copy(gb.mem.data[0:0x100], gb.mem.rom[0:0x100])
}

do_frame :: proc(gb: ^GB) {
	for f := 0; f < gb.speed; f += 1 {
		for t: u64 = 0; t < FrameDots; t += 1 {
			cpu_tick(&gb.cpu, &gb.mem)

			if gb.cpu.breakpoint > 0 {
				return
			}

			mem_tick(&gb.mem)
			ppu_tick(gb, t)
			timer_tick(gb)
		}
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

	return (ROM_Header){rom_type, rom_size, ram_size}
}

print_rom_info :: proc(header: ROM_Header) {
	log.infof("ROM Type: 0x%x", header.rom_type)
	log.infof("ROM Size (Kb): %d", header.rom_size)
	log.infof("RAM Size (Kb): %d", header.ram_size)
}
