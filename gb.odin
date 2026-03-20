package gb_emu

import "core:flags"
import "core:log"
import "core:mem"
import "core:os"
import rl "vendor:raylib"

FrameDots: u64 = ScanelineDots * FrameScanlines
ScreenWidth :: 160
ScreenHeight :: 144

GB :: struct {
	rom_header:    ROM_Header,
	mem:           GB_Memory,
	cpu:           CPU,
	ppu:           PPU,
	apu:           APU,
	booting:       bool,
	booted:        bool,
	div_acc:       u64,
	timer_acc:     u64,
	timer_enabled: bool,
	timer_dots:    u64,
	speed:         int,
	inputs:        Input_State,
	color:         bool,
}

Input_State :: struct {
	left:   bool,
	right:  bool,
	up:     bool,
	down:   bool,
	a:      bool,
	b:      bool,
	start:  bool,
	select: bool,
}

GB_Debug_Context :: struct {
	tileset_texture:      rl.RenderTexture2D,
	tilemap_texture:      rl.RenderTexture2D,
	show_tileset_texture: bool,
	show_tilemap_texture: bool,
}

GB_Error :: enum u32 {
	None,
	ROM_FileError,
	ROM_TooBig,
	ROM_InvalidSize,
	ROM_Unsupported,
	ROM_FailedToLoadBoot,
	MBC_Unsupported,
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
	VBK    = 0xFF4F,
	WBK    = 0xFF70,
	DMA    = 0xFF46,
	HDMA1  = 0xFF51,
	HDMA2  = 0xFF52,
	HDMA3  = 0xFF53,
	HDMA4  = 0xFF54,
	HDMA5  = 0xFF55,
	BCPS   = 0xFF68,
	BCPD   = 0xFF69,
	OCPS   = 0xFF6A,
	OCPD   = 0xFF6B,
}

GB_Audio_Registers :: enum u16 {
	NR10 = 0xFF10, // Channel 1 sweep
	NR11 = 0xFF11, // Channel 1 length timer & duty cycle
	NR12 = 0xFF12, // Channel 1 volume & enveloppe
	NR13 = 0xFF13, // Channel 1 period low
	NR14 = 0xFF14, // Channel 1 period high & control
	NR21 = 0xFF16, // Channel 2 length timer & duty cycle
	NR22 = 0xFF17, // Channel 2 volume & enveloppe
	NR23 = 0xFF18, // Channel 2 period low
	NR24 = 0xFF19, // Channel 2 period high & control
	NR30 = 0xFF1A, // Channel 3 DAC enable
	NR31 = 0xFF1B, // Channel 3 length timer [write-only]
	NR32 = 0xFF1C, // Channel 3 output level
	NR33 = 0xFF1D, // Channel 3 period low [write-only]
	NR34 = 0xFF1E, // Channel 3 period high & control
	NR41 = 0xFF20, // Channel 4 length timer [write-only]
	NR42 = 0xFF21, // Channel 4 output level
	NR43 = 0xFF22, // Channel 4 period low [write-only]
	NR44 = 0xFF23, // Channel 4 period high & control
	NR50 = 0xFF24, // Master volume & VIN panning
	NR51 = 0xFF25, // Sound panning
	NR52 = 0xFF26, // Audio master control
}

gb_init :: proc(gb: ^GB, color: bool) {
	log.debugf("CGB mode: %w", color)

	gb.color = color

	mem_init(&gb.mem, color)
	cpu_init(&gb.cpu)
	ppu_init(&gb.ppu, &gb.mem, color)
	apu_init(&gb.apu)
}

gb_deinit :: proc(gb: ^GB) {
	apu_deinit(&gb.apu)
}

gb_run :: proc(gb: ^GB) {
	screen_texture := rl.LoadRenderTexture(ScreenWidth, ScreenHeight)
	defer rl.UnloadRenderTexture(screen_texture)
	screen_rect := rl.Rectangle{0, 0, ScreenWidth, ScreenHeight}
	debug_ctx := GB_Debug_Context{}
	debug_ctx.tileset_texture = create_debug_tileset_texture()
	defer rl.UnloadRenderTexture(debug_ctx.tileset_texture)
	debug_ctx.tilemap_texture = create_debug_tilemap_texture()
	defer rl.UnloadRenderTexture(debug_ctx.tilemap_texture)

	screen_texture.texture.format = rl.PixelFormat.UNCOMPRESSED_R8G8B8A8

	for !rl.WindowShouldClose() {
		if gb.booting && !gb.booted && mem_read(&gb.mem, 0xFF50) != 0 {
			log.infof("Boot process completed")

			gb.booted = true
			gb.booting = false
			gb_map_rom(gb)
		}

		process_debug_inputs(gb, &debug_ctx)

		if gb.cpu.breakpoint == 0 {
			read_inputs(gb)
			do_frame(gb)
		} else {
			process_debugger_inputs(&gb.cpu)
		}

		apu_update_audio_stream(&gb.apu)

		rl.UpdateTexture(screen_texture.texture, raw_data(gb.ppu.framebuffer[:]))

		rl.BeginDrawing()
		{
			rl.ClearBackground(rl.LIGHTGRAY)

			if debug_ctx.show_tileset_texture {
				draw_tileset_texture(&debug_ctx)
			} else if debug_ctx.show_tilemap_texture {
				draw_tilemap_texture(&debug_ctx)
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
			draw_debugger_info(gb)
		}
		rl.EndDrawing()
	}
}

gb_load_rom :: proc(gb: ^GB, rom_path: string) -> GB_Error {
	rom, success := os.read_entire_file(rom_path)

	if !success {
		return .ROM_FileError
	}

	gb.rom_header = read_rom_header(rom)

	if (len(rom) != gb.rom_header.rom_size) {
		return .ROM_InvalidSize
	}

	gb.mem.rom = rom

	print_rom_info(gb.rom_header)

	return .None
}

gb_init_mbc :: proc(gb: ^GB) -> GB_Error {
	switch gb.rom_header.rom_type {
	case 0:
		mbc0_init(&gb.mem)
		log.info("Memory controller: none")
	case 0x1:
		mbc1_init(&gb.mem, false)
		log.info("Memory controller: MBC1")
	case 0x2:
		mbc1_init(&gb.mem, true)
		log.info("Memory controller: MBC1+RAM")
	case 0x3:
		mbc1_init(&gb.mem, true)
		log.info("Memory controller: MBC1+RAM+BATTERY")
	case:
		return .MBC_Unsupported
	}

	return .None
}

gb_boot :: proc(gb: ^GB, boot_rom_path: string) -> GB_Error {
	gb.booting = true

	err := gb.color ? load_cgb_boot_rom(gb, boot_rom_path) : load_dmg_boot_rom(gb, boot_rom_path)

	if err != .None {
		log.errorf("Failed to load boot ROM: %w", err)
		return .ROM_FailedToLoadBoot
	}

	gb_init_mbc(gb) or_return

	return .None
}

gb_map_rom :: proc(gb: ^GB) {
	rom := gb.mem.rom

	switch gb.rom_header.rom_type {
	case 0:
		copy(gb.mem.data[:], rom[:0x8000]) // map the full rom to 0 - 0x7FFF
	case 0x1:
		copy(gb.mem.data[:], rom[:0x4000]) // load the first 16kb bank
	case 0x2:
		copy(gb.mem.data[:], rom[:0x4000]) // load the first 16kb bank
	case 0x3:
		copy(gb.mem.data[:], rom[:0x4000]) // load the first 16kb bank
	case:
		unreachable()
	}
}

gb_init_dmg_registers :: proc(gb: ^GB) {
	gb.cpu.pc = 0x100
	gb.cpu.sp = 0xFFFE
	gb.cpu.af = 0x01B0
	gb.cpu.bc = 0x0013
	gb.cpu.de = 0x00D8
	gb.cpu.hl = 0x014D

	mem_write(gb, u16(GB_HardRegister.LCDC), 0x91)
	mem_write(gb, u16(GB_HardRegister.IF), 0xE1)
	mem_write(gb, u16(GB_HardRegister.IE), 0x0)
	mem_write(gb, u16(GB_HardRegister.JOYPAD), 0xCF)
}

gb_init_cgb_registers :: proc(gb: ^GB) {
	gb_init_dmg_registers(gb)

	gb.cpu.af = 0x1180
	gb.cpu.bc = 0x0000
	gb.cpu.de = 0xFF56
	gb.cpu.hl = 0x000D

	mem_write(gb, u16(GB_HardRegister.VBK), 0)
	mem_write(gb, u16(GB_HardRegister.WBK), 1)
	mem_write(gb, u16(GB_HardRegister.HDMA5), 0xFF)
	mem_write(gb, u16(GB_HardRegister.BCPS), 0)
	mem_write(gb, u16(GB_HardRegister.OCPS), 0)
}

load_dmg_boot_rom :: proc(gb: ^GB, rom_path: string) -> GB_Error {
	data, success := os.read_entire_file(rom_path)

	if !success {
		return .ROM_FileError
	}

	if (len(data) > 0x100) {
		return .ROM_TooBig
	}

	copy(gb.mem.data[:], data)
	copy(gb.mem.data[0x100:], gb.mem.rom[0x100:0x8000])

	return .None
}

load_cgb_boot_rom :: proc(gb: ^GB, rom_path: string) -> GB_Error {
	// https://gbdev.io/pandocs/Power_Up_Sequence.html#size
	boot_rom, success := os.read_entire_file(rom_path)

	if !success {
		return .ROM_FileError
	}

	copy(gb.mem.data[:0x100], boot_rom[:0x100]) // Boot ROM first part
	copy(gb.mem.data[0x100:0x200], gb.mem.rom[0x100:0x200]) // ROM header
	copy(gb.mem.data[0x200:], boot_rom[0x200:]) // Boot ROM second part

	return .None
}

do_frame :: proc(gb: ^GB) {
	for f := 0; f < gb.speed; f += 1 {
		for t: u64 = 0; t < FrameDots; t += 1 {
			cpu_tick(gb)

			if gb.cpu.breakpoint > 0 {
				return
			}

			mem_tick(gb)
			ppu_tick(gb, t)
			apu_tick(gb)
			timer_tick(gb)
		}
	}
}

read_inputs :: proc(gb: ^GB) {
	gb.inputs.right = rl.IsKeyDown(rl.KeyboardKey.RIGHT)
	gb.inputs.left = rl.IsKeyDown(rl.KeyboardKey.LEFT)
	gb.inputs.up = rl.IsKeyDown(rl.KeyboardKey.UP)
	gb.inputs.down = rl.IsKeyDown(rl.KeyboardKey.DOWN)
	gb.inputs.a = rl.IsKeyDown(rl.KeyboardKey.Q)
	gb.inputs.b = rl.IsKeyDown(rl.KeyboardKey.B)
	gb.inputs.select = rl.IsKeyDown(rl.KeyboardKey.S)
	gb.inputs.start = rl.IsKeyDown(rl.KeyboardKey.SPACE)
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
	log.infof("ROM Size (Kb): %d", header.rom_size / 1024)
	log.infof("RAM Size (Kb): %d", header.ram_size)
}
