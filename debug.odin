package gb_emu

import "core:fmt"
import "core:log"
import "core:strings"
import rl "vendor:raylib"

Tile_Count :: 384
Tiles_Per_Row :: 16

print_vram :: proc(vram: []u8) {
	for i := 0; i < 384; i += 1 {
		tile_addr := 0x8000 + (i * 16)

		log.debugf("Tile %d: ")

		for j := 0; j < 16; j += 1 {
			log.debugf("%x", vram[tile_addr + j])
		}
	}
}

print_cpu :: proc(cpu: ^CPU) {
	z := int(read_flag(cpu, Flags.Z))
	h := int(read_flag(cpu, Flags.H))
	n := int(read_flag(cpu, Flags.N))
	c := int(read_flag(cpu, Flags.C))
	ime := int(cpu.ime)

	log.debugf(
		"AF: %4x, BC: %4x, DE: %4x, HL: %4x, SP: %4x, PC: %4x | Z: %d, H: %d, N: %d, C: %d | IME: %d",
		cpu.af,
		cpu.bc,
		cpu.de,
		cpu.hl,
		cpu.sp,
		cpu.pc,
		z,
		h,
		n,
		c,
		ime,
	)
}

dump_vram :: proc(cpu: ^CPU, mem: ^GB_Memory) {
	dump_memory(cpu, mem, 0x8000, 0x9FFF)
}

dump_memory :: proc(cpu: ^CPU, mem: ^GB_Memory, start_addr: u16, end_addr: u16) {
	addr := start_addr
	sb := strings.Builder{}

	strings.builder_init(&sb)
	defer strings.builder_destroy(&sb)

	for addr < end_addr {
		strings.write_string(&sb, fmt.aprintf("0x%x\t", addr))

		for i: u16 = 0; i < 16; i += 1 {
			byte := mem_read(mem, addr + i)

			strings.write_string(&sb, fmt.aprintf("%2x ", byte))
		}

		strings.write_string(&sb, fmt.aprintln(" "))
		addr += 16
	}

	mem_str := strings.to_string(sb)

	fmt.println(mem_str)
}

draw_debugger_info :: proc(gb: ^GB) {
	// draw info about the CPU state

	flags_str := get_flags_str(&gb.cpu)
	defer delete(flags_str)
	ime_str := gb.cpu.ime ? "Enabled" : "Disabled"

	cpu_str := rl.TextFormat(
		"AF: %4x, BC: %4x, DE: %4x, HL: %4x, SP: %4x, PC: %4x | %s | IME: %s",
		gb.cpu.af,
		gb.cpu.bc,
		gb.cpu.de,
		gb.cpu.hl,
		gb.cpu.sp,
		gb.cpu.pc,
		flags_str,
		ime_str,
	)

	rl.DrawText(cpu_str, 5, rl.GetScreenHeight() - 80, 16, rl.RED)

	// draw info about timers

	div := mem_read(&gb.mem, u16(GB_HardRegister.DIV))
	timers_str := rl.TextFormat("DIV: %2x", div)

	rl.DrawText(timers_str, 5, rl.GetScreenHeight() - 50, 16, rl.RED)

	// draw info about the current breakpoint

	if gb.cpu.breakpoint > 0 {
		bp_str := rl.TextFormat("Breakpoint: %4x", gb.cpu.breakpoint)

		rl.DrawText(bp_str, 5, rl.GetScreenHeight() - 20, 16, rl.RED)
	} else {
		rl.DrawText("Breakpoint: None", 5, rl.GetScreenHeight() - 20, 16, rl.DARKGRAY)
	}
}

process_debug_inputs :: proc(gb: ^GB, debug_ctx: ^GB_Debug_Context) {
	if rl.IsKeyDown(rl.KeyboardKey.ENTER) {
		gb.speed = 8
	} else {
		gb.speed = 1
	}

	if rl.IsKeyPressed(rl.KeyboardKey.D) {
		dump_memory(&gb.cpu, &gb.mem, 0xFE00, 0xFE9F)
	}

	if rl.IsKeyPressed(rl.KeyboardKey.T) {
		debug_ctx.show_tileset_texture = !debug_ctx.show_tileset_texture

		if debug_ctx.show_tileset_texture {
			update_tileset_texture(&gb.ppu, &gb.mem, debug_ctx.tileset_texture.texture)
		}
	}
}

process_debugger_inputs :: proc(cpu: ^CPU) {
	if rl.IsKeyPressed(rl.KeyboardKey.S) {
		cpu_step(cpu)
	} else if rl.IsKeyPressed(rl.KeyboardKey.C) {
		cpu_continue(cpu)
	}
}

get_flags_str :: proc(cpu: ^CPU) -> string {
	sb := strings.Builder{}

	strings.builder_init(&sb)
	defer strings.builder_destroy(&sb)

	for flag in Flags {
		if read_flag(cpu, flag) {
			strings.write_string(&sb, flag_to_string(flag))
		} else {
			strings.write_string(&sb, "-")
		}
	}

	return strings.clone(strings.to_string(sb))
}

flag_to_string :: proc(flag: Flags) -> string {
	switch flag {
	case Flags.Z:
		return "Z"
	case Flags.N:
		return "N"
	case Flags.H:
		return "H"
	case Flags.C:
		return "C"
	}

	unreachable()
}

update_tileset_texture :: proc(
	ppu: ^PPU,
	mem: ^GB_Memory,
	texture: rl.Texture2D,
	addr: u16 = 0x8000,
) {
	img_data := make([]rl.Color, texture.width * texture.height)
	defer delete(img_data)

	for i := 0; i < Tile_Count; i += 1 {
		x := (i % Tiles_Per_Row) * 8
		y := (i / Tiles_Per_Row) * 8
		tile_addr := addr + u16(i * 16)

		for tile_x := 0; tile_x < 8; tile_x += 1 {
			for tile_y := 0; tile_y < 8; tile_y += 1 {
				pixel_color_id := get_pixel_color(ppu, mem, tile_addr, tile_x, tile_y)
				pixel_color := get_color_from_palette(ppu, mem, BGP_Addr, pixel_color_id)
				img_x := x + tile_x
				img_y := y + tile_y

				img_data[img_y * int(texture.width) + img_x] = pixel_color
			}
		}
	}

	rl.UpdateTexture(texture, raw_data(img_data[:]))
}

create_debug_tileset_texture :: proc() -> rl.RenderTexture2D {
	width :: Tiles_Per_Row * 8
	height :: (Tile_Count / Tiles_Per_Row) * 8
	texture := rl.LoadRenderTexture(width, height)
	texture.texture.format = rl.PixelFormat.UNCOMPRESSED_R8G8B8A8

	return texture
}
