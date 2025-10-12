package gb_emu

import sa "core:container/small_array"
import "core:fmt"
import "core:log"
import "core:time"
import rl "vendor:raylib"

// https://gbdev.io/pandocs/Rendering.html

ScanelineDots :: 456
FrameScanlines :: 154
VBlank_ScanelineCount :: 10
DrawFrameScanlines :: FrameScanlines - VBlank_ScanelineCount
MaxObjectsPerScanline :: 10
OAM_EntryCount :: 40
OAM_Scan_Dots :: 80

PPU :: struct {
	scanline:         u8,
	mode:             PPU_Mode,
	colors:           [4]rl.Color,
	framebuffer:      [ScreenWidth * ScreenHeight]rl.Color,
	scanline_objects: sa.Small_Array(MaxObjectsPerScanline, PPU_Object),
	states:           [4]PPU_State,
}

PPU_Mode :: enum {
	HBlank,
	VBlank,
	OAM_Scan,
	DrawPixels,
}

PPU_State :: struct {
	on_enter:       proc(gb: ^GB),
	on_update:      proc(gb: ^GB, tick: u64),
	current_tick:   int,
	duration_ticks: int,
	next_mode:      PPU_Mode,
}

// https://gbdev.io/pandocs/LCDC.html
PPU_Control :: enum {
	BG_WindowEnablePrio,
	OBJ_Enable,
	OBJ_Size,
	BG_TileMap,
	BG_WindowTiles,
	Window_Enable,
	Window_TileMap,
	PPU_Enable,
}

PPU_Object :: struct {
	x:       u8,
	y:       u8,
	tile_id: u8,
	attrs:   u8,
}

ppu_init :: proc(ppu: ^PPU, mem: ^GB_Memory) {
	ppu.scanline = 0
	ppu.mode = PPU_Mode.OAM_Scan

	vblank_duration := VBlank_ScanelineCount * ScanelineDots

	ppu.states[PPU_Mode.HBlank] = create_state(nil, nil, 204, PPU_Mode.OAM_Scan)
	ppu.states[PPU_Mode.VBlank] = create_state(
		vlank_on_enter,
		nil,
		vblank_duration,
		PPU_Mode.OAM_Scan,
	)
	ppu.states[PPU_Mode.OAM_Scan] = create_state(
		oam_scan_on_enter,
		nil,
		OAM_Scan_Dots,
		PPU_Mode.DrawPixels,
	)
	ppu.states[PPU_Mode.DrawPixels] = create_state(
		nil,
		draw_pixels_on_update,
		172,
		PPU_Mode.HBlank,
	)

	// TODO: add palette support
	color0 := rl.WHITE
	color1 := rl.ORANGE
	color2 := rl.RED
	color3 := rl.BLACK
	ppu.colors = [4]rl.Color{color0, color1, color2, color3}
}

ppu_tick :: proc(gb: ^GB, tick: u64) {
	ppu := &gb.ppu

	ppu.scanline = u8(tick / ScanelineDots)
	mem_write(&gb.mem, u16(GB_HardRegister.LY), u8(ppu.scanline))

	current_state := &ppu.states[ppu.mode]

	if current_state.current_tick >= current_state.duration_ticks {
		current_state.current_tick = 0

		change_mode(ppu)

		current_state = &ppu.states[ppu.mode]

		if current_state.on_enter != nil {
			current_state.on_enter(gb)
		}
	}

	if current_state.on_update != nil {
		current_state.on_update(gb, tick)
	}

	current_state.current_tick += 1
}

change_mode :: proc(ppu: ^PPU) {
	current_state := &ppu.states[ppu.mode]

	// transition to VBlank is handled differently because it
	// only happens after the first 144 scanlines have completed

	if ppu.scanline >= DrawFrameScanlines {
		ppu.mode = PPU_Mode.VBlank
	} else {
		ppu.mode = current_state.next_mode
	}
}

oam_scan_on_enter :: proc(gb: ^GB) {
	assert(gb.ppu.mode == PPU_Mode.OAM_Scan)
	scan_oam(&gb.ppu, &gb.mem)
}

vlank_on_enter :: proc(gb: ^GB) {
	assert(gb.ppu.mode == PPU_Mode.VBlank)
	cpu_request_interrupt(&gb.cpu, &gb.mem, Interrupt.VBlank)
}

draw_pixels_on_update :: proc(gb: ^GB, tick: u64) {
	ppu := &gb.ppu

	assert(ppu.mode == PPU_Mode.DrawPixels)

	x := ppu.states[PPU_Mode.DrawPixels].current_tick // output one pixel per dot

	if x > 159 {
		return
	}

	y := int(gb.ppu.scanline)
	pixel_color := get_pixel_color_at(ppu, &gb.mem, x, y)
	framebuffer_offset := y * ScreenWidth + x

	ppu.framebuffer[framebuffer_offset] = pixel_color
}

get_pixel_color_at :: proc(ppu: ^PPU, mem: ^GB_Memory, x: int, y: int) -> rl.Color {
	scx := int(mem.read(mem, u16(GB_HardRegister.SCX)))
	scy := int(mem.read(mem, u16(GB_HardRegister.SCY)))
	tilemap_x := (scx + x) % 256
	tilemap_y := (scy + y) % 256
	bg_tile_id := get_bg_tile_id(mem, tilemap_x, tilemap_y)
	pixel_color: rl.Color = rl.WHITE

	// if LCDC.0 bit is 0, the background / window pixel color is white
	if get_control_flag(mem, PPU_Control.BG_WindowEnablePrio) {
		bg_tile_addr := get_tile_addr(mem, bg_tile_id, is_object = false)
		pixel_color = get_pixel_color(ppu, mem, bg_tile_addr, tilemap_x, tilemap_y)
	}

	if get_control_flag(mem, PPU_Control.OBJ_Enable) {
		obj_tile_id, draw_obj := get_object_tile_id(ppu, x)

		// TODO: background pixel may take priority over the object pixel
		if draw_obj {
			fmt.ensuref(
				get_control_flag(mem, PPU_Control.OBJ_Size) == false,
				"8x16 objects are not supported",
			)
			obj_tile_addr := get_tile_addr(mem, obj_tile_id, is_object = true)
			pixel_color = get_pixel_color(ppu, mem, obj_tile_addr, x, y)
		}
	}

	return pixel_color
}

get_bg_tile_id :: proc(mem: ^GB_Memory, x: int, y: int) -> u8 {
	bg_tilemap_addr: u16 = 0
	in_window := false

	if get_control_flag(mem, PPU_Control.Window_Enable) {
		// https://gbdev.io/pandocs/Scrolling.html#ff4aff4b--wy-wx-window-y-position-x-position-plus-7
		wx := mem_read(mem, u16(GB_HardRegister.WX))
		wy := mem_read(mem, u16(GB_HardRegister.WY))

		wx -= 7

		in_window = x >= int(wx) && y >= int(wy)
	}

	if in_window {
		win_tilemap := get_control_flag(mem, PPU_Control.Window_TileMap)

		bg_tilemap_addr = win_tilemap ? 0x9C00 : 0x9800
	} else {
		bg_tilemap := get_control_flag(mem, PPU_Control.BG_TileMap)

		bg_tilemap_addr = bg_tilemap ? 0x9C00 : 0x9800
	}

	tile_x := x / 8
	tile_y := y / 8
	tilemap_offset := (tile_y * 32) + tile_x

	return mem.read(mem, bg_tilemap_addr + u16(tilemap_offset))
}

get_object_tile_id :: proc(ppu: ^PPU, x: int) -> (u8, bool) {
	objects := sa.slice(&ppu.scanline_objects)

	for obj in objects {
		obj_x := int(obj.x)

		if x >= obj_x && x <= obj_x + 7 {
			return obj.tile_id, true
		}
	}

	return 0, false
}

// https://gbdev.io/pandocs/Tile_Data.html
get_pixel_color :: proc(ppu: ^PPU, mem: ^GB_Memory, tile_addr: u16, x: int, y: int) -> rl.Color {
	// a tile is 16 bytes
	// each 2 bytes represent 1 line of the tile

	x_in_tile := x % 8
	y_in_tile := y % 8
	x_mask := u8(1 << (7 - u8(x_in_tile)))
	y_offset := u16(y_in_tile * 2)
	tile_data_low := mem.read(mem, tile_addr + y_offset)
	tile_data_high := mem.read(mem, tile_addr + y_offset + 1)
	color_low := (tile_data_low & x_mask) > 0 ? 1 : 0
	color_high := (tile_data_high & x_mask) > 0 ? 1 : 0
	color_id := u8(color_high << 1) | u8(color_low)

	return ppu.colors[color_id]
}

get_tile_addr :: proc(mem: ^GB_Memory, tile_id: u8, is_object: bool) -> u16 {
	// https://gbdev.io/pandocs/Tile_Data.html

	window_bg_data := get_control_flag(mem, PPU_Control.BG_WindowTiles)
	tile_offset := u16(tile_id) * 16

	// objects always use 8000 addressing
	if window_bg_data || is_object {
		// 8000 addressing
		return 0x8000 + u16(tile_offset)
	}

	// 8800 addressing

	if tile_id < 128 {
		// use block 2 for 0 - 127 ids
		return 0x9000 + u16(tile_offset)
	} else {
		// use block 1 for 128 - 255 ids
		return 0x8800 + u16(tile_offset)
	}
}

// TODO: add support for 8x16 objects
scan_oam :: proc(ppu: ^PPU, mem: ^GB_Memory) {
	oam_addr: u16 = 0xFE00

	sa.clear(&ppu.scanline_objects)

	for i: u16 = 0;
	    i < OAM_EntryCount && sa.len(ppu.scanline_objects) < MaxObjectsPerScanline;
	    i += 1 {
		obj_addr := oam_addr + (i * 4)
		y_pos := mem_read(mem, obj_addr) - 16

		if ppu.scanline < y_pos || ppu.scanline > y_pos + 7 {
			continue
		}

		x_pos := mem_read(mem, obj_addr + 1) - 8
		tile_id := mem_read(mem, obj_addr + 2)
		attrs := mem_read(mem, obj_addr + 3)

		sa.append(&ppu.scanline_objects, PPU_Object{x_pos, y_pos, tile_id, attrs})
	}
}

get_control_flag :: proc(mem: ^GB_Memory, flag: PPU_Control) -> bool {
	mask := u8(1 << u8(flag))

	return (mem.read(mem, u16(GB_HardRegister.LCDC)) & mask) > 0
}

create_state :: proc(
	on_enter: proc(gb: ^GB),
	on_update: proc(gb: ^GB, tick: u64),
	duration_ticks: int,
	next_mode: PPU_Mode,
) -> PPU_State {
	return PPU_State{on_enter, on_update, 0, duration_ticks, next_mode}
}
