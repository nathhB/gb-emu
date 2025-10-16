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
BGP_Addr :: 0xFF47
OBP0_Addr :: 0xFF48
OBP1_Addr :: 0xFF49

PPU :: struct {
	scanline:         u8,
	mode:             PPU_Mode,
	framebuffer:      [ScreenWidth * ScreenHeight]rl.Color,
	scanline_objects: sa.Small_Array(MaxObjectsPerScanline, PPU_Object),
	states:           [4]PPU_State,
	interrupt_line:   bool,
	bg_palette:       PPU_Palette,
	obj0_palette:     PPU_Palette,
	obj1_palette:     PPU_Palette,
}

PPU_Mode :: enum {
	HBlank     = 0,
	VBlank     = 1,
	OAM_Scan   = 2,
	DrawPixels = 3,
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
	BG_WindowEnablePrio = 0,
	OBJ_Enable          = 1,
	OBJ_Size            = 2,
	BG_TileMap          = 3,
	BG_WindowTiles      = 4,
	Window_Enable       = 5,
	Window_TileMap      = 6,
	PPU_Enable          = 7,
}

PPU_Status :: enum {
	LYC_EQ_LY           = 2,
	HBLANK_INT_SELECT   = 3,
	VBLANK_INT_SELECT   = 4,
	OAM_SCAN_INT_SELECT = 5,
	LYC_INT_SELECT      = 6,
}

PPU_Object :: struct {
	x:       int,
	y:       int,
	height:  int,
	tile_id: u8,
	attrs:   u8,
}

PPU_Object_Attr :: enum {
	Priority    = 7,
	Y_Flip      = 6,
	X_Flip      = 5,
	DMG_Palette = 4,
}

PPU_Palette :: [4]rl.Color

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

	ppu.bg_palette = [4]rl.Color{rl.WHITE, rl.ORANGE, rl.RED, rl.BLACK}
	ppu.obj0_palette = [4]rl.Color {
		rl.Color{136, 192, 112, 255},
		rl.Color{52, 104, 86, 255},
		rl.Color{8, 24, 32, 255},
		rl.Color{224, 248, 208, 255},
	}
	ppu.obj1_palette = [4]rl.Color {
		rl.Color{0, 170, 188, 255},
		rl.Color{0, 110, 132, 255},
		rl.Color{0, 68, 75, 255},
		rl.Color{224, 248, 208, 255},
	}
}

ppu_tick :: proc(gb: ^GB, tick: u64) {
	ppu := &gb.ppu
	stat := mem_read(&gb.mem, u16(GB_HardRegister.STAT))
	lyc := mem_read(&gb.mem, u16(GB_HardRegister.LYC))
	ppu.scanline = u8(tick / ScanelineDots)

	mem_write(&gb.mem, u16(GB_HardRegister.LY), ppu.scanline)
	handle_lcd_interrupts(gb, stat, lyc)
	update_lcd_status(gb, stat, lyc)

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

handle_lcd_interrupts :: proc(gb: ^GB, stat: u8, lyc: u8) {
	interrupt_line :=
		(gb.ppu.mode == PPU_Mode.HBlank &&
			check_lcd_interrupt_enabled(stat, PPU_Status.HBLANK_INT_SELECT)) ||
		(gb.ppu.mode == PPU_Mode.VBlank &&
				check_lcd_interrupt_enabled(stat, PPU_Status.VBLANK_INT_SELECT)) ||
		(gb.ppu.mode == PPU_Mode.OAM_Scan &&
				check_lcd_interrupt_enabled(stat, PPU_Status.OAM_SCAN_INT_SELECT)) ||
		(lyc == gb.ppu.scanline && check_lcd_interrupt_enabled(stat, PPU_Status.LYC_INT_SELECT))


	if !gb.ppu.interrupt_line && interrupt_line {
		cpu_request_interrupt(&gb.cpu, &gb.mem, Interrupt.LCD)
	}

	gb.ppu.interrupt_line = interrupt_line
}

update_lcd_status :: proc(gb: ^GB, stat: u8, lyc: u8) {
	stat := (stat & 0xFC) | (u8(gb.ppu.mode) & 0x03)

	if gb.ppu.scanline == lyc {
		stat |= (1 << u8(PPU_Status.LYC_EQ_LY))
	} else {
		stat &= ~u8(1 << u8(PPU_Status.LYC_EQ_LY))
	}

	// don't use mem_write intentionally
	gb.mem.write(&gb.mem, u16(GB_HardRegister.STAT), stat)
}

check_lcd_interrupt_enabled :: proc(stat: u8, interrupt: PPU_Status) -> bool {
	return (stat & (1 << u8(interrupt))) > 0
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
	bg_pixel_color: u8 = 0
	obj_pixel_color: u8 = 0
	drawn_obj: PPU_Object

	// if LCDC.0 bit is 0, the background / window pixel color is white
	if get_control_flag(mem, PPU_Control.BG_WindowEnablePrio) {
		bg_tile_addr := get_tile_addr(mem, bg_tile_id, is_object = false)
		bg_pixel_color = get_pixel_color(ppu, mem, bg_tile_addr, tilemap_x % 8, tilemap_y % 8)
	}

	if get_control_flag(mem, PPU_Control.OBJ_Enable) {
		objects := make([dynamic]PPU_Object)
		defer delete(objects)

		get_objects_at_x(ppu, x, &objects)
		obj_pixel_color, drawn_obj = get_object_pixel_color(ppu, mem, objects[:], x, y)
	}

	output_pixel_color: u8
	palette_addr: u16

	if obj_pixel_color > 0 {
		// priority == true means BG/Window color 1-3 (non-transparent) is drawn over this object
		priority := get_object_attr(drawn_obj, PPU_Object_Attr.Priority)

		if priority && bg_pixel_color > 0 {
			output_pixel_color = bg_pixel_color
			palette_addr = BGP_Addr
		} else {
			output_pixel_color = obj_pixel_color
			obj_palette := get_object_attr(drawn_obj, PPU_Object_Attr.DMG_Palette)
			palette_addr = obj_palette ? OBP1_Addr : OBP0_Addr
		}
	} else {
		output_pixel_color = bg_pixel_color
		palette_addr = BGP_Addr
	}

	return get_color_from_palette(ppu, mem, palette_addr, output_pixel_color)
}

get_color_from_palette :: proc(
	ppu: ^PPU,
	mem: ^GB_Memory,
	palette_addr: u16,
	color_id: u8,
) -> rl.Color {
	mask := u8(0x3) << (color_id * 2)
	palette_data := mem_read(mem, BGP_Addr)
	palette_color_id := (palette_data & mask) >> (color_id * 2)
	palette: ^PPU_Palette

	if palette_addr == BGP_Addr {
		palette = &ppu.bg_palette
	} else if palette_addr == OBP0_Addr {
		palette = &ppu.obj0_palette
	} else if palette_addr == OBP1_Addr {
		palette = &ppu.obj1_palette
	} else {
		panic("unexpected palette address")
	}

	return palette[palette_color_id]
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

get_objects_at_x :: proc(ppu: ^PPU, x: int, res: ^[dynamic]PPU_Object) {
	objects := sa.slice(&ppu.scanline_objects)

	for obj in objects {
		if x >= obj.x && x <= obj.x + 7 {
			append(res, obj)
		}
	}
}

get_object_attr :: proc(obj: PPU_Object, attr: PPU_Object_Attr) -> bool {
	mask := u8(1) << u8(attr)

	return (obj.attrs & mask) > 0
}

// https://gbdev.io/pandocs/Tile_Data.html
get_pixel_color :: proc(
	ppu: ^PPU,
	mem: ^GB_Memory,
	tile_addr: u16,
	x_in_tile: int,
	y_in_tile: int,
	x_flip := false,
	y_flip := false,
) -> u8 {
	// a tile is 16 bytes
	// each 2 bytes represent 1 line of the tile

	x_in_tile := x_in_tile
	y_in_tile := y_in_tile

	if x_flip {
		x_in_tile = 7 - x_in_tile
	}

	if y_flip {
		// TODO: 8x16 objects?
		y_in_tile = 7 - y_in_tile
	}

	x_mask := u8(1 << (7 - u8(x_in_tile)))
	y_offset := u16(y_in_tile * 2)
	tile_data_low := mem.read(mem, tile_addr + y_offset)
	tile_data_high := mem.read(mem, tile_addr + y_offset + 1)
	color_low := (tile_data_low & x_mask) > 0 ? 1 : 0
	color_high := (tile_data_high & x_mask) > 0 ? 1 : 0
	color_id := u8(color_high << 1) | u8(color_low)

	return color_id
}

get_object_pixel_color :: proc(
	ppu: ^PPU,
	mem: ^GB_Memory,
	objects: []PPU_Object,
	x: int,
	y: int,
) -> (
	pixel_color: u8,
	selected_obj: PPU_Object,
) {
	ensure(len(objects) <= MaxObjectsPerScanline)

	pixel_color = 0

	for &obj in objects {
		x_in_tile := x - obj.x
		y_in_obj := y - obj.y
		y_in_tile: int
		tile_id: u8

		if obj.height == 16 {
			if y_in_obj < 8 {
				tile_id = obj.tile_id & 0xFE
				y_in_tile = y_in_obj
			} else {
				tile_id = obj.tile_id | 0x01
				y_in_tile = y_in_obj - 8
			}
		} else {
			tile_id = obj.tile_id
			y_in_tile = y_in_obj
		}

		obj_tile_addr := get_tile_addr(mem, tile_id, is_object = true)
		x_flip := get_object_attr(obj, PPU_Object_Attr.X_Flip)
		y_flip := get_object_attr(obj, PPU_Object_Attr.Y_Flip)
		obj_pixel_color := get_pixel_color(
			ppu,
			mem,
			obj_tile_addr,
			x_in_tile,
			y_in_tile,
			x_flip,
			y_flip,
		)

		if obj_pixel_color == 0 {
			continue
		}

		if pixel_color == 0 || (obj.x < selected_obj.x) {
			selected_obj = obj
			pixel_color = obj_pixel_color
		}
	}

	return
}

get_tile_addr :: proc(mem: ^GB_Memory, tile_id: u8, is_object: bool) -> u16 {
	// https://gbdev.io/pandocs/Tile_Data.html

	window_bg_data := get_control_flag(mem, PPU_Control.BG_WindowTiles)

	// objects always use 8000 addressing
	if window_bg_data || is_object {
		// 8000 addressing
		tile_offset := u16(tile_id) * 16

		return 0x8000 + u16(tile_offset)
	}

	// 8800 addressing

	if tile_id < 128 {
		// use block 2 for 0 - 127 ids
		tile_offset := u16(tile_id) * 16

		return 0x9000 + u16(tile_offset)
	} else {
		// use block 1 for 128 - 255 ids
		tile_offset := u16(tile_id - 128) * 16

		return 0x8800 + u16(tile_offset)
	}
}

// https://gbdev.io/pandocs/OAM.html
scan_oam :: proc(ppu: ^PPU, mem: ^GB_Memory) {
	oam_addr: u16 = 0xFE00

	sa.clear(&ppu.scanline_objects)

	for i: u16 = 0;
	    i < OAM_EntryCount && sa.len(ppu.scanline_objects) < MaxObjectsPerScanline;
	    i += 1 {
		obj_addr := oam_addr + (i * 4)
		y_pos := int(mem_read(mem, obj_addr)) - 16
		obj_height := get_control_flag(mem, PPU_Control.OBJ_Size) ? 16 : 8

		if int(ppu.scanline) >= y_pos && int(ppu.scanline) < y_pos + obj_height {
			x_pos := int(mem_read(mem, obj_addr + 1)) - 8
			tile_id := mem_read(mem, obj_addr + 2)
			attrs := mem_read(mem, obj_addr + 3)

			sa.append(&ppu.scanline_objects, PPU_Object{x_pos, y_pos, obj_height, tile_id, attrs})
			// log.debugf("%x %x %d %x %x", x_pos, y_pos, obj_height, tile_id, attrs)
		}
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
