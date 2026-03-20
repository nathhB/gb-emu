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
	color:            bool,
	scanline:         u8,
	mode:             PPU_Mode,
	framebuffer:      [ScreenWidth * ScreenHeight]rl.Color,
	scanline_objects: sa.Small_Array(MaxObjectsPerScanline, PPU_Object),
	states:           [4]PPU_State,
	interrupt_line:   bool,
	bg_colors:        DMG_Colors,
	obp0_colors:      DMG_Colors,
	obp1_colors:      DMG_Colors,
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

DMG_Colors :: [4]rl.Color

DMG_Palette :: enum {
	BGP,
	OBP0,
	OBP1,
}

ppu_init :: proc(ppu: ^PPU, mem: ^GB_Memory, color: bool) {
	ppu.color = color
	ppu.scanline = 0
	ppu.mode = PPU_Mode.OAM_Scan

	vblank_duration := VBlank_ScanelineCount * ScanelineDots

	ppu.states[PPU_Mode.HBlank] = create_state(hblank_on_enter, nil, 204, PPU_Mode.OAM_Scan)
	ppu.states[PPU_Mode.VBlank] = create_state(
		vblank_on_enter,
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

	ppu.bg_colors = [4]rl.Color{rl.WHITE, rl.ORANGE, rl.RED, rl.BLACK}
	ppu.obp0_colors = [4]rl.Color {
		rl.Color{136, 192, 112, 255},
		rl.Color{52, 104, 86, 255},
		rl.Color{8, 24, 32, 255},
		rl.Color{224, 248, 208, 255},
	}
	ppu.obp1_colors = [4]rl.Color {
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

	mem_write(gb, u16(GB_HardRegister.LY), ppu.scanline)
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
		cpu_request_interrupt(gb, Interrupt.LCD)
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
	gb.mem.write(gb, u16(GB_HardRegister.STAT), stat)
}

check_lcd_interrupt_enabled :: proc(stat: u8, interrupt: PPU_Status) -> bool {
	return (stat & (1 << u8(interrupt))) > 0
}

oam_scan_on_enter :: proc(gb: ^GB) {
	assert(gb.ppu.mode == PPU_Mode.OAM_Scan)
	scan_oam(&gb.ppu, &gb.mem)
}

hblank_on_enter :: proc(gb: ^GB) {
	if hdma_transfer_is_active(&gb.mem) && gb.mem.hdma_transfer.mode == .HBlank {
		hdma_copy_one_block(gb)
	}
}

vblank_on_enter :: proc(gb: ^GB) {
	assert(gb.ppu.mode == PPU_Mode.VBlank)
	cpu_request_interrupt(gb, Interrupt.VBlank)
}

draw_pixels_on_update :: proc(gb: ^GB, tick: u64) {
	ppu := &gb.ppu

	assert(ppu.mode == PPU_Mode.DrawPixels)

	x := ppu.states[PPU_Mode.DrawPixels].current_tick // output one pixel per dot

	if x > 159 {
		return
	}

	y := int(gb.ppu.scanline)
	pixel_color := get_pixel_color_at(gb, x, y)
	framebuffer_offset := y * ScreenWidth + x

	ppu.framebuffer[framebuffer_offset] = pixel_color
}

get_pixel_color_at :: proc(gb: ^GB, x: int, y: int) -> rl.Color {
	mem := &gb.mem
	ppu := &gb.ppu
	scx := int(mem_read(mem, u16(GB_HardRegister.SCX)))
	scy := int(mem_read(mem, u16(GB_HardRegister.SCY)))
	tilemap_x := (scx + x) % 256
	tilemap_y := (scy + y) % 256
	bg_tilemap_addr := get_bg_tilemap_addr(gb, tilemap_x, tilemap_y)
	bg_tile_id, bg_tile_props := get_bg_tile(gb, bg_tilemap_addr, tilemap_x, tilemap_y)
	bg_pixel_color: u8 = 0
	obj_pixel_color: u8 = 0
	drawn_obj: PPU_Object

	// in DMG mode, if LCDC.0 bit is 0, the background / window pixel color is white
	// in CGB mode, if LCDC.0 bit is 0, background / window pixel never has priority over object pixels
	bg_win_enable_prio := get_control_flag(mem, PPU_Control.BG_WindowEnablePrio)

	if ppu.color || bg_win_enable_prio {
		bg_tile_addr := get_tile_addr(mem, bg_tile_id, is_object = false)
		x_flip := false
		y_flip := false
		bank := 0

		if ppu.color {
			x_flip = get_tile_x_flip(bg_tile_props)
			y_flip = get_tile_y_flip(bg_tile_props)
			bank = get_tile_bank(bg_tile_props)
		}

		bg_pixel_color = get_pixel_color(
			ppu,
			mem,
			bg_tile_addr,
			tilemap_x % 8,
			tilemap_y % 8,
			x_flip = x_flip,
			y_flip = y_flip,
			bank = bank,
		)
	}

	if get_control_flag(mem, PPU_Control.OBJ_Enable) {
		objects := make([dynamic]PPU_Object)
		defer delete(objects)

		get_objects_at_x(ppu, x, &objects)
		obj_pixel_color, drawn_obj = get_object_pixel_color(ppu, mem, objects[:], x, y)
	}

	draw_obj := false

	if obj_pixel_color > 0 {
		// priority == 1 means BG/Window color 1-3 (non-transparent) is drawn over this object
		// in CGB mode, background priority of 1 takes precedence over OAM priority
		bg_priority := ppu.color && bg_win_enable_prio ? get_tile_priority(bg_tile_props) : 0
		priority := bg_priority == 1 ? 1 : get_tile_priority(drawn_obj.attrs)

		if priority == 0 || bg_pixel_color == 0 {
			draw_obj = true
		}
	}

	if draw_obj {
		if ppu.color {
			palette := get_tile_color_palette(drawn_obj.attrs)

			return get_color_from_obj_color_palette(ppu, mem, palette, obj_pixel_color)
		} else {
			palette_id := get_tile_dmg_palette(drawn_obj.attrs)
			palette := palette_id == 0 ? DMG_Palette.OBP0 : DMG_Palette.OBP1

			return get_color_from_dmg_palette(ppu, mem, palette, obj_pixel_color)
		}
	} else {
		if ppu.color {
			palette := get_tile_color_palette(bg_tile_props)

			return get_color_from_bg_color_palette(ppu, mem, palette, bg_pixel_color)
		} else {
			return get_color_from_dmg_palette(ppu, mem, .BGP, bg_pixel_color)
		}
	}
}

get_color_from_dmg_palette :: proc(
	ppu: ^PPU,
	mem: ^GB_Memory,
	palette: DMG_Palette,
	color_id: u8,
) -> rl.Color {
	colors: ^DMG_Colors
	palette_addr: u16

	switch palette {
	case DMG_Palette.BGP:
		colors = &ppu.bg_colors
		palette_addr = BGP_Addr
	case DMG_Palette.OBP0:
		colors = &ppu.obp0_colors
		palette_addr = OBP0_Addr
	case DMG_Palette.OBP1:
		colors = &ppu.obp1_colors
		palette_addr = OBP1_Addr
	}

	mask := u8(0x3) << (color_id * 2)
	palette_data := mem_read(mem, palette_addr)
	palette_color_id := (palette_data & mask) >> (color_id * 2)

	return colors[palette_color_id]
}

get_color_from_bg_color_palette :: proc(
	ppu: ^PPU,
	mem: ^GB_Memory,
	palette: u8,
	color: u8,
) -> rl.Color {
	// https://gbdev.io/pandocs/Palettes.html#ff69--bcpdbgpd-cgb-mode-only-background-color-palette-data--background-palette-data

	assert(color >= 0 && color < 4)

	addr := (palette * 8) + (color * 2)
	color_byte_lo := mem.bg_palettes[addr]
	color_byte_hi := mem.bg_palettes[addr + 1]
	color16 := (u16(color_byte_hi) << 8) | u16(color_byte_lo)
	r5 := u8(color16 & 0x1F)
	g5 := u8((color16 >> 5) & 0x1F)
	b5 := u8((color16 >> 10) & 0x1F)
	r8 := r5 << 3
	g8 := g5 << 3
	b8 := b5 << 3

	return rl.Color{r8, g8, b8, 255}
}

get_color_from_obj_color_palette :: proc(
	ppu: ^PPU,
	mem: ^GB_Memory,
	palette: u8,
	color: u8,
) -> rl.Color {
	// https://gbdev.io/pandocs/Palettes.html#ff69--bcpdbgpd-cgb-mode-only-background-color-palette-data--background-palette-data

	assert(color > 0 && color < 4)

	addr := (palette * 8) + (color * 2)
	color_byte_lo := mem.obj_palettes[addr]
	color_byte_hi := mem.obj_palettes[addr + 1]
	color16 := (u16(color_byte_hi) << 8) | u16(color_byte_lo)
	r5 := u8(color16 & 0x1F)
	g5 := u8((color16 >> 5) & 0x1F)
	b5 := u8((color16 >> 10) & 0x1F)
	r8 := r5 << 3
	g8 := g5 << 3
	b8 := b5 << 3

	return rl.Color{r8, g8, b8, 255}
}

get_bg_tile :: proc(gb: ^GB, tilemap_addr: u16, x: int, y: int) -> (id: u8, props: u8) {
	tile_x := x / 8
	tile_y := y / 8
	tilemap_offset := (tile_y * 32) + tile_x
	tile_addr := tilemap_addr + u16(tilemap_offset)

	id = mem_read(&gb.mem, tile_addr, override_vram_bank = 0)

	if gb.color {
		// CGB: read BG tile attribute in VRAM bank 1
		props = mem_read(&gb.mem, tile_addr, override_vram_bank = 1)
	}

	return
}

get_bg_tilemap_addr :: proc(gb: ^GB, x: int, y: int) -> u16 {
	mem := &gb.mem
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

	return bg_tilemap_addr
}

get_objects_at_x :: proc(ppu: ^PPU, x: int, res: ^[dynamic]PPU_Object) {
	objects := sa.slice(&ppu.scanline_objects)

	for obj in objects {
		if x >= obj.x && x <= obj.x + 7 {
			append(res, obj)
		}
	}
}

get_tile_priority :: proc(attrs: u8) -> u8 {
	mask := u8(1) << 7

	return (attrs & mask) >> 7
}

get_tile_y_flip :: proc(attrs: u8) -> bool {
	mask := u8(1) << 6

	return (attrs & mask) > 0
}

get_tile_x_flip :: proc(attrs: u8) -> bool {
	mask := u8(1) << 5

	return (attrs & mask) > 0
}

get_tile_dmg_palette :: proc(attrs: u8) -> u8 {
	mask := u8(1) << 4

	return (attrs & mask) >> 4
}

// CGB only
get_tile_bank :: proc(attrs: u8) -> int {
	mask := u8(1) << 3

	return int((attrs & mask) >> 3)
}

// CGB only
get_tile_color_palette :: proc(attrs: u8) -> u8 {
	return attrs & 0x07
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
	bank := 0,
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
	tile_data_low := mem_read(mem, tile_addr + y_offset, override_vram_bank = bank)
	tile_data_high := mem_read(mem, tile_addr + y_offset + 1, override_vram_bank = bank)
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
	final_pixel_color: u8,
	selected_obj: PPU_Object,
) {
	ensure(len(objects) <= MaxObjectsPerScanline)

	final_pixel_color = 0

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
		x_flip := get_tile_x_flip(obj.attrs)
		y_flip := get_tile_y_flip(obj.attrs)
		bank := ppu.color ? get_tile_bank(obj.attrs) : 0
		obj_pixel_color := get_pixel_color(
			ppu,
			mem,
			obj_tile_addr,
			x_in_tile,
			y_in_tile,
			x_flip,
			y_flip,
			bank = bank,
		)

		if obj_pixel_color > 0 {
			if ppu.color {
				// in CGB, object priority is based on position in OAM
				final_pixel_color = obj_pixel_color
				return
			}

			// in DMG mode, object priority is based on X position

			if final_pixel_color == 0 || (obj.x < selected_obj.x) {
				selected_obj = obj
				final_pixel_color = obj_pixel_color
			}
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

	return (mem_read(mem, u16(GB_HardRegister.LCDC)) & mask) > 0
}

create_state :: proc(
	on_enter: proc(gb: ^GB),
	on_update: proc(gb: ^GB, tick: u64),
	duration_ticks: int,
	next_mode: PPU_Mode,
) -> PPU_State {
	return PPU_State{on_enter, on_update, 0, duration_ticks, next_mode}
}
