package gb_emu

import "core:log"
import "core:time"
import rl "vendor:raylib"

PPU :: struct {
    scanline: int,
    mode: PPU_Mode,
    colors: [4]rl.Color,
    framebuffer: [ScreenWidth*ScreenHeight]u32
}

PPU_Mode :: enum {
    HBlank,
    VBlank,
    OAM_Scan,
    DrawPixels
}

// https://gbdev.io/pandocs/LCDC.html
PPU_Control :: enum {
    BG_WindowEnablePrio,
    OBJ_Enable,
    OBJ_Size,
    BG_TileMap,
    BG_WindowTiles,
    WindowEnable,
    WindowTileMap,
    PPU_Enable
}

ppu_init :: proc(ppu: ^PPU, mem: []u8) {
    ppu.scanline = 0
    ppu.mode = PPU_Mode.OAM_Scan
    ppu.colors = [4]rl.Color{rl.WHITE, rl.BLACK, rl.BLACK, rl.BLACK}
}

ppu_tick :: proc(ppu: ^PPU, mem: []u8, tick: i64) {
    scanline := int(tick / ScanelineDots)
    scanline_tick := int(tick % ScanelineDots)

    ppu.scanline = scanline
    ppu.mode = get_mode(ppu.scanline, scanline_tick)
    mem[GB_HardRegister.LY] = u8(scanline)

    if ppu.mode == PPU_Mode.DrawPixels {
        x := scanline_tick - 80

        draw_pixel(ppu, mem, x, ppu.scanline)
    }
}

get_mode :: proc(scanline: int, scanline_tick: int) -> PPU_Mode {
    if scanline >= FrameDrawScanlines {
        return PPU_Mode.VBlank
    }

    if scanline_tick < 80 {
        return PPU_Mode.OAM_Scan
    } else if scanline_tick < 80 + 172 {
        return PPU_Mode.DrawPixels
    }

    return PPU_Mode.HBlank
}

draw_pixel :: proc(ppu: ^PPU, mem: []u8, x: int, y: int) {
    if x > 159 {
        return
    }

    scx := int(mem[GB_HardRegister.SCX])
    scy := int(mem[GB_HardRegister.SCY])
    tilemap_x := (scx + x) % 256
    tilemap_y := (scy + y) % 256 
    tile_id := get_tile_id(mem, tilemap_x, tilemap_y)
    color := get_pixel_color(ppu, mem, tile_id, tilemap_x, tilemap_y)
    framebuffer_offset := y * ScreenWidth + x

    ppu.framebuffer[framebuffer_offset] = color
}

get_tile_id :: proc(mem: []u8, x: int, y: int) -> u8 {
    // TODO: handle window rendering
    // for now assume the pixel is never inside the window

    tile_x := x / 8
    tile_y := y / 8
    bg_tilemap_addr := get_control_flag(mem, PPU_Control.BG_TileMap) ? 0x9C00 : 0x9800
    tilemap_offset := (tile_y * 32) + tile_x

    return mem[bg_tilemap_addr + tilemap_offset]
}

// https://gbdev.io/pandocs/Tile_Data.html
get_pixel_color :: proc(ppu: ^PPU, mem: []u8, tile_id: u8, x: int, y: int) -> u32 {
    // a tile is 16 bytes
    // each 2 bytes represent 1 line of the tile

    tile_addr := get_tile_addr(mem, tile_id)
    x_in_tile := x % 8
    y_in_tile := y % 8
    x_mask := u8(1 << (7 - u8(x_in_tile)))
    y_offset := u16(y_in_tile * 2)
    tile_data_low := mem[tile_addr + y_offset]
    tile_data_high := mem[tile_addr + y_offset + 1] 
    color_low := (tile_data_low & x_mask) > 0 ? 1 : 0
    color_high := (tile_data_high & x_mask) > 0 ? 1 : 0

    return rl.ColorToInt(ppu.colors[u8(color_high << 8) | u8(color_low)])
}

get_tile_addr :: proc(mem: []u8, tile_id: u8) -> u16 {
    // https://gbdev.io/pandocs/Tile_Data.html

    window_bg_data := get_control_flag(mem, PPU_Control.BG_WindowTiles)
    tile_offset := tile_id * 16

    if window_bg_data {
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

get_control_flag :: proc(mem: []u8, flag: PPU_Control) -> bool {
    return (mem[GB_HardRegister.LCDC] & (1 << u8(flag))) > 0
}
