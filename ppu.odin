package gb_emu

import "core:log"
import "core:time"

PPU :: struct {
    scanline: int,
    mode: PPU_Mode,
}

PPU_Mode :: enum {
    HBlank,
    VBlank,
    OAM_Scan,
    DrawPixels
}

ppu_init :: proc(ppu: ^PPU, mem: []u8) {
    ppu.scanline = 0
    ppu.mode = PPU_Mode.OAM_Scan
}

ppu_tick :: proc(ppu: ^PPU, mem: []u8, tick: i64) {
    scanline := int(tick / ScanelineDots)
    scanline_tick := int(tick % ScanelineDots)

    ppu.scanline = scanline
    ppu.mode = get_mode(ppu.scanline, scanline_tick)
    mem[GB_HardRegister.LY] = u8(scanline)

    if ppu.mode == PPU_Mode.DrawPixels {
        x := scanline_tick - 80

        get_pixel(x)
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

get_pixel :: proc(x: int) {
    if x > 159 {
        return
    }
}

get_tile :: proc(ppu: ^PPU, mem: []u8) {
    lcd_control := mem[GB_HardRegister.LCDC]
    ppu_enabled := (lcd_control & (1 << 7)) > 0
    window_tilemap := (lcd_control & (1 << 6)) > 0
    window_enabled := (lcd_control & (1 << 5)) > 0
    window_bg_data := (lcd_control & (1 << 4)) > 0
    bg_tilemap := (lcd_control & (1 << 3)) > 0
    obj_size := (lcd_control & (1 << 2)) > 0
    obj_enabled := (lcd_control & (1 << 1)) > 0
    bg_window_enabled_priority := (lcd_control & 1) > 0

    // log.debug(lcd_control)
}
