package gb_emu

PPU :: struct {
    scanline: int,
    tick: int,
    mode: int
}

ppu_begin_frame :: proc(ppu: ^PPU) {
    ppu.scanline = 0
    ppu.tick = 0
    ppu.mode = 2
}

ppu_tick :: proc(ppu: ^PPU) {
    if ppu.scanline < 144 {
        if (ppu.tick >= 80 + 172) {
            ppu.mode = 0
        } else if (ppu.tick >= 80) {
            ppu.mode = 3
        } else {
            ppu.mode = 2
        }
    } else {
        ppu.mode = 1
    }
    
    ppu.tick += 1

    if ppu.tick >= 456 {
        ppu.scanline += 1
        ppu.tick = 0
    }
}
