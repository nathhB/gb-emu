package gb_emu

import "core:testing"

@(test)
ppu_modes :: proc(t: ^testing.T) {
    gb := GB{}
    tick: u64 = 0

    mbc_dummy_init(&gb.mem)
    ppu_init(&gb.ppu, &gb.mem)

    // test modes for the first 144 scanlines of the frame

    for i := 0; i < 144; i += 1 {
        for j := 0; j < 80; j += 1 {
            ppu_tick(&gb, tick)
            tick += 1

            testing.expect_value(t, gb.ppu.mode, PPU_Mode.OAM_Scan)
        }

        for j := 0; j < 172; j += 1 {
            ppu_tick(&gb, tick)
            tick += 1

            testing.expect_value(t, gb.ppu.mode, PPU_Mode.DrawPixels)
        }

        for j := 0; j < 204; j += 1 {
            ppu_tick(&gb, tick)
            tick += 1

            testing.expect_value(t, gb.ppu.mode, PPU_Mode.HBlank)
        }
    }

    testing.expect_value(t, gb.ppu.scanline, 143)

    // then for the last 10 scanlines of the frame
    for i := 0; i < 10; i += 1 {
        for j := 0; j < 456; j += 1 {
            ppu_tick(&gb, tick)
            tick += 1

            testing.expect_value(t, gb.ppu.mode, PPU_Mode.VBlank)
        }
    }

    testing.expect_value(t, gb.ppu.scanline, 153)
}

@(private="file")
run_ppu :: proc(gb: ^GB, ticks: u64) {
    for i: u64 = 0; i < ticks; i += 1 {
        ppu_tick(gb, i)
    }
}
