package gb_emu

import "core:testing"

@(test)
ppu_modes :: proc(t: ^testing.T) {
    ppu := PPU{}
    mem: [0xFFFF+1]u8
    tick: i64 = 0

    ppu_init(&ppu, mem[:])

    // test modes for the first 144 scanlines of the frame

    for i := 0; i < 144; i += 1 {
        for j := 0; j < 80; j += 1 {
            ppu_tick(&ppu, mem[:], tick)
            tick += 1

            testing.expect_value(t, ppu.mode, PPU_Mode.OAM_Scan)
        }

        for j := 0; j < 172; j += 1 {
            ppu_tick(&ppu, mem[:], tick)
            tick += 1

            testing.expect_value(t, ppu.mode, PPU_Mode.DrawPixels)
        }

        for j := 0; j < 204; j += 1 {
            ppu_tick(&ppu, mem[:], tick)
            tick += 1

            testing.expect_value(t, ppu.mode, PPU_Mode.HBlank)
        }
    }

    testing.expect_value(t, ppu.scanline, 143)

    // then for the last 10 scanlines of the frame
    for i := 0; i < 10; i += 1 {
        for j := 0; j < 456; j += 1 {
            ppu_tick(&ppu, mem[:], tick)
            tick += 1

            testing.expect_value(t, ppu.mode, PPU_Mode.VBlank)
        }
    }

    testing.expect_value(t, ppu.scanline, 153)
}

@(private="file")
run_ppu :: proc(ppu: ^PPU, mem: []u8, ticks: i64) {
    for i: i64 = 0; i < ticks; i += 1 {
        ppu_tick(ppu, mem[:], i)
    }
}
