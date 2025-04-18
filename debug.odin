package gb_emu

import "core:log"

print_vram :: proc(vram: []u8) {
    for i := 0; i < 384; i += 1 {
        tile_addr := 0x8000 + (i * 16)

        log.debugf("Tile %d: ");

        for j := 0; j < 16; j += 1 {
            log.debugf("%x", vram[tile_addr + j])
        }
    }
}
