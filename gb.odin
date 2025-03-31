package gb_emu

import "core:time"
import "core:log"

DotNs: i64 = 238 // 1 dot = 2^22 Hz, 1 dot = 1 T-Cycle, 1 M-Cycle = 4 T-Cycle

GB :: struct {
    mem: [0xFFFF]u8,
    cpu: CPU,
    ppu: PPU
}

gb_init :: proc(gb: ^GB) {
    cpu_init(&gb.cpu)
}

gb_run :: proc(gb: ^GB) {
    t := time.tick_now()
    acc_ns: i64 = 0

    for true {
        dt := time.tick_diff(t, time.tick_now())
        t = time.tick_now()
        dt_ns := time.duration_nanoseconds(dt)

        acc_ns += dt_ns

        if acc_ns > DotNs {
            tick(gb)

            acc_ns -= DotNs
        }
    }
}

@(private="file")
tick :: proc(gb: ^GB) {
    cpu_tick(&gb.cpu, gb.mem[:])
}
