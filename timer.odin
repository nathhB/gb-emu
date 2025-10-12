package gb_emu

import "core:log"

// https://gbdev.io/pandocs/Timer_and_Divider_Registers.html#ff05--tima-timer-counter

DivDots :: 256 // 16384 Hz
ClockOO_Dots :: 1024
ClockO1_Dots :: 16
Clock10_Dots :: 64
Clock11_Dots :: 256

timer_tick :: proc(gb: ^GB) {
	process_div(gb)

	tac := mem_read(&gb.mem, u16(GB_HardRegister.TAC))

	clock_type := tac & 3
	clock_enabled := (tac & 4) > 0

	gb.timer_dots = get_clock_dots(clock_type)

	if clock_enabled {
		process_timer(gb)
	}
}

get_clock_dots :: proc(type: u8) -> u64 {
	if type == 0 {
		return ClockOO_Dots
	} else if type == 1 {
		return ClockO1_Dots
	} else if type == 2 {
		return Clock10_Dots
	} else if type == 3 {
		return Clock11_Dots
	}

	unreachable()
}

process_div :: proc(gb: ^GB) {
	div := mem_read(&gb.mem, u16(GB_HardRegister.DIV))

	gb.div_acc += 1

	if gb.div_acc >= DivDots {
		div += 1
		gb.div_acc = 0
	}

	// can't use mem_write here or it will reset the register to 0
	gb.mem.data[u16(GB_HardRegister.DIV)] = div
}

process_timer :: proc(gb: ^GB) {
	gb.timer_acc += 1

	if gb.timer_acc >= gb.timer_dots {
		tima := u16(mem_read(&gb.mem, u16(GB_HardRegister.TIMA)))
		tima += 1

		if tima > 0xFF {
			cpu_request_interrupt(&gb.cpu, &gb.mem, Interrupt.Timer)

			tima = 0
		}

		mem_write(&gb.mem, u16(GB_HardRegister.TIMA), u8(tima))
		gb.timer_acc = 0
	}
}
