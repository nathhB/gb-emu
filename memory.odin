package gb_emu

import "core:log"
import rl "vendor:raylib"

GB_Memory :: struct {
	data:         [0xFFFF + 1]u8,
	rom:          []u8,
	rom_bank:     int,
	ram_bank:     int,
	external_ram: bool,
	read:         proc(mem: ^GB_Memory, addr: u16) -> u8,
	write:        proc(gb: ^GB, addr: u16, byte: u8),
	get_ptr:      proc(mem: ^GB_Memory, addr: u16) -> ^u8,
	oam_transfer: struct {
		active:   bool,
		src_addr: u16,
		current:  u8,
	},
}

mem_write :: proc(gb: ^GB, addr: u16, byte: u8) {
	if addr >= 0xFF00 {
		write_to_hardware_register(gb, addr, byte)

		return
	}

	gb.mem.write(gb, addr, byte)
}

mem_read :: proc(mem: ^GB_Memory, addr: u16) -> u8 {
	if addr == 0xFF00 {
		byte := mem.read(mem, addr)

		return byte
	} else {
		return mem.read(mem, addr)
	}
}

mem_get_ptr :: proc(mem: ^GB_Memory, addr: u16) -> ^u8 {
	return mem.get_ptr(mem, addr)
}

mem_tick :: proc(gb: ^GB) {
	if gb.mem.oam_transfer.active {
		do_oam_transfer(gb)
	}
}

// https://gbdev.io/pandocs/Hardware_Reg_List.html
write_to_hardware_register :: proc(gb: ^GB, addr: u16, byte: u8) {
	reg := GB_HardRegister(addr)

	if reg == GB_HardRegister.JOYPAD {
		write_to_joypad(gb, byte)
	} else if reg == GB_HardRegister.DIV {
		write_to_div(gb)
	} else if reg == GB_HardRegister.DMA {
		start_dma_transfer(&gb.mem, byte)
	} else if reg == GB_HardRegister.STAT {
		gb.mem.write(gb, addr, byte & 0x78)
	} else if reg >= GB_HardRegister.NR10 && reg <= GB_HardRegister.NR34 {
		write_to_audio_registers(gb, reg, byte)
	} else {
		gb.mem.write(gb, addr, byte)
	}
}

write_to_joypad :: proc(gb: ^GB, byte: u8) {
	data := u8(0xCF | byte)
	select_buttons := (data & (1 << 5)) == 0
	select_dpad := (data & (1 << 4)) == 0

	if select_dpad {
		data &= ~u8(gb.inputs.right)
		data &= ~(u8(gb.inputs.left) << 1)
		data &= ~(u8(gb.inputs.up) << 2)
		data &= ~(u8(gb.inputs.down) << 3)
	}

	if select_buttons {
		data &= ~u8(gb.inputs.a)
		data &= ~(u8(gb.inputs.b) << 1)
		data &= ~(u8(gb.inputs.select) << 2)
		data &= ~(u8(gb.inputs.start) << 3)
	}

	gb.mem.write(gb, u16(GB_HardRegister.JOYPAD), data)
}

write_to_audio_registers :: proc(gb: ^GB, reg: GB_HardRegister, byte: u8) {
	// log.debugf("WRITE TO AUDIO: %w: %x", reg, byte)
}

write_to_div :: proc(gb: ^GB) {
	gb.mem.write(gb, u16(GB_HardRegister.DIV), 0) // writing to DIV resets it
}

// https://gbdev.io/pandocs/OAM_DMA_Transfer.html#ff46--dma-oam-dma-source-address--start
start_dma_transfer :: proc(mem: ^GB_Memory, byte: u8) {
	// Source:      $XX00-$XX9F   ;XX = $00 to $DF
	// Destination: $FE00-$FE9F

	mem.oam_transfer.active = true
	mem.oam_transfer.src_addr = u16(byte) * 0x100
	mem.oam_transfer.current = 0
}

do_oam_transfer :: proc(gb: ^GB) {
	mem := &gb.mem
	current := u16(mem.oam_transfer.current)
	src_byte := mem_read(mem, mem.oam_transfer.src_addr + current)

	mem_write(gb, 0xFE00 + current, src_byte)
	mem.oam_transfer.current += 1

	if mem.oam_transfer.current == 160 {
		mem.oam_transfer.active = false
	}
}
