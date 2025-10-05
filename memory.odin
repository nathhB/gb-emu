package gb_emu

import "core:log"

GB_Memory :: struct {
	data:         [0xFFFF + 1]u8,
	rom:          []u8,
	rom_bank:     int,
	read:         proc(mem: ^GB_Memory, addr: u16) -> u8,
	write:        proc(mem: ^GB_Memory, addr: u16, byte: u8),
	get_ptr:      proc(mem: ^GB_Memory, addr: u16) -> ^u8,
	oam_transfer: struct {
		active:   bool,
		src_addr: u16,
		current:  u8,
	},
}

mem_write :: proc(mem: ^GB_Memory, addr: u16, byte: u8) {
	if addr >= 0xFF00 {
		write_to_hardware_register(mem, addr, byte)

		return
	}

	mem.write(mem, addr, byte)
}

mem_read :: proc(mem: ^GB_Memory, addr: u16) -> u8 {
	return mem.read(mem, addr)
}

mem_get_ptr :: proc(mem: ^GB_Memory, addr: u16) -> ^u8 {
	return mem.get_ptr(mem, addr)
}

mem_tick :: proc(mem: ^GB_Memory) {
	if mem.oam_transfer.active {
		do_oam_transfer(mem)
	}
}

// https://gbdev.io/pandocs/Hardware_Reg_List.html
write_to_hardware_register :: proc(mem: ^GB_Memory, addr: u16, byte: u8) {
	reg := GB_HardRegister(addr)

	if reg == GB_HardRegister.JOYPAD {
		write_to_joypad(mem, byte)
	} else if reg == GB_HardRegister.DIV {
		write_to_div(mem)
	} else if reg == GB_HardRegister.DMA {
		start_dma_transfer(mem, byte)
	} else {
		mem.write(mem, addr, byte)
	}
}

write_to_joypad :: proc(mem: ^GB_Memory, byte: u8) {
	reg := mem_read(mem, u16(GB_HardRegister.JOYPAD))

	reg = (reg & 0x0F) | (0xC0 | (byte & 0xF0))

	mem.write(mem, u16(GB_HardRegister.JOYPAD), reg)
}

write_to_div :: proc(mem: ^GB_Memory) {
	mem.write(mem, u16(GB_HardRegister.DIV), 0)
}

write_to_tac :: proc(mem: ^GB_Memory, byte: u8) {
	clock_type := byte & 3
	clock_enabled := (byte & 4) > 0
}

// https://gbdev.io/pandocs/OAM_DMA_Transfer.html#ff46--dma-oam-dma-source-address--start
start_dma_transfer :: proc(mem: ^GB_Memory, byte: u8) {
	// Source:      $XX00-$XX9F   ;XX = $00 to $DF
	// Destination: $FE00-$FE9F

	mem.oam_transfer.active = true
	mem.oam_transfer.src_addr = u16(byte) * 0x100
	mem.oam_transfer.current = 0
}

do_oam_transfer :: proc(mem: ^GB_Memory) {
	current := u16(mem.oam_transfer.current)
	src_byte := mem_read(mem, mem.oam_transfer.src_addr + current)

	mem_write(mem, 0xFE00 + current, src_byte)
	mem.oam_transfer.current += 1

	if mem.oam_transfer.current == 160 {
		mem.oam_transfer.active = false
	}
}
