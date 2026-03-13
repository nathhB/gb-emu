package gb_emu

import "core:log"
import rl "vendor:raylib"

VRAM_Bank_Size :: 0x2000 + 1
VRAM_Size :: VRAM_Bank_Size * 2
WRAM_Bank_Size :: 0xFFF + 1
WRAM_Size :: WRAM_Bank_Size * 7

GB_Memory :: struct {
	vram_banking:  bool, // CGB
	wram_banking:  bool, // CGB
	data:          [0xFFFF + 1]u8,
	rom:           []u8,
	vram:          [VRAM_Size]u8,
	wram:          [WRAM_Size]u8,
	rom_bank:      int,
	ram_bank:      int,
	vram_bank:     int,
	wram_bank:     int,
	external_ram:  bool,
	read:          proc(mem: ^GB_Memory, addr: u16) -> u8,
	write:         proc(gb: ^GB, addr: u16, byte: u8),
	get_ptr:       proc(mem: ^GB_Memory, addr: u16) -> ^u8,
	oam_transfer:  struct {
		active:   bool,
		src_addr: u16,
		current:  u8,
	},
	hdma_transfer: struct {
		active:  bool,
		mode:    HDMA_Mode,
		length:  int,
		current: int,
	},
}

HDMA_Mode :: enum {
	General_Purpose,
	HBlank,
}

mem_write :: proc(gb: ^GB, addr: u16, byte: u8) {
	if addr >= 0x8000 && addr <= 0x9FFF {
		if gb.mem.vram_banking {
			vram_addr := get_vram_addr(&gb.mem, addr)

			gb.mem.vram[vram_addr] = byte
		} else {
			gb.mem.write(gb, addr, byte)
		}
	} else if addr >= 0xD000 && addr <= 0xDFFF {
		if gb.mem.wram_banking {
			wram_addr := get_wram_addr(&gb.mem, addr)

			gb.mem.wram[wram_addr] = byte
		} else {
			gb.mem.write(gb, addr, byte)
		}
	} else if addr >= 0xFF00 {
		write_to_hardware_register(gb, addr, byte)
	} else {
		gb.mem.write(gb, addr, byte)
	}
}

mem_read :: proc(mem: ^GB_Memory, addr: u16) -> u8 {
	if addr == u16(GB_HardRegister.VBK) {
		// pandocs: Reading from this register will return the number of the
		// currently loaded VRAM bank in bit 0, and all other bits will be set to 1

		return mem.vram_bank == 1 ? 0xFF : 0xFE
	} else if addr >= 0x8000 && addr <= 0x9FFF {
		if mem.vram_banking {
			vram_addr := get_vram_addr(mem, addr)

			return mem.vram[vram_addr]
		}
	} else if addr >= 0xD000 && addr <= 0xDFFF {
		if mem.wram_banking {
			wram_addr := get_wram_addr(mem, addr)

			return mem.wram[wram_addr]
		}
	}

	return mem.read(mem, addr)
}

mem_get_ptr :: proc(mem: ^GB_Memory, addr: u16) -> ^u8 {
	return mem.get_ptr(mem, addr)
}

mem_tick :: proc(gb: ^GB) {
	if gb.mem.oam_transfer.active {
		do_oam_transfer(gb)
	}
}

get_vram_addr :: proc(mem: ^GB_Memory, addr: u16) -> u16 {
	bank_offset := u16(mem.vram_bank * VRAM_Bank_Size)

	return bank_offset + (addr - 0x8000)
}

get_wram_addr :: proc(mem: ^GB_Memory, addr: u16) -> u16 {
	assert(mem.wram_bank > 0)

	bank := mem.wram_bank - 1
	bank_offset := u16(bank * WRAM_Bank_Size)

	return bank_offset + (addr - 0xD000)
}

// https://gbdev.io/pandocs/Hardware_Reg_List.html
write_to_hardware_register :: proc(gb: ^GB, addr: u16, byte: u8) {
	if is_audio_register(addr) {
		apu_write_register(gb, GB_Audio_Registers(addr), byte)
		return
	}

	reg := GB_HardRegister(addr)

	if reg == .JOYPAD {
		write_to_joypad(gb, byte)
	} else if reg == .DIV {
		write_to_div(gb)
	} else if reg == .DMA {
		start_dma_transfer(&gb.mem, byte)
	} else if reg == .STAT {
		gb.mem.write(gb, addr, byte & 0x78)
	} else if reg == .VBK {
		write_to_vbk(&gb.mem, byte)
	} else if reg == .WBK {
		write_to_wbk(&gb.mem, byte)
	} else if reg == .HDMA5 {
		start_hdma_transfer(&gb.mem, byte)
	} else {
		gb.mem.write(gb, addr, byte)
	}
}

is_audio_register :: proc(addr: u16) -> bool {
	if addr >= u16(GB_Audio_Registers.NR10) && addr <= u16(GB_Audio_Registers.NR14) {
		return true
	}

	if addr >= u16(GB_Audio_Registers.NR21) && addr <= u16(GB_Audio_Registers.NR24) {
		return true
	}

	if addr >= u16(GB_Audio_Registers.NR30) && addr <= u16(GB_Audio_Registers.NR34) {
		return true
	}

	if addr >= u16(GB_Audio_Registers.NR41) && addr <= u16(GB_Audio_Registers.NR44) {
		return true
	}

	if addr >= u16(GB_Audio_Registers.NR50) && addr <= u16(GB_Audio_Registers.NR52) {
		return true
	}

	return false
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

write_to_div :: proc(gb: ^GB) {
	div := mem_read(&gb.mem, u16(GB_HardRegister.DIV))

	// https://gbdev.io/pandocs/Audio_details.html#div-apu
	// resetting DIV while bit 4 is set triggers DIV-APU
	if (div & 0x10) > 0 {
		panic("disable for now")
		// apu_div_timer(gb)
	}

	gb.mem.write(gb, u16(GB_HardRegister.DIV), 0) // writing to DIV resets it
}

write_to_vbk :: proc(mem: ^GB_Memory, byte: u8) {
	mem.vram_bank = int(byte & 0x1)
}

write_to_wbk :: proc(mem: ^GB_Memory, byte: u8) {
	mem.wram_bank = max(1, int(byte & 0b00000111))
}

// https://gbdev.io/pandocs/OAM_DMA_Transfer.html#ff46--dma-oam-dma-source-address--start
start_dma_transfer :: proc(mem: ^GB_Memory, byte: u8) {
	// Source:      $XX00-$XX9F   ;XX = $00 to $DF
	// Destination: $FE00-$FE9F

	mem.oam_transfer.active = true
	mem.oam_transfer.src_addr = u16(byte) * 0x100
	mem.oam_transfer.current = 0
}

// https://gbdev.io/pandocs/CGB_Registers.html#ff55--hdma5-cgb-mode-only-vram-dma-lengthmodestart
start_hdma_transfer :: proc(mem: ^GB_Memory, byte: u8) {
	mode := byte & 0x80 > 0 ? HDMA_Mode.HBlank : HDMA_Mode.General_Purpose
	length := (int(byte & 0x7F) + 1) * 0x10

	mem.hdma_transfer.active = true
	mem.hdma_transfer.mode = mode
	mem.hdma_transfer.length = length
	mem.hdma_transfer.current = 0
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
