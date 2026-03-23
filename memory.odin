package gb_emu

import "core:fmt"
import "core:log"
import "core:os"

VRAM_Bank_Size :: 0x2000
VRAM_Size :: VRAM_Bank_Size * 2
WRAM_Bank_Size :: 0x1000
WRAM_Size :: WRAM_Bank_Size * 7
Ext_RAM_Bank_Size :: 0x2000

GB_Memory :: struct {
	vram_banking:  bool, // CGB
	wram_banking:  bool, // CGB
	data:          [0xFFFF + 1]u8,
	rom:           []u8,
	vram:          [VRAM_Size]u8,
	wram:          [WRAM_Size]u8,
	ext_ram:       []u8,
	bg_palettes:   [64]u8, // CGB: background palettes RAM
	obj_palettes:  [64]u8, // CGB: object palettes RAM
	bgp_addr:      u8, // CGB
	bgp_auto_incr: bool, // CGB
	obp_addr:      u8, // CGB
	obp_auto_incr: bool, // CGB
	rom_bank:      int,
	ext_ram_bank:  int,
	vram_bank:     int,
	wram_bank:     int,
	save_ext_ram:  bool,
	read:          proc(mem: ^GB_Memory, addr: u16) -> u8,
	write:         proc(gb: ^GB, addr: u16, byte: u8),
	oam_transfer:  struct {
		active:   bool,
		src_addr: u16,
		current:  u8,
	},
	hdma_transfer: struct {
		mode:      HDMA_Mode,
		blocks:    u8,
		current:   u8,
		src_addr:  u16,
		dest_addr: u16,
		status:    u8, // returned when register HDMA5 (FF55) is read
	},
}

HDMA_Mode :: enum {
	General_Purpose,
	HBlank,
}

mem_init :: proc(mem: ^GB_Memory, color: bool, external_ram := 0) {
	mem.data[GB_HardRegister.JOYPAD] = 0xFF

	if color {
		mem.vram_banking = true
		mem.wram_banking = true
	}

	mem.hdma_transfer.status = 0xFF

	if external_ram > 0 {
		mem.ext_ram = make([]u8, external_ram)
	}
}

mem_write :: proc(gb: ^GB, addr: u16, byte: u8) {
	if addr >= 0x8000 && addr <= 0x9FFF {
		if gb.mem.vram_banking {
			vram_addr := get_vram_addr(&gb.mem, addr)

			gb.mem.vram[vram_addr] = byte
		} else {
			gb.mem.write(gb, addr, byte)
		}
	} else if addr >= 0xA000 && addr <= 0xBFFF {
		if gb.mem.ext_ram != nil {
			ext_ram_addr := get_ext_ram_addr(&gb.mem, addr)

			gb.mem.ext_ram[ext_ram_addr] = byte
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

mem_read :: proc(mem: ^GB_Memory, addr: u16, override_vram_bank := -1) -> u8 {
	if addr == u16(GB_HardRegister.VBK) {
		// pandocs: Reading from this register will return the number of the
		// currently loaded VRAM bank in bit 0, and all other bits will be set to 1

		return mem.vram_bank == 1 ? 0xFF : 0xFE
	} else if addr == u16(GB_HardRegister.HDMA5) {
		return mem.hdma_transfer.status
	} else if addr >= 0x8000 && addr <= 0x9FFF {
		if mem.vram_banking {
			vram_addr := get_vram_addr(mem, addr, override_vram_bank)

			return mem.vram[vram_addr]
		}
	} else if addr >= 0xA000 && addr <= 0xBFFF {
		if mem.ext_ram != nil {
			ext_ram_addr := get_ext_ram_addr(mem, addr)

			return mem.ext_ram[ext_ram_addr]
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
	if addr < 8000 && (addr < 0x100 || addr > 0x200) {
		fmt.panicf("Tried to get a pointer to ROM: 0x%x", addr)
	}

	if addr >= 0x8000 && addr <= 0x9FFF {
		if mem.vram_banking {
			return &mem.vram[get_vram_addr(mem, addr)]
		}
	} else if addr >= 0xA000 && addr <= 0xBFFF {
		return &mem.ext_ram[get_ext_ram_addr(mem, addr)]
	} else if addr >= 0xD000 && addr <= 0xDFFF {
		if mem.wram_banking {
			return &mem.wram[get_wram_addr(mem, addr)]
		}
	}

	return &mem.data[addr]
}

mem_tick :: proc(gb: ^GB) {
	if gb.mem.oam_transfer.active {
		do_oam_transfer(gb)
	}
}

mem_save_external_ram :: proc(mem: ^GB_Memory) -> GB_Error {
	if mem.ext_ram == nil {
		return .None
	}

	fd, err := os.open("foo.sav", os.O_RDWR | os.O_CREATE | os.O_TRUNC, os.S_IRUSR | os.S_IWUSR)
	defer os.close(fd)

	if err != nil {
		log.errorf("os.open(): %w", err)
		return .Save_Failed
	}

	start_addr := mem.ext_ram_bank * Ext_RAM_Bank_Size
	end_addr := start_addr + Ext_RAM_Bank_Size

	_, err = os.write(fd, mem.ext_ram[start_addr:end_addr])

	if err != nil {
		log.errorf("os.write(): %w", err)
		return .Save_Failed
	}

	return .None
}

mem_load_saved_external_ram :: proc(mem: ^GB_Memory) -> GB_Error {
	if mem.ext_ram == nil {
		return .None
	}

	data, success := os.read_entire_file("foo.sav")

	if !success {
		return .Load_Failed
	}

	// copy saved RAM into all external banks

	banks := len(mem.ext_ram) / Ext_RAM_Bank_Size

	for i := 0; i < banks; i += 1 {
		start := Ext_RAM_Bank_Size * i
		end := start + Ext_RAM_Bank_Size

		copy(mem.ext_ram[start:end], data)
	}

	return .None
}

get_vram_addr :: proc(mem: ^GB_Memory, addr: u16, override_vram_bank := -1) -> u16 {
	bank := override_vram_bank >= 0 ? override_vram_bank : mem.vram_bank
	bank_offset := u16(bank * VRAM_Bank_Size)

	return bank_offset + (addr - 0x8000)
}

get_wram_addr :: proc(mem: ^GB_Memory, addr: u16) -> u16 {
	assert(mem.wram_bank > 0)

	bank := mem.wram_bank - 1
	bank_offset := u16(bank * WRAM_Bank_Size)

	return bank_offset + (addr - 0xD000)
}

get_ext_ram_addr :: proc(mem: ^GB_Memory, addr: u16) -> u16 {
	bank := mem.ext_ram_bank
	bank_offset := u16(bank * Ext_RAM_Bank_Size)

	return bank_offset + (addr - 0xA000)
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
		write_to_hdma_transfer(&gb.mem, byte)
	} else if reg == .BCPS {
		write_to_bcps(&gb.mem, byte)
	} else if reg == .OCPS {
		write_to_ocps(&gb.mem, byte)
	} else if reg == .BCPD {
		write_to_bcpd(&gb.mem, byte)
	} else if reg == .OCPD {
		write_to_ocpd(&gb.mem, byte)
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
write_to_hdma_transfer :: proc(mem: ^GB_Memory, byte: u8) {
	if hdma_transfer_is_active(mem) && (byte & (1 << 7)) == 0 {
		// stop HDMA transfer
		mem.hdma_transfer.status |= 1 << 7
		return
	}

	mode := byte & 0x80 > 0 ? HDMA_Mode.HBlank : HDMA_Mode.General_Purpose
	blocks := (byte & 0x7F) + 1
	src_addr_hi := u16(mem_read(mem, u16(GB_HardRegister.HDMA1)))
	src_addr_lo := u16(mem_read(mem, u16(GB_HardRegister.HDMA2)) & 0xF0)
	dest_addr_hi := u16(mem_read(mem, u16(GB_HardRegister.HDMA3)) & 0b00011111)
	dest_addr_lo := u16(mem_read(mem, u16(GB_HardRegister.HDMA4)) & 0xF0)

	mem.hdma_transfer.src_addr = src_addr_hi << 8 | src_addr_lo
	mem.hdma_transfer.dest_addr = 0x8000 + (dest_addr_hi << 8 | dest_addr_lo)
	mem.hdma_transfer.status = 0
	mem.hdma_transfer.mode = mode
	mem.hdma_transfer.blocks = blocks
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

hdma_copy_one_block :: proc(gb: ^GB) {
	transfer := &gb.mem.hdma_transfer

	ensure(hdma_transfer_is_active(&gb.mem))

	from := transfer.src_addr + u16(transfer.current * 0x10)
	to := transfer.dest_addr + u16(transfer.current * 0x10)

	for i: u16 = 0; i < 0x10; i += 1 {
		mem_write(gb, to + i, mem_read(&gb.mem, from + i))
	}

	transfer.current += 1
	remaining := transfer.blocks - transfer.current
	transfer.status = remaining & 0x7F

	if remaining == 0 {
		transfer.status |= 1 << 7
	}
}

hdma_transfer_is_active :: proc(mem: ^GB_Memory) -> bool {
	return (mem.hdma_transfer.status & (1 << 7)) == 0
}

write_to_bcps :: proc(mem: ^GB_Memory, byte: u8) {
	mem.bgp_auto_incr = (byte & (1 << 7)) > 0
	mem.bgp_addr = byte & 0b00111111
}

write_to_ocps :: proc(mem: ^GB_Memory, byte: u8) {
	mem.obp_auto_incr = (byte & (1 << 7)) > 0
	mem.obp_addr = byte & 0b00111111
}

write_to_bcpd :: proc(mem: ^GB_Memory, byte: u8) {
	mem.bg_palettes[mem.bgp_addr] = byte

	if mem.bgp_auto_incr {
		mem.bgp_addr = (mem.bgp_addr + 1) % 64
	}
}

write_to_ocpd :: proc(mem: ^GB_Memory, byte: u8) {
	mem.obj_palettes[mem.obp_addr] = byte

	if mem.obp_auto_incr {
		mem.obp_addr = (mem.obp_addr + 1) % 64
	}
}
