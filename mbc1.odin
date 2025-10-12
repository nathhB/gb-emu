package gb_emu

import "core:fmt"
import "core:log"

// https://gbdev.io/pandocs/MBC1.html

mbc1_init :: proc(mem: ^GB_Memory, rom: []u8, ram_size: int) {
	mem.rom = rom
	mem.write = mbc1_write
	mem.read = mbc1_read
	mem.get_ptr = mbc1_get_ptr
	mem.external_ram = ram_size > 0

	select_rom_bank(mem, 1)
}

mbc1_write :: proc(mem: ^GB_Memory, addr: u16, byte: u8) {
	if addr >= 0x8000 {
		// write to RAM
		mem.data[addr] = byte

		return
	}

	if addr >= 0xC000 && addr <= 0xCFFF {
		log.debug("write to WRAM")
	}

	if addr <= 0x1FFF {
		// log.debugf("Writing to RAM enable register: 0x%x", byte)
		mem.external_ram = byte == 0xA
	} else if addr >= 0x2000 && addr <= 0x3FFF {
		// writing to ROM bank number register
		select_rom_bank(mem, byte)
		log.debugf("selected rom bank: %d (%x)", mem.rom_bank, addr)
	} else if addr >= 0x4000 && addr <= 0x5FFF {
		// TODO: ram bank
	} else if addr >= 0x6000 && addr <= 0x7FFF {
		// TODO: check rom and ram size
		// banking_mode := byte & 0x1
		//
		// log.debug("MBC1, set banking mode: %d", banking_mode)
	} else {
		panic(fmt.aprintf("Attempted to write into ROM at addr 0x%x (value: 0x%x)", addr, byte))
	}
}

select_rom_bank :: proc(mem: ^GB_Memory, byte: u8) {
	// only consider the 5 lower bits
	mem.rom_bank = byte == 0 ? 1 : int(byte & 0b00011111)
	bank_start := u32(mem.rom_bank * 0x4000)
	bank_end := bank_start + 0x4000

	copy(mem.data[0x4000:], mem.rom[bank_start:bank_end])
}

mbc1_read :: proc(mem: ^GB_Memory, addr: u16) -> u8 {
	if addr <= 0x7FFF {
		// read from ROM bank
		return mem.data[addr]
	} else if addr >= 0xA000 && addr <= 0xBFFF {
		if mem.external_ram {
			// TODO: implement RAM banking
			panic("unsupported")
		} else {
			return 0xFF
		}
	}

	return mem.data[addr]
}

mbc1_get_ptr :: proc(mem: ^GB_Memory, addr: u16) -> ^u8 {
	if addr < 8000 {
		panic("Tried to get a pointer to ROM")
	}

	return &mem.data[addr]
}
