package gb_emu

import "core:fmt"
import "core:log"

// https://gbdev.io/pandocs/MBC1.html

mbc1_init :: proc(mem: ^GB_Memory) {
	mem.write = mbc1_write
	mem.read = mbc1_read

	select_rom_bank(mem, 1)
	select_ram_bank(mem, 0)
}

mbc1_write :: proc(gb: ^GB, addr: u16, byte: u8) {
	if addr >= 0x8000 {
		// write to RAM
		gb.mem.data[addr] = byte

		return
	}

	if addr <= 0x1FFF {
		// enabled := byte == 0x0A
		//
		// if !gb.mem.external_ram && enabled {
		// 	log.debug("Enabled external RAM")
		// }
		//
		// if gb.mem.external_ram && !enabled {
		// 	log.debug("Disabed external RAM")
		// }
		//
		// gb.mem.external_ram = enabled
	} else if addr >= 0x2000 && addr <= 0x3FFF {
		// writing to ROM bank number register
		select_rom_bank(&gb.mem, byte)
	} else if addr >= 0x4000 && addr <= 0x5FFF {
		select_ram_bank(&gb.mem, byte)
	} else if addr >= 0x6000 && addr <= 0x7FFF {
		fmt.panicf("Selecting banking mode unsupported: %x (addr: %x)", byte, addr)
		// TODO: check rom and ram size
		// banking_mode := byte & 0x1
		// https://gbdev.io/pandocs/MBC1.html#60007fff--banking-mode-select-write-only
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
	// log.debugf("Selected ROM bank: %d", mem.rom_bank)
}

select_ram_bank :: proc(mem: ^GB_Memory, byte: u8) {
	mem.ext_ram_bank = int(byte & 0x3)
	log.debugf("Selected external RAM bank: %d", mem.ext_ram_bank)
}

mbc1_read :: proc(mem: ^GB_Memory, addr: u16) -> u8 {
	return mem.data[addr]
}
