package gb_emu

// Memory controller using for unit tests

mbc_dummy_init :: proc(mem: ^GB_Memory) {
	mem.write = mbc_dummy_write
	mem.read = mbc_dummy_read
}

mbc_dummy_write :: proc(gb: ^GB, addr: u16, byte: u8) {
	gb.mem.data[addr] = byte
}

mbc_dummy_read :: proc(mem: ^GB_Memory, addr: u16) -> u8 {
	return mem.data[addr]
}
