package gb_emu

import "core:fmt"
import "core:log"
import rl "vendor:raylib"

Audio_Sample_Rate :: 44_100
Pulse_Sample_Rate :: 1048576
Pulse_Sample_Ticks :: Pulse_Sample_Rate / Audio_Sample_Rate
Audio_Chunk_Size :: 1024
Audio_Buffer_Size :: Audio_Chunk_Size * 8

APU :: struct {
	enabled:             bool,
	channel1:            Square_Channel,
	channel2:            Square_Channel,
	dacs:                [4]APU_DAC,
	audio_stream:        rl.AudioStream,
	ticks:               u8,
	left_master_volume:  f32, // 0 - 1
	right_master_volume: f32, // 0 - 1
	buffer:              Audio_Buffer,
	sample_tick:         int,
	dots:                int,
}

Audio_Frame :: [2]i16

Audio_Buffer :: struct {
	frames:      [Audio_Buffer_Size]Audio_Frame,
	write_index: int,
	read_index:  int,
}

Square_Channel :: struct {
	enabled:            bool,
	expire_enabled:     bool,
	expire_timer:       u8,
	period_timer:       u16,
	volume:             u8,
	init_volume:        u8,
	init_expire_timer:  u8,
	wave_duty:          u8,
	env_dir:            u8,
	env_enabled:        bool,
	sweep_pace:         u8,
	tone_freq:          int,
	current_frame:      int,
	acc_frame_value:    f32,
	current_wave_value: f32,
	env_sweep_timer:    u8,
	left_panning:       f32, // 0 / 1
	right_panning:      f32, // 0 / 1

	// only for channel 1
	period_sweep_pace:  u8,
	period_sweep_dir:   u8,
	period_sweep_step:  u8,
}

APU_DAC :: struct {
	enabled: bool,
}

apu_init :: proc(apu: ^APU) {
	rl.InitAudioDevice()
	rl.SetAudioStreamBufferSizeDefault(Audio_Chunk_Size)
	apu.audio_stream = rl.LoadAudioStream(Audio_Sample_Rate, size_of(i16) * 8, 2)
	rl.PlayAudioStream(apu.audio_stream)

	apu.ticks = 8
}

apu_deinit :: proc(apu: ^APU) {
	rl.UnloadAudioStream(apu.audio_stream)
}

apu_update_audio_stream :: proc(apu: ^APU) {
	if !rl.IsAudioStreamProcessed(apu.audio_stream) {
		return
	}

	buffer := make([]i16, Audio_Chunk_Size * 2)
	defer delete(buffer)
	available_frames := 0

	if apu.buffer.write_index < apu.buffer.read_index {
		// write index has wrapped around
		available_frames =
			(len(apu.buffer.frames) - 1 + apu.buffer.write_index) - apu.buffer.read_index
	} else {
		available_frames = apu.buffer.write_index - apu.buffer.read_index
	}

	if available_frames > Audio_Chunk_Size {
		available_frames = Audio_Chunk_Size
	}

	for i := 0; i < Audio_Chunk_Size; i += 1 {
		left_value: i16 = 0
		right_value: i16 = 0

		if i < available_frames {
			frame := apu.buffer.frames[apu.buffer.read_index]
			left_value = frame[0]
			right_value = frame[1]
			apu.buffer.read_index = (apu.buffer.read_index + 1) % len(apu.buffer.frames)
		}

		buffer[i * 2] = left_value
		buffer[(i * 2) + 1] = right_value
	}

	rl.UpdateAudioStream(apu.audio_stream, raw_data(buffer), Audio_Chunk_Size)
}

apu_write_register :: proc(gb: ^GB, reg: GB_Audio_Registers, byte: u8) {
	if reg == .NR52 {
		write_to_master_control(gb, byte)
	} else if reg == .NR51 {
		// apu_log_debug("Writing to sound panning %w register: 0x%x", reg, byte)

		write_to_panning(gb, byte)
	} else if reg == .NR50 {
		apu_log_debug("Writing to master volume %w register: 0x%x", reg, byte)

		write_to_master_volume(gb, byte)
	} else if reg >= .NR10 && reg <= .NR14 {
		apu_log_debug("Writing to APU channel 1's %w register: 0x%x", reg, byte)
		write_to_channel1(gb, reg, byte)
	} else if reg >= .NR21 && reg <= .NR24 {
		apu_log_debug("Writing to APU channel 2's %w register: 0x%x", reg, byte)
		write_to_channel2(gb, reg, byte)
	}
}

apu_tick :: proc(gb: ^GB) {
	apu := &gb.apu

	apu.dots += 1

	// apu is ticked every 4 dots (1048576 Hz)
	if apu.dots < 4 {
		return
	}

	apu.dots = 0

	if apu.channel1.enabled {
		channel1_tick(gb)
	}

	if apu.channel2.enabled {
		channel2_tick(gb)
	}

	// Sample at 44100 Hz
	if apu.sample_tick >= Pulse_Sample_Ticks {
		left_value: f32
		right_value: f32

		for i := 0; i < 2; i += 1 {
			channel := i == 0 ? &apu.channel1 : &apu.channel2

			frame_value := channel.acc_frame_value / f32(apu.sample_tick)

			left_value += frame_value * channel.left_panning
			right_value += frame_value * channel.right_panning

			channel.acc_frame_value = 0
		}

		// TODO: multiply by 0.5 because there are only two channels for now
		// update this when more channels are implemented
		left_value *= 0.5
		right_value *= 0.5

		left_value *= apu.left_master_volume
		right_value *= apu.right_master_volume

		write_to_audio_buffer(&gb.apu, left_value, right_value)

		apu.sample_tick = 0
	}

	apu.sample_tick += 1
}

apu_div_timer :: proc(gb: ^GB) {
	apu := &gb.apu

	apu.ticks -= 1

	if apu.ticks % 2 == 0 {
		if gb.apu.channel1.enabled {
			channel1_expire_tick(gb)
		}

		if gb.apu.channel2.enabled {
			channel2_expire_tick(gb)
		}
	}

	if apu.ticks % 4 == 0 {
		channel1_period_sweep_tick(gb)
	}

	if apu.ticks % 8 == 0 {
		// envelopes

		if gb.apu.channel1.enabled {
			channel1_env_tick(gb)
		}

		if gb.apu.channel2.enabled {
			channel2_env_tick(gb)
		}
	}

	if apu.ticks == 0 {
		apu.ticks = 8
	}
}

clear_audio_registers :: proc(gb: ^GB) {
	for reg in GB_Audio_Registers {
		if reg != .NR52 {
			gb.mem.write(gb, u16(reg), 0)
		}
	}
}

write_to_master_control :: proc(gb: ^GB, byte: u8) {
	gb.apu.enabled = byte & 0x80 > 0
	nr52 := read_audio_register(gb, GB_Audio_Registers.NR52)

	gb.mem.write(gb, u16(GB_Audio_Registers.NR52), (byte & 0x80) | (nr52 & 0x7F))

	if gb.apu.enabled {
		apu_log_debug("APU enabled")
	} else {
		apu_log_debug("APU disabled")
		clear_audio_registers(gb)
		report_channel_status(gb, 1, false)
		report_channel_status(gb, 2, false)
		// TODO: do that for all 4 channels
	}
}

write_to_master_volume :: proc(gb: ^GB, byte: u8) {
	apu := &gb.apu

	right_volume := byte & 0x7
	left_volume := (byte & 0x70) >> 4

	if right_volume == 0 {
		right_volume = 1
	}

	if left_volume == 0 {
		left_volume = 1
	}

	apu.left_master_volume = f32(left_volume) / 0x7
	apu.right_master_volume = f32(right_volume) / 0x7
}

write_to_panning :: proc(gb: ^GB, byte: u8) {
	apu := &gb.apu

	apu.channel1.right_panning = byte & 0x1 > 0 ? 1 : 0
	apu.channel1.left_panning = byte & 0x10 > 0 ? 1 : 0

	apu.channel2.right_panning = byte & 0x2 > 0 ? 1 : 0
	apu.channel2.left_panning = byte & 0x20 > 0 ? 1 : 0
}

write_to_channel1 :: proc(gb: ^GB, reg: GB_Audio_Registers, byte: u8) {
	channel := &gb.apu.channel1
	dac := &gb.apu.dacs[0]
	apu := &gb.apu

	if reg == .NR11 {
		channel.init_expire_timer = byte & 0x1F
		channel.wave_duty = byte & 0xC0 >> 6

		apu_log_debug(
			"Channel 1 init expire timer & wave duty: expire timer = %x, wave duty = %x",
			channel.init_expire_timer,
			channel.wave_duty,
		)
	} else if reg == .NR12 {
		channel.init_volume = u8((byte & 0xF0) >> 4)
		channel.env_dir = (byte & 0x8) > 0 ? 1 : 0
		channel.sweep_pace = byte & 0x7

		apu_log_debug(
			"Channel 1 volume & envelope (v: %x): init volume = %d, env dir = %x, sweep pace = %x",
			byte,
			channel.init_volume,
			channel.env_dir,
			channel.sweep_pace,
		)

		dac_enabled := channel.init_volume > 0 || channel.env_dir > 0

		if !dac.enabled && dac_enabled {
			apu_log_debug("DAC 1 enabled")
		} else if dac.enabled && !dac_enabled {
			apu_log_debug("DAC 1 disabled, disabling channel 2")
			channel.enabled = false
			channel.acc_frame_value = 0
			report_channel_status(gb, 1, false)
		}

		dac.enabled = dac_enabled
	} else if reg == .NR13 {
		apu_log_debug("Channel 1 period low: %x", byte)
	} else if reg == .NR14 {
		apu_log_debug("Channel 1 period high: %x", byte & 0x07)

		if dac.enabled && (byte & 0x80 > 0) {
			apu_log_debug("Channel 1 triggered")

			nrx3 := read_audio_register(gb, .NR13)
			period_value := read_period_value(gb, nrx3, byte)
			tone_freq := compute_tone_freq(period_value)

			if tone_freq != channel.tone_freq {
				apu_log_debug(
					"Channel 1 tone frequency: %d Hz (period value: 0x%x)",
					tone_freq,
					period_value,
				)
			}

			channel.enabled = true
			channel.acc_frame_value = 0
			channel.current_frame = 0
			channel.expire_timer = channel.init_expire_timer
			channel.period_timer = 2048 - period_value
			channel.tone_freq = tone_freq
			channel.volume = channel.init_volume
			channel.env_sweep_timer = channel.sweep_pace == 0 ? 8 : channel.sweep_pace
			channel.env_enabled = channel.sweep_pace > 0

			read_period_sweep_register(gb, channel)
			report_channel_status(gb, 1, true)
		}

		channel.expire_enabled = byte & 0x40 > 0

		apu_log_debug(
			"Channel 1 control: length enabled = %w, period timer = %x",
			channel.expire_enabled,
			channel.period_timer,
		)
	}

	gb.mem.write(gb, u16(reg), byte)
}

read_period_sweep_register :: proc(gb: ^GB, channel: ^Square_Channel) {
	nr10 := read_audio_register(gb, .NR10)

	channel.period_sweep_pace = (nr10 & 0b01110000) >> 4
	channel.period_sweep_dir = (nr10 & 0b00001000) >> 3
	channel.period_sweep_step = nr10 & 0b00000111

	apu_log_debug(
		"Channel 1 period sweep: pace = %d, dir = %d, step = %d",
		channel.period_sweep_pace,
		channel.period_sweep_dir,
		channel.period_sweep_step,
	)
}

write_to_channel2 :: proc(gb: ^GB, reg: GB_Audio_Registers, byte: u8) {
	channel := &gb.apu.channel2
	dac := &gb.apu.dacs[1]
	apu := &gb.apu

	if reg == .NR21 {
		channel.init_expire_timer = byte & 0x1F
		channel.wave_duty = byte & 0xC0 >> 6

		apu_log_debug(
			"Channel 2 init expire timer & wave duty: expire timer = %x, wave duty = %x",
			channel.init_expire_timer,
			channel.wave_duty,
		)
	} else if reg == .NR22 {
		channel.init_volume = u8((byte & 0xF0) >> 4)
		channel.env_dir = (byte & 0x8) > 0 ? 1 : 0
		channel.sweep_pace = byte & 0x7

		apu_log_debug(
			"Channel 2 volume & envelope (v: %x): init volume = %d, env dir = %x, sweep pace = %x",
			byte,
			channel.init_volume,
			channel.env_dir,
			channel.sweep_pace,
		)

		dac_enabled := channel.init_volume > 0 || channel.env_dir > 0

		if !dac.enabled && dac_enabled {
			apu_log_debug("DAC 2 enabled")
		} else if dac.enabled && !dac_enabled {
			apu_log_debug("DAC 2 disabled, disabling channel 2")
			channel.enabled = false
			channel.acc_frame_value = 0
			report_channel_status(gb, 2, false)
		}

		dac.enabled = dac_enabled
	} else if reg == .NR23 {
		apu_log_debug("Channel 2 period low: %x", byte)
	} else if reg == .NR24 {
		apu_log_debug("Channel 2 period high: %x", byte & 0x07)

		if dac.enabled && (byte & 0x80 > 0) {
			apu_log_debug("Channel 2 triggered")

			nrx3 := read_audio_register(gb, .NR23)
			period_value := read_period_value(gb, nrx3, byte)
			tone_freq := compute_tone_freq(period_value)

			if tone_freq != channel.tone_freq {
				apu_log_debug(
					"Channel 2 tone frequency: %d Hz (period value: 0x%x)",
					tone_freq,
					period_value,
				)
			}

			channel.enabled = true
			channel.acc_frame_value = 0
			channel.current_frame = 0
			channel.expire_timer = channel.init_expire_timer
			channel.period_timer = 2048 - period_value
			channel.tone_freq = tone_freq
			channel.volume = channel.init_volume
			channel.env_sweep_timer = channel.sweep_pace == 0 ? 8 : channel.sweep_pace
			channel.env_enabled = channel.sweep_pace > 0

			report_channel_status(gb, 2, true)
		}

		channel.expire_enabled = byte & 0x40 > 0

		apu_log_debug(
			"Channel 2 control: length enabled = %w, period timer = %x",
			channel.expire_enabled,
			channel.period_timer,
		)
	}

	gb.mem.write(gb, u16(reg), byte)
}

report_channel_status :: proc(gb: ^GB, channel_id: u8, enabled: bool) {
	nr52 := read_audio_register(gb, GB_Audio_Registers.NR52)

	if enabled {
		nr52 |= (1 << channel_id)
	} else {
		nr52 &= ~u8(1 << channel_id)
	}

	gb.mem.write(gb, u16(GB_Audio_Registers.NR52), nr52)
}

channel1_tick :: proc(gb: ^GB) {
	apu := &gb.apu
	channel := &gb.apu.channel1

	channel.period_timer -= 1

	if channel.period_timer <= 0 {
		nrx3 := read_audio_register(gb, .NR13)
		nrx4 := read_audio_register(gb, .NR14)
		period_value := read_period_value(gb, nrx3, nrx4)
		tone_freq := compute_tone_freq(period_value)

		if tone_freq != channel.tone_freq {
			apu_log_debug(
				"Channel 1 tone frequency: %d Hz (period value: 0x%x)",
				tone_freq,
				period_value,
			)
		}

		channel.period_timer = 2048 - period_value
		channel.tone_freq = tone_freq
		channel.current_frame = (channel.current_frame + 1) % 8
		channel.current_wave_value = get_pulse_wave_value(channel.wave_duty, channel.current_frame)
	}

	volume_level := f32(channel.volume) / 0xF
	frame_value := channel.current_wave_value * volume_level

	channel.acc_frame_value += frame_value
}

channel2_tick :: proc(gb: ^GB) {
	apu := &gb.apu
	channel := &gb.apu.channel2

	channel.period_timer -= 1

	if channel.period_timer <= 0 {
		nrx3 := read_audio_register(gb, .NR23)
		nrx4 := read_audio_register(gb, .NR24)
		period_value := read_period_value(gb, nrx3, nrx4)
		tone_freq := compute_tone_freq(period_value)

		if tone_freq != channel.tone_freq {
			apu_log_debug(
				"Channel 2 tone frequency: %d Hz (period value: 0x%x)",
				tone_freq,
				period_value,
			)
		}

		channel.period_timer = 2048 - period_value
		channel.tone_freq = tone_freq
		channel.current_frame = (channel.current_frame + 1) % 8
		channel.current_wave_value = get_pulse_wave_value(channel.wave_duty, channel.current_frame)
	}

	volume_level := f32(channel.volume) / 0xF
	frame_value := channel.current_wave_value * volume_level

	channel.acc_frame_value += frame_value
}

channel1_period_sweep_tick :: proc(gb: ^GB) {
	// TODO:
}

channel1_env_tick :: proc(gb: ^GB) {
	channel := &gb.apu.channel1

	channel.env_sweep_timer -= 1

	if channel.env_sweep_timer == 0 {
		channel.env_sweep_timer = channel.sweep_pace == 0 ? 8 : channel.sweep_pace

		if channel.volume == 0 || channel.volume == 0xF {
			channel.env_enabled = false
		}

		if channel.env_enabled {
			if channel.env_dir == 0 {
				channel.volume -= 1
			} else if channel.env_dir == 1 {
				channel.volume += 1
			}
		}
	}
}

channel1_expire_tick :: proc(gb: ^GB) {
	channel := &gb.apu.channel1

	if channel.expire_enabled {
		channel.expire_timer -= 1

		if channel.expire_timer == 0 {
			apu_log_debug("Channel 1 expired")

			channel.enabled = false
			channel.acc_frame_value = 0
			report_channel_status(gb, 1, false)
		}
	}
}

channel2_env_tick :: proc(gb: ^GB) {
	channel := &gb.apu.channel2

	channel.env_sweep_timer -= 1

	if channel.env_sweep_timer == 0 {
		channel.env_sweep_timer = channel.sweep_pace == 0 ? 8 : channel.sweep_pace

		if channel.volume == 0 || channel.volume == 0xF {
			channel.env_enabled = false
		}

		if channel.env_enabled {
			if channel.env_dir == 0 {
				channel.volume -= 1
			} else if channel.env_dir == 1 {
				channel.volume += 1
			}
		}
	}
}

channel2_expire_tick :: proc(gb: ^GB) {
	channel := &gb.apu.channel2

	if channel.expire_enabled && channel.expire_timer > 0 {
		channel.expire_timer -= 1

		if channel.expire_timer == 0 {
			apu_log_debug("Channel 2 expired")

			channel.enabled = false
			channel.acc_frame_value = 0
			report_channel_status(gb, 2, false)
		}
	}
}

read_period_value :: proc(gb: ^GB, nrx3: byte, nrx4: byte) -> u16 {
	return (u16(nrx4 & 0x07) << 8) | u16(nrx3)
}

compute_tone_freq :: proc(period_value: u16) -> int {
	return 131072 / (2048 - int(period_value))
}

read_audio_register :: proc(gb: ^GB, reg: GB_Audio_Registers) -> u8 {
	return mem_read(&gb.mem, u16(reg))
}

get_pulse_wave_value :: proc(wave_duty: u8, frame: int) -> f32 {
	ensure(frame >= 0 && frame < 8)

	if wave_duty == 0 {
		// 12.5% of time low, 1 frame out of 8
		return frame == 7 ? -1 : 1
	} else if wave_duty == 1 {
		// 25% of time low, 2 frames out of 8
		return frame >= 6 ? -1 : 1
	} else if wave_duty == 2 {
		// 50% of time low, 4 frames out of 8
		return frame >= 3 ? -1 : 1
	} else if wave_duty == 3 {
		// 75% of time low, 6 frames out of 8
		// no audible difference with 25%
		return frame >= 6 ? 1 : -1
	}

	fmt.panicf("unspported wave duty: %d", wave_duty)
}

write_to_audio_buffer :: proc(apu: ^APU, left_value, right_value: f32) {
	frame := Audio_Frame{i16(left_value * f32(32_767)), i16(right_value * f32(32_767))}

	apu.buffer.frames[apu.buffer.write_index] = frame
	apu.buffer.write_index = (apu.buffer.write_index + 1) % len(apu.buffer.frames)
}

@(disabled = true)
apu_log_debug :: proc(fmt_str: string, args: ..any, location := #caller_location) {
	log.debugf(fmt_str, args, location)
}
