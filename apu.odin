package gb_emu

import intrinsics "base:intrinsics"
import "base:runtime"
import "core:c"
import libc "core:c/libc"
import "core:fmt"
import "core:log"
import "core:sync"
import rl "vendor:raylib"

Audio_Sample_Rate :: 44_100
Audio_Sample_Size :: 16
Pulse_Sample_Rate :: 1048576
Pulse_Sample_Ticks :: Pulse_Sample_Rate / Audio_Sample_Rate
Audio_Chunk_Size :: 512
Audio_Buffer_Size :: Audio_Chunk_Size * 8

APU :: struct {
	enabled:      bool,
	channel2:     APU_Channel2,
	dacs:         [4]APU_DAC,
	audio_stream: rl.AudioStream,
}

audio_buffer: [Audio_Buffer_Size]i16
audio_buffer_write_index: int
audio_buffer_read_index: int

APU_Channel2 :: struct {
	enabled:             bool,
	expire_enabled:      bool,
	expire_timer:        u8,
	env_timer:           u8,
	period_div:          u16,
	volume:              uint,
	init_volume:         uint,
	init_expire_timer:   u8,
	wave_duty:           u8,
	env_dir:             u8,
	sweep_pace:          u8,
	div_tick:            u8,
	apu_dots:            int,
	sample_tick:         int,
	tone_freq:           int,
	current_frame:       int,
	current_frame_value: f32,
}

APU_DAC :: struct {
	enabled: bool,
}

audio_callback :: proc(buffer_ptr: rawptr, frames: c.uint) {
	context = runtime.default_context()

	buffer := (^i16)(buffer_ptr)
	available_frames := 0
	write_index := sync.atomic_load(&audio_buffer_write_index)

	if write_index < audio_buffer_read_index {
		// write index has wrapped around
		available_frames = (len(audio_buffer) - 1 + write_index) - audio_buffer_read_index
	} else {
		available_frames = write_index - audio_buffer_read_index
	}

	if available_frames > int(frames) {
		available_frames = int(frames)
	}

	// fmt.printfln(
	// 	"write_index = %d, read_index = %d, requested_frames = %d, available_frames = %d",
	// 	write_index,
	// 	audio_buffer_read_index,
	// 	int(frames),
	// 	available_frames,
	// )

	for i := 0; i < int(frames); i += 1 {
		frame_value: i16 = 0

		if i < available_frames {
			frame_value = audio_buffer[audio_buffer_read_index]
			audio_buffer_read_index = (audio_buffer_read_index + 1) % len(audio_buffer)
		}

		libc.memcpy(intrinsics.ptr_offset(buffer, i), &frame_value, 2)
	}
}

apu_init :: proc(apu: ^APU) {
	rl.InitAudioDevice()
	rl.SetAudioStreamBufferSizeDefault(Audio_Chunk_Size)
	apu.audio_stream = rl.LoadAudioStream(Audio_Sample_Rate, Audio_Sample_Size, 1)
	rl.SetAudioStreamCallback(apu.audio_stream, rl.AudioCallback(audio_callback))
	rl.PlayAudioStream(apu.audio_stream)
}

apu_deinit :: proc(apu: ^APU) {
	rl.UnloadAudioStream(apu.audio_stream)
}

apu_write_register :: proc(gb: ^GB, reg: GB_Audio_Registers, byte: u8) {
	if reg == .NR52 {
		write_to_master_control(gb, byte)
	} else if reg >= .NR21 && reg <= .NR24 {
		log.debugf("Writing to APU channel 2's %w register: 0x%x", reg, byte)
		write_to_channel2(gb, reg, byte)
	}
}

apu_tick :: proc(gb: ^GB) {
	apu := &gb.apu

	if apu.channel2.enabled {
		channel2_tick(gb)
	}
}

apu_div_timer :: proc(gb: ^GB) {
	if gb.apu.channel2.enabled {
		channel2_div_tick(gb)
	}
}

write_to_master_control :: proc(gb: ^GB, byte: u8) {
	gb.apu.enabled = byte & 0x80 > 0
	nr52 := read_audio_register(gb, GB_Audio_Registers.NR52)

	gb.mem.write(gb, u16(GB_Audio_Registers.NR52), (byte & 0x80) | (nr52 & 0x7F))

	if gb.apu.enabled {
		log.debug("APU enabled")
	} else {
		log.debug("APU disabled")
		clear_audio_registers(gb)
		report_channel_status(gb, 1, false)
		// TODO: do that for all 4 channels
	}
}

clear_audio_registers :: proc(gb: ^GB) {
	for reg in GB_Audio_Registers {
		if reg != .NR52 {
			gb.mem.write(gb, u16(reg), 0)
		}
	}
}

write_to_channel2 :: proc(gb: ^GB, reg: GB_Audio_Registers, byte: u8) {
	channel := &gb.apu.channel2
	dac := &gb.apu.dacs[1]
	apu := &gb.apu

	if reg == .NR21 {
		channel.init_expire_timer = byte & 0x1F
		channel.wave_duty = byte & 0xC0 >> 6

		log.debugf(
			"Channel 2 init expire timer & wave duty: expire timer = %x, wave duty = %x",
			channel.init_expire_timer,
			channel.wave_duty,
		)
	} else if reg == .NR22 {
		channel.init_volume = uint((byte & 0xF0) >> 4)
		channel.env_dir = (byte & 0x8) > 0 ? 1 : 0
		channel.sweep_pace = byte & 0x7

		log.debugf(
			"Channel 2 volume & envelope: init volume = %d, env dir = %x, sweep pace = %x",
			channel.init_volume,
			channel.env_dir,
			channel.sweep_pace,
		)

		dac_enabled := channel.init_volume > 0 || channel.env_dir > 0

		if !dac.enabled && dac_enabled {
			log.debug("DAC 2 enabled")
		} else if dac.enabled && !dac_enabled {
			log.debug("DAC 2 disabled, disabling channel 2")
			channel.enabled = false
			report_channel_status(gb, 1, false)
		}

		dac.enabled = dac_enabled
	} else if reg == .NR23 {
		log.debugf("Channel 2 period low: %x", byte)
	} else if reg == .NR24 {
		log.debugf("Channel 2 period high: %x", byte & 0x07)

		if dac.enabled && !channel.enabled && (byte & 0x80 > 0) {
			log.debug("Channel 2 triggered")

			nrx3 := read_audio_register(gb, .NR23)
			period_value := read_period_value(gb, nrx3, byte)
			tone_freq := compute_tone_freq(period_value)

			if tone_freq != channel.tone_freq {
				log.debugf(
					"Channel 2 tone frequency: %d Hz (period value: 0x%x)",
					tone_freq,
					period_value,
				)
			}

			channel.enabled = true
			channel.apu_dots = 0
			channel.current_frame_value = 0
			channel.current_frame = 0
			channel.sample_tick = 0
			channel.expire_timer = channel.init_expire_timer
			channel.env_timer = 0
			channel.period_div = period_value
			channel.tone_freq = tone_freq
			channel.volume = channel.init_volume

			audio_buffer_write_index = 0
			audio_buffer_read_index = 0

			report_channel_status(gb, 1, true)
		}

		channel.expire_enabled = byte & 0x40 > 0

		log.debugf(
			"Channel 2 control: length enabled = %w, period divider = %x",
			channel.expire_enabled,
			channel.period_div,
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

channel2_tick :: proc(gb: ^GB) {
	channel := &gb.apu.channel2

	channel.apu_dots += 1

	// pulse channel are ticked every 4 dots
	// 1048576 Hz
	if channel.apu_dots < 4 {
		return
	}

	channel.apu_dots = 0
	channel.sample_tick += 1

	// Sample at 44100 Hz
	if channel.sample_tick == Pulse_Sample_Ticks {
		volume_level := f32(channel.volume) / f32(15)
		write_frame_to_audio_buffer(&gb.apu, channel.current_frame_value * volume_level)
		channel.sample_tick = 0
	}

	channel.period_div += 1

	if channel.period_div < 2048 {
		return
	}

	nrx3 := read_audio_register(gb, .NR23)
	nrx4 := read_audio_register(gb, .NR24)
	period_value := read_period_value(gb, nrx3, nrx4)
	tone_freq := compute_tone_freq(period_value)

	if tone_freq != channel.tone_freq {
		log.debugf("Channel 2 tone frequency: %d Hz (period value: 0x%x)", tone_freq, period_value)
	}

	channel.period_div = period_value
	channel.tone_freq = tone_freq
	channel.current_frame = (channel.current_frame + 1) % 8
	channel.current_frame_value =
		get_pulse_wave_value(channel.wave_duty, channel.current_frame) * 0.5
}

channel2_div_tick :: proc(gb: ^GB) {
	channel := &gb.apu.channel2

	channel.div_tick = (channel.div_tick + 1) % 8

	if channel.div_tick == 0 {
		// envelope update
		if channel.sweep_pace > 0 {
			channel.env_timer = (channel.env_timer + 1) % channel.sweep_pace

			if channel.env_timer == 0 {
				if channel.env_dir == 0 {
					channel.volume -= 1
				} else {
					channel.volume += 1
				}
			}
		}
	} else if channel.div_tick == 2 {
		// expire update
		if channel.expire_enabled {
			channel.expire_timer += 1

			if channel.expire_timer >= 64 {
				channel.enabled = false
				report_channel_status(gb, 1, false)
				log.debug("Channel 2 expired")
			}
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

write_frame_to_audio_buffer :: proc(apu: ^APU, frame_value: f32) {
	// frame_value is [-1.0, +1.0]

	audio_buffer[audio_buffer_write_index] = i16(frame_value * f32(32_767))
	audio_buffer_write_index = (audio_buffer_write_index + 1) % len(audio_buffer)
}
