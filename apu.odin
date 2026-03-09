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
Audio_Buffer_Size :: Audio_Chunk_Size * 16

APU :: struct {
	enabled:             bool,
	channel2:            APU_Channel2,
	dacs:                [4]APU_DAC,
	audio_stream:        rl.AudioStream,
	env_timer:           u8,
	left_master_volume:  f32, // 0 - 1
	right_master_volume: f32, // 0 - 1
}

audio_buffer: [Audio_Buffer_Size][2]i16
audio_buffer_write_index: int
audio_buffer_read_index: int

APU_Channel2 :: struct {
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
	apu_dots:           int,
	sample_tick:        int,
	tone_freq:          int,
	current_frame:      int,
	acc_frame_value:    f32,
	current_wave_value: f32,
	env_sweep_timer:    u8,
	trigger_time:       f64,
	left_panning:       f32, // 0 / 1
	right_panning:      f32, // 0 / 1
}

APU_DAC :: struct {
	enabled: bool,
}

audio_callback :: proc(buffer_ptr: rawptr, frames: c.uint) {
	// context = runtime.default_context()

	buffer := (^i16)(buffer_ptr)
	available_frames := 0
	write_index := sync.atomic_load(&audio_buffer_write_index)

	if write_index < audio_buffer_read_index {
		// write index has wrapped around
		available_frames = (len(audio_buffer) - 1 + write_index) - audio_buffer_read_index
	} else {
		available_frames = write_index - audio_buffer_read_index
	}

	frames := int(frames)

	if available_frames > frames {
		available_frames = frames
	}

	// fmt.printfln(
	// 	"write_index = %d, read_index = %d, requested_frames = %d, available_frames = %d",
	// 	write_index,
	// 	audio_buffer_read_index,
	// 	int(frames),
	// 	available_frames,
	// )

	for i := 0; i < frames; i += 1 {
		left_value: i16 = 0
		right_value: i16 = 0

		if i < available_frames {
			left_value = audio_buffer[audio_buffer_read_index][0]
			right_value = audio_buffer[audio_buffer_read_index][1]
			audio_buffer_read_index = (audio_buffer_read_index + 1) % len(audio_buffer)
		}

		libc.memcpy(intrinsics.ptr_offset(buffer, i * 2), &left_value, 2)
		libc.memcpy(intrinsics.ptr_offset(buffer, i * 2 + 1), &right_value, 2)
	}
}

apu_init :: proc(apu: ^APU) {
	rl.InitAudioDevice()
	rl.SetAudioStreamBufferSizeDefault(Audio_Chunk_Size)
	apu.audio_stream = rl.LoadAudioStream(Audio_Sample_Rate, Audio_Sample_Size, 2)
	rl.SetAudioStreamCallback(apu.audio_stream, rl.AudioCallback(audio_callback))
	rl.PlayAudioStream(apu.audio_stream)

	apu.env_timer = 8
}

apu_deinit :: proc(apu: ^APU) {
	rl.UnloadAudioStream(apu.audio_stream)
}

apu_write_register :: proc(gb: ^GB, reg: GB_Audio_Registers, byte: u8) {
	if reg == .NR52 {
		write_to_master_control(gb, byte)
	} else if reg == .NR51 {
		// log.debugf("Writing to sound panning %w register: 0x%x", reg, byte)

		write_to_panning(gb, byte)
	} else if reg == .NR50 {
		log.debugf("Writing to master volume %w register: 0x%x", reg, byte)

		write_to_master_volume(gb, byte)
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
	apu := &gb.apu

	apu.env_timer -= 1

	if apu.env_timer == 0 {
		apu.env_timer = 8

		// log.debugf("APU DIV: %f", dt)

		channel2_div_tick(gb)
	}

	// if gb.apu.channel2.enabled {
	// 	channel2_div_tick(gb)
	// }
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
		log.debug("APU enabled")
	} else {
		log.debug("APU disabled")
		clear_audio_registers(gb)
		report_channel_status(gb, 1, false)
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

	apu.channel2.right_panning = byte & 0x2 > 0 ? 1 : 0
	apu.channel2.left_panning = byte & 0x20 > 0 ? 1 : 0
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
		channel.init_volume = u8((byte & 0xF0) >> 4)
		channel.env_dir = (byte & 0x8) > 0 ? 1 : 0
		channel.sweep_pace = byte & 0x7

		log.debugf(
			"Channel 2 volume & envelope (v: %x): init volume = %d, env dir = %x, sweep pace = %x",
			byte,
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

		if dac.enabled && (byte & 0x80 > 0) {
			log.debug("Channel 2 triggered")

			channel.trigger_time = rl.GetTime()

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
			channel.acc_frame_value = 0
			channel.current_frame = 0
			channel.sample_tick = 0
			channel.expire_timer = channel.init_expire_timer
			channel.period_timer = 2048 - period_value
			channel.tone_freq = tone_freq
			channel.volume = channel.init_volume
			channel.env_sweep_timer = channel.sweep_pace == 0 ? 8 : channel.sweep_pace
			channel.env_enabled = channel.sweep_pace > 0

			audio_buffer_write_index = 0
			audio_buffer_read_index = 0

			report_channel_status(gb, 1, true)
		}

		channel.expire_enabled = byte & 0x40 > 0

		log.debugf(
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

channel2_tick :: proc(gb: ^GB) {
	apu := &gb.apu
	channel := &gb.apu.channel2

	channel.apu_dots += 1

	// pulse channel are ticked every 4 dots
	// 1048576 Hz
	if channel.apu_dots < 4 {
		return
	}

	channel.apu_dots = 0
	channel.sample_tick += 1
	channel.period_timer -= 1

	if channel.period_timer <= 0 {
		nrx3 := read_audio_register(gb, .NR23)
		nrx4 := read_audio_register(gb, .NR24)
		period_value := read_period_value(gb, nrx3, nrx4)
		tone_freq := compute_tone_freq(period_value)

		if tone_freq != channel.tone_freq {
			log.debugf(
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

	// Sample at 44100 Hz
	if channel.sample_tick >= Pulse_Sample_Ticks {
		frame_value := channel.acc_frame_value / f32(channel.sample_tick)
		left_panning := channel.left_panning * apu.left_master_volume
		right_panning := channel.right_panning * apu.right_master_volume
		left_value := frame_value * left_panning * volume_level
		right_value := frame_value * right_panning * volume_level

		write_to_audio_buffer(&gb.apu, left_value, right_value)
		channel.sample_tick = 0
		channel.acc_frame_value = 0
	}
}

channel2_div_tick :: proc(gb: ^GB) {
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

	if channel.expire_enabled && channel.expire_timer > 0 {
		channel.expire_timer -= 1

		if channel.expire_timer == 0 {
			log.debug("Channel 2 expired")

			channel.enabled = false
			report_channel_status(gb, 1, false)
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
	// frame_value is [-1.0, +1.0]

	audio_buffer[audio_buffer_write_index][0] = i16(left_value * f32(32_767))
	audio_buffer[audio_buffer_write_index][1] = i16(right_value * f32(32_767))
	audio_buffer_write_index = (audio_buffer_write_index + 1) % len(audio_buffer)
}
