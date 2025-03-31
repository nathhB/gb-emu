package gb_emu

import "core:log"

main :: proc() {
    context.logger = log.create_console_logger()
    gb := GB{}

    gb_init(&gb)
    gb_run(&gb)
}
