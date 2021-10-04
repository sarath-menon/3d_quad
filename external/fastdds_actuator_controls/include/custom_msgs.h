#pragma once

namespace motor_command_sub {

int main();

static uint32_t index;
static float motor_commands[4];
static bool new_data = false;

} // namespace motor_command_sub