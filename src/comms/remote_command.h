#pragma once

#include <string>
using std::string;

#include "serial_link.h"

uint16_t last_command_seq_num = 0;

int execute_command(ns_message::command_v1_t *msg, SerialLink *serial);
