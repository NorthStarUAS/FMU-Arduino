#pragma once

#include <string>
using std::string;

#include "../nst_messages.h"
#include "serial_link.h"

int execute_command(nst_message::command_v1_t *msg, SerialLink *serial);
