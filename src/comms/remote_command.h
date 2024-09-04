#pragma once

#include <string>
using std::string;

#include "serial_link.h"

int execute_command(ns_message::command_v1_t *msg, SerialLink *serial);
