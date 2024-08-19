#pragma once

#include <string>
using std::string;

#include "serial_link.h"

int execute_command(string command, SerialLink *serial);