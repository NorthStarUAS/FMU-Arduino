// pid.h - a standard pid controller
//
// Written by Curtis Olson, started January 2004.
//
// Copyright (C) 2004-2017  Curtis L. Olson  - curtolson@flightgear.org
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//

#pragma once

#include <string>
using std::string;

#include "component.h"

class ap_pid_t : public ap_component_t {

private:

    bool do_reset;

    double y_n;         // measured process value (input)
    double y_n_1;		// previous process value (input)
    double iterm;		// integral summer

public:

    ap_pid_t( string config_path );
    ~ap_pid_t() {}

    void reset() override;
    void update( double dt ) override;
};