// pid.cpp - a standard pid controller
//
// Written by Curtis Olson, started January 2004.
//
// Copyright (C) 2004-2025 Curtis L. Olson - curtolson@flightgear.org
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

#define _USE_MATH_DEFINES  // make msvc happy
#include <math.h>

#include "../props2.h"

#include "pid.h"

ap_pid_t::ap_pid_t( string config_path ):
    do_reset(true),
    y_n( 0.0 ),
    y_n_1( 0.0 ),
    iterm( 0.0 )
{
    size_t pos;

    component_node = PropertyNode( config_path );

    // enable
    string enable_prop = component_node.getString("enable");
    pos = enable_prop.rfind("/");
    if ( pos != string::npos ) {
        string path = enable_prop.substr(0, pos);
        enable_node = PropertyNode(path);
        enable_attr = enable_prop.substr(pos+1);
        printf("enable: %s / %s\n", path.c_str(), enable_attr.c_str());
    }

    // input
    string input_prop = component_node.getString("input");
    pos = input_prop.rfind("/");
    if ( pos != string::npos ) {
        string path = input_prop.substr(0, pos);
        input_attr = input_prop.substr(pos+1);
        input_node = PropertyNode( path );
        printf("input: %s / %s\n", path.c_str(), input_attr.c_str());
    }

    // reference
    string ref_prop = component_node.getString("reference");
    pos = ref_prop.rfind("/");
    if ( pos != string::npos ) {
        string path = ref_prop.substr(0, pos);
        ref_attr = ref_prop.substr(pos+1);
        ref_node = PropertyNode( path );
        printf("reference: %s / %s\n", path.c_str(), ref_attr.c_str());
    }

    // output
    string output_prop = component_node.getString("output");
    pos = output_prop.rfind("/");
    if ( pos != string::npos ) {
        string path = output_prop.substr(0, pos);
        output_attr = output_prop.substr(pos+1);
        output_node = PropertyNode( path );
        printf("output: %s / %s\n", path.c_str(), output_attr.c_str());
    }

    // config
    config_node = component_node.getChild( "config" );
}

void ap_pid_t::reset() {
    do_reset = true;
}

void ap_pid_t::update( double dt ) {
    enabled = enable_node.getBool(enable_attr.c_str());

    bool debug = component_node.getBool("debug");
    if ( debug ) printf("Updating %s\n", get_name().c_str());
    y_n = input_node.getDouble(input_attr.c_str());
    double r_n = ref_node.getDouble(ref_attr.c_str());

    double error = r_n - y_n;

    string wrap = component_node.getString("wrap");
    if ( wrap == "180" ) {
        // wrap error (by +/- 360 degrees to put the result in [-180, 180]
        if ( error < -180 ) { error += 360; }
        if ( error > 180 ) { error -= 360; }
    } else if ( wrap == "pi" ) {
        // wrap error (by +/- 2*pi degrees to put the result in [-pi, pi]
        if ( error < -M_PI ) { error += 2*M_PI; }
        if ( error > M_PI ) { error -= 2*M_PI; }
    }

    if ( debug ) printf("input = %.3f reference = %.3f error = %.3f\n",
                        y_n, r_n, error);

    double u_trim = config_node.getDouble("u_trim");
    double u_min = config_node.getDouble("u_min");
    double u_max = config_node.getDouble("u_max");

    double Kp = config_node.getDouble("Kp");
    double Ti = config_node.getDouble("Ti");
    double Td = config_node.getDouble("Td");
    double Ki = 0.0;
    if ( Ti > 0.0001 ) {
        Ki = Kp / Ti;
    }
    double Kd = Kp * Td;

    // proportional term
    double pterm = Kp * error + u_trim;

    // integral term
    if ( Ti > 0.0001 ) {
        iterm += Ki * error * dt;
    } else {
        iterm = 0.0;
    }

    // if the reset flag is set, back compute an iterm that will
    // produce zero initial transient (overwriting the existing
    // iterm) then unset the do_reset flag.
    if ( do_reset ) {
        if ( Ti > 0.0001 ) {
            double u_n = output_node.getDouble(output_attr.c_str());
            // and clip
            if ( u_n < u_min ) { u_n = u_min; }
            if ( u_n > u_max ) { u_n = u_max; }
            iterm = u_n - pterm;
        } else {
            iterm = 0.0;
        }
        do_reset = false;
    }

    // derivative term: observe that dError/dt = -dInput/dt (except
    // when the setpoint changes (which we don't want to react to
    // anyway.)  This approach avoids "derivative kick" when the set
    // point changes.
    double dy = y_n - y_n_1;
    y_n_1 = y_n;
    double dterm = Kd * -dy / dt;

    // anti-windup
    double output = pterm + iterm + dterm;
    if ( output < u_min ) {
        if ( Ti > 0.0001 ) {
            iterm += u_min - output;
        }
        output = u_min;
    }
    if ( output > u_max ) {
        if ( Ti > 0.0001 ) {
            iterm -= output - u_max;
        }
        output = u_max;
    }

    if ( debug ) printf("pterm = %.3f iterm = %.3f\n", pterm, iterm);

    if ( !enabled ) {
        // this will force a reset when component becomes enabled
        do_reset = true;
    } else {
        output_node.setDouble( output_attr.c_str(), output );
    }
}
