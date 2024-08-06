// control_mgr.cpp - high level control/autopilot interface
//
// Written by Curtis Olson, started January 2006.
//
// Copyright (C) 2006  Curtis L. Olson  - http://www.flightgear.org/~curt
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

#include <stdio.h>

#include "../nodes.h"

#include "tecs.h"
#include "fcs_mgr.h"

void fcs_mgr_t::init() {
    // initialize and build the autopilot controller from the property
    // tree config (/config/autopilot)
    ap.init();
    eff.init();
    set_mode("basic");

    printf("Autopilot initialized\n");
}

// send a reset signal to all ap modules that support it.  This gives each
// component a chance to update it's state to reset for current conditions,
// eliminate transients, etc.
void fcs_mgr_t::reset() {
    // FIXME: events->log("controls", "global reset called");
    ap.reset();
}

void fcs_mgr_t::update(float dt) {
    // sanity check
    if ( dt > 1.0 ) { dt = 0.01; }
    if ( dt < 0.00001 ) { dt = 0.01; }

    // call for a global fcs component reset when activating master switch
    bool master_switch = inceptors_node.getBool("master_switch");
    if ( master_switch and !last_master_switch ) {
        // reset the ap; transient mitigation
        reset();
        printf("ap enabled\n");
    } else if ( !master_switch and last_master_switch ) {
        printf("ap disabled (manaul flight)\n");
    }
    last_master_switch = master_switch;

    // update tecs (total energy) values and error metrics
    update_tecs();

    // update the autopilot stages (even in manual flight mode.)  This
    // keeps the differential value up to date, tracks manual inputs,
    // and keeps more continuity in the flight when the mode is
    // switched to autopilot.
    ap.update( dt );

    // copy pilot inputs to flight control outputs when not in
    // autopilot mode
    if ( !master_switch ) {
        eff.write(inceptors_node);
    } else {
        eff.write(outputs_node);
    }
}

string fcs_mgr_t::get_mode() {
    return fcs_node.getString("mode");
}

// manage detailed enable switches from high level mode
void fcs_mgr_t::set_mode( string fcs_mode ) {
    fcs_node.setString("mode", fcs_mode);
    // fixme: comms.events.log("control", "mode change: " + fcs_mode)
    if ( fcs_mode == "basic" ) {
        // set lock modes for "basic" inner loops only
        locks_node.setBool( "roll", true );
        locks_node.setBool( "yaw", true );
        locks_node.setBool( "pitch", true );
        locks_node.setBool( "tecs", false );
    } else if ( fcs_mode == "roll" ) {
        // set lock modes for roll only
        locks_node.setBool( "roll", true );
        locks_node.setBool( "yaw", false );
        locks_node.setBool( "pitch", false );
        locks_node.setBool( "tecs", false );
    } else if ( fcs_mode == "roll+pitch" ) {
        // set lock modes for roll and pitch
        locks_node.setBool( "roll", true );
        locks_node.setBool( "yaw", false );
        locks_node.setBool( "pitch", true );
        locks_node.setBool( "tecs", false );
    } else if ( fcs_mode == "basic+tecs" ) {
        // set lock modes for "basic" + alt hold + speed hold
        locks_node.setBool( "roll", true );
        locks_node.setBool( "yaw", true );
        locks_node.setBool( "pitch", true );
        locks_node.setBool( "tecs", true );
    } else {
        // fixme: comms.events.log("control", "unknown fcs mode attempted: " + fcs_mode)
    }
}

// global shared instance
fcs_mgr_t *fcs_mgr = nullptr;