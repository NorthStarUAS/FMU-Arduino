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

#include "tecs.h"
#include "control_mgr.h"

void control_mgr_t::init() {
    // initialize the autopilot class and build the structures from the
    // configuration file values

    status_node = PropertyNode( "/status" );
    ap_node = PropertyNode( "/autopilot" );
    pilot_node = PropertyNode( "/sensors/pilot_input" );
    flight_node = PropertyNode( "/controls/flight" );
    engine_node = PropertyNode( "/controls/engine" );
    switches_node = PropertyNode( "/switches" );

    // initialize and build the autopilot controller from the property
    // tree config (/config/autopilot)
    ap.init();

    printf("Autopilot initialized\n");
}

// send a reset signal to all ap modules that support it.  This gives each
// component a chance to update it's state to reset for current conditions,
// eliminate transients, etc.
void control_mgr_t::reset() {
    // FIXME: events->log("controls", "global reset called");
    ap.reset();
}

void control_mgr_t::copy_pilot_inputs() {
    // This function copies the pilot inputs to the flight/engine
    // outputs.  This creates a manual pass through mode.  Consider
    // that manaul pass-through is handled with less latency directly
    // on APM2/BFS/Aura3 hardware if available.
    
    float aileron = pilot_node.getDouble("aileron");
    flight_node.setDouble( "aileron", aileron );

    float elevator = pilot_node.getDouble("elevator");
    flight_node.setDouble( "elevator", elevator );

    float rudder = pilot_node.getDouble("rudder");
    flight_node.setDouble( "rudder", rudder );

    double flaps = pilot_node.getDouble("flaps");
    flight_node.setDouble("flaps", flaps );

    double gear = pilot_node.getDouble("gear");
    flight_node.setDouble("gear", gear );

    double throttle = pilot_node.getDouble("throttle");
    engine_node.setDouble("throttle", throttle );
}

void control_mgr_t::update(float dt) {
    // sanity check
    if ( dt > 1.0 ) { dt = 0.01; }
    if ( dt < 0.00001 ) { dt = 0.01; }
    
    // call for a global fcs component reset when activating ap master
    // switch
    static bool last_master_switch = false;
    bool master_switch = switches_node.getBool("master_switch");
    if ( master_switch != last_master_switch ) {
	if ( switches_node.getBool("master_switch") ) {
            reset();            // reset the ap; transient mitigation
	}
	last_master_switch = switches_node.getBool("master_switch");
    }
    
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
        copy_pilot_inputs();
    }
}
