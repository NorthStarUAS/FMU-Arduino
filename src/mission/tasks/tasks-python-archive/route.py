import math

from PropertyTree import PropertyNode

import comms.events
import control.route
from mission.task.task import Task
from mission.task import fcsmode
import mission.task.state

class Route(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.route_node = PropertyNode("/task/route")
        self.ap_node = PropertyNode("/autopilot")
        self.nav_node = PropertyNode("/navigation")
        self.targets_node = PropertyNode("/autopilot/targets")

        self.alt_agl_ft = 0.0
        self.speed_kt = 30.0

        self.name = config_node.getString("name")
        self.coord_path = config_node.getString("coord_path")
        self.alt_agl_ft = config_node.getDouble("altitude_agl_ft")
        self.speed_kt = config_node.getDouble("speed_kt")

        # load a route if included in config tree
        if control.route.build(config_node):
            control.route.swap()
        else:
            print("Detected an internal inconsistency in the route")
            print(" configuration.  See earlier errors for details.")
            quit()

    def activate(self):
        self.active = True

        # save existing state
        mission.task.state.save(modes=True, targets=True)

        # set modes
        fcsmode.set("basic+tecs")
        self.nav_node.setString("mode", "route")

        if self.alt_agl_ft > 0.1:
            self.targets_node.setDouble("altitude_agl_ft", self.alt_agl_ft)

        self.route_node.setString("follow_mode", "leader");
        self.route_node.setString("start_mode", "first_wpt");
        self.route_node.setString("completion_mode", "loop");

        comms.events.log("mission", "route")

    # build route from a property tree node
    def build(self, config_node):
        self.standby_route = []       # clear standby route
        num = config_node.getLen("wpt")
        for i in range(num):
            child = config_node.getChild("wpt/%d" % i)
            wp = waypoint.Waypoint()
            wp.build(child)
            self.standby_route.append(wp)
        print("loaded %d waypoints" % len(self.standby_route))
        return True

    def update(self, dt):
        if not self.active:
            return False

    def is_complete(self):
        return False

    def close(self):
        # restore the previous state
        mission.task.state.restore()

        self.active = False
        return True
