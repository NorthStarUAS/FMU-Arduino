# Task: semi-smart 'do nothing' function.  On the ground, shutdown
# motor and center controls.  If this tasks is initiated while we are
# flying, it means the mission manager ran out of things to do and has
# requested a circle hold over the current position.

from PropertyTree import PropertyNode

import comms.events
import mission.mission_mgr
from mission.task.task import Task
from mission.task import fcsmode
import mission.task.state

class Idle(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.airdata_node = PropertyNode("/sensors/airdata")
        self.ap_node = PropertyNode("/autopilot")
        self.engine_node = PropertyNode("/controls/engine")
        self.name = config_node.getString("name")

    def activate(self):
        self.active = True

        # save existing state
        mission.task.state.save(modes=True)

        # if not in the air, set a simple flight control mode that
        # does not touch the throttle, and set throttle to idle.
        if not self.airdata_node.getBool("is_airborne"):
            fcsmode.set("basic")
            self.engine_node.setDouble("throttle", 0.0)
            
        comms.events.log("mission", "idle")

    def update(self, dt):
        if not self.active:
            return False

        # If we are in the air, push a circle hold on the front of the
        # seq task queue
        if self.airdata_node.getBool("is_airborne"):
            # request a circle hold if we are in the air
            mission.mission_mgr.m.request_task_circle()

    def is_complete(self):
        return False

    def close(self):
        # restore the previous state
        mission.task.state.restore()

        self.active = False
        return True
