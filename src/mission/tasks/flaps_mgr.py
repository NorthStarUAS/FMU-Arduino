from PropertyTree import PropertyNode

import comms.events
from mission.task.task import Task

class FlapsMgr(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.flight_node = PropertyNode("/controls/flight")
        self.imu_node = PropertyNode("/sensors/imu/0")
        
        self.name = config_node.getString("name")
        if config_node.hasChild("speed_secs"):
            self.speed_secs = float(config_node.getString("speed_secs"))
            if self.speed_secs < 1.0:
                self.speed_secs = 1.0
        else:
            self.speed_secs = 5

    def activate(self):
        self.active = True
        comms.events.log("flap_mgr", "active")
    
    def update(self, dt):
        if not self.active:
            return False

        target_value = self.flight_node.getDouble("flaps_setpoint")
        current_value = self.flight_node.getDouble("flaps")
        df = target_value - current_value
        max = dt / self.speed_secs
        if df > max: df = max
        if df < -max: df = -max
        current_value += df
        self.flight_node.setDouble("flaps", current_value)

    def is_complete(self):
        return False
    
    def close(self):
        self.active = False
        return True
