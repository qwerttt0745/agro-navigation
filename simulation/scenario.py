"""
scenario.py: Scenario management for navigation system testing
"""
from enum import Enum


class ScenarioType(Enum):
    """Available scenarios"""
    NORMAL = "normal"
    GNSS_LOSS = "gnss_loss"
    EXTENDED_LOSS = "extended_loss"


class Scenario:
    """Scenario configuration and execution"""
    
    def __init__(self, scenario_type: ScenarioType):
        self.scenario_type = scenario_type
        self.is_active = False
        self.start_time = 0.0
    
    def trigger(self, current_time: float):
        """Trigger scenario at given time"""
        self.is_active = True
        self.start_time = current_time
    
    def stop(self):
        """Stop scenario"""
        self.is_active = False
