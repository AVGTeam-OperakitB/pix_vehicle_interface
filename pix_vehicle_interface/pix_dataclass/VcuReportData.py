from dataclasses import dataclass, fields

@dataclass
class VcuReportData:
    acc: float = 0.0
    brake_light_actual: int = 0
    steer_mode_sts: int = 0
    speed: float = 0
    drive_mode_sts: int = 0
    vehicle_mode_sts: int = 0
    back_crash_state: int = 0
    front_crash_state: int = 0
    aeb_state: int = 0
    chassis_errcode: int = 0
    turn_light_actual: int = 0
    hazard_light_actual: int = 0


    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCtrlData' object has no attribute '{field_name}'")
        return getattr(self, field_name)
