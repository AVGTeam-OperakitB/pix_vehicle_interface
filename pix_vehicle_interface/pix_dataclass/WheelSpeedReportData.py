from dataclasses import dataclass, fields



@dataclass
class WheelSpeedReportData:
    front_left: float = 0.0
    front_right: float = 0.0
    rear_left: float = 0.0
    rear_right: float = 0.0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCtrlData' object has no attribute '{field_name}'")
        return getattr(self, field_name)
