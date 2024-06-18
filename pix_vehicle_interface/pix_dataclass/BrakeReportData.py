from dataclasses import dataclass, fields

@dataclass
class BrakeReportData:
    brake_en_state: int = 0
    brake_flt1: int = 0
    brake_flt2: int = 0
    brake_pedal_actual: float = 0.0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCtrlData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

