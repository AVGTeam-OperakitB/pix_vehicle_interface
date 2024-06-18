from dataclasses import dataclass, fields


@dataclass
class SteerReportData:
    steer_en_state: int = 0
    steer_flt1: int = 0
    steer_flt2: int = 0
    steer_angle_actual: float = 0.0
    steer_angle_spd_actual: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'SteerReportData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

