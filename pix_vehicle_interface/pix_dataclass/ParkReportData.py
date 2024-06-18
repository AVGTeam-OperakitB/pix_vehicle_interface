from dataclasses import dataclass, fields
from time import time_ns
from pix_dataclass.data_utils import generate_byte_array


@dataclass
class ParkReportData:
    parking_actual: int = 0
    park_flt: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCtrlData' object has no attribute '{field_name}'")
        return getattr(self, field_name)