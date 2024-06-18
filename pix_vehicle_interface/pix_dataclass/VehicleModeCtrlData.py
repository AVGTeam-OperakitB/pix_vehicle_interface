from dataclasses import dataclass, fields
from time import time_ns
from pix_dataclass.data_utils import generate_byte_array


@dataclass
class VehicleModeCtrlData:
    steer_mode_ctrl: int = 0
    drive_mode_ctrl: int = 0
    turn_light_ctrl: int = 0
    checksum: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'VehicleModeCtrlData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def add_cycle_count(self):

        if self.checksum >= 255:
            self.checksum = 0

        self.checksum += 1

    def get_bytearray(self):
        steer_mode_ctrl = (self.steer_mode_ctrl, 0, 2)
        drive_mode_ctrl = (self.drive_mode_ctrl, 8, 10)
        turn_light_ctrl = (self.turn_light_ctrl, 16, 17)
        checksum = (self.checksum, 56, 63)

        return generate_byte_array(8,
                                   steer_mode_ctrl,
                                   drive_mode_ctrl,
                                   turn_light_ctrl,
                                   checksum,
                                   xor_checksum=False)
