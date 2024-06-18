from dataclasses import dataclass, fields
from time import time_ns
from pix_dataclass.data_utils import generate_byte_array


@dataclass
class SteerCtrlData:
    steer_en_ctrl: int = 0
    steer_angle_spd: int = 0
    steer_angle_target: int = 0
    checksum: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'SteerCtrlData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def add_cycle_count(self):

        if self.checksum >= 255:
            self.checksum = 0

        self.checksum += 1

    def get_bytearray(self):
        steer_en_ctrl = (self.steer_en_ctrl, 0, 0)
        steer_angle_spd = (self.steer_angle_spd, 8, 15)
        steer_angle_target_upper = ((self.steer_angle_target >> 8) & 0xFF, 24, 31)
        steer_angle_target_lower = (self.steer_angle_target & 0xFF, 32, 39)
        checksum = (self.checksum, 56, 63)

        return generate_byte_array(8,
                                   steer_en_ctrl,
                                   steer_angle_spd,
                                   steer_angle_target_upper,
                                   steer_angle_target_lower,
                                   checksum,
                                   xor_checksum=False)
