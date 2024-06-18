from dataclasses import dataclass, fields
from time import time_ns
from pix_dataclass.data_utils import generate_byte_array


@dataclass
class ThrottleCtrlData:
    throttle_en_ctrl: int = 0
    throttle_acc: int = 0
    throttle_pedal_target: int = 0
    vel_target: int = 0
    checksum: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCtrlData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def add_cycle_count(self):

        if self.checksum >= 255:
            self.checksum = 0

        self.checksum += 1

    def get_bytearray(self):

        throttle_en_ctrl = (self.throttle_en_ctrl, 0, 0)
        throttle_acc_upper = ((self.throttle_acc >> 8) & 0xFF, 8, 15)
        throttle_acc_lower = (self.throttle_acc & 0b00000011, 22, 23)
        throttle_pedal_target_upper = ((self.throttle_pedal_target >> 8) & 0xFF, 24, 31)
        throttle_pedal_target_lower = (self.throttle_pedal_target & 0xFF, 32, 39)
        vel_target_upper = ((self.vel_target >> 8) & 0xFF, 40, 47)
        vel_target_lower = (self.vel_target & 0b00000011, 54, 55)
        checksum = (self.checksum, 56, 63)

        return generate_byte_array(8,
                                   throttle_en_ctrl,
                                   throttle_acc_upper,
                                   throttle_acc_lower,
                                   throttle_pedal_target_upper,
                                   throttle_pedal_target_lower,
                                   vel_target_upper,
                                   vel_target_lower,
                                   checksum,
                                   xor_checksum=False)
