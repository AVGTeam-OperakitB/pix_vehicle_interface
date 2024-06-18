from dataclasses import dataclass, fields
from time import time_ns
from pix_dataclass.data_utils import generate_byte_array


@dataclass
class GearCtrlData:
    gear_en_ctrl: int = 0
    gear_target: int = 0
    checksum: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'GearCtrlData' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def add_cycle_count(self):

        if self.checksum >= 255:
            self.checksum = 0

        self.checksum += 1

    def get_bytearray(self):
        gear_en_ctrl = (self.gear_en_ctrl, 0, 0)
        gear_target = (self.gear_target, 8, 10)
        checksum = (self.checksum, 56, 63)
        return generate_byte_array(8,
                                   gear_en_ctrl,
                                   gear_target,
                                   checksum,
                                   xor_checksum=False)
