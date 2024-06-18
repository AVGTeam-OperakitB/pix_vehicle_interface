import math

from rclpy.node import Node

from tier4_vehicle_msgs.msg import BatteryStatus
from autoware_auto_vehicle_msgs.msg import (ControlModeReport, GearReport, HazardLightsReport, TurnIndicatorsReport,
                                            SteeringReport, VelocityReport)
from sensor_msgs.msg import Imu
import can
import rclpy
import threading
from struct import unpack

from pix_dataclass.BMSReportData import BMSReportData
from pix_dataclass.BrakeReportData import BrakeReportData
from pix_dataclass.GearReportData import GearReportData
from pix_dataclass.ParkReportData import ParkReportData
from pix_dataclass.SteerReportData import SteerReportData
from pix_dataclass.ThrottleReportData import ThrottleReportData
from pix_dataclass.VcuReportData import VcuReportData
from pix_dataclass.WheelSpeedReportData import WheelSpeedReportData


class CANReceiverNode(Node):
    """
    Node responsible for receiving CAN messages and publishing them as ROS2 messages.

    TODO: Check publisher queue size.
    """

    def __init__(self):
        super().__init__('CANReportNode')

        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.running = False
        self.receive_thread = threading.Thread(target=self.receive_data)

        # Report message object
        self.msg_obj_battery_rpt = BatteryStatus()
        self.msg_obj_control_mode_rpt = ControlModeReport()
        self.msg_obj_gear_rpt = GearReport()
        self.msg_obj_hazardLights_rpt = HazardLightsReport()
        self.msg_obj_indicators_rpt = TurnIndicatorsReport()
        self.msg_obj_steering_rpt = SteeringReport()
        self.msg_obj_velocity_rpt = VelocityReport()
        self.msg_obj_imu = Imu()

        # Report data class
        self.bms_rpt_data = BMSReportData()
        self.brake_rpt_data = BrakeReportData()
        self.gear_rpt_data = GearReportData()
        self.park_rpt_data = ParkReportData()
        self.steer_rpt_data = SteerReportData()
        self.throttle_rpt_data = ThrottleReportData()
        self.vcu_rpt_data = VcuReportData()
        self.wheel_spd_rpt_data = WheelSpeedReportData()

        # vehicle status report publisher
        self.battery_rpt_publisher = self.create_publisher(BatteryStatus, '/vehicle/status/battery_charge', 10)
        self.control_mode_rpt_publisher = self.create_publisher(ControlModeReport, '/vehicle/status/control_mode', 10)
        self.gear_rpt_publisher = self.create_publisher(GearReport, '/vehicle/status/gear_status', 10)
        self.hazard_lights_rpt_publisher = self.create_publisher(HazardLightsReport,
                                                                 '/vehicle/status/hazard_lights_status', 10)
        self.turn_indicators_rpt_publisher = self.create_publisher(TurnIndicatorsReport,
                                                                   '/vehicle/status/turn_indicators_status', 10)
        self.steering_rpt_publisher = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 10)
        self.velocity_rpt_publisher = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 100)

        # subscriber
        self.imu_subscriber = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # publisher timer
        # self.throttle_ctrl_send_timer = self.create_timer(0.02, self.throttle_ctrl_data_timer_callback)
        self.battery_rpt_timer = self.create_timer(0.02, self.battery_rpt_timer_callback)
        self.control_mode_rpt_timer = self.create_timer(0.02, self.control_mode_rpt_timer_callback)
        self.gear_rpt_timer = self.create_timer(0.02, self.gear_rpt_timer_callback)
        self.hazard_lights_rpt_timer = self.create_timer(0.02, self.hazard_lights_rpt_timer_callback)
        self.turn_indicators_rpt_timer = self.create_timer(0.02, self.turn_indicators_rpt_timer_callback)
        self.steering_rpt_timer = self.create_timer(0.02, self.steering_rpt_timer_callback)
        self.velocity_rpt_timer = self.create_timer(0.02, self.velocity_rpt_timer_callback)

        # global member init
        self.heading_rate = 0.00
        self.start()

    def start(self):
        self.running = True
        self.receive_thread.start()

    def stop(self):
        self.running = False
        self.receive_thread.join()

    def imu_callback(self, msg):
        self.heading_rate = msg.angular_velocity.z

    def receive_data(self):
        """
        Continuously receives CAN message and processes them.
        """
        while self.running:
            try:
                message = self.bus.recv()
                can_id = message.arbitration_id
                data = message.data

                if message.is_error_frame:
                    pass
                # TODO : If the message contains an error frame, additional code for data processing is required.
                if message.is_remote_frame:
                    pass
                # TODO : Requires processing when the message contains a remote request frame
                if not message.is_extended_id and message.dlc == 8 and not message.is_remote_frame:
                    self.process_can_data_and_publish(can_id, data)
            except can.CanError as _:
                pass

    def process_can_data_and_publish(self, can_id, data):

        # Throttle Report parsing
        if can_id == 0x500:
            throttle_en_state = unpack('>B', data[0:1])[0] & 0b00000011
            throttle_flt1 = unpack('>B', data[1:2])[0]
            throttle_flt2 = unpack('>B', data[2:3])[0]
            throttle_pedal_actual = unpack('>H', data[3:5])[0] * 0.1

            data = {
                'throttle_en_state': throttle_en_state,
                'throttle_flt1': throttle_flt1,
                'throttle_flt2': throttle_flt2,
                'throttle_pedal_actual': throttle_pedal_actual
            }

            self.throttle_rpt_data.update_value(**data)


        # Brake Report parsing
        elif can_id == 0x501:
            brake_en_state = unpack('>B', data[0:1])[0] & 0b00000011
            brake_flt1 = unpack('>B', data[1:2])[0]
            brake_flt2 = unpack('>B', data[2:3])[0]
            brake_pedal_actual = unpack('>H', data[3:5])[0] * 0.1

            data = {
                'brake_en_state': brake_en_state,
                'brake_flt1': brake_flt1,
                'brake_flt2': brake_flt2,
                'brake_pedal_actual': brake_pedal_actual
            }

            self.brake_rpt_data.update_value(**data)

        # Steering Report parsing
        elif can_id == 0x502:
            steer_en_state = unpack('>B', data[0:1])[0] & 0b00000011
            steer_flt1 = unpack('>B', data[1:2])[0]
            steer_flt2 = unpack('>B', data[2:3])[0]
            steer_angle_actual = (unpack('>H', data[3:5])[0] - 500) * 0.001
            steer_angle_spd_actual = unpack('>B', data[6:7])[0]

            data = {
                'steer_en_state': steer_en_state,
                'steer_flt1': steer_flt1,
                'steer_flt2': steer_flt2,
                'steer_angle_actual': steer_angle_actual,
                'steer_angle_spd_actual': steer_angle_spd_actual
            }

            self.steer_rpt_data.update_value(**data)

        # Gear Report parsing
        elif can_id == 0x503:
            gear_actual = unpack('>B', data[0:1])[0] & 0b00000111
            gear_flt = unpack('>B', data[1:2])[0]

            gear_actual_rpt = 0

            # DRIVE
            if gear_actual == 4:
                gear_actual_rpt = 2

            # NEUTRAL
            elif gear_actual == 3:
                gear_actual_rpt = 1

            # REVERSE
            elif gear_actual == 2:
                gear_actual_rpt = 20

            # PARKING
            elif gear_actual == 1:
                gear_actual_rpt = 22

            data = {
                'gear_actual': gear_actual_rpt,
                'gear_flt': gear_flt
            }

            self.gear_rpt_data.update_value(**data)

        # Park Report parsing
        elif can_id == 0x504:
            parking_actual = unpack('>B', data[0:1])[0] & 0b00000001
            park_flt = unpack('>B', data[1:2])[0]

            data = {
                'parking_actual': parking_actual,
                'park_flt': park_flt
            }

            self.park_rpt_data.update_value(**data)

        # VCU Report parsing
        # TODO : Check AEB_STATE Report
        elif can_id == 0x505:
            acc = (unpack('>b', data[0:1])[0] | unpack('>b', data[1:2])[0] & 0b11110000) * 0.001
            brake_light_actual = (unpack('>b', data[1:2])[0] >> 3) & 0b00000001
            steer_mode_sts = unpack('>B', data[1:2])[0] & 0b0000111
            speed = unpack('>h', data[2:4])[0] * 0.001
            drive_mode_sts = (unpack('>B', data[4:5])[0] >> 5) & 0b00000111
            vehicle_mode_sts = (unpack('>B', data[4:5])[0] >> 3) & 0b00000011
            back_crash_state = unpack('>B', data[4:5])[0] & 0b00000100
            front_crash_state = unpack('>B', data[4:5])[0] & 0b00000010
            aeb_state = unpack('>B', data[4:5])[0] & 0b00000001
            chassis_errcode = unpack('>B', data[5:6])[0]
            turn_light_actual = unpack('>B', data[7:8])[0] & 0b00000011
            vehicle_mode_sts_rpt = 0
            turn_light_actual_rpt = 0
            hazard_light_actual_rpt = 0

            # Manual/Remote Mode
            if vehicle_mode_sts == 0:
                vehicle_mode_sts_rpt = 4

            # Auto Mode
            elif vehicle_mode_sts == 1:
                vehicle_mode_sts_rpt = 1

            # Turn indicator Left
            if turn_light_actual == 1:
                turn_light_actual_rpt = 2

            # Turn indicator Right
            elif turn_light_actual == 2:
                turn_light_actual_rpt = 3

            # Hazard Light ON
            elif turn_light_actual == 3:
                hazard_light_actual_rpt = 2

            else:
                turn_light_actual_rpt = 0
                hazard_light_actual_rpt = 0

            data = {
                'acc': acc,
                'brake_light_actual': brake_light_actual,
                'steer_mode_sts': steer_mode_sts,
                'speed': speed,
                'drive_mode_sts': drive_mode_sts,
                'vehicle_mode_sts': vehicle_mode_sts_rpt,
                'back_crash_state': back_crash_state,
                'front_crash_state': front_crash_state,
                'aeb_state': aeb_state,
                'chassis_errcode': chassis_errcode,
                'turn_light_actual': turn_light_actual_rpt,
                'hazard_light_actual_rpt': hazard_light_actual_rpt,

            }

            self.vcu_rpt_data.update_value(**data)


        # Wheel Speed Report parsing
        elif can_id == 0x506:
            front_left = unpack('>H', data[0:2])[0] * 0.001
            front_right = unpack('>H', data[2:4])[0] * 0.001
            rear_left = unpack('>H', data[4:6])[0] * 0.001
            rear_right = unpack('>H', data[6:8])[0] * 0.001

            data = {
                'front_left': front_left,
                'front_right': front_right,
                'rear_left': rear_left,
                'rear_right': rear_right
            }

            self.wheel_spd_rpt_data.update_value(**data)

        # BMS Report parsing
        elif can_id == 0x512:
            batter_voltage = unpack('>H', data[0:2])[0] * 0.01
            battery_current = (unpack('>H', data[2:4])[0] * 0.1) - 3200
            battery_soc = unpack('>B', data[4:5])[0]

            data = {
                'batter_voltage': batter_voltage,
                'battery_current': battery_current,
                'battery_soc': battery_soc
            }

            self.bms_rpt_data.update_value(**data)

    # def throttle_ctrl_data_timer_callback(self):
    #     pass

    def battery_rpt_timer_callback(self):
        self.msg_obj_battery_rpt.energy_level = float(self.bms_rpt_data.get_value('battery_soc'))
        self.battery_rpt_publisher.publish(self.msg_obj_battery_rpt)

    def control_mode_rpt_timer_callback(self):
        self.msg_obj_control_mode_rpt.mode = self.vcu_rpt_data.get_value('vehicle_mode_sts')
        self.control_mode_rpt_publisher.publish(self.msg_obj_control_mode_rpt)

    def hazard_lights_rpt_timer_callback(self):
        self.msg_obj_hazardLights_rpt.report = self.vcu_rpt_data.get_value('hazard_light_actual')
        self.hazard_lights_rpt_publisher.publish(self.msg_obj_hazardLights_rpt)

    def turn_indicators_rpt_timer_callback(self):
        self.msg_obj_indicators_rpt.report = self.vcu_rpt_data.get_value('turn_light_actual')
        self.turn_indicators_rpt_publisher.publish(self.msg_obj_indicators_rpt)

    def steering_rpt_timer_callback(self):
        self.msg_obj_steering_rpt.steering_tire_angle = self.steer_rpt_data.get_value('steer_angle_actual')
        self.steering_rpt_publisher.publish(self.msg_obj_steering_rpt)

    def gear_rpt_timer_callback(self):
        self.msg_obj_gear_rpt.report = self.gear_rpt_data.get_value('gear_actual')
        self.gear_rpt_publisher.publish(self.msg_obj_gear_rpt)

    def velocity_rpt_timer_callback(self):
        self.msg_obj_velocity_rpt.header.stamp = self.get_clock().now().to_msg()
        self.msg_obj_velocity_rpt.header.frame_id = "base_link"
        self.msg_obj_velocity_rpt.longitudinal_velocity = float(self.vcu_rpt_data.get_value('speed'))
        self.msg_obj_velocity_rpt.heading_rate = self.heading_rate
        self.velocity_rpt_publisher.publish(self.msg_obj_velocity_rpt)


def main(args=None):
    rclpy.init(args=args)
    node = CANReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
