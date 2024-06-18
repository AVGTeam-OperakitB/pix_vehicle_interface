from setuptools import find_packages, setup
import os
from glob import glob

# Other imports ...


package_name = 'pix_vehicle_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('pix_vehicle_interface/launch', '*launch.[py]*'))),
        ('lib/' + package_name + '/can_utils', ['pix_vehicle_interface/can_utils/can_sender.py']),
        ('lib/' + package_name + '/pix_dataclass', ['pix_vehicle_interface/pix_dataclass/data_utils.py',
                                                    'pix_vehicle_interface/pix_dataclass/BrakeCtrlData.py',
                                                    'pix_vehicle_interface/pix_dataclass/data_utils.py',
                                                    'pix_vehicle_interface/pix_dataclass/GearCtrlData.py',
                                                    'pix_vehicle_interface/pix_dataclass/ParkCtrlData.py',
                                                    'pix_vehicle_interface/pix_dataclass/SteerCtrlData.py',
                                                    'pix_vehicle_interface/pix_dataclass/ThrottleCtrlData.py',
                                                    'pix_vehicle_interface/pix_dataclass/VehicleModeCtrlData.py',
                                                    'pix_vehicle_interface/pix_dataclass/BMSReportData.py',
                                                    'pix_vehicle_interface/pix_dataclass/BrakeReportData.py',
                                                    'pix_vehicle_interface/pix_dataclass/GearReportData.py',
                                                    'pix_vehicle_interface/pix_dataclass/ParkReportData.py',
                                                    'pix_vehicle_interface/pix_dataclass/SteerReportData.py',
                                                    'pix_vehicle_interface/pix_dataclass/ThrottleReportData.py',
                                                    'pix_vehicle_interface/pix_dataclass/VcuReportData.py',
                                                    'pix_vehicle_interface/pix_dataclass/WheelSpeedReportData.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JEON YANG HO',
    maintainer_email='yhjeon@avgenius.kr',
    description='pix vehicle interface by CAN',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pix_interface_rpt = pix_vehicle_interface.pix_interface_rpt_node:main',
            'pix_interface_cmd = pix_vehicle_interface.pix_interface_cmd_node:main',
            'pix_test = pix_vehicle_interface.test_cmd:main',
        ],
    },
)
