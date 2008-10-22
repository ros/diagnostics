from sample_test.sample_test import SampleTest
from hokuyo_test.hokuyo_test import HokuyoTest
from imu_test.imu_test import ImuTest
from ethernet_test.ethernet_test import EthernetTest
from gripper_test.gripper_test import GripperTest

test_list = {'00000': SampleTest, '6801024': HokuyoTest, '6801050':ImuTest,  'etherne':EthernetTest, '6804015': GripperTest}
