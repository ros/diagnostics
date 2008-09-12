from sample_test.sample_test import SampleTest
from hokuyo_test.hokuyo_test import HokuyoTest
from imu_test.imu_test import ImuTest
from motor_test.motor_test import MotorTest
from ethernet_test.ethernet_test import EthernetTest

test_list = {'00000': SampleTest, '6801024': HokuyoTest, '6801050':ImuTest, '6803009':MotorTest,'6803018':MotorTest,'6803019':MotorTest,'6803020':MotorTest,'6803022':MotorTest,'6803023':MotorTest, '6803025':MotorTest, 'etherne':EthernetTest}
