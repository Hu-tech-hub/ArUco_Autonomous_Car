from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from src.hardware import ExpansionBoard


class FakeMessages:
    @staticmethod
    def write(address,payload):
        return address,list(payload)


class FakeBus:
    def __init__(self): self.writes=[]
    def i2c_rdwr(self,*messages): self.writes.extend(messages)
    def close(self): pass


class HardwareTests(unittest.TestCase):
    def setUp(self):
        self.bus=FakeBus()
        self.board=ExpansionBoard(transport=self.bus,message_factory=FakeMessages)

    def test_motor_protocol_direction_and_limits(self):
        self.board.set_motor(1,60)
        self.board.set_motor(2,150)
        self.assertEqual(self.bus.writes,[(0x7A,[31,196]),(0x7A,[32,100])])
        with self.assertRaises(ValueError): self.board.set_motor(5,1)

    def test_servo_protocol(self):
        self.board.set_servo(1,1500,1000)
        self.assertEqual(self.bus.writes[-1],(0x7A,[40,1,232,3,1,220,5]))

    def test_hiwonder_chassis_uses_adapter(self):
        sdk=Path(__file__).resolve().parents[1]/'vendor/hiwonder'
        sys.path.insert(0,str(sdk))
        from HiwonderSDK import Board
        from HiwonderSDK.mecanum import MecanumChassis
        with patch.object(Board,'_board',self.board):
            chassis=MecanumChassis()
            chassis.set_velocity(40,90,0)
            self.assertEqual(len(self.bus.writes),4)
            self.assertTrue(all(addr==0x7A for addr,_ in self.bus.writes))
            self.assertLess(self.board.motors[0],0)
            self.assertGreater(self.board.motors[1],0)
            Board.stopAllMotors()
            self.assertEqual(self.board.motors,[0,0,0,0])


if __name__ == '__main__':
    unittest.main()
