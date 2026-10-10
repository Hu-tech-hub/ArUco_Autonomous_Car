"""Board-compatible I2C interface for the TurboPi motor/servo expansion board.
Protocol derived from Hiwonder Board.py; original in ../reference/Board.py.
"""
from src.hardware import ExpansionBoard
_board = ExpansionBoard()

def setMotor(index, speed):
    return _board.set_motor(index, speed)

def getMotor(index):
    if not 1 <= index <= 4:
        raise ValueError('Motor index must be 1..4')
    return _board.motors[index-1]

def setPWMServoPulse(servo_id, pulse=1500, use_time=1000):
    return _board.set_servo(servo_id,pulse,use_time)

def getPWMServoPulse(servo_id):
    if not 1 <= servo_id <= 6:
        raise ValueError('Servo index must be 1..6')
    return _board.servos[servo_id-1]

def getBattery():
    return _board.battery_mv()

def stopAllMotors():
    _board.stop()

def close():
    _board.close()
