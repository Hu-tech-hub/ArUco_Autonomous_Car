"""TurboPi expansion-board I2C transport, based on Hiwonder Board.py."""


class ExpansionBoard:
    def __init__(self, bus=1, address=0x7A, transport=None, message_factory=None):
        self.bus_number,self.address = bus,address
        self.transport,self.message_factory = transport,message_factory
        self.motors = [0]*4
        self.servos = [1500]*6

    def _dependencies(self):
        if self.message_factory is None:
            from smbus2 import SMBus, i2c_msg
            self.message_factory = i2c_msg
            if self.transport is None:
                self.transport = SMBus(self.bus_number)

    def _write(self, payload):
        self._dependencies()
        message = self.message_factory.write(self.address,payload)
        self.transport.i2c_rdwr(message)

    def set_motor(self, index, speed):
        if index not in (1,2,3,4):
            raise ValueError('Motor index must be 1..4')
        speed = max(-100,min(100,int(round(speed))))
        if index in (1,3):
            speed = -speed
        self._write([31+index-1,speed.to_bytes(1,'little',signed=True)[0]])
        self.motors[index-1] = speed
        return speed

    def set_servo(self, index, pulse=1500, duration_ms=1000):
        if not 1 <= index <= 6:
            raise ValueError('Servo index must be 1..6')
        pulse = max(500,min(2500,int(pulse)))
        duration_ms = max(0,min(30000,int(duration_ms)))
        self._write([40,1,*duration_ms.to_bytes(2,'little'),index,*pulse.to_bytes(2,'little')])
        self.servos[index-1] = pulse
        return pulse

    def battery_mv(self):
        self._dependencies()
        write = self.message_factory.write(self.address,[0])
        read = self.message_factory.read(self.address,2)
        self.transport.i2c_rdwr(write,read)
        return int.from_bytes(bytes(read),'little')

    def stop(self):
        first_error = None
        for index in range(1,5):
            try:
                self.set_motor(index,0)
            except Exception as error:
                first_error = first_error or error
        if first_error:
            raise first_error

    def close(self):
        if self.transport is not None:
            self.transport.close()
