import time
from motor_driver import SerialPort, MotorCmd, MotorData


ser7 = SerialPort("/dev/ttyUSB3")  #电机3
ser8 = SerialPort("/dev/ttyUSB0")  #电机0
ser9 = SerialPort("/dev/ttyUSB1")  #电机1
ser10 = SerialPort("/dev/ttyUSB2") #电机2

mt0 = MotorCmd(id=0, direction=1, offset=0)
mt1 = MotorCmd(id=0, direction=-1, offset=3.82)
mt2 = MotorCmd(id=0, direction=1, offset=3.137)
mt3 = MotorCmd(id=0, direction=-1, offset=0.31)
dt0 = MotorData()
dt1 = MotorData()
dt2 = MotorData()
dt3 = MotorData()

try:
    while True:
    
        ser8.sendRecv(mt0, dt0)
        ser9.sendRecv(mt1, dt1)
        ser10.sendRecv(mt2, dt2)
        ser7.sendRecv(mt3, dt3)
        
        print(f"ID:0 | P:{dt0.q:>+7.2f} | V:{dt0.dq:>+7.2f}  "
              f"ID:1 | P:{dt1.q:>+7.2f} | V:{dt1.dq:>+7.2f}  "
              f"ID:2 | P:{dt2.q:>+7.2f} | V:{dt2.dq:>+7.2f}  "
              f"ID:3 | P:{dt3.q:>+7.2f} | V:{dt3.dq:>+7.2f}   ", end='\r')

        time.sleep(0.01)






except KeyboardInterrupt:
        mt0.mode = 0
        mt0.kp = 0.0        
        mt0.kd = 0.0       
        mt0.dq = 0.0       
        mt0.tau = 0.0
        ser8.sendRecv(mt0, dt0)
        print("\n程序停止")



