import serial
import struct
import math
import time
import concurrent.futures
import socket
import keyboard
from motor_driver import SerialPort, MotorCmd, MotorData, move

class WiFiGripper:
    def __init__(self, ip, port=8888):
        self.ip = ip
        self.port = port
        # 创建一个 UDP Socket (数据报协议)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"WiFi 夹爪发射器已就绪，目标地址: {self.ip}:{self.port}")

    def set_angle(self, angle):
        """发送 0~180 的角度值给 ESP32"""
        # 限制在合法范围内
        angle = max(0, min(180, int(angle)))
        # 把角度变成字符串，然后编码成字节流发出去
        message = str(angle).encode('utf-8')
        
        try:
            # UDP 是无连接的，直接把包朝那个 IP 扔过去就行了
            self.sock.sendto(message, (self.ip, self.port))
        except Exception as e:
            print(f"发送夹爪指令失败: {e}")

    def close(self):
        # 释放 socket 资源
        self.sock.close()

def inverse_kinematics(r, z):
    """
    逆运动学：已知目标腕部位置(r, z)，求关节角度 q1, q2
    """
    # 限制到原点的距离，防止数学溢出
    d_square = r**2 + z**2
    if d_square > (LINK1_LENGTH + LINK2_LENGTH)**2:
        return None, None # 够不到，超出最大臂展
    if d_square < (LINK1_LENGTH - LINK2_LENGTH)**2:
        return None, None # 靠太近，结构干涉

    # 余弦定理求 q2
    cos_q2 = (d_square - LINK1_LENGTH**2 - LINK2_LENGTH**2) / (2 * LINK1_LENGTH * LINK2_LENGTH)
    # 浮点数防越界
    cos_q2 = max(-1.0, min(1.0, cos_q2)) 
    
    # 采用"肘部朝上"姿态（若实际表现相反，将负号去掉即可）
    q2 = -math.acos(cos_q2) 

    # 求 q1
    k1 = LINK1_LENGTH + LINK2_LENGTH * math.cos(q2)
    k2 = LINK2_LENGTH * math.sin(q2)
    q1 = math.atan2(z, r) - math.atan2(k2, k1)

    return q1, q2
    
def forward_kinematics(q1, q2):
    """
    正运动学：已知角度，求腕部位置
    假设原点在肩部电机轴心，正前方为r轴正方向，正上方为z轴正方向。
    角度0度时手臂水平向前。
    """
    r = LINK1_LENGTH * math.cos(q1) + LINK2_LENGTH * math.cos(q1 + q2)
    z = LINK1_LENGTH * math.sin(q1) + LINK2_LENGTH * math.sin(q1 + q2)
    return r, z


def verify_motor_init(ser, mt, dt, motor_name):
    success_count = 0
    # 循环读取，直到连续获得 10 次稳定反馈
    while success_count < 10:
        ser.sendRecv(mt, dt)
        
        # 验证条件：根据你的驱动库，如果通信失败 dt.q 可能是 None，或者 id 对不上
        # 这里做一个基础的防空值判断（如果你的库失败时返回0.0，你需要根据实际情况调整）
        if dt is not None: 
            success_count += 1
        else:
            success_count = 0 # 一旦断掉，重新计数
            print(f"[{motor_name}] 读取失败，重试中...")
            
        time.sleep(0.005) # 留出 5ms 给 Linux 底层 USB 驱动喘息

def torque_soft_start(ser_list, mt_list, dt_list, pi_coeffs, duration=1.0, steps=100):
    """
    平滑加载重力补偿力矩，防止电机在启动瞬间发生力矩阶跃和震动。
    """
    PI_1, PI_2, PI_3 = pi_coeffs
    ser8, ser9, ser10, ser7 = ser_list
    mt0, mt1, mt2, mt3 = mt_list
    dt0, dt1, dt2, dt3 = dt_list
    
    # 1. 继承当前物理位置，记录下初始状态
    # 注意：这些初始角度在整个缓启动期间都不会改变，确保电机“原地绷紧”
    initial_q0 = dt0.q
    initial_q1 = dt1.q
    initial_q2 = dt2.q
    initial_q3 = dt3.q

    mt0.q = initial_q0
    mt1.q = initial_q1
    mt2.q = initial_q2
    mt3.q = initial_q3
    
    # 设置 3号电机的 PD 参数（提前设好，避免后续突变）
    mt3.kp = 0.5
    mt3.kd = 0.02
    
    # 2. 开启电机模式
    mt0.mode = 1
    mt1.mode = 1
    mt2.mode = 1
    mt3.mode = 1
    
    # 纯位置环首次上电锁死
    ser8.sendRecv(mt0, dt0)
    ser9.sendRecv(mt1, dt1)
    ser10.sendRecv(mt2, dt2)
    ser7.sendRecv(mt3, dt3)
    
    print(f"开始力矩缓启动，预计耗时 {duration} 秒...")
    step_delay = duration / steps
    
    # 3. 缓启动插值主循环
    for i in range(1, steps + 1):
        ratio = i / steps  # 从 0.01 逐渐增加到 1.0
        
        # 计算满负荷受力，并乘以当前步的缓启动比例 ratio
        # 注意：这里我们用实时的反馈 dt.q 来计算重力，更精确
        mt3.tau = ratio * (PI_3 * math.cos(dt1.q + dt2.q + dt3.q))
        mt2.tau = ratio * (PI_2 * math.cos(dt1.q + dt2.q) + mt3.tau)
        mt1.tau = ratio * (PI_1 * math.cos(dt1.q) + mt2.tau)
        
        # 【核心修复】：绝对不能在这里改变 mt3.q 的值！
        # 让所有电机保持在 initial_q 的位置
        mt0.q = initial_q0
        mt1.q = initial_q1
        mt2.q = initial_q2
        mt3.q = initial_q3
        
        # 发送插值指令并获取最新反馈
        ser8.sendRecv(mt0, dt0)
        ser9.sendRecv(mt1, dt1)
        ser10.sendRecv(mt2, dt2)
        ser7.sendRecv(mt3, dt3)
        
        time.sleep(step_delay)
        
    print("重力补偿 100% 加载完毕，进入正常运行状态！")

# ================= 主控制流程 =================

LINK1_LENGTH = 0.20448  # 肩部电机轴心 到 肘部电机轴心 的距离
LINK2_LENGTH = 0.23969  # 肘部电机轴心 到 腕部电机轴心 的距离


if __name__ == "__main__":
    
    # 实例化电机
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

    gripper = WiFiGripper(ip="10.175.31.214", port=8888)

    # 动力学系数
    PI_1 = 3.1909
    PI_2 = 1.8932
    PI_3 = 0.0234
    
    #初始化角度
    verify_motor_init(ser8, mt0, dt0, "肩部(mt0)")
    verify_motor_init(ser9, mt1, dt1, "肘部1(mt1)")
    verify_motor_init(ser10, mt2, dt2, "肘部2(mt2)")
    verify_motor_init(ser7, mt3, dt3, "手腕(mt3)")
    torque_soft_start(
        ser_list=[ser8, ser9, ser10, ser7], 
        mt_list=[mt0, mt1, mt2, mt3], 
        dt_list=[dt0, dt1, dt2, dt3], 
        pi_coeffs=(PI_1, PI_2, PI_3),
        duration=0.3,  # 你可以自由修改这里的启动时间，比如 1.5 秒
        steps=50      # 步数跟着等比调整
    )

    move(ser7, mt3, dt3, target=-(dt2.q + dt1.q), duration=0.5)
    time.sleep(0.5)



    try:
        while True:
            # 1. 重力补偿 (这里最好用真实的反馈位置 q1, q2, q3 来计算，因为这是当下的物理受力)
            
            mt3.tau = PI_3 * math.cos(dt1.q + dt2.q + dt3.q)
            mt2.tau = PI_2 * math.cos(dt1.q + dt2.q) + mt3.tau
            mt1.tau = PI_1 * math.cos(dt1.q) + mt2.tau
            mt3.q = -(dt2.q + dt1.q)
            mt3.kp = 0.5
            mt3.kd = 0.02

            

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
        mt1.mode = 0
        mt2.mode = 0
        mt3.mode = 0
        ser8.sendRecv(mt0, dt0)
        ser9.sendRecv(mt1, dt1)
        ser10.sendRecv(mt2, dt2)
        ser7.sendRecv(mt3, dt3)

        print("\n程序停止")
