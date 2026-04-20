#!/usr/bin/env python3
"""
WuK Project SITL Test Script
自动化测试WuK变形无人机在SITL中的行为
"""

import time
import sys
from pymavlink import mavutil

class WuKSITLTester:
    def __init__(self, connection_string='tcp:127.0.0.1:5760'):
        """初始化MAVLink连接"""
        print(f"连接到 {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        print("收到心跳!")
        
    def set_mode(self, mode_name):
        """设置飞行模式"""
        # 模式映射 (ArduCopter)
        mode_map = {
            'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4,
            'LOITER': 5, 'RTL': 6, 'CIRCLE': 7, 'LAND': 9, 'DRIFT': 11,
            'SPORT': 13, 'FLIP': 14, 'AUTOTUNE': 15, 'POSHOLD': 16,
            'BRAKE': 17, 'THROW': 18, 'AVOID_ADSB': 19, 'GUIDED_NOGPS': 20,
            'SMART_RTL': 21, 'FLOWHOLD': 22, 'FOLLOW': 23, 'ZIGZAG': 24,
            'SYSTEMID': 25, 'AUTOROTATE': 26, 'AUTO_RTL': 27, 'TURTLE': 28,
            'MORPH': 50, 'GROUND': 51,
        }
        
        if mode_name.upper() not in mode_map:
            print(f"错误: 未知模式 {mode_name}")
            return False
            
        mode_id = mode_map[mode_name.upper()]
        
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
        print(f"请求切换到模式: {mode_name} (ID: {mode_id})")
        time.sleep(0.5)
        return True
    
    def arm(self):
        """解锁"""
        print("发送解锁命令...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("已解锁!")
        
    def disarm(self):
        """上锁"""
        print("发送上锁命令...")
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        print("已上锁!")
    
    def set_rc(self, channel, pwm):
        """设置RC通道值 (1-16, PWM 1000-2000)"""
        channels = [65535] * 18  # 65535 = no change
        channels[channel - 1] = pwm
        
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *channels
        )
        print(f"设置 RC{channel} = {pwm}")
    
    def monitor_messages(self, duration=5):
        """监控消息"""
        print(f"\n监控消息 {duration} 秒...")
        start_time = time.time()
        while time.time() - start_time < duration:
            msg = self.master.recv_match(blocking=True, timeout=1)
            if msg:
                msg_type = msg.get_type()
                if msg_type == 'STATUSTEXT':
                    print(f"[TEXT] {msg.text}")
                elif msg_type == 'NAMED_VALUE_FLOAT':
                    print(f"[NAMED] {msg.name}: {msg.value}")
    
    def run_test(self):
        """运行测试"""
        print("\n" + "="*60)
        print("WuK SITL 测试")
        print("="*60)
        
        try:
            print("\n1. 设置 LOITER 模式")
            self.set_mode('LOITER')
            time.sleep(2)
            
            print("\n2. 测试 RC7/8/9")
            self.set_rc(7, 1100)
            time.sleep(1)
            self.set_rc(7, 1900)
            time.sleep(1)
            self.set_rc(7, 1500)
            time.sleep(1)
            
            self.set_rc(8, 1100)
            time.sleep(1)
            self.set_rc(8, 1900)
            time.sleep(1)
            self.set_rc(8, 1500)
            time.sleep(1)
            
            print("\n3. 触发变形 (RC9 = 1900)")
            self.set_rc(9, 1900)
            self.monitor_messages(duration=10)
            
            print("\n测试完成!")
            
        except KeyboardInterrupt:
            print("\n用户中断")
        except Exception as e:
            print(f"\n错误: {e}")

def main():
    if len(sys.argv) > 1:
        conn = sys.argv[1]
    else:
        conn = 'tcp:127.0.0.1:5760'
    
    tester = WuKSITLTester(conn)
    tester.run_test()

if __name__ == '__main__':
    main()
