import sys
import select
import termios
import tty
import time

# 假设之前的RS485通讯及MODBUS协议代码保存在 driver.py 中
# 如：from driver import RS485Communicator, ModbusDevice
# 如果不分文件，也可以直接把之前的类代码放在此文件中

# 动态添加项目根目录到sys.path
from pathlib import Path
root_path = Path(__file__).parent.parent
sys.path.append(str(root_path))

from scripts.rs import RS485Communicator, ModbusDevice


INIT_SPEED = 6600.0

def get_key(timeout=0.1):
    """
    非阻塞方式读取一个字符，若超时则返回 None
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key



def main():
    # 创建RS485通讯对象（根据实际情况修改串口号和波特率）
    communicator = RS485Communicator(port="/dev/ttyUSB0", baudrate=9600, timeout=0.2)
    
    # 创建左右两个从站设备对象，分别对应从站地址 0x01（左轮）和 0x02（右轮）
    device_left = ModbusDevice(communicator, 0x01)
    device_right = ModbusDevice(communicator, 0x02)
    
    reg_value = INIT_SPEED
    
    # print(device_right.read_holding_registers(0x0073, 2))

    try:
        # 设置初始目标转速到两轮（写入寄存器 0x0043）
        device_left.write_speed(reg_value)
        device_right.write_speed(reg_value)
        print("启动时目标转速：{:.2f} 0.1hz".format(reg_value))
    except Exception as e:
        print("设置初始速度错误：", e)
        raise
    
    print("测试说明：按 's' 提升 10% 转速，按 'd' 降低 10% 转速，按 'q' 退出测试。")
    
    try:
        while True:
            # 读取左右轮当前转速（假设寄存器 0x0034 返回转速，单位为 rpm）
            have_speed = True
            try:
                left_hz = device_left.read_speed()
                left_rpm = device_left.read_speed(hz=False)
                right_hz = device_right.read_speed()
                right_rpm = device_right.read_speed(hz=False)
            except Exception as e:
                print("读取速度错误：", e)
                have_speed = False
            
            if have_speed:
                # print("当前转速 --> 左轮: {} rpm,  {} 0.1hz; "
                # "右轮： {} rpm,  {} 0.1hz".format(left_rpm, left_hz, right_rpm, right_hz))
                print(f"\r左轮: {left_hz} Hz ({left_rpm} RPM) | "
                      f"右轮: {right_hz} Hz ({right_rpm} RPM)", end="")
                
            # 检查键盘输入
            key = get_key()
            if key:
                if key == 'q':
                    print("退出测试。")
                    break
                elif key == 's':
                    reg_value *= 1.1
                    print("\n提升速度，新目标转速：{:.2f} 0.1hz".format(reg_value))
                elif key == 'd':
                    reg_value *= 0.9
                    print("\n降低速度，新目标转速：{:.2f} 0.1hz".format(reg_value))
                else:
                    continue
                try:
                    device_left.write_speed(reg_value)
                    device_right.write_speed(reg_value)
                except Exception as e:
                    print("写入新速度错误：", e)

            time.sleep(0.5)  # 循环延时，避免过于频繁的通讯
    except KeyboardInterrupt:
        print("测试中断。")
    finally:
        device_left.stop()
        device_right.stop()
        communicator.close()


if __name__ == '__main__':
    main()
