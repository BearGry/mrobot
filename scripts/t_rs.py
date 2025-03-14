import sys
import select
import termios
import tty
import time

# 假设之前的RS485通讯及MODBUS协议代码保存在 driver.py 中
# 如：from driver import RS485Communicator, ModbusDevice
# 如果不分文件，也可以直接把之前的类代码放在此文件中

from rs import RS485Communicator, ModbusDevice

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

def rpm_to_register_value(rpm):
    """
    根据目标转速（单位rpm）转换为写入寄存器的数值。
    本例假设寄存器 0x0043 设定闭环目标转速，其值 = rpm * 2 （例如 30rpm 对应 60）
    """
    return int(round(rpm * 2))

def main():
    # 创建RS485通讯对象（根据实际情况修改串口号和波特率）
    communicator = RS485Communicator(port="/dev/ttyUSB0", baudrate=9600, timeout=0.2)
    
    # 创建左右两个从站设备对象，分别对应从站地址 0x01（左轮）和 0x02（右轮）
    device_left = ModbusDevice(communicator, 0x01)
    device_right = ModbusDevice(communicator, 0x02)
    
    target_rpm = 2000  # 初始目标转速，单位 rpm
    reg_value = rpm_to_register_value(target_rpm)
    
    try:
        # 设置初始目标转速到两轮（写入寄存器 0x0043）
        device_left.write_single_register(0x0043, -reg_value, signed=True)
        device_right.write_single_register(0x0043, reg_value, signed=True)
        print("启动时目标转速：{:.2f} rpm".format(target_rpm))
    except Exception as e:
        print("设置初始速度错误：", e)
        raise
    
    print("测试说明：按 's' 提升 10% 转速，按 'd' 降低 10% 转速，按 'q' 退出测试。")
    
    try:
        while True:
            # 读取左右轮当前转速（假设寄存器 0x0034 返回转速，单位为 rpm）
            have_speed = True
            try:
                left_hz = device_left.read_holding_registers(0x0022, 1, signed=True)[0]
                left_rpm = device_left.read_holding_registers(0x0034, 1)[0]
                right_hz = device_right.read_holding_registers(0x0022, 1, signed=True)[0]
                right_rpm = device_right.read_holding_registers(0x0034, 1)[0]
            except Exception as e:
                print("读取速度错误：", e)
                have_speed = False
            
            if have_speed:
                print("当前转速 --> 左轮: {} rpm,  {} 0.1hz; "
                "右轮： {} rpm,  {} 0.1hz".format(left_rpm, left_hz, right_rpm, right_hz))
            
            # 检查键盘输入
            key = get_key()
            if key:
                if key == 's':
                    target_rpm *= 1.1
                    print("提升速度，新目标转速：{:.2f} rpm".format(target_rpm))
                    reg_value = rpm_to_register_value(target_rpm)
                    try:
                        device_left.write_single_register(0x0043, -reg_value, signed=True)
                        device_right.write_single_register(0x0043, reg_value, signed=True)
                    except Exception as e:
                        print("写入新速度错误：", e)
                elif key == 'd':
                    target_rpm *= 0.9
                    print("降低速度，新目标转速：{:.2f} rpm".format(target_rpm))
                    reg_value = rpm_to_register_value(target_rpm)
                    try:
                        device_left.write_single_register(0x0043, -reg_value, signed=True)
                        device_right.write_single_register(0x0043, reg_value, signed=True)
                    except Exception as e:
                        print("写入新速度错误：", e)
                elif key == 'q':
                    print("退出测试。")
                    break
            time.sleep(0.5)  # 循环延时，避免过于频繁的通讯
    except KeyboardInterrupt:
        print("测试中断。")
    finally:
        device_left.write_single_register(0x0043, 0)
        device_right.write_single_register(0x0043, 0)
        communicator.close()


if __name__ == '__main__':
    main()
