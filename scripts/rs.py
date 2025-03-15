import serial
import time
import struct

def modbus_crc16(data):
    """
    计算MODBUS RTU协议的CRC16校验码。
    :param data: 待校验的字节数组
    :return: CRC16值（整数）
    """
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 0x0001):
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

class RS485Communicator(object):
    """
    封装RS485底层通讯，负责串口的打开、发送和接收。
    """
    def __init__(self, port="/dev/ttyUSB0", baudrate=9600, timeout=0.2):
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_EVEN,   # 根据协议要求，数据位8，偶校验
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=timeout
        )
    
    def send_frame(self, frame):
        """
        发送一帧数据
        :param frame: 要发送的字节序列
        """
        self.serial.write(frame)
        self.serial.flush()
    
    def read_frame(self, expected_length):
        """
        读取一帧数据
        :param expected_length: 预期接收的字节数
        :return: 接收到的字节数据
        """
        frame = self.serial.read(expected_length)
        return frame
    
    def close(self):
        """
        关闭串口
        """
        self.serial.close()

class ModbusDevice(object):
    """
    封装MODBUS RTU协议操作，包括读保持寄存器、写单个寄存器、写多个寄存器。
    此外提供更常用的读写速度接口，上层使用更加便捷
    每个对象对应一个从站设备，通过slave_addr指定。
    """
    def __init__(self, communicator, slave_addr):
        """
        :param communicator: RS485Communicator对象，用于底层通讯
        :param slave_addr: 从站地址（如0x01或0x02）
        """
        self.comm = communicator
        self.slave_addr = slave_addr

    def _build_frame(self, function_code, payload):
        """
        构建MODBUS RTU帧：包含从站地址、功能码、数据域和CRC校验
        :param function_code: MODBUS功能码（例如0x03、0x06、0x10）
        :param payload: 数据部分（例如寄存器地址、寄存器数量等），类型为bytearray
        :return: 完整的帧（bytearray）
        """
        frame = bytearray()
        frame.append(self.slave_addr)
        frame.append(function_code)
        frame.extend(payload)
        crc = modbus_crc16(frame)
        frame.append(crc & 0xFF)           # CRC低字节
        frame.append((crc >> 8) & 0xFF)      # CRC高字节
        return frame

    def _send_and_receive(self, function_code, payload, response_length):
        """
        发送帧并等待接收响应，同时校验CRC
        :param function_code: MODBUS功能码
        :param payload: 数据域
        :param response_length: 预期响应帧的长度（字节数）
        :return: 响应帧（字节数组）
        """
        frame = self._build_frame(function_code, payload)
        self.comm.send_frame(frame)
        # 根据波特率和数据长度等待足够时间后再读数据
        time.sleep(0.05)
        response = self.comm.read_frame(response_length)
        if len(response) < response_length:
            raise Exception("响应超时或数据不完整")
        # 校验响应帧的CRC
        received_crc = response[-2] | (response[-1] << 8)
        calculated_crc = modbus_crc16(response[:-2])
        if received_crc != calculated_crc:
            raise Exception("CRC校验错误")
        # 检查是否返回错误码（功能码高位加0x80表示异常）
        if response[1] & 0x80:
            error_code = response[2]
            raise Exception("MODBUS异常，错误码: {}".format(error_code))
        return response

    def read_holding_registers(self, start_address, quantity, raw_data=False, signed=False):
        """
        使用功能码0x03读保持寄存器
        :param start_address: 起始寄存器地址（整数）
        :param quantity: 寄存器数量（整数）
        :param raw_data: 是否要原始接收到的字节数组数据
        :param signal: 解码处理时，数值是否带符号
        :return: 包含各寄存器值的列表，每个值为一个整数
        """
        payload = bytearray()
        payload.extend(struct.pack('>H', start_address))  # 高字节在前
        payload.extend(struct.pack('>H', quantity))
        # 响应格式：从站地址、功能码、字节数、数据（寄存器数*2字节）+CRC2字节
        expected_length = 5 + quantity * 2
        try:
            response = self._send_and_receive(0x03, payload, expected_length)
        except Exception as e:
            print(f'{self.slave_addr} - {start_address} 读取出错')
            raise Exception("读保持寄存器异常") from e
        # 第3字节为数据字节数
        byte_count = response[2]
        # 返回数据部分（原始字节数组）
        if raw_data:
            data_bytes = response[3:3+byte_count]
            return data_bytes
        registers = []
        # 返回前统一处理好
        for i in range(quantity):
            reg_bytes = response[3 + i*2: 3 + (i+1)*2]
            if signed:
                reg_val = struct.unpack('>h', reg_bytes)[0]
            else:
                reg_val = struct.unpack('>H', reg_bytes)[0]
            registers.append(reg_val)
        return registers


    def write_single_register(self, register_address, value, signed=False):
        """
        使用功能码0x06写单个寄存器
        :param register_address: 寄存器地址（整数）
        :param value: 要写入的数值（整数）
        :param signed: 如果为True，则以有符号方式打包寄存器值，否则以无符号方式
        :return: 响应帧
        """
        payload = bytearray()
        payload.extend(struct.pack('>H', register_address))
        if signed:
            payload.extend(struct.pack('>h', value))
        else:
            payload.extend(struct.pack('>H', value))
        expected_length = 8
        try:
            response = self._send_and_receive(0x06, payload, expected_length)
        except Exception as e:
            print(f'{self.slave_addr} - {register_address} 写出错')
            raise Exception("写单个寄存器异常") from e
        return response


    def write_multiple_registers(self, start_address, values):
        """
        使用功能码0x10写多个寄存器
        :param start_address: 起始寄存器地址
        :param values: 待写入的寄存器值列表，每个值为整数
        :return: 响应帧
        """
        quantity = len(values)
        payload = bytearray()
        payload.extend(struct.pack('>H', start_address))
        payload.extend(struct.pack('>H', quantity))
        byte_count = quantity * 2
        payload.append(byte_count)
        for val in values:
            payload.extend(struct.pack('>H', val))
        # 响应帧长度固定为8字节
        expected_length = 8
        response = self._send_and_receive(0x10, payload, expected_length)
        return response
    
    def read_speed(self, hz=True):
        '''
        读取速度值，换向频率（单位0.1Hz） 或 电机转速（单位RPM）
        :param hz: 是否要换向频率版本，False的话为电机转速版本
        '''
        result = 0
        if hz:
            result = self.read_holding_registers(0x0022, 1, signed=True)[0]
            # 这里取反是为了统一读写速度时，正数为向前走，负数为向后走
            if self.slave_addr == 0x01: # 左轮
                result = -result
        else:
            result = self.read_holding_registers(0x0034, 1)[0]
        return result

    def write_speed(self, speed):
        '''
        速度闭环控制目标速度-换向频率（单位0.1Hz）
        :param speed: 目标速度值-换向频率（单位0.1Hz）  (浮点数会被转化为整数)
        '''
        speed = int(round(speed))
        if self.slave_addr == 0x01:
            speed = -speed
        self.write_single_register(0x0043, speed, signed=True)

    def stop(self):
        '''
        让电机停下来
        '''
        self.write_speed(0)
        

if __name__ == '__main__':
    # 示例：创建RS485通讯对象及两个从站设备对象
    try:
        # 初始化RS485通讯（根据实际情况修改串口号和波特率）
        communicator = RS485Communicator(port="/dev/ttyUSB0", baudrate=9600, timeout=0.2)
        # 创建两个Modbus设备实例，分别对应从站地址0x01和0x02
        device1 = ModbusDevice(communicator, 0x01)
        device2 = ModbusDevice(communicator, 0x02)
        
        # 示例操作：从设备1读取2个保持寄存器（地址0x0000开始）
        regs = device1.read_holding_registers(start_address=0x0000, quantity=2)
        print("设备1的寄存器数据：", regs)
        
        # 示例操作：向设备2写入单个寄存器（例如寄存器0x0041写入值500）
        resp = device2.write_single_register(register_address=0x0041, value=500)
        print("设备2写单寄存器响应：", list(resp))
        
    except Exception as e:
        print("通讯出现错误：", e)
    finally:
        communicator.close()
