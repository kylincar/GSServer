import serial
from enum import Enum
import os
from socket import timeout
import time

import math
def axes_to_ha_dec(ra_axis, dec_axis, latitude, southern_hemisphere=False):
    """
    将德式赤道仪轴读数转换为时角/赤纬
    
    参数:
    ra_axis: RA轴读数 (度)
    dec_axis: Dec轴读数 (度)
    latitude: 观测点地理纬度 (度)
    southern_hemisphere: 是否在南半球 (布尔值)
    
    返回:
    tuple: (ha, dec) 时角(小时), 赤纬(度)
    """
    # 标准化轴读数
    ra_axis = ra_axis % 360
    if ra_axis < 0:
        ra_axis += 360
    
    # 处理Dec轴读数，确保在合理的范围内
    # Dec轴读数在德式赤道仪中通常在-90到+90度范围内
    # 但如果发生了极点翻转，可能在90到270度范围内
    adjusted_dec = dec_axis
    if adjusted_dec > 270:
        adjusted_dec -= 360
    elif adjusted_dec < -90:
        adjusted_dec += 360
    
    # 检查是否需要极点翻转调整
    ha_deg = ra_axis
    final_dec = adjusted_dec
    if ra_axis > 180.0:
        ha_deg = ra_axis - 180
        final_dec = 180 - adjusted_dec
    
    # 转换为时角
    ha = ha_deg / 15.0
    
    # 南半球调整
    if southern_hemisphere:
        final_dec = -final_dec
    
    return ha, final_dec

def ha_dec_to_az_alt(ha, dec, latitude = 42.4708):
    """
    将时角/赤纬转换为方位角/高度角
    
    参数:
    ha: 时角 (小时)
    dec: 赤纬 (度)
    latitude: 观测点地理纬度 (度)
    
    返回:
    tuple: (azimuth, altitude) 方位角(度), 高度角(度)
    """
    # 输入验证
    if not (-90 <= latitude <= 90):
        raise ValueError("纬度应在-90到+90度范围内")
    
    # 转换为弧度
    ha_rad = math.radians(ha * 15.0)  # 时角转换为角度再转为弧度
    dec_rad = math.radians(dec)
    lat_rad = math.radians(latitude)
    
    # 计算高度角
    sin_alt = math.sin(lat_rad) * math.sin(dec_rad) + math.cos(lat_rad) * math.cos(dec_rad) * math.cos(ha_rad)
    # 限制sin_alt在[-1, 1]范围内，防止计算误差
    sin_alt = max(-1, min(1, sin_alt))
    alt_rad = math.asin(sin_alt)
    altitude = math.degrees(alt_rad)
    
    # 计算方位角
    # 处理极点情况
    cos_alt = math.cos(alt_rad)
    if abs(cos_alt) < 1e-10:  # 高度角接近90度或-90度
        azimuth = 0  # 在天顶或天底时，方位角定义为0
    else:
        sin_az = -math.sin(ha_rad) * math.cos(dec_rad) / math.cos(alt_rad)
        cos_az = (math.cos(lat_rad) * math.sin(dec_rad) - math.sin(lat_rad) * math.cos(dec_rad) * math.cos(ha_rad)) / math.cos(alt_rad)
        # 限制sin_az和cos_az在[-1, 1]范围内，防止计算误差
        sin_az = max(-1, min(1, sin_az))
        cos_az = max(-1, min(1, cos_az))
        
        az_rad = math.atan2(sin_az, cos_az)
        azimuth = math.degrees(az_rad)
    
    # 确保方位角在0-360度范围内
    azimuth = azimuth % 360
    if azimuth < 0:
        azimuth += 360
    
    return azimuth, altitude

def equatorial_axes_to_az_alt(ra_axis, dec_axis, latitude = 42.4708, southern_hemisphere=False):
    """
    将德式赤道仪轴读数直接转换为方位角/高度角
    
    参数:
    ra_axis: RA轴读数 (度)
    dec_axis: Dec轴读数 (度)
    latitude: 观测点地理纬度 (度)
    southern_hemisphere: 是否在南半球 (布尔值)
    
    返回:
    tuple: (azimuth, altitude) 方位角(度), 高度角(度)
    """
    # 第一步：轴读数 → HA/Dec
    ha, dec = axes_to_ha_dec(ra_axis, dec_axis, latitude, southern_hemisphere)
    print(f"ha-dec:{ha}\t{dec}")
    # 第二步：HA/Dec → Az/Alt
    azimuth, altitude = ha_dec_to_az_alt(ha, dec, latitude)
    
    return azimuth, altitude

class Comma(Enum):
    START_CHAR = ':'
    END_CHAR = '\r'
    ANSWER_CHAR = '='
    ERROR_CHAR = '!'

class Switch(Enum):
    OFF = '0'
    ON  = '1'

class Speed(Enum):
    X_1000  = '0'
    X_750   = '1'
    X_500   = '2'
    X_250   = '3'
    X_125   = '4'

class MotionMode(Enum):
    GOTO_FAST_SF                = '0'
    TRACKING_SLOW_SF            = '1'
    GOTO_SLOW_SF                = '2'
    TRACKING_FAST_SF            = '3'
    GOTO_FAST_MEDIUM            = '4'
    TRACKING_SLOW_MEDIUM        = '5'
    GOTO_SLOW_MEDIUM            = '6'
    TRACKING_FAST_MEDIUM        = '7'
    GOTO_FAST_SF_1X             = '8'
    TRACKING_SLOW_SF_1X         = '9'
    GOTO_SLOW_SF_1X             = 'A'
    TRACKING_FAST_SF_1X         = 'B'
    GOTO_FAST_MEDIUM_1X         = 'C'
    TRACKING_SLOW_MEDIUM_1X     = 'D'
    GOTO_SLOW_MEDIUM_1X         = 'E'
    TRACKING_FAST_MEDIUM_1X     = 'F'

class MotionModeDirection(Enum):
    CW_NORTH_NORMAL             = '0'
    CCW_NORTH_NORMAL            = '1'
    CW_SOUTH_NORMAL             = '2'
    CCW_SOUTH_NORMAL            = '3'
    CW_NORTH_COARSE             = '4'
    CCW_NORTH_COARSE            = '5'
    CW_SOUTH_COARSE             = '6'
    CCW_SOUTH_COARSE            = '7'

class Axis(Enum):
    CH1_RA_AZ = '1'
    CH2_DEC_ALT = '2'
    BOTH = '3'

class OpCode(Enum):
    SET_POSITION            = 'E'
    INIT_DONE               = 'F'
    SET_MOTION_MODE         = 'G'
    SET_GOTO_TARGET         = 'S'
    SET_STEP_PERIOD         = 'I'
    START_MOTION            = 'J'
    STOP_MOTION             = 'K'
    INSTANT_STOP            = 'L'
    SET_AUX_SWITCH          = 'O'
    SET_AUTOGUIDE_SPEED     = 'P'
    RUN_BOOTLOADER_MODE     = 'Q'
    SET_LED_BRIGHTNESS      = 'V'
    GET_CPR                 = 'a'
    GET_TIMER_FREQ          = 'b'
    GET_GOTO_TARGET_POS     = 'h'
    GET_STEP_PERIOD         = 'i'
    GET_POS                 = 'j'
    GET_STATUS              = 'f'
    GET_HIGH_SPEED_RATIO    = 'g'
    GET_1X_TRACKING_PERIOD  = 'D'
    GET_TELE_AXIS_POS       = 'd'
    GET_MOTOR_BOARD_VERSION = 'e'
    SET_EXTEND              = 'W'
    GET_EXTEND              = 'q'

def normalize_degree(input):
    output = input + 0.0
    if input > 180:
        output = input - 360
    elif input <= -180:
        output = input + 360
    return output

class SkyWathcher:
    serialport = None
    portname = None
    baudrate = 115200
    timetout = 0.05
    CH1_CPR = 0
    CH2_CPR = 0

    CH1_POS = 0.0
    CH2_POS = 0.0

    CH1_TARGET_POS = 0.0
    CH2_TARGET_POS = 0.0

    TIMER_FREQ = 0

    def __init__(self, _portname, _baudrate, _timeout):
        self.portname = _portname
        self.baudrate = _baudrate
        self.timeout = _timeout
        self.serialport = serial.Serial(self.portname, self.baudrate, timeout=self.timeout)
        self.get_cpr()
        self.get_timer_freq()
        print(self.CH1_CPR)
        print(self.CH2_CPR)
        print(self.TIMER_FREQ)
    def __del__(self):
        if not self.serialport == None:
            self.stop_motion()
            self.serialport.close()

    def hex2skywatch(self, hexstring ):
        db_string = ''
        if len(hexstring) == 8:
            db_string = hexstring[6] + hexstring[7] + hexstring[4] + hexstring[5] + hexstring[2] + hexstring[3]
        elif len(hexstring) == 6:
            db_string = hexstring[4] + hexstring[5] + hexstring[2] + hexstring[3]  
        elif len(hexstring) == 4:
            db_string = hexstring[2] + hexstring[3]  
        return db_string

    def get_reply(self, is_success, reply_str):
        hex_char = '0x'
        if is_success:
            if len(reply_str) == 8:     #Type B
                hex_char = hex_char + reply_str[5] + reply_str[6] + reply_str[3] + reply_str[4] + reply_str[1] + reply_str[2]
            elif len(reply_str) == 6:   #Type C
                hex_char = hex_char + reply_str[3] + reply_str[4] + reply_str[1] + reply_str[2]
            elif len(reply_str) == 4:   #Type D
                hex_char = hex_char + reply_str[1] + reply_str[2]
            elif len(reply_str) == 5:   #Type E
                hex_char = hex_char + reply_str[3] + reply_str[1] + reply_str[2]
            elif len(reply_str) == 2:   #Type A
                hex_char = hex_char
        else:
            hex_char = ''
            print("error relpy::" + reply_str)
        return hex_char

    def send_cmd_and_get_reply(self, op_header, channel, value):
        str_cmd = Comma.START_CHAR.value + op_header + channel + value + '\r'
        while True:
            self.serialport.write(str_cmd.encode('utf-8'))
            reply_bytes = self.serialport.read(10)
            reply_str = reply_bytes.decode(encoding="utf-8")
            len_reply = len(reply_str)

            if len_reply > 0:
                if len_reply == 1:
                    print(str_cmd)
                    self.get_reply(False,reply_str)
                    time.sleep(0.1)
                    continue
                if reply_str[0] == Comma.ANSWER_CHAR.value \
                    and reply_str[len_reply - 1] == Comma.END_CHAR.value:
                    return self.get_reply(True,reply_str)
                elif reply_str[0] == Comma.ERROR_CHAR.value \
                    and reply_str[len_reply - 1] == Comma.END_CHAR.value:
                    print(str_cmd)
                    self.get_reply(False,reply_str)
                    time.sleep(0.1)
                    continue
            
            print(str_cmd)
            self.get_reply(False,reply_str)
            time.sleep(0.1)

    def get_cpr(self,):
        ch1_cpr_str = self.send_cmd_and_get_reply(OpCode.GET_CPR.value, Axis.CH1_RA_AZ.value, '')
        if not ch1_cpr_str == '':
            self.CH1_CPR = int(ch1_cpr_str, 16)

        ch2_cpr_str = self.send_cmd_and_get_reply(OpCode.GET_CPR.value, Axis.CH2_DEC_ALT.value, '')
        if not ch2_cpr_str == '':
            self.CH2_CPR = int(ch2_cpr_str, 16)

    def get_pos(self,):
        ch1_pos_str = self.send_cmd_and_get_reply(OpCode.GET_POS.value, Axis.CH1_RA_AZ.value, '')
        if not ch1_pos_str == '':
            self.CH1_POS = (int(ch1_pos_str, 16) - int('0x800000', 16)) * 360.0 / self.CH1_CPR
            #self.CH1_POS = self.CH1_POS * (-1.0)

        ch2_pos_str = self.send_cmd_and_get_reply(OpCode.GET_POS.value, Axis.CH2_DEC_ALT.value, '')
        if not ch2_pos_str == '':
            self.CH2_POS = (int(ch2_pos_str, 16) - int('0x800000', 16)) * 360.0 / self.CH2_CPR

    def get_pos_by_channel(self, channel):
        ch1_pos_str = self.send_cmd_and_get_reply(OpCode.GET_POS.value, channel.value, '')
        if not ch1_pos_str == '':
            channel_pos = (int(ch1_pos_str, 16) - int('0x800000', 16)) * 360.0 / self.CH1_CPR
            return normalize_degree(channel_pos)

    def get_target_pos(self,):
        ch1_pos_str = self.send_cmd_and_get_reply(OpCode.GET_GOTO_TARGET_POS.value, Axis.CH1_RA_AZ.value, '')
        if not ch1_pos_str == '':
            self.CH1_TARGET_POS = (int(ch1_pos_str, 16) - int('0x800000', 16)) * 360.0 / self.CH1_CPR
            self.CH1_TARGET_POS = normalize_degree(self.CH1_TARGET_POS)

        ch1_pos_str = self.send_cmd_and_get_reply(OpCode.GET_GOTO_TARGET_POS.value, Axis.CH2_DEC_ALT.value, '')
        if not ch1_pos_str == '':
            self.CH2_TARGET_POS = (int(ch1_pos_str, 16) - int('0x800000', 16)) * 360.0 / self.CH2_CPR
            self.CH2_TARGET_POS = normalize_degree(self.CH2_TARGET_POS)

    def set_motion_mode(self, motion_mode):
        # self.send_cmd_and_get_reply(OpCode.SET_MOTION_MODE.value, Axis.CH1_RA_AZ.value, motion_mode)
        # self.send_cmd_and_get_reply(OpCode.SET_MOTION_MODE.value, Axis.CH2_DEC_ALT.value, motion_mode)
        self.set_motion_mode_by_channel(Axis.CH1_RA_AZ, motion_mode)
        self.set_motion_mode_by_channel(Axis.CH2_DEC_ALT, motion_mode)
    
    def set_motion_mode_by_channel(self, channel, motion_mode):
        self.send_cmd_and_get_reply(OpCode.SET_MOTION_MODE.value, channel.value, motion_mode)

    def set_target_pos(self, azimuth, altitude):
        self.set_target_pos_by_channel(Axis.CH1_RA_AZ, azimuth)
        self.set_target_pos_by_channel(Axis.CH2_DEC_ALT, altitude)

    def set_target_pos_by_channel(self, channel, value):
        # todo 
        value = normalize_degree(value)
        set_pos_ch1 = int(value * self.CH1_CPR / 360.0 + int('0x800000', 16))
        set_pos_ch1_hex_str = self.hex2skywatch(hex(set_pos_ch1)).upper()
        self.send_cmd_and_get_reply(OpCode.SET_GOTO_TARGET.value, channel.value, set_pos_ch1_hex_str)
    
    def start_motion(self,):
        self.start_motion_by_channel(Axis.CH1_RA_AZ)
        self.start_motion_by_channel(Axis.CH2_DEC_ALT)

    def start_motion_by_channel(self, channel):
        self.send_cmd_and_get_reply(OpCode.START_MOTION.value, channel.value, '')

    def stop_motion(self,):
        self.stop_motion_by_channel(Axis.CH1_RA_AZ)
        self.stop_motion_by_channel(Axis.CH2_DEC_ALT)

    def stop_motion_by_channel(self, channel):
        self.send_cmd_and_get_reply(OpCode.STOP_MOTION.value, channel.value, '')

    def instant_stop(self,):
        self.instant_stop_by_channel(OpCode.INSTANT_STOP.value, Axis.CH1_RA_AZ)
        self.instant_stop_by_channel(OpCode.INSTANT_STOP.value, Axis.CH2_DEC_ALT)

    def instant_stop_by_channel(self, channel):
        self.send_cmd_and_get_reply(OpCode.INSTANT_STOP.value, channel.value, '')

    def set_aux_switch(self, _channel, _switch):
        self.send_cmd_and_get_reply(OpCode.SET_AUX_SWITCH.value, _channel, _switch)

    def set_autoguide_speed(self, _channel, _speed):
        self.send_cmd_and_get_reply(OpCode.SET_AUTOGUIDE_SPEED.value, _channel, _speed)

    def set_polarscope_led(self, _brightness):
        self.send_cmd_and_get_reply(OpCode.SET_LED_BRIGHTNESS.value, Axis.CH1_RA_AZ.value, _brightness)
        self.send_cmd_and_get_reply(OpCode.SET_LED_BRIGHTNESS.value, Axis.CH2_DEC_ALT.value, _brightness)

    def get_timer_freq(self, ):
        ch1_str = self.send_cmd_and_get_reply(OpCode.GET_TIMER_FREQ.value, '1', '')
        if not ch1_str == '':
            self.TIMER_FREQ = int(ch1_str, 16)

    def get_speed(self, ):
        ch1_str = self.send_cmd_and_get_reply(OpCode.GET_STEP_PERIOD.value, Axis.CH1_RA_AZ.value, '')
        #print(ch1_str)
        if not ch1_str == '':
            speed_ch1 =  self.TIMER_FREQ * 360.0 / (self.CH1_CPR * int(ch1_str, 16))

        ch1_str = self.send_cmd_and_get_reply(OpCode.GET_STEP_PERIOD.value, Axis.CH2_DEC_ALT.value, '')
        if not ch1_str == '':
            speed_ch2 =  self.TIMER_FREQ * 360.0 / (self.CH2_CPR * int(ch1_str, 16))
        
        #print("speed ch1=%f\tch2=%f" % (speed_ch1, speed_ch2))

    def set_speed(self, speed_ch1, speed_ch2):
        self.set_speed_by_channel(Axis.CH1_RA_AZ, speed_ch1)
        self.set_speed_by_channel(Axis.CH2_DEC_ALT, speed_ch2)

    def set_speed_by_channel(self, channel, speed_ch1):

        if abs(speed_ch1) < 1e-5:
            speed_ch1 = speed_ch1 + 1e-4
            self.send_cmd_and_get_reply(OpCode.STOP_MOTION.value, channel.value, '')

        t1_ch1 = int(self.TIMER_FREQ * 360.0 / (self.CH1_CPR * speed_ch1))
        t1_ch1_hex = '0x%.6X' % t1_ch1
        set_speed_ch1_hex_str = self.hex2skywatch( t1_ch1_hex ).upper()
        self.send_cmd_and_get_reply(OpCode.SET_STEP_PERIOD.value, channel.value, set_speed_ch1_hex_str)
        

    def get_status_by_channel(self, channel):
        status_ch1_reply = self.send_cmd_and_get_reply(OpCode.GET_STATUS.value, channel.value, '')
        
        status_ch1_dec = int(status_ch1_reply, 16)
        status_ch1_bin = "{:0>12b}".format(status_ch1_dec)

        status = dict()
        status['IS_LEVEL_SWITCH'] = status_ch1_bin[2]
        status['IS_INIT_DONE'] = status_ch1_bin[3]
        status['IS_SPEED_FAST'] = status_ch1_bin[5]
        status['IS_DIRECTION_CCW'] = status_ch1_bin[6]
        status['IS_MODE_TRACKING'] = status_ch1_bin[7]
        status['IS_BLOCKED'] = status_ch1_bin[10]
        status['IS_RUNNING'] = status_ch1_bin[11]

        return status
        
    def waiting_until_stop(self, _channel):
        status = self.get_status_by_channel(_channel)
        while status['IS_RUNNING'] == '1':
            time.sleep(0.1)
            status = self.get_status_by_channel(_channel)

    def start_tracking(self,):
        self.set_motion_mode(MotionMode.TRACKING_SLOW_MEDIUM.value + MotionModeDirection.CW_NORTH_NORMAL.value)
        self.stop_motion_by_channel(Axis.CH2_DEC_ALT)
        self.start_motion_by_channel(Axis.CH1_RA_AZ)

    def stop_tracking(self,):
        self.stop_motion()

    def find_home(self,):
        
        mount.set_motion_mode(MotionMode.GOTO_FAST_MEDIUM.value + MotionModeDirection.CW_NORTH_NORMAL.value)
        mount.start_motion()
        mount.set_target_pos(0,0)
# serial_port = serial.Serial('COM5', 115200, timeout=0.05)

# is_success, reply_str = send_cmd_and_get_reply(serial_port, OpCode.GET_CPR.value, Axis.CH1_RA_AZ.value, '')

# serial_port.close()


if __name__ == '__main__':
    mount = SkyWathcher('COM9', 9600, 0.05)
    #mount.find_home()
    #mount.start_tracking()
    mount.set_motion_mode(MotionMode.GOTO_FAST_MEDIUM.value + MotionModeDirection.CW_NORTH_NORMAL.value)
    #mount.set_speed(0.01,0)
    mount.set_target_pos(0,0)
    mount.start_motion()
    time.sleep(5)
    mount.stop_motion()
    time.sleep(1)
    while True:
        #mount.get_speed()
        #mount.set_speed(0,0)
        #mount.start_motion()
        #mount.start_motion_by_channel(Axis.CH1_RA_AZ)
        mount.get_pos()
        #mount.get_target_pos()
        print(f"{mount.CH1_POS}\t{mount.CH2_POS}")
        #azimuth, altitude = equatorial_axes_to_az_alt(mount.CH1_POS, mount.CH2_POS)

        #print(f"az-alt:{azimuth}\t{altitude}")
    mount.get_status_by_channel(Axis.CH2_DEC_ALT)
    #time.sleep(5)
    mount.set_motion_mode(MotionMode.TRACKING_SLOW_MEDIUM.value + MotionModeDirection.CW_NORTH_NORMAL.value)
    mount.set_speed(3,3)
    mount.get_pos()
    print(f"{mount.CH1_POS}\t{mount.CH2_POS}")
    mount.start_motion()
    mount.set_speed(5,5)
    time.sleep(1)
    mount.get_pos()
    print(f"{mount.CH1_POS}\t{mount.CH2_POS}")
    mount.set_speed(3,3)
    time.sleep(1)
    mount.get_pos()
    print(f"{mount.CH1_POS}\t{mount.CH2_POS}")
    mount.set_speed(1,1)
    time.sleep(1)
    mount.get_pos()
    print(f"{mount.CH1_POS}\t{mount.CH2_POS}")
    mount.set_speed(3,3)
    time.sleep(1)
    mount.get_pos()
    print(f"{mount.CH1_POS}\t{mount.CH2_POS}")
    mount.set_speed(5,5)
    time.sleep(5)
    mount.get_pos()
    print(f"{mount.CH1_POS}\t{mount.CH2_POS}")
    mount.stop_motion()
    time.sleep(2)
    mount.get_pos()
    print(f"{mount.CH1_POS}\t{mount.CH2_POS}")
    time.sleep(10000)
    # time.sleep(5)
    mount.set_motion_mode(MotionMode.GOTO_SLOW_MEDIUM.value + MotionModeDirection.CW_NORTH_NORMAL.value)
    mount.set_target_pos(0,0)
    mount.start_motion()
    target_altitude = 0
    while True:
        mount.get_target_pos() 
        mount.get_pos()
        mount.get_speed()
        status = mount.get_status_by_channel(Axis.CH2_DEC_ALT)
        print("diff ch1 = %f\tch2 = %f" % (mount.CH1_TARGET_POS - mount.CH1_POS, mount.CH2_TARGET_POS - mount.CH2_POS))
        if status['IS_RUNNING'] == '0':
            target_altitude = target_altitude + 1
            mount.set_motion_mode(MotionMode.GOTO_SLOW_MEDIUM.value + MotionModeDirection.CW_NORTH_NORMAL.value)
            mount.set_target_pos(0,target_altitude)
            mount.start_motion()
        time.sleep(0.1)