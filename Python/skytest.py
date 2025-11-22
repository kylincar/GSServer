from AM5Controller  import AM5Controller
from PIDController import  PIDController
from TimeManager import TimeManager
from VitualAxis import VitualAxis, list_txt_files, select_file

import datetime
import time
import sys
import os

#controller = AM5Controller()
controller = AM5Controller(driver_name="ASCOM.GS.Sky.Telescope")
controller.connect()
#controller.telescope.SiteElevation = 8.0 #meters
lat, lon, alt = controller.get_location()
print(alt)
print(controller.can_pulse_guide())
print(controller.can_set_guide_rates())

#controller.telescope.GuideRateRA = 0.5 
#controller.telescope.GuideRateDec = 0.5

#参数说明：
# Direction (int): 方向，使用 ASCOM 定义的枚举值：
# 0: guideNorth (北)
# 1: guideSouth (南)
# 2: guideEast (东)
# 3: guideWest (西)
# Duration (int): 脉冲持续时间，单位为毫秒 (ms)。

while True:
    guideSouth = 0
    guideWest = 3
    duration_ms = 500
    controller.telescope.PulseGuide(guideSouth, duration_ms)
    time.sleep(1)


simulation = True
ra_dec_mode = True #赤道仪模式，否则为经纬仪模式

try:
    # 指定目录
    directory = './output/'
    
    # 列出所有 .txt 文件
    txt_files = list_txt_files(directory)
    if not txt_files:
        sys.exit(1)
    
    # 用户选择文件
    selected_file = select_file(txt_files)
    
    # 返回文件路径
    file_path = os.path.join(directory, selected_file)
    print(f"您选择的文件路径是: {file_path}")
    
    time_manager = TimeManager(file_path)
    
    controller.telescope.Tracking = True    
    # 初始化 PID 参数（这些参数可能需要根据具体系统进行调整）
    kp = 1.5
    ki = 0.3
    kd = 0#1e-2

    # 初始化 PID 控制器
    pid_altitude_dec = PIDController(kp, ki, kd)
    pid_azimuth_ra = PIDController(kp, ki, kd)

    
    
    # while controller.is_at_home() == False:
    #     controller.telescope.FindHome()
    #     print("waitting for the telescope to find home position")
    #     time.sleep(1)
    
    if ra_dec_mode:
        init_ra, init_dec = time_manager.get_init_radec()
        #controller.slew_to_coordinates(init_ra, init_dec)
        # 初始目标角度
        pid_altitude_dec.set_target(init_dec)
        pid_azimuth_ra.set_target(init_ra*15)



    start_time = datetime.datetime.now()
    while True:    
        current_time = datetime.datetime.now() #+ datetime.timedelta(seconds=100)
        if simulation:
            #use related time
            relate_time = current_time.timestamp() - start_time.timestamp()
            c_related_time, c_utc_time, target_azimuth, target_altitude, target_ra, target_dec, c_distance, c_velocity = time_manager.get_pos_by_related_time(relate_time)
            if c_related_time == None:
                break
        else :
            if current_time.timestamp() < time_manager.utc_times_list[0]:
                print(f"waitting for the time rise, time left:{round(time_manager.utc_times_list[0] - current_time.timestamp())}s")
                time.sleep(1)
                continue
            
            c_related_time, c_utc_time, target_azimuth, target_altitude, target_ra, target_dec, c_distance, c_velocity = time_manager.get_pos_by_utc_time(current_time.timestamp())
            if c_related_time == None:
                break

        pid_altitude_dec.set_target(target_dec)
        pid_azimuth_ra.set_target(target_ra*15)
        
        declination = controller.get_declination()
        custom_dec_rate, real_dec = pid_altitude_dec.update(declination, -1)
        
        right_ascension = controller.get_right_ascension()
        custom_ra_rate, real_ra = pid_azimuth_ra.update(right_ascension*15, 1)

        # 使用 GuideRate 调整跟踪速率（假设望远镜驱动支持）（例如，每秒移动 0.0041780741 度在RA）
        controller.telescope.GuideRateRightAscension = custom_ra_rate  # 转换为度/秒
        controller.telescope.GuideRateDeclination = custom_dec_rate # 转换为度/秒
        #controller.telescope.MoveAxis(0, custom_ra_rate)  # 0 表示 RA ，fangweijiao 
        #controller.telescope.MoveAxis(1, custom_dec_rate)  # 0 表示 RA 轴
        #controller.set_right_ascension_rate(custom_ra_rate)
        print(f"target_ra:{target_ra:.4f},real_ra:{right_ascension:.4f},ra_rate:{custom_ra_rate}")
        #print(f"target_dec:{target_dec:.4f},real_dec:{declination:.4f},dec_rate:{custom_dec_rate}")

        #print(f"target_ra: {target_ra:.2f} real_dec: {real_dec:.2f} real_ra: {real_ra:.4f} error_dec:{pid_altitude_dec.limit_altitude_error(real_dec-target_dec):.4f} error_ra:{pid_azimuth_ra.limit_azimuth_error(real_ra-target_ra):.4f}")
        
        time.sleep(0.2)
finally:
    # 确保断开与望远镜的连接
    controller.abort_slew()
    #controller.telescope.AbortSlew()
    controller.telescope.Tracking = False    
    #controller.telescope.Connected = False