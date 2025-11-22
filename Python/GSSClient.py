import win32com.client
from astropy.coordinates import SkyCoord, EarthLocation, AltAz, Angle
from astropy.time import Time
import astropy.units as u
import datetime
import time
import json


class AM5Controller:
    def __init__(self, driver_name="ASCOM.GS.Sky.Telescope"):
        self.driver_name = driver_name
        self.telescope = None
        self.is_connected = False

    def connect(self):
        try:
            self.telescope = win32com.client.Dispatch(self.driver_name)
            self.telescope.Connected = True
            self.is_connected = True
        except Exception as e:
            print("连接赤道仪时出现错误：", e)

    def disconnect(self):
        if self.is_connected:
            try:
                self.telescope.Connected = False
                self.is_connected = False
            except Exception as e:
                print("断开连接时出现错误：", e)
        else:
            print("尚未连接赤道仪，无需断开")

    def get_location(self):
        if self.is_connected:
            observer_lat = float(self.telescope.SiteLatitude)
            observer_lon = float(self.telescope.SiteLongitude)
            observer_alt = float(self.telescope.SiteElevation)
            return observer_lat, observer_lon, observer_alt
        
    def get_description(self):
        if self.is_connected:
            return self.telescope.Description
        print("请先连接赤道仪再获取描述信息")
        return None

    def get_driver_info(self):
        if self.is_connected:
            return self.telescope.DriverInfo
        print("请先连接赤道仪再获取驱动信息")
        return None

    def get_driver_version(self):
        if self.is_connected:
            return self.telescope.DriverVersion
        print("请先连接赤道仪再获取驱动版本号")
        return None

    def get_name(self):
        if self.is_connected:
            return self.telescope.Name
        print("请先连接赤道仪再获取设备名称")
        return None

    def is_at_home(self):
        if self.is_connected:
            return self.telescope.AtHome
        print("请先连接赤道仪再检查是否归位")
        return None

    def get_axis_rates(self):
        if self.is_connected:
            return self.telescope.AxisRates
        print("请先连接赤道仪再获取轴速率")
        return None

    def can_pulse_guide(self):
        if self.is_connected:
            return self.telescope.CanPulseGuide
        print("请先连接赤道仪再检查是否支持脉冲导星")
        return None

    def can_set_declination_rate(self):
        if self.is_connected:
            return self.telescope.CanSetDeclinationRate
        print("请先连接赤道仪再检查是否能设置赤纬轴速率")
        return None

    def can_set_guide_rates(self):
        if self.is_connected:
            return self.telescope.CanSetGuideRates
        print("请先连接赤道仪再检查是否能设置导星速率")
        return None

    def can_set_right_ascension_rate(self):
        if self.is_connected:
            return self.telescope.CanSetRightAscensionRate
        print("请先连接赤道仪再检查是否能设置赤经轴速率")
        return None

    def get_declination(self):
        if self.is_connected:
            return self.telescope.Declination
        print("请先连接赤道仪再获取赤纬位置")
        return None

    def get_declination_rate(self):
        if self.is_connected:
            return self.telescope.DeclinationRate
        print("请先连接赤道仪再获取赤纬轴速率")
        return None

    def set_declination_rate(self, rate):
        if self.is_connected:
            try:
                self.telescope.DeclinationRate = rate
                print("已成功设置赤纬轴运动速率")
            except Exception as e:
                print("设置赤纬轴运动速率时出现错误：", e)
        else:
            print("请先连接赤道仪再设置赤纬轴速率")

    def get_equatorial_system(self):
        if self.is_connected:
            return self.telescope.EquatorialSystem
        print("请先连接赤道仪再获取赤道坐标系类型")
        return None

    def get_right_ascension(self):
        if self.is_connected:
            return self.telescope.RightAscension
        print("请先连接赤道仪再获取赤经位置")
        return None

    def get_right_ascension_rate(self):
        if self.is_connected:
            return self.telescope.RightAscensionRate
        print("请先连接赤道仪再获取赤经轴速率")
        return None

    def set_right_ascension_rate(self, rate):
        if self.is_connected:
            try:
                self.telescope.RightAscensionRate = rate
                print("已成功设置赤经轴运动速率")
            except Exception as e:
                print("设置赤经轴运动速率时出现错误：", e)
        else:
            print("请先连接赤道仪再设置赤经轴速率")

    def get_sidereal_time(self):
        if self.is_connected:
            return self.telescope.SiderealTime
        print("请先连接赤道仪再获取恒星时")
        return None

    def abort_slew(self):
        if self.is_connected:
            try:
                self.telescope.AbortSlew()
                print("已中止赤道仪的快速移动操作")
            except Exception as e:
                print("中止移动操作时出现错误：", e)
        else:
            print("请先连接赤道仪再中止移动操作")

    def can_move_axis(self, axis_identifier):
        if self.is_connected:
            return self.telescope.CanMoveAxis(axis_identifier)
        print("请先连接赤道仪再检查轴是否可移动")
        return None

    def can_slew(self):
        if self.is_connected:
            return self.telescope.CanSlew
        print("请先连接赤道仪再检查是否能快速移动")
        return None

    def can_sync(self):
        if self.is_connected:
            return self.telescope.CanSync
        print("请先连接赤道仪再检查是否支持同步操作")
        return None

    def find_home(self):
        if self.is_connected:
            try:
                self.telescope.FindHome()
                print("已启动赤道仪的归位操作")
            except Exception as e:
                print("启动归位操作时出现错误：", e)
        else:
            print("请先连接赤道仪再启动归位操作")

    def move_axis(self, axis_identifier, direction):
        if self.is_connected:
            try:
                self.telescope.MoveAxis(axis_identifier, direction)
                print("已尝试移动赤道仪指定轴")
            except Exception as e:
                print("移动轴操作时出现错误：", e)
        else:
            print("请先连接赤道仪再移动轴")

    def slew_to_coordinates(self, ra_hours, dec_degrees):
        if self.is_connected:
            try:
                self.telescope.SlewToCoordinates(ra_hours, dec_degrees)
                print("赤道仪正在移动到指定坐标位置（同步方法）")
            except Exception as e:
                print("移动到坐标位置时出现错误：", e)
        else:
            print("请先连接赤道仪再移动到坐标位置")

    def slew_to_coordinates_async(self, ra_hours, dec_degrees):
        if self.is_connected:
            try:
                self.telescope.SlewToCoordinatesAsync(ra_hours, dec_degrees)
                print("赤道仪正在移动到指定坐标位置（异步方法）")
            except Exception as e:
                print("启动异步移动到坐标位置时出现错误：", e)
        else:
            print("请先连接赤道仪再启动异步移动到坐标位置")

    def sync_to_coordinates(self, ra_hours, dec_degrees):
        if self.is_connected:
            try:
                self.telescope.SyncToCoordinates(ra_hours, dec_degrees)
                print("已尝试将赤道仪位置与指定坐标同步")
            except Exception as e:
                print("同步坐标操作时出现错误：", e)
        else:
            print("请先连接赤道仪再进行坐标同步")

    def get_altitude(self):
        if self.is_connected:
            return self.telescope.Altitude
        print("请先连接赤道仪再获取高度角")
        return None

    def get_azimuth(self):
        if self.is_connected:
            return self.telescope.Azimuth
        print("请先连接赤道仪再获取方位角")
        return None

    def is_pulse_guiding(self):
        if self.is_connected:
            return self.telescope.IsPulseGuiding
        print("请先连接赤道仪再检查是否正在脉冲导星")
        return None
    def set_altitude(self, altitude):
        if self.is_connected:
            try:
                self.telescope.SetAltitude(altitude)
                print("已成功设置高度角")
            except Exception as e:
                print("设置高度角时出现错误：", e)
        else:
            print("请先连接赤道仪再设置高度角")

    def set_azimuth(self, azimuth):
        if self.is_connected:
            try:
                self.telescope.SetAzimuth(azimuth)
                print("已成功设置方位角")
            except Exception as e:
                print("设置方位角时出现错误：", e)
        else:
            print("请先连接赤道仪再设置方位角")

    #根据赤道仪的经纬度
    def slew_to_altaz(self, altitude, azimuth):

        observer_lat = float(self.telescope.SiteLatitude)  # observer's latitude in degrees
        observer_lon = float(self.telescope.SiteLongitude)  # observer's longitude in degrees
        observer_alt = float(self.telescope.SiteElevation)
        observation_time = (datetime.datetime.now() - datetime.timedelta(hours=8)).isoformat()  # observation time in ISO format
        # observation_time = self.telescope.UTCDate

        # Create a Time object for the observation time
        obstime = Time(observation_time)

        location = EarthLocation(lon=observer_lon, lat=observer_lat, height=observer_alt)

        # 定义高度角和方位角（示例值，你可按实际情况修改）
        #如何从角度浮点数创建角度对象

        altitude_obj = Angle(altitude, unit=u.deg)
        azimuth_obj = Angle(azimuth, unit=u.deg)
        altaz = AltAz(alt=altitude_obj, az=azimuth_obj, obstime=obstime, location=location)


        # 创建一个 SkyCoord 对象来转换坐标
        coord = SkyCoord(altaz)

        # 将高度角和方位角坐标转换为赤经赤纬坐标
        icrs_coord = coord.transform_to('icrs')
        
        # Convert RA from degrees to hours
        ra = icrs_coord.ra.hour
        dec = icrs_coord.dec.deg
        #print(f"Right Ascension: {ra}, Declination: {dec}")
        # self.move_axis(0, 1)
        # self.slew_to_coordinates(ra, dec)
        # 将目标坐标设置给望远镜
        self.telescope.TargetRightAscension = ra
        self.telescope.TargetDeclination = dec

        # 移动望远镜到目标坐标
        self.telescope.SlewToTarget()
        return

def test_telescope_functions():
    controller = AM5Controller()
    # 连接赤道仪测试
    controller.connect()
    assert controller.is_connected, "赤道仪连接失败"

    # 获取设备描述信息测试
    description = controller.get_description()
    assert isinstance(description, str), "获取设备描述信息类型错误"
    print(f"设备描述信息: {description}")

    # 获取驱动信息测试
    driver_info = controller.get_driver_info()
    assert isinstance(driver_info, str), "获取驱动信息类型错误"
    print(f"驱动信息: {driver_info}")

    # 获取驱动版本号测试
    driver_version = controller.get_driver_version()
    assert isinstance(driver_version, str) or isinstance(driver_version, int), "获取驱动版本号类型错误"
    print(f"驱动版本号: {driver_version}")

    # 获取设备名称测试
    name = controller.get_name()
    assert isinstance(name, str), "获取设备名称类型错误"
    print(f"设备名称: {name}")

    # 获取赤纬位置测试
    declination = controller.get_declination()
    assert isinstance(declination, (float, int)), "获取赤纬位置类型错误"
    print(f"当前赤纬位置: {declination}")

    # 获取赤经位置测试
    right_ascension = controller.get_right_ascension()
    assert isinstance(right_ascension, (float, int)), "获取赤经位置类型错误"
    print(f"当前赤经位置: {right_ascension}")

    support_actions = controller.telescope.SupportedActions
    print(f"当前赤经位置: {support_actions}")

    # contruct json params struct
    #park_params = {"Name":"TestParkName","X":90.0,"Y":90.0}
    #result = controller.telescope.Action("telescope:setparkposition", json.dumps(park_params))
    #print(result)
    slew_params = {"1":-0.1,"2":-0.2}
    result = controller.telescope.Action("telescope:axis_slew", json.dumps(slew_params))
    print(result)
    time.sleep(100000)
    controller.move_axis(0, 0) #赤经，RA
    controller.move_axis(1, 3.5) #赤纬，DEC

    count = 0
    while count < 7:
        time.sleep(1)
        controller.move_axis(1, 3.5 - count*0.4) #赤纬，DEC
        #controller.set_right_ascension_rate()
        count = count + 1
        print(count)
        # 获取赤纬位置测试
        declination = controller.get_declination()
        assert isinstance(declination, (float, int)), "获取赤纬位置类型错误"
        print(f"当前赤纬位置: {declination}")

        # 获取赤经位置测试
        right_ascension = controller.get_right_ascension()
        assert isinstance(right_ascension, (float, int)), "获取赤经位置类型错误"
        print(f"当前赤经位置: {right_ascension}")
    # 中止快速移动操作测试
    controller.abort_slew()

    # 断开赤道仪连接测试
    controller.disconnect()
    assert not controller.is_connected, "赤道仪断开连接失败"
if __name__ == "__main__":
    test_telescope_functions()