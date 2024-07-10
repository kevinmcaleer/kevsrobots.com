class Robot:
    __battery_max_voltage = 4.2
    __battery_min_voltage = 3.0
    _battery_voltage = 3.7
    __serial_number = "123456"

    def _battery_level(self):
        battery_percentage = ((self._battery_voltage - self.__battery_min_voltage) / (self.__battery_max_voltage - self.__battery_min_voltage)) * 100
        return int(battery_percentage)
    
    def battery(self):
        return f"{self._battery_level()}%"
    
r = Robot()
print(r.battery())

print(r._battery_voltage)
