import time

class Lander:
    def __init__(self):
        self.accelerometerX = 60 # RPH (rotations per hour)
        self.accelerometerY = 0  # RPH
        self.accelerometerZ = 0  # RPH
        self.temperatureEngine = 100   # Degrees Fahrenheit
        self.temperatureVehicle = 2700 # Degrees Fahrenheit
        self.gyroscopeX = 60.0     # Degrees
        self.gyroscopeY = 15.0     # Degrees
        self.gyroscopeZ = 10.0     # Degrees
        self.rollEngineOne = Engine() # Rotation Engine: Percentage of Thrust
        self.rollEngineTwo = Engine() # Rotation Engine: Percentage of Thrust
        self.rollEngineThree = Engine() # Rotation Engine: Percentage of Thrust
        self.axialThrustOne = Engine()  # Descent Engine: Percentage of Thrust
        self.axialThrustTwo = Engine()  # Descent Engine: Percentage of Thrust
        self.axialThrustThree = Engine()# Descent Engine: Percentage of Thrust
        self.parachute = False          # If the parachute is deployed
        self.touchDown = False          # If the vehicle has landed
        self.bank_angle = 0  # Current bank angle in degrees
        self.cg_offset = 0   # CG offset for stability

class Engine:
    def __init__(self):
        self.thrust = 0.0

    def setThrust(self, thrust):
        self.thrust = thrust

    def getEngineData(self, lander):
        pass

class DataLog:
    def logData(self, lander):
        # Log data in a text file and send to console output simulator
        pass

class Simulator:
    def display(self, lander):
        # Display data on the simulator
        pass

class Control:
    def __init__(self):
        self.lander = Lander()
        self.count = 0
        self.initialize()

    def initialize(self):
        self.lander.accelerometerX = 60
        self.lander.accelerometerY = 0
        self.lander.accelerometerZ = 0        #Accelerometer(z-axis): RPH
        self.lander.dopplerRadar = 10750      #Velocity: 10,750 miles per hour
        self.lander.altimeter = 28.1          # 28.1 miles above surface
        self.lander.temperatureEngine = 100   #Temperature of Engines: Degrees Fahrenheit
        self.lander.temperatureVehicle = 2700 #Temperature of Vehicle: Degrees Fahrenheit
        self.lander.gyroscopeX = 60.0         #X-Axis Gyroscope: Degrees
        self.lander.gyroscopeY = 15.0
        self.lander.gyroscopeZ = 10.0
        self.lander.rollEngineOne.setThrust(0.0) #Rotation Engine: Percentage of Thrust
        self.lander.rollEngineTwo.setThrust(0.0)
        self.lander.rollEngineThree.setThrust(0.0)
        self.lander.axialThrustOne.setThrust(0.0)
        self.lander.axialThrustTwo.setThrust(0.0)
        self.lander.axialThrustThree.setThrust(0.0)
        self.lander.parachute = False           #If the parachute is deployed
        self.lander.touchDown = False           #If the vehicle has landed 

    """
This program will simulate a Mars Lander entering the Mars Atmosphere"
		"   The lander enters the atmosphere at an altitude of 28.1 miles and is "
		"   travelling at 10,750 miles per hour. The lander will read sensor data "
		"   and will activate a parachute and engines as needed to control the descent"
		"   The following sensors and engines are used:"
		"   SENSORS:"
		"      - 3 Accelerometers: one for an X-Axis, Y-Axis, and Z-Axis."
		"      - 1 Altimeter: reads the altitude."
		"      - 1 Doppler Radar: reads the descent velocity."
		"      - 3 Gyroscopes: tells orientation of X-Axis, Y-Axis, and Z-Axis."
		"      - 2 Temperature Sensors: reads temperature of the engine and vehicle."
		"      - 1 Touch-Down Sensor: reads if vehicle has landed."
		"   ENGINES:"
		"      - 3 Roll Engines: controls the X-Axis and Z-Axis spin."
		"      - 3 Axial Engines: controls descent speed, Y-Axis and Z-Axis spin."
        """


    def landMarsLander(self):
        dataLog = DataLog()
        engine = Engine()
        simulator = Simulator()

        while not self.lander.touchDown:
            dataLog.logData(self.lander)    # logs data in txt file and sends to console output simulator
            simulator.display(self.lander)
            self.calcSensorData(self.lander) # Calls for updated data for the next position
            dataLog.logData(self.lander)     # Sends lander data to be logged in a file and sent to the simulator
            simulator.display(self.lander)
            engine.getEngineData(self.lander)  # Sends lander data for engines to react, and update lander object data
            dataLog.logData(self.lander)
            simulator.display(self.lander)
            self.calcSensorData(self.lander)
            time.sleep(1)

    def calcSensorData(self, vehicle):
        landerWeight = 437
        self.calcRotationPerHour(vehicle, landerWeight)
        self.calcAxialData(vehicle, landerWeight)
        self.calcVelocity(vehicle, landerWeight)
        self.calcAltitude(vehicle)
        self.calcTemperature(vehicle)
        
        def adjust_bank_angle(self, current_angle, lateral_acceleration):
            # Adjust bank angle based on lateral acceleration
            max_bank_angle = 30  # Maximum allowed bank angle
            bank_angle_rate = 2  # Rate of change of bank angle
            if lateral_acceleration > 0:
                current_angle = min(current_angle + bank_angle_rate, max_bank_angle)
            elif lateral_acceleration < 0:
                current_angle = max(current_angle - bank_angle_rate, -max_bank_angle)
            else:
                current_angle = 0  # No lateral acceleration, maintain zero bank angle
            return current_angle


    def calcRotationPerHour(self, vehicle, lbs):
        parachute = vehicle.parachute
        altitude = vehicle.altimeter
        rotationX = vehicle.accelerometerX
        rotationZ = vehicle.accelerometerZ
        rollOne = vehicle.rollEngineOne.thrust
        rollTwo = vehicle.rollEngineTwo.thrust
        rollThree = vehicle.rollEngineThree.thrust
        powerOutput = rollOne + rollTwo + rollThree
        gyroX = vehicle.gyroscopeX
        gyroZ = vehicle.gyroscopeZ
        
        if not parachute and altitude < 6:
            rotationX = rotationX / 3600 # must convert accelerometerX rotation per hour to per second
            rotationZ = rotationZ / 3600

            rotationX = rotationX * lbs
            rotationZ = rotationZ * lbs

            rotationX = rotationX - powerOutput
            rotationZ = rotationZ - (0.33 * powerOutput) # roll engin accounts for 1/3rd z-axis spin

            rotationX = rotationX / lbs
            rotationZ = rotationZ / lbs

            gyroX = 360 * rotationX
            gyroZ = 360 * rotationZ
            gyroX = int(vehicle.gyroscopeX + gyroX) % 360
            gyroZ = int(vehicle.gyroscopeZ + gyroZ) % 360

            vehicle.gyroscopeX = gyroX
            vehicle.gyroscopeZ = gyroZ

            rotationX = rotationX * 3600
            rotationZ = rotationZ * 3600

            vehicle.accelerometerX = rotationX
            vehicle.accelerometerZ = rotationZ
            
            # Calculate lateral acceleration
            lateral_acceleration = rotationZ - (0.33 * powerOutput)  # Assuming roll engine accounts for 1/3rd Z-axis spin
            # Adjust bank angle based on lateral acceleration
            vehicle.bank_angle = self.adjust_bank_angle(vehicle.bank_angle, lateral_acceleration)

        else:
            vehicle.rollEngineOne.setThrust(0.0)
            vehicle.rollEngineTwo.setThrust(0.0)
            vehicle.rollEngineThree.setThrust(0.0)

    def calcAxialData(self, vehicle, lbs):
        parachute = vehicle.parachute
        altitude = vehicle.altimeter
        rotationY = vehicle.accelerometerY
        rotationZ = vehicle.accelerometerZ
        gyroY = vehicle.gyroscopeY
        gyroZ = vehicle.gyroscopeZ
        axialOne = vehicle.axialThrustOne.thrust
        axialTwo = vehicle.axialThrustTwo.thrust
        axialThree = vehicle.axialThrustThree.thrust

        counterClockPower = axialOne + axialThree
        clockPower = axialTwo

        if not parachute and altitude < 6:
            rotationY = rotationY / 3600
            rotationZ = rotationZ / 3600

            rotationY = rotationY * lbs
            rotationZ = rotationZ * lbs

            rotationY = rotationY + (clockPower - counterClockPower)
            rotationZ = rotationZ + (0.66 * (clockPower - counterClockPower)) #Axial counts for 2/3rds of Z-axis

            rotationY = rotationY / lbs
            rotationZ = rotationZ / lbs

            gyroY = rotationY * 360
            gyroZ = rotationZ * 360
            gyroY = int(vehicle.gyroscopeY + gyroY) % 360
            gyroZ = int(vehicle.gyroscopeZ + gyroZ) % 360

            vehicle.gyroscopeY = gyroY
            vehicle.gyroscopeZ = gyroZ

            rotationY = rotationY * 3600
            rotationZ = rotationZ * 3600

            vehicle.accelerometerY = rotationY
            vehicle.accelerometerZ = rotationZ
            
            cg_offset_adjustment = 0.5 * vehicle.cg_offset
            # Adjust axial thrust based on CG offset
            rotationZ = rotationZ + (0.66 * (clockPower - counterClockPower)) + cg_offset_adjustment

        else:
            vehicle.axialThrustOne.setThrust(0.0)
            vehicle.axialThrustTwo.setThrust(0.0)
            vehicle.axialThrustThree.setThrust(0.0)

    def calcVelocity(self, vehicle, weight):
        velocity = vehicle.dopplerRadar
        axialOne = vehicle.axialThrustOne.thrust
        axialTwo = vehicle.axialThrustTwo.thrust
        axialThree = vehicle.axialThrustThree.thrust
        totalPower = axialOne + axialTwo + axialThree
        altitude = vehicle.altimeter
        parachute = vehicle.parachute
        changeInVelocity = 0

        if altitude > 1:
            if not parachute:
                distance = altitude - 7.5
                speed = velocity - 3800
                changeInVelocity = speed / distance
            elif parachute:
                if altitude > 7:
                    distance = altitude - 7
                    speed = velocity - 861
                    changeInVelocity = speed / distance
                if 5.7 < altitude <= 7:
                    distance = altitude - 5.7
                    speed = velocity - 268
                    changeInVelocity = speed / distance
                if 0.62 < altitude <= 5.7:
                    distance = altitude - 0.62
                    speed = velocity - 134
                    changeInVelocity = speed / distance

            speed = velocity - changeInVelocity
            if speed > 0:
                vehicle.dopplerRadar = speed
            else:
                print("Error: Lander is flying back out of Atmosphere!")
                input()
        elif altitude < 1 and not parachute:
            velocity = velocity - (totalPower * 0.25) * 0.25
            if velocity >= 0:
                vehicle.dopplerRadar = velocity
            elif velocity <= 0:
                vehicle.dopplerRadar = 20
            else:
                print("Error: Lander Error has occurred")
                vehicle.touchDown = True
                input()

    def calcAltitude(self, vehicle):
        altitude = vehicle.altimeter
        velocity = vehicle.dopplerRadar
        velocity = velocity / 3600
        altitude = altitude - velocity

        if 0.03 < altitude <= 7.5:
            vehicle.parachute = True
            self.slowDownAccelerometer(vehicle)
        elif altitude <= 0.03:
            vehicle.parachute = False
            if altitude <= 0.03 and velocity > 17:
                for _ in range(8):
                    vehicle.touchDown = True
                    print("ERROR: Lander crashed! Speed too Fast! | SPEED:", velocity, "mph")
                    time.sleep(1)
                input()
            elif altitude <= 0.03 and velocity <= 17:
                vehicle.touchDown = True
                print("CONGRATULATIONS! We have touch down!")
                input()

        vehicle.altimeter = altitude


    def calcTemperature(self, vehicle):
        parachute = vehicle.parachute
        velocity = vehicle.dopplerRadar
        altitude = vehicle.altimeter
        maxEngineTemp = 250
        maxVehicleTemp = 2700
        percentComplete = 0.0

        if parachute:
            dataHolder = altitude - 2
            percentComplete = int(dataHolder / 5) % 5

            vehicle.temperatureEngine = maxEngineTemp * percentComplete

            dataHolder = altitude - 2
            percentComplete = int(dataHolder / 20) % 20
            dataHolder = percentComplete * maxVehicleTemp

            vehicle.temperatureVehicle = maxVehicleTemp - dataHolder
        else:
            if altitude > 7.5:
                vehicle.temperatureEngine = 100.00
                vehicle.temperatureVehicle = 2700
            else:
                vehicle.temperatureEngine = maxEngineTemp
                vehicle.temperatureVehicle = maxVehicleTemp

    def slowDownAccelerometer(self, vehicle):
        rotationX = vehicle.accelerometerX
        rotationY = vehicle.accelerometerY
        rotationZ = vehicle.accelerometerZ

        rotationX = self.adjustRotation(rotationX)
        rotationY = self.adjustRotation(rotationY)
        rotationZ = self.adjustRotation(rotationZ)

        vehicle.accelerometerX = rotationX
        vehicle.accelerometerY = rotationY
        vehicle.accelerometerZ = rotationZ

    def adjustRotation(self, rot):
        if rot < 0:
            return rot + 20
        elif rot > 0:
            return rot - 20
        else:
            return rot


def main():
    control = Control()
    control.landMarsLander()

if __name__ == "__main__":
    main()


