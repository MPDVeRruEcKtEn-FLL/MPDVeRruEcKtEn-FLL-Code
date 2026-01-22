from hub import motion_sensor
import motor
import motor_pair
import color_sensor

import time
import math

from Logger import Logger

class DriveBase:
    """

    Every function we use to control our robot! \n
    Use this class as code to be added and used in every project. \n
    Alle Funktionen mit denen Wir unseren Roboter steuern! \n
    Diese Klasse in den Code hinzufügen und benutzen.

    """

    # CONSTANTS

    TYPEMOTOR = 'motor'
    TYPECOLORSENS = 'color_sensor'

    TANKTURN = 0
    LEFTTURN = 1
    RIGHTTURN = 2

    CLOCKWISE = 0
    COUNTERCLOCKWISE = 1
    SHORTEST_PATH = 2
    LONGEST_PATH = 3
    
    LEFT = 1
    RIGHT = -1
    
    RED = 0
    GREEN = 1
    BLUE = 2
    
    # PID CONSTANTS
    
    PREGLER = 0
    IREGLER = 0
    DREGLER = 0

    # CONFIGS

    MOTORR = 0
    MOTORL = 4
    ADDITION = 3
    ACTION = 2
    COLORSENS = 5

    MOTPAIR = 0

    WHEELCIRC = 17.5 / (24/8) # Übersetzung von 3 [Rad:Motor] 24:8

    def __init__(self, initial_yaw: int = 0):
        self.gyroSens = motion_sensor
        self.stop = False

        self.global_turn_value = initial_yaw
        self.gyroSens.reset_yaw(initial_yaw * -10)
        self.addition_state = False

        motor_pair.pair(self.MOTPAIR, self.MOTORL, self.MOTORR)
        

    def configure(
        self,
        motor_right_port: int = MOTORR,
        motor_left_port: int = MOTORL,
        addition_port: int = ADDITION,
        action_port: int = ACTION,
        color_sensor_port: int = COLORSENS,
        motor_pair_id: int = MOTPAIR,
        wheel_circumference: float = WHEELCIRC,
    ):
        """
        Configure DriveBase

        Konfiguriere DriveBase

        Params / Parameter
        -----------------

        #### motor_right_port: int = 0
            The port of the right motor. \n
            Der Port des rechten Motors.

        #### motor_left_port: int = 4
            The port of the left motor. \n
            Der Port des linken Motors.

        #### addition_port: int = 3
            The port of the addition motor. \n
            Der Port des Zusatzmotors.

        #### action_right_port: int = 5
            The port of the right action motor. \n
            Der Port des rechten Aktionsmotors.

        #### color_sensor_port: int = 2
            The port of the color sensor. \n
            Der Port von dem Farbsensor.

        #### motor_pair_id: int = 0
            The ID of the main motorpair. \n
            Die ID des Haupt Motorpaares.

        #### wheel_circumference: float = 17.6
            The circumference of the wheels. \n
            Der Umfang der Räder.
        """
        self.MOTORR = motor_right_port
        self.MOTORL = motor_left_port
        self.ADDITION = addition_port
        self.ACTION = action_port
        self.COLORSENS = color_sensor_port
        self.MOTPAIR = motor_pair_id
        self.WHEELCIRC = wheel_circumference
        
    def configure_pid(
        self,
        p_regler: float = 0,
        i_regler: float = 0,
        d_regler: float = 0
    ):
        """Configure PID Constants
        Configure PID Constants
        
        Konfiguriere PID Konstanten
        
        Params / Parameter
        -----------------

        #### p_regler: float = 0
            The P-Constant
            Die P Konstante
        
        #### i_regler: float = 0
            The I-Constant
            Die I Konstante
            
        #### d_regler: float = 0
            The D-Constant
            Die D Konstante
        """
        
        if p_regler != 0:
            self.PREGLER = p_regler
        if i_regler != 0:
            self.IREGLER = i_regler
        if d_regler != 0:
            self.DREGLER = d_regler
    #########################
    # Complex GyroFunctions #
    #########################

    def drive_distance(
        self,
        distance: float = 100,
        mainspeed: int = 1110,
        stopspeed: int = 700,
        re_align: bool = True,
        isolated_drive: bool = False,
        stop: bool = True,
        *,
        brake_start: float = 0.95,
        timestep: int = 1,
        avoid_collision: bool = False,
        ) -> bool:
        """
        Drive a specific distance and correct unwanted changes with the gyrosensor.

        Fahre eine bestimmte Distanz und gleiche unbeabsichtigte Änderungen mit dem GyroSensor aus.

        Params / Parameter
        -----------------
        #### distance : int = 100
            The distance that the robot is supposed to drive. \n
            Die Distanz die der Roboter fahren soll. If 0 it drives endlessly, if negative drives backwards.

        #### mainspeed: int = 600
            The maximum speed the robot reaches. (0 - 1110) \n
            Die maximale Geschwindigkeit, die der Roboter erreicht. (0 - 1110)

        #### stopspeed : float = 300
            The target speed while braking; the minimum speed at the end of the program. (0 - 1110) \n
            Die minimale Geschwindigkeit am Ende des Programms. (0 - 1110)
        
        #### re_align : bool = True
            If the robot should realign itself at the end of the driving to correct changes. \n
            Ob der Roboter sich neu ausrichten sollte, nachdem er gefahren ist, um Änderungen auszugleichen.

        #### isolated_drive : bool = False
            If the robot should drive independantly of the global turn_value. \n
            Ob der Roboter unabhängig von dem globalen Winkel fahren soll.
        
        #### stop : bool = True
            If the robot should stop at the end of the driven distance. \n
            Ob der Roboter am Ende der Distanz stoppen soll.

        ##### brake_start : int = 0.95
            Percentage of the driven distance after which the robot starts braking. \n
            Prozentsatz der zurückgelegten Strecke, nach der der Roboter mit dem Bremsen beginnt. \n
        ##### timestep : int = 100
            The timestep in ms between every single calculation and correction to prevent to fast reactions. \n
            Der Zeitabstand in ms zwischen jeder Berechnung und Ausgleichung, um zu schnelle Reaktionen zu verhindern. \n
        ##### avoid_collision : bool = False    ---> UNUSED
            If the robot should try to avoid every collision. \n
            Ob der Roboter versuchen sollte, Kollisionen auszuweichen.
        """
        motor.reset_relative_position(self.MOTORL, 0)
        motor.reset_relative_position(self.MOTORR, 0)
        mainspeed = abs(mainspeed)
        stopspeed = abs(stopspeed)

        def get_gyro_value() -> float:
            return self.gyroSens.tilt_angles()[0] / 10

        def get_driven():
            return (
                abs(motor.relative_position(self.MOTORL))
                + abs(motor.relative_position(self.MOTORR))
            ) / 2
            
        def error() -> int:
            return int(get_gyro_value() - start_value)
                
        start_value = get_gyro_value() if isolated_drive else self.global_turn_value

        # Set starting speed of robot
        speed = mainspeed
        # Sets PID values

        old_change = 0
        integral = 0
        steering_sum = 0

        invert = 1

        # Sets values based on user inputs
        loop = True
        
        # Inversion of target rotation value for negative values
        if distance < 0:
            invert = -1
            distance = abs(distance)

        # Calulation of degrees the motors should turn to
        # 17.6 is wheel circumference in cm. You might need to adapt it
        rotate_distance = (distance / self.WHEELCIRC) * 360
        deccelerate_distance = rotate_distance * (1 - brake_start)

        # Calculation of braking point
        brake_start_value = brake_start * rotate_distance
        driven_distance = get_driven()


        while loop:
            steering_sum += error()
            integral += error() - old_change
            # Calculation of driven distance and PID values
            old_driven_distance = driven_distance
            driven_distance = get_driven()
            
            p_regler, i_regler, d_regler = self.get_pids(speed)
            # yaw angle used due to orientation of the hub
            curren_steering = (
                error() * p_regler
                + integral * i_regler
                + d_regler * (error() - old_change) / (timestep / 1000)
            )
            
            old_change = error()

            curren_steering = max(-100, min(curren_steering, 100))

            # Calculation of speed based on acceleration and braking, calculation of steering value for robot to drive perfectly straight
            if distance == 0 or not stop:
                speed = mainspeed
            else:
                speed = self.speed_calculation(
                    speed,
                    deccelerate_distance,
                    brake_start_value,
                    int(driven_distance),
                    int(old_driven_distance),
                    mainspeed=mainspeed,
                    stopspeed=stopspeed,
                )
                braking = True if driven_distance > brake_start_value else False
                # curren_steering = 0 if braking else curren_steering

            motor_pair.move(
                self.MOTPAIR, int(invert * curren_steering), velocity=int(invert * -speed)
            )

            if distance == 0:
                if self.stop:
                    loop = False
                    motor_pair.stop(self.MOTPAIR)
                    self.stop = False
            elif rotate_distance < driven_distance and stop:
                loop = False
                motor_pair.stop(self.MOTPAIR)
            elif rotate_distance < driven_distance:
                loop = False
            time.sleep_ms(timestep)
        if re_align:
            # Removed isolated turn because not needed
            self.turn_to_angle(self.global_turn_value)
        time.sleep_ms(timestep)
        return True

    def turn_to_angle(
        self,
        target_angle: float = 90,
        turn_type: int = TANKTURN,
        minspeed: int = 100,
        maxspeed: int = 1110,
        isolated_turn: bool = False,
        smart_stop: bool = True,
        *,
        pGain: float = 20,
        iGain: float = 0,
        dGain: float = 6,
        powerExp: float = 6,
        tolerance: float = 0.5,
        timestep: int = 10
        ):
        """
        Turn to a specific angle and correct overturning with the gyrosensor.

        Drehe den Roboter auf einen bestimmten Winkel und korrigiere Überdrehen mit dem GyroSensor.

        Params / Parameter:
        -----------------------

        #### target_angle : float = 90
            The angle the robot should turn to. \n
            Der Winkel auf die sich der Roboter drehen soll.
        
        #### turn_type : int = DriveBase.TANKTURN
            The type of how the robot should turn. \n
            Wie der Roboter sich drehen soll. \n
                -> TANKTURN Turn both wheels / Drehe um beide Räder \n
                -> LEFTTURN Turn over the left wheel / Drehe über das linke Rad \n
                -> RIGHTTURN Turn over the right wheel / Drehe über das rechte Rad

        #### minspeed : int = 100
            The minimal speed, which the robot never falls below. (0 - 1110) \n
            Die minimale Geschwindigkeit, welche der Roboter niemals unterschreitet. (0 - 1110)
        
        #### maxspeed : int = 1000
            The maximum speed, which the robot never exceeds. (0 - 1110) \n
            Die maximale Geschwindigkeit, welche der Roboter niemals überschreitet. (0 - 1110)

        #### isolated_turn : bool = False
            If the robot should turn independently of the global turn_value. \n
            Ob der Roboter sich unabhängig von dem globalen Winkle drehen soll.
        
        #### smart_stop : bool = True
            If the robot should stop at the end. \n
            Ob der Roboter am Ende stoppen soll.

        ##### pGain : float = 40
            The proporional gain of the pid-turn. \n
            Die Proportionale von dem PID-Turn. \n
        ##### iGain : float = 0
            The integral gain of the pid-turn. \n
            Die Integral von dem PID-Turn. \n
        ##### dGain : float = 9
            The derivative gain of the pid-turn. \n
            Die Derivative von dem PID-Turn. \n
        ##### power_exp : float = 6
            The power gain. \n
            Die Verstärkung der Leistung. \n
        ##### tolerance : float = 0.5
            The tolerance of the output value. \n
            Die Toleranz beim Output. \n
        ##### timestep : int = 10
            The timestep between every single calculation and correction to prevent to fast reactions. \n
            Der Zeitabstand zwischen jeder Berechnung und Ausgleichung, um zu schnelle Reaktionen zu verhindern.
        """

        def get_gyro_value():
            return self.gyroSens.tilt_angles()[0] / 10

        def error() -> float:
            raw_error = target_angle - get_gyro_value()
            if raw_error > 180:
                raw_error -= 360
            elif raw_error < -180:
                raw_error += 360
            return raw_error

        def calc_power():
            return (
                abs(motor.get_duty_cycle(self.MOTORL))
                + abs(motor.get_duty_cycle(self.MOTORR))
            ) / 2
            

        integral = 0
        power = 0
        prev_error = 0
        invert = 1
        
        if turn_type != self.TANKTURN:
            dGain = 0

        if not isolated_turn:
            self.global_turn_value = target_angle

        while True:
            integral += error() * (timestep / 1000)
            derivative = (error() - prev_error) / (timestep / 1000)
            prev_error = error()
            power = calc_power()

            output = (pGain * error()) + (iGain * integral) + \
                (dGain * derivative) * (1 - (power / 10000) ** powerExp)
            if output < 0:
                invert = -1
            else:
                invert = 1
            output = int(max(minspeed, min(abs(output), maxspeed)))

            if abs(error()) <= tolerance:
                output = 0

            # Set motor speeds based on output
            if turn_type == self.TANKTURN:
                motor_pair.move(self.MOTPAIR, 100, velocity=invert * output)

            elif turn_type == self.LEFTTURN:
                motor.run(self.MOTORR, invert * -output)

            elif turn_type == self.RIGHTTURN:
                motor.run(self.MOTORL, invert * -output)

            # Stop when close to the target angle
            if abs(error()) <= tolerance and smart_stop:
                motor_pair.stop(self.MOTPAIR)
                time.sleep_ms(90)
                if abs(error()) <= tolerance:
                    Logger.debug(
                        "Successful Turn: {}/{} offset: {}".format(target_angle, -int(get_gyro_value()), error()))
                    break

            time.sleep_ms(timestep)
        motor_pair.stop(self.MOTPAIR)

    def turn_till_color(self, direction: int = LEFT, speed: int = 900, color_type: int = RED, color_gate: int = 700, timeout: int = -1):
        """Turn the robot until a specific color is detected.
        
        Params / Parameter
        -----------------
        #### direction: int = DriveBase.LEFT
            The direction in which the robot should turn. \n
            Die Richtung in die sich der Roboter drehen soll. \n
                -> DriveBase.LEFT Turn left / Drehe links \n
                -> DriveBase.RIGHT Turn right / Drehe rechts
        #### speed: int = 900
            The speed with which the robot should turn. \n
            Die Geschwindigkeit mit der sich der Roboter drehen soll. \n
            
        #### color_type: int = DriveBase.RED
            The type of color value that should be detected. \n
            Der Typ des Farbwertes der erkannt werden soll. \n
        
        #### color_gate: int = 700
            The gate value for the color detection. \n
            Der Grenzwert für die Farberkennung. \n
        #### timeout: int = -1
            The maximum time the robot should turn. If -1, turns endlessly. \n
            Die maximale Zeit die der Roboter sich drehen soll. Wenn -1, dreht endlos. \n
        """

        colorsens = self.detect_device(self.TYPECOLORSENS)
        self.COLORSENS = colorsens[0]
        if timeout > 0:
            motor_pair.move_for_time(
                self.MOTPAIR, timeout, direction * 100, velocity=speed)
        else:
            motor_pair.move(self.MOTPAIR, direction * 100, velocity=speed)

        start_time = time.ticks_ms()

        while True:
            color_val = color_sensor.rgbi(self.COLORSENS)[color_type]

            if color_val <= color_gate:
                break
            elif (time.ticks_ms() - start_time) / 1000 > timeout:
                Logger.debug((time.ticks_ms() - start_time) / 1000)
                break
            else:
                time.sleep_ms(50)
        motor.stop(self.MOTPAIR)

    def turn_till_reflect(self, direction: int = LEFT, speed: int = 900, reflection_gate: int = 700, smaller_than: int = True, timeout: int = -1):
        """Turn the robot until a specific reflection is detected.
        
        Params / Parameter
        -----------------
        #### direction: int = DriveBase.LEFT
            The direction in which the robot should turn. \n
            Die Richtung in die sich der Roboter drehen soll. \n
                -> DriveBase.LEFT Turn left / Drehe links \n
                -> DriveBase.RIGHT Turn right / Drehe rechts
        #### speed: int = 900
            The speed with which the robot should turn. \n
            Die Geschwindigkeit mit der sich der Roboter drehen soll. \n
        #### reflection_gate: int = 700
            The gate value for the reflection detection. \n
            Der Grenzwert für die Reflektionserkennung. \n
        #### smaller_than: bool = True
            If the robot should stop when the reflection is smaller than the gate. \n
            Ob der Roboter stoppen soll, wenn die Reflektion kleiner als der Grenzwert ist. \n
        #### timeout: int = -1
            The maximum time the robot should turn. If -1, turns endlessly. \n
            Die maximale Zeit die der Roboter sich drehen soll. Wenn -1, dreht endlos. \n
        """

        colorsens = self.detect_device(self.TYPECOLORSENS)
        self.COLORSENS = colorsens[0]
        motor_pair.move(self.MOTPAIR, direction * 100, velocity=speed)

        start_time = time.ticks_ms()

        while True:
            reflection_val = color_sensor.reflection(self.COLORSENS)

            if smaller_than and reflection_val <= reflection_gate:
                break
            elif not smaller_than and reflection_val >= reflection_gate:
                break
            elif (time.ticks_ms() - start_time) / 1000 > timeout:
                Logger.debug((time.ticks_ms() - start_time) / 1000)
                break
            else:
                time.sleep_ms(50)
        print("Finish")
        motor.stop(self.MOTPAIR)

    def till_collide(self, speed: int = 900, gate: int = 300, timeout: int = -1, *, start_delay: int = 1000) -> float:
        """Drive the robot until a collision is detected bigger than the given gate value based on the motor duty cycle. \n
        
        Fahre den Roboter bis eine Kollision erkannt wird, die größer als der gegebene Grenzwert ist, basierend auf dem Motordutycycle. \n
        
        Params / Parameter
        -----------------
        #### speed: int = 900
            The speed with which the robot should drive. \n
            Die Geschwindigkeit mit der sich der Roboter fahren soll. \n
        #### gate: int = 300
            The gate value for the collision detection. \n
            Der Grenzwert für die Kollisionserkennung. \n
        #### timeout: int = -1
            The maximum time the robot should drive. If -1, drives endlessly. \n
            Die maximale Zeit die der Roboter sich fahren soll. Wenn -1, fährt endlos. \n
        
        """
        def cycl() -> float:
            return (
                abs(motor.get_duty_cycle(self.MOTORL))
                + abs(motor.get_duty_cycle(self.MOTORR))
            ) / 2

        def get_driven() -> float:
            return (
                abs(motor.relative_position(self.MOTORL))
                + abs(motor.relative_position(self.MOTORR))
            ) / 2
        start_dist = get_driven()

        motor_pair.move(self.MOTPAIR, 0, velocity=speed)
        time.sleep_ms(start_delay)
        start_cycl = cycl()
        start_time = time.ticks_ms()
        while True:
            if self.collided(cycl(), start_cycl, gate):
                print(cycl())
                break
            elif (time.ticks_ms() - start_time) / 1000 > timeout and timeout > 0:
                Logger.debug(
                    abs(time.ticks_diff(start_time, time.ticks_ms)) / 1000)
                break
            else:
                time.sleep_ms(50)
        motor_pair.stop(self.MOTPAIR)

        distance = ((get_driven() - start_dist) * self.WHEELCIRC) / 360
        return distance

    def till_color(self, speed: int, color_type: int = 0, color_gate: int = 700, timeout: int = -1):
        """Drive the robot until a specific color is detected.
        
        Fahre den Roboter bis eine bestimmte Farbe erkannt wird.
        
        Params / Parameter
        -----------------
        #### speed: int = 900
            The speed with which the robot should drive. \n
            Die Geschwindigkeit mit der sich der Roboter fahren soll. \n
        #### color_type: int = DriveBase.RED
            The type of color value that should be detected. \n
            Der Typ des Farbwertes der erkannt werden soll. \n
        #### color_gate: int = 700
            The gate value for the color detection. \n
            Der Grenzwert für die Farberkennung. \n
        #### timeout: int = -1
            The maximum time the robot should drive. If -1, drives endlessly. \n
            Die maximale Zeit die der Roboter sich fahren soll. Wenn -1, fährt endlos. \n
        """
        colorsens = self.detect_device(self.TYPECOLORSENS)
        self.COLORSENS = colorsens[0]
        # if timeout > 0:
        #     motor_pair.move_for_time(self.MOTPAIR, timeout, 0, velocity = speed)
        # else:
        motor_pair.move(self.MOTPAIR, 0, velocity=speed)

        start_time = time.ticks_ms()

        loop = True

        while loop:
            color_val = color_sensor.rgbi(self.COLORSENS)[color_type]

            if color_val <= color_gate:
                loop = False
                break
            elif (time.ticks_ms() - start_time) / 1000 > timeout:
                loop = False
                break
            else:
                time.sleep_ms(50)
            if not loop:
                print("IDK what happens")
        print("Finish")
        motor.stop(self.MOTPAIR)

    def around_kollision(self, timestamp, power, old_power, steering, speed):
        """
        IN PROGRESS, DO NOT USE!!!
        """
        return
        # Logger.debug((timestamp, power, old_power))
        motor_pair.move(self.MOTPAIR, steering, velocity=speed)

    #######################
    # Simple Interactions #
    #######################

    def run_motor_duration(
        self, 
        speed: int = 500, 
        duration: float = 5,
        *ports: int,
        acceleration: int = 1000
        ) -> bool:
        """Run the given Motor

        Start the given ports for a specified time duration.
        If the duration is <= 0 do not stop.

        Starte die gegebenen ports für eine angegebene Zeit.
        Wenn die Zeit <= 0 ist, stoppt der Motor nicht.

        Params / Parameter
        -----------------

        #### speed: int = 500
            How fast the motor should turn. \n
            Wie schnell sich der Motor drehen soll. \n
        #### duration: float = 5
            How long the motor should run, if <= 0 no stopping. \n
            Wie lange der Motor sich drehen soll, wenn <= 0 stoppt er nicht. \n
        #### ports: int
            The ports which will be controlled, needs to be specified, otherwise throws Error. \n
            Die Ports die gesteuert werden sollen, muss angegeben sein, sonst kommt ein Fehler. \n
        #### acceleration: int = 1000
            The acceleration of the motor. \n
            Die Beschleunigung des Motors. \n
        """
        if len(ports) == 0:
            Logger.exception(40, "Please give ports")
            return False
        ports_list = list(ports)

        try:
            for port in ports_list:
                motor.run(port, speed, acceleration=acceleration)
            if duration > 0:
                time.sleep(duration)
                for port in ports_list:
                    motor.stop(port, stop=motor.SMART_COAST)
            return True
        except:
            Logger.exception(
                421, "Given unavailable port {}".format(str(ports)))
            return False

    def run_motor_degree(
        self, 
        speed: int = 500, 
        degree: float = 90, 
        *ports: int, 
        tolerance: float = 5,
        acceleration: int = 1000
        ) -> bool:
        """Run the given Motor

        Start the given ports for a specified angle.

        Starte die gegebenen ports für einen angegebenen Winkel.

        Params / Parameter
        -----------------
        #### speed: int = 500 [degree/second]
            How fast the motor should turn. \n
            Wie schnell sich der Motor drehen soll. \n
        #### angle: float = 5 [degree]
            How much the motor should turn. \n
            Wie viel sich der Motor drehen soll. \n
        #### ports: int
            The ports which will be controlled, needs to be specified, otherwise throws Error. \n
            Die Ports die gesteuert werden sollen, muss angegeben sein, sonst kommt ein Fehler. \n
        ##### tolerance: float = 5
            The tolerance the motor checks for between the given and measured angle. \n
            Die Toleranz der Motor überprüft zwischen dem gegebenen und gemessenen Winkel. \n
        #### acceleration: int = 1000
            The acceleration of the motor. \n
            Die Beschleunigung des Motors. \n
        """

        def reached() -> bool:
            if abs(current_pos - target_pos) <= tolerance:
                return True
            else:
                return False

        try:
            if degree > 0:
                invert = 1
            else:
                invert = -1

            ports_list = [port for port in ports]
            if len(ports) == 0:
                Logger.exception(40, "Please give ports")
                return False

            target_pos = degree

            for port in ports_list:
                start_pos = motor.relative_position(
                    port)  # Startposition speichern
                motor.run(port, invert * speed, acceleration=acceleration)  # Motor starten
                # Zielposition berechnen
                target_pos = start_pos + degree

            while True:
                for port in ports_list:
                    current_pos = motor.relative_position(port)
                    if reached():
                        ports_list.remove(port)
                        motor.stop(port, stop=motor.SMART_COAST)
                if len(ports_list) == 0:
                    break
                time.sleep_ms(100)
            return True
        except Exception as e:
            Logger.exception(
                421, "Error with motor port(s) {}: {}".format(str(ports), e)
            )
            return False

    def run_action_duration(self, speed: int = 700, duration: float = 5, acceleration: int = 1000) -> bool:
        """Run the action/ability motor for time.

        Run the action motor with given speed, for the given time.

        Drehe den Motor mit dem gegebenen Speed, die gegebene Zeit.

        Params / Parameter
        --------------

        #### speed: float = 700 [degree/second]
            The given speed, with which the motor turns. \n
            Der gegebene Speed mit dem der Motor sich dreht. \n
        #### time: float = 5 [seconds]
            The given time, for which the motor should turn. \n
            Die gegebene Zeit, die sich der Motor drehen soll. \n
        #### acceleration: int = 1000
            The acceleration of the motor. \n
            Die Beschleunigung des Motors. \n
        """
        return self.run_motor_duration(speed, duration, self.ACTION, acceleration=acceleration)

    def run_action_degree(self, speed: int = 700, degree: float = 90, acceleration: int = 1000) -> bool:
        """Run the action/ability motor for degree

        Run the action motor until it has turned the given degree. (not a turn to-, but a turn for-action)

        Drehe den Motor mit gegebener Geschwindigkeit bis er sich um den gegebenen Winkel dreht. (Kein drehen bis auf Position, aber ein drehen um Grad)

        Params / Parameter
        --------------

        #### speed: int = 700 [degree / second]
            The given speed, with which the motor turns. \n
            Der gegebene Speed mit dem der Motor sich dreht. \n
        #### degree: float = 90 [degree]
            The given angle, for which the motor should turn. \n
            Den gegebenen Winkel, um die sich der Motor drehen soll. \n
        #### acceleration: int = 1000
            The acceleration of the motor. \n
            Die Beschleunigung des Motors. \n
        """
        return self.run_motor_degree(speed, degree, self.ACTION, acceleration=acceleration)

    def run_to_absolute_position(
        self, 
        position: int = 0, 
        speed: int = 500, 
        *ports: int, 
        acceleration: int = 1000, 
        direction: int = 2,
        fine_correction: bool = True,
        correction_timeout: int = 2000,
        correction_tolerance: int = 0
    ) -> bool:
        """Run motor(s) to given absolute position

        Run the given motors to the position, waits until position is reached

        Drehe die Motoren auf die Position, wartet bis die Position erreicht ist

        Params / Parameter
        ------------

        #### position: int = 0 [degree]
            Where the robot should turn to. \n
            Auf welchen Wert sich der Roboter drehen soll. \n
        #### speed: int = 500 [degree / second]
            With which speed the robot should turn. \n
            Mit welcher Geschwindigkeit der Roboter sich drehen soll. \n
        #### ports: tuple[int, ...]
            Which port should be used. \n
            Welche Ports angesteuert werden sollen. \n
        #### acceleration: int = 1000
            The acceleration of the motor. \n
            Die Beschleunigung des Motors. \n
        #### direction: int = 2
            The direction strategy to use. \n
            Die Richtungsstrategie, die verwendet werden soll. \n
            CLOCKWISE = 0, COUNTERCLOCKWISE = 1, SHORTEST_PATH = 2, LONGEST_PATH = 3
        #### fine_correction: bool = True
            If the robot should correct its position at the end. \n
            Ob der Roboter am Ende seine Position korrigieren soll.
        #### correction_timeout: int = 2000
            Max time in ms for correction. \n
            Maximalzeit in ms für die Korrektur.
        #### correction_tolerance: int = 0
            The tolerance in degrees for the correction. \n
            Die Toleranz in Grad für die Korrektur.
        """

        ports_list = [port for port in ports]
        start_ticks = {}
        if len(ports) == 0:
            Logger.exception(40, "Please give ports")
            return False
            
        try:
            sub_tasks = []
            for port in ports_list:
                current_abs = (motor.absolute_position(port) % 360)
                target_abs = (position % 360)
                
                diff = (target_abs - current_abs) % 360
                
                delta = 0
                if direction == self.CLOCKWISE:
                    delta = diff
                elif direction == self.COUNTERCLOCKWISE:
                    delta = diff - 360 if diff != 0 else 0
                elif direction == self.SHORTEST_PATH:
                    if diff <= 180:
                        delta = diff
                    else:
                        delta = diff - 360
                elif direction == self.LONGEST_PATH:
                    if diff > 180:
                        delta = diff
                    else:
                        delta = diff - 360
                
                start_rel = motor.relative_position(port)
                target_rel = start_rel + delta
                
                move_speed = abs(speed)
                if delta < 0:
                    move_speed = -move_speed
                elif delta == 0:
                    move_speed = 0
                
                if delta != 0:
                    motor.run(port, int(move_speed), acceleration=acceleration)
                    start_ticks[port] = time.ticks_ms()
                    sub_tasks.append((port, target_rel, delta))

            while len(sub_tasks) > 0:
                for task in sub_tasks[:]:
                    port, target_rel, delta = task
                    current_rel = motor.relative_position(port)
                    
                    finished = False
                    if delta > 0:
                        if current_rel >= target_rel:
                            finished = True
                    else:
                        if current_rel <= target_rel:
                            finished = True
                    
                    if motor.get_duty_cycle(port) == 0 and time.ticks_diff(time.ticks_ms(), start_ticks[port]) > 100:
                        motor.stop(port)
                        sub_tasks.remove(task)
                     
                    if finished:
                         motor.stop(port)
                         sub_tasks.remove(task)
                         
                
                if len(sub_tasks) == 0:
                    break
                    
                time.sleep_ms(10)
            
            # Fine correction phase
            if fine_correction:
                # Give motors time to settle after stop
                time.sleep_ms(100) 
                
                start_time = time.ticks_ms()
                ports_to_correct = list(ports)
                
                while time.ticks_diff(time.ticks_ms(), start_time) < correction_timeout and len(ports_to_correct) > 0:
                    for port in ports_to_correct[:]:
                        # Check absolute position error
                        current_abs = motor.absolute_position(port)
                        # Calc shortest distance to target
                        diff = (position - current_abs + 180) % 360 - 180
                        
                        if abs(diff) <= correction_tolerance:
                            motor.stop(port)
                            ports_to_correct.remove(port)
                        else:
                            # Simple proportional controller or fixed low speed
                            # Using fixed low speed to avoid oscillation if not using PID
                            correct_speed = 100
                            if diff < 0:
                                correct_speed = -100
                            
                            motor.run(port, correct_speed)
                            
                    if len(ports_to_correct) == 0:
                        break
                        
                    time.sleep_ms(10)
                
                # Stop any remaining motors
                for port in ports_to_correct:
                    motor.stop(port)

            return True
        except Exception as e:
            Logger.exception(
                12, "run to absolute position had following error: {}".format(
                    e)
            )
            raise e
            return False

    def run_to_relative_position(
        self, 
        position: int = 0, 
        speed: int = 500, 
        *ports: int, 
        acceleration: int = 1000, 
        direction: int = 2,
        fine_correction: bool = True,
        correction_timeout: int = 2000,
        correction_tolerance: int = 0
    ) -> bool:
        """Run motor(s) to given relative position

        Run the given motors to the position, waits until position is reached

        Drehe die Motoren auf die Position, wartet bis die Position erreicht ist

        Params / Parameter
        ------------

        #### position: int = 0 [degree]
            Where the robot should turn to. \n
            Auf welchen Wert sich der Roboter drehen soll. \n
        #### speed: int = 500 [degree / second]
            With which speed the robot should turn. \n
            Mit welcher Geschwindigkeit der Roboter sich drehen soll. \n
        #### ports: tuple[int, ...]
            Which port should be used. \n
            Welche Ports angesteuert werden sollen. \n
        #### acceleration: int = 1000
            The acceleration of the motor. \n
            Die Beschleunigung des Motors. \n
        #### direction: int = 2
            Unused for relative position, but kept for consistency. logic is always Shortest/Linear.
        #### fine_correction: bool = True
            If the robot should correct its position at the end. \n
            Ob der Roboter am Ende seine Position korrigieren soll.
        #### correction_timeout: int = 2000
            Max time in ms for correction. \n
            Maximalzeit in ms für die Korrektur.
        #### correction_tolerance: int = 0
            The tolerance in degrees for the correction. \n
            Die Toleranz in Grad für die Korrektur.
        """

        ports_list = [port for port in ports]
        if len(ports_list) == 0:
            Logger.exception(40, "Please give ports")
            return False
            
        try:
            sub_tasks = []
            for port in ports_list:
                start_rel = motor.relative_position(port)
                delta = position - start_rel
                
                move_speed = abs(speed)
                if delta < 0:
                    move_speed = -move_speed
                elif delta == 0:
                    move_speed = 0
                
                if delta != 0:
                    motor.run(port, int(move_speed), acceleration=acceleration)
                    sub_tasks.append((port, position, delta))

            while len(sub_tasks) > 0:
                for task in sub_tasks[:]:
                    port, target_rel, delta = task
                    current_rel = motor.relative_position(port)
                    
                    finished = False
                    if delta > 0:
                        if current_rel >= target_rel:
                            finished = True
                    else:
                        if current_rel <= target_rel:
                            finished = True
                            
                    if finished:
                         motor.stop(port)
                         sub_tasks.remove(task)
                
                if len(sub_tasks) == 0:
                    break
                    
                time.sleep_ms(10) # 10ms for faster update than 100ms
            
            # Fine correction phase
            if fine_correction:
                # Give motors time to settle after stop
                time.sleep_ms(100) 
                
                start_time = time.ticks_ms()
                ports_to_correct = list(ports)
                
                while time.ticks_diff(time.ticks_ms(), start_time) < correction_timeout and len(ports_to_correct) > 0:
                    for port in ports_to_correct[:]:
                        # Check relative position error
                        current_rel = motor.relative_position(port)
                        diff = position - current_rel
                        
                        if abs(diff) <= correction_tolerance:
                            motor.stop(port)
                            ports_to_correct.remove(port)
                        else:
                            # Simple proportional controller or fixed low speed
                            correct_speed = 100
                            if diff < 0:
                                correct_speed = -100
                            
                            motor.run(port, correct_speed)
                            
                    if len(ports_to_correct) == 0:
                        break
                        
                    time.sleep_ms(10)
                
                # Stop any remaining motors
                for port in ports_to_correct:
                    motor.stop(port)
                
            return True
        except Exception as e:
            Logger.exception(
                12, "run to relative position had following error: {}".format(
                    e)
            )
            return False

    def attach_addition(self, attach: bool = True) -> bool:
        """Attach/Detach the addition.

        Attach or detach the addition of the robot.

        Befestige oder Löse Aufsatz vom Roboter.

        Params/Parameter
        --------
        #### attach: bool
            In which state the addition should be set. \n
            In welchen Zustand der Aufsatz gesetzt werden soll.
        """
        old_state = self.addition_state
        if attach and not old_state:
            motor.run(3, 600, acceleration=10000)
            Logger.info("WAITING", code = "START")
            while (motor.get_duty_cycle(self.ADDITION) / 100) < 100:
                pass
            time.sleep(1)
            motor.stop(self.ADDITION)
            self.addition_state = True
            return True
        elif not attach and old_state:
            self.run_motor_duration(-600, 1, self.ADDITION)
            self.addition_state = False
            return True
        else:
            return False

    def reset_null(self, *ports: int):
        """
        Reset the position of a given motor to absolute position zero.

        Setze die Position von einem gegebenen Motor auf die absolute Position Null.

        Params
        ------

        #### ports: tuple[int]
        """
        for port in ports:
            motor.reset_relative_position(port, 0)
            while True:
                current_pos = motor.relative_position(port)
                if abs(current_pos) == 0:
                    break
                
    def reset_gyro(self):
        """Reset the gyro sensor to zero position.

        Setze den Gyrosensor auf die Nullposition.
        """
        self.gyroSens.reset_yaw(0)
        self.global_turn_value = 0
        time.sleep_ms(500)

    def stop_motor(self, *ports) -> bool:
        """Stop given motor

        Stop the motor(s) with given port(s)
        
        Stoppe den/die Motor(en) mit gegebenen Port(s)
        
        Params / Parameter
        ------

        #### ports: tuple[int]
        """
        try:
            for port in ports:
                motor.stop(port)
            return True
        except OSError:
            Logger.exception(
                621, "Given unavailable port(s) {}".format(str(ports)))
            return False

    #########################
    # Calculating Functions #
    #########################

    def detect_device(self, device_type: str) -> list[int]:
        """Detect specific connected device_types
        
        Scan all ports and indentify which of the ports are the device_type searched for
        
        Scanne alle Ports und erkenne welche der Ports mit dem gesuchten device_type übereinstimmen

        Params / Parameter
        -----------------
        
        #### device_type: str
            The type of device to search for ('motor' or 'color_sensor'). \n
            Der Gerätetyp, nach dem gesucht werden soll ('motor' oder 'color_sensor'). \n

        Returns / Ausgabe
        -----
        #### list[int]: 
            List of ports where the specified device_type is connected. \n
            Liste der Ports, an denen der angegebene Gerätetyp angeschlossen ist. \
        """
        devices = self.detect_all_devices()
        searched_devices: list[int] = []
        for device in devices:
            if devices[device] == device_type:
                searched_devices.append(device)
        return searched_devices

    def detect_all_devices(self) -> dict[int, str]:
        """Detect all connected devices on all ports

        Scan all ports and identify which type of device is connected to each port.

        Scanne alle Ports und identifiziere, welche Art von Gerät an jedem Port angeschlossen ist.

        Returns / Ausgabe
        -----
        #### dict[int, str]
            A dictionary mapping port numbers to device types ('motor', 'color_sensor', or 'none'). \n
            Ein Dictionary, das Portnummern zu Gerätetypen zuordnet ('motor', 'color_sensor' oder 'none').
        """
        devices = {}
        for port in range(6):
            device_type = 'none'
            
            # Check for motor
            try:
                motor.relative_position(port)
                device_type = 'motor'
            except:
                pass
            
            # Check for color sensor
            if device_type == 'none':
                try:
                    color_sensor.rgbi(port)
                    device_type = 'color_sensor'
                except:
                    pass
            
            devices[port] = device_type
        
        Logger.debug("Device scan: {}".format(devices))
        return devices

    def speed_calculation(
        self,
        speed: int,
        deccelerate_distance: float,
        brake_start_value: float,
        driven: int,
        old_driven: int,
        mode: int = 0,
        rotate_mode: int = 0,
        mainspeed: int = 300,
        stopspeed: int = 300,
    ):
        """Calculating the speed depending on all given Params

        Used to calculate all the speeds in our programs.
        Executed separately to reduce redundancy.

        Wird verwendet, um alle Geschwindigkeiten in unseren Programmen zu berechnen.
        Wird separat ausgeführt, um Redundanz zu reduzieren.

        Params / Parameter
        ----------------------
        #### speed : int
            The current speed of the robot. \n
            Die aktuelle Geschwindigkeit des Roboters. \n
        #### deccelerate_distance: float
            The distance at which the robot starts to deccelerate. \n
            Die Distanz, ab welcher der Roboter anfängt zu bremsen. \n
        #### brakeStartValue : float
            Percentage of the driven distance after which the robot starts braking. \n
            Prozentsatz der zurückgelegten Strecke, nach dem der Roboter mit dem Bremsen beginnt. \n
        #### driven : int
            Distance the robot has currently traveled. \n
            Strecke, die der Roboter aktuell zurückgelegt hat. \n
        #### old_driven : int
            Distance the robot traveled during the last function call. \n
            Strecke, die der Roboter beim letzten Aufruf zurückgelegt hat. \n
        #### mode : int = 0
            The mode the robot operates in: turn[0] or drive[1]. \n
            Der Modus, in dem der Roboter arbeitet: turn[0] oder drive[1]. \n
        #### rotate_mode : int = 0
            The turning mode: normal_turn[0] or tank_turn[1]. \n
            Der Drehmodus: normal_turn[0] oder tank_turn[1]. \n
        #### mainspeed : int = 300
            The maximum speed the robot reaches. \n
            Die maximale Geschwindigkeit, die der Roboter erreicht. \n
        #### stopspeed : int = 300
            The target speed while braking; the minimum speed at the end of the program. \n
            Die Zielgeschwindigkeit beim Bremsen; die minimale Geschwindigkeit am Ende des Programms. \n
        """

        if rotate_mode == 1:
            if mainspeed in range(-300, 300):
                return mainspeed
            else:
                return int(math.copysign(1, mainspeed)) * 300

        if mode == 0:
            deccelerate_distance = max(deccelerate_distance, 1)
            sub_speed_per_degree = (
                mainspeed - stopspeed) / deccelerate_distance

            subtraction = (
                abs(driven) - abs(old_driven)
                if abs(driven) - abs(old_driven) >= 1
                else 1
            ) * sub_speed_per_degree

            if abs(driven) > abs(brake_start_value):

                if abs(speed) > abs(stopspeed):
                    speed = int(speed - subtraction)

            return speed
        else:
            deccelerate_distance = max(deccelerate_distance, 1)
            sub_speed_per_degree = (
                mainspeed - stopspeed) / deccelerate_distance

            subtraction = (
                abs(driven) - abs(old_driven)
                if abs(driven) - abs(old_driven) >= 1
                else 1
            ) * sub_speed_per_degree

            if abs(driven) > abs(brake_start_value):
                if abs(speed) > abs(stopspeed):
                    speed = int(speed - subtraction)
            return speed

    def get_pids(self, speed: float) -> tuple[float, float, float]:
        """Calculation of PID Values.

        Return the PID Values depending on the given speed. \n
        Gib die PID-Werte aus, abhängig davon, wie schnell der Roboter fährt. \n
        
        Params / Parameter:
        ----------------------
        #### speed : float
            The current speed of the robot. \n
            Die aktuelle Geschwindigkeit des Roboters. \n

        Returns / Ausgabe
        -----
        (pRegler, iRegler, dRegler=1)

        """

        speed = abs(speed)

        def pRegler():
            return (
                14.59
                - 0.177132762 * speed
                + 0.000920045989 * speed**2
                - 2.34879006e-6 * speed**3
                + 3.15365919e-9 * speed**4
                - 2.15176282e-12 * speed**5
                + 5.90277778e-16 * speed**6
            )

        def iRegler():
            return (
                4.30433333
                - 0.0374442063 * speed
                + 0.00018870942 * speed**2
                - 5.52917468e-7 * speed**3
                + 8.790625e-10 * speed**4
                - 6.96201923e-13 * speed**5
                + 2.14583333e-16 * speed**6
            )

        if self.PREGLER == 0:
            p_regler = 3
        else:
            p_regler = self.PREGLER
            
        if self.IREGLER == 0:
            i_regler = 1
        else:
            i_regler = self.IREGLER
            
        if self.DREGLER == 0:
            d_regler = 3
        else:
            d_regler = self.DREGLER
        

        return (p_regler, i_regler, d_regler)

    
    def get_heading(self):
        """EXPERIMENTAL"""
        def get_gyro() -> float:
            return self.gyroSens.tilt_angles()[0] / 10
        
        Logger.debug("Gyro Heading: {}".format(get_gyro()))
        
        
        
    def collided(self, cycl, start_cycl, gate: int = 300):
        """Detect collision based on change in motor cycle.
        Determine if a collision has occurred by comparing the current motor cycle 
        with the starting cycle and a defined gate threshold. \n
        
        Kollisionserkennung basierend auf Änderung des Motorzyklus.
        Bestimme, ob eine Kollision aufgetreten ist, indem der aktuelle Motorzyklus
        mit dem Startzyklus und einem definierten Gate-Schwellenwert verglichen wird. \n
        
        Params / Parameter:
        ----------------------
        #### cycl : float
            The current motor cycle. \n
            Der aktuelle Motorzyklus. \n
        #### start_cycl : float
            The motor cycle at the start of monitoring. \n
            Der Motorzyklus zu Beginn der Überwachung. \n
        #### gate : int = 300
            The threshold for detecting a collision. \n
            Der Schwellenwert zur Erkennung einer Kollision. \n
        
        """
        diff = cycl - start_cycl
        if diff > gate:
            return True
        else:
            return False

    def __convert_abs__(self, abs_pos: int = 0) -> int:
        return (abs_pos % 360)
