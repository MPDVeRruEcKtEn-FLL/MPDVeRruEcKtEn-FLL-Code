import hub
import motor
import time

import sys
import io


from DriveBase import DriveBase
from Logger import Logger

"""

Our Personal Main Code

Here you can also find our specific exercises and also some examples.

Those are the ports we used for the specific tasks:
Ports:
    A = 0: MotorRight
    B = 1: ColorSensor
    C = 2: ActionRight
    D = 3: ActionLeft
    E = 4: MotorLeft
    F = 5: ColorSensor

"""
print()
print("-" * 50)
print()

class Controller:
    def __init__(self):
        self._kill_ = False

        self.driveBase = DriveBase()
        
        self.DIVES = [
            self.combined,
            self.blue_path,
            self.golden_path,
            self.green_path,
            self.lilac_path,
                      ]

        Logger.info("Started Program", code = 0)

    #############
    # Internals #
    #############

    def __button_check__(self, button: int) -> bool:
        """"""
        if button == 0:
            return bool(
                hub.button.pressed(hub.button.LEFT)
                or hub.button.pressed(hub.button.RIGHT)
            )
        elif button == 1:
            return bool(hub.button.pressed(hub.button.LEFT))
        elif button == 2:
            return bool(hub.button.pressed(hub.button.RIGHT))
        else:
            Logger.exception(message = "UNKNOWN WHICH BUTTON", code = 303)
            return False

    def __wait_for_button__(self):
        Logger.info("WAITING", code = "START")
        while not self.__button_check__(0):
            time.sleep(0.1)
        time.sleep(0.5)
        
    def get_program_slot(self):
        buf = io.StringIO()
        try:
            raise Exception("probe")
        except Exception as e:
            sys.print_exception(e, buf)

        text = buf.getvalue()
        for line in text.split('\n'):
            if line.startswith('  File "'):
                path = line.split('"')[1]
                parts = path.split('/')
                if len(parts) >= 2 and parts[-2].isdigit():
                    return int(parts[-2])
        Logger.exception(98, "Could not determine program slot")
        return None

    ##################
    # MAIN FUNCTIONS #
    ##################

    def kill(self):
        Logger.info("Killed program", code = -1)
        self._kill_ = True

    ###############
    # RUN PROGRAM #
    ###############

    def run_program(self, dive: int = None):
        if dive is None:
            dive = self.get_program_slot()
            if dive is None:
                Logger.exception(99, "No program slot found, cannot run program")
                return
        Logger.info(f"Starting program {dive}: {self.DIVES[dive].__name__}", code = 100 + dive)
        self.DIVES[dive]()
        self.driveBase.attach_addition(False)
        Logger.info("Finished program", code = 200)

    ########################
    # Tasks for Robot Game #
    ########################
    
    def combined(self):
        for i in range(1, len(self.DIVES[1:])):
            self.run_program(i)
            self.__wait_for_button__()
        
    def blue_path(self):
        db = self.driveBase
        db.attach_addition(True)
        db.drive_distance(58, re_align=False)
        db.drive_distance(-6)
        db.run_motor_duration(-1110, 5, 2)
        for i in range(2):
            db.drive_distance(8, stop=False, re_align=False)
            db.drive_distance(-8, stop=False, re_align=False)
        db.drive_distance(10, re_align=False)
        db.run_motor_duration(1110, 5, 2)
        db.drive_distance(-60, re_align=False)
    
    def golden_path(self):
        def arm_up(speed=300, acc=1000):
            db.run_to_absolute_position(145, speed, 2, acceleration=acc)
        def arm_down(speed=300, acc=1000):
            db.run_to_absolute_position(45, speed, 2, acceleration=acc)
        db = self.driveBase
        db.run_to_absolute_position(45, 100, 2)
        db.attach_addition(True)
        db.drive_distance(-4)
        db.reset_gyro()
        arm_up()
        db.drive_distance(52)
        db.turn_to_angle(-51, pGain=40, dGain=9)
        arm_down()
        db.drive_distance(28)
        arm_up(400, acc=900)
        arm_up(200, acc=500)
        arm_down()
        db.turn_to_angle(-90, DriveBase.RIGHTTURN)
        arm_up()
        db.drive_distance(5)

        db.turn_to_angle(-157, DriveBase.RIGHTTURN)
        db.drive_distance(-20)
        arm_down()
        db.drive_distance(8)
        db.run_to_absolute_position(145, 1110, 2, acceleration=10000)
        time.sleep_ms(300)
        arm_up()
        db.turn_to_angle(-90, DriveBase.RIGHTTURN, iGain=2)
        db.drive_distance(50)
        db.turn_to_angle(-143, DriveBase.RIGHTTURN)
        db.drive_distance(-4)
        arm_down(1110, 10000)
        arm_down(200)
        time.sleep_ms(300)
        arm_up(200)
        db.turn_to_angle(-133, dGain=0)
        db.drive_distance(5)
        arm_down()
        motor.run(2, -200)
        db.drive_distance(20, 800)
        db.drive_distance(-30)

    def green_path(self):
        db = self.driveBase
        db.run_to_absolute_position(0, 1000, 2)
        db.attach_addition(True)
        db.drive_distance(-5)
        db.reset_gyro()
        db.run_to_absolute_position(-90, 10000, 2, acceleration=400)
        db.drive_distance(80, mainspeed=1110, stopspeed=800)
        db.turn_to_angle(90, pGain=30)
        db.drive_distance(43)
        db.turn_to_angle(0, pGain=30)
        db.drive_distance(10)
        
        db.drive_distance(-4)
        db.turn_to_angle(-90, pGain=30)
        db.drive_distance(44.5)
        db.turn_to_angle(-100, pGain=30)
        db.drive_distance(-4, 800, re_align=False)
        db.run_to_absolute_position(-30, 100, 2)
        db.drive_distance(-7, re_align=False)
        db.turn_to_angle(-170, pGain=30)
        db.drive_distance(75,1100)

    def red_path(self):
        db = self.driveBase
        db.attach_addition(True)
        db.drive_distance(-5)
        db.reset_gyro()
        db.drive_distance(80)
        db.drive_distance(-82)

    def lilac_path(self):
        db = self.driveBase
        db.run_to_absolute_position(45, 100, 2)
        db.attach_addition(True)
        db.drive_distance(-5)
        db.reset_gyro()
        motor.run(2, 50)
        db.drive_distance(53)
        motor.stop(2)
        for i in range(0,3):
            db.run_to_absolute_position(45, 600, 2, acceleration=2000)
            db.run_to_absolute_position(135, 100, 2)
        db.drive_distance(-53)
        db.run_to_absolute_position(45, 100, 2)

    ###########
    # TESTING #
    ###########

    def get_current_heading(self):
        while not self.__button_check__(0):
            self.driveBase.get_heading()
            time.sleep(1)


ctrl = Controller()

def main():
    ctrl.run_program()
    ctrl.kill()

# Start the main async function
if __name__ == "__main__":
    main()
    raise Exception("Program Ended")
