# LEGO slot:2

import hub # type: ignore
import motor  # type: ignore
import time


import sys
import io


from DriveBase import DriveBase
from Logger import Logger

"""

Our Personal Main Code

Here you can also find our specific exercises and also some examples.

Those are the ports we used for the specific tasks:
MotorPorts:
    A = 0: MotorRight
    B = 1: Unused
    C = 2: Unused
    D = 3: Addition
    E = 4: MotorLeft
    F = 5: AbilityRight

"""
print()
print("-" * 50)
print()

class Controller:
    def __init__(self):
        self._kill_ = False

        self.driveBase = DriveBase()
        
        self.DIVES = [self.detach_pinsel, self.schiffbergung]

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

    def __connect_addition__(self, direction):
        self.driveBase.run_motor_duration(direction * 1110, 0, DriveBase.ADDITION)
        Logger.info("WAITING", code = "START")
        while not self.__button_check__(0):
            pass
        self.driveBase.stop_motor(DriveBase.ADDITION)
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

    def run_program(self):
        dive = self.get_program_slot()
        if dive is None:
            Logger.info("No program slot found", code=404)
            return
        self.driveBase.attach_addition(True)
        time.sleep(1)  # Give some time to start
        self.DIVES[dive]()
        self.driveBase.attach_addition(False)
        Logger.info("Finished program", code = 200)
        time.sleep(1)  # Give some time to read the message

    ########################
    # Tasks for Robot Game #
    ########################

    ###

    ###########
    # TESTING #
    ###########
    
    
    def detach_pinsel(self):
        self.driveBase.drive_distance(83, 1110, 800)
        self.driveBase.turn_to_angle(-5, minspeed=100, maxspeed=1110)
        self.driveBase.drive_distance(-80, 1110, 800)
        
    def schiffbergung(self):
        self.driveBase.turn_to_angle(-5)
        self.driveBase.drive_distance(45)
        self.driveBase.drive_distance(-45)
    ###


ctrl = Controller()

# ctrl.driveBase.configure_pid(5, 1, 1)


def main():
    ctrl.run_program()
    ctrl.kill()


# Start the main async function
if __name__ == "__main__":
    main()
    raise Exception("Program Ended")
