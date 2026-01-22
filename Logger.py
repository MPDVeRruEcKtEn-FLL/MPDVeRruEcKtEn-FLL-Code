import hub # type: ignore

import time
import math

class Logger:
    @staticmethod
    def debug(message, *args):
        """
            Print a debug message to the Console. 
            Use as many arguments as you want! \n
            Gebe eine Debug-Nachricht in die Konsole aus.
            Nutze so viele Argumente wie du möchtest!
        """
        try:
            print("[DEBUG] {}".format(str(message) + " " + " ".join(map(str, args))))
        except:
            print("[DEBUG] {}".format(str(message)))

    @staticmethod
    def info(message, *args, code = None):
        """
            Print a info message to the Console. 
            Use as many arguments as you want! 
            Additionally you can add a code to be displayed
            on the Spike Prime Hub. \n
            Gebe eine Info-Nachricht in die Konsole aus.
            Nutze so viele Argumente wie du möchtest!
            Zusätzlich kannst du einen "code" auf dem
            Spike Prime Hub anzeigen lassen.
        """
        if code != None:
            hub.light_matrix.write(str(code))
        try:
            print("[INFO] {}".format(message + " " + " ".join(map(str, args))))
        except:
            print("[INFO] {}".format(message))

    @staticmethod
    def exception(code: int, message, *args):
        """
            Print a error message to the Console. 
            Use as many arguments as you want! 
            Every Exception also has a error number! \n
            Gebe eine Error-Nachricht in die Konsole aus.
            Nutze so viele Argumente wie du möchtest!
            Immer einen Fehlercode angeben!
        """
        hub.light_matrix.write(str(code))
        try:
            print("[ERROR] {}".format(message + " " + " ".join(map(str, args))))
        except:
            print("[ERROR] {}".format(message))