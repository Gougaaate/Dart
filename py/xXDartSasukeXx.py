import dartv2b
from time import sleep

class xXDartSasukeXx:

    def __init__(self):
        self.dart = dartv2b.DartV2()
        init_odo = self.dart.get_front_encoders()

        # Initialisation des variables

        self.odo = (init_odo[0], init_odo[1])
        self.cmd = (0,0)
        self.battery = self.dart.encoders.battery_voltage()

    def update_odometer(self):
        """
        Sauvegarde les données de l'odomètre
        """

        odo_values = self.dart.get_front_encoders()
        self.odo = (odo_values[0], odo_values[1])
        return self.odo

    def update_battery(self):
        """
        Sauvegarde les données de la batterie
        """
        self.battery = self.dart.encoders.battery_voltage()
        return self.battery

    def set_motor_cmd(self, left, right):
        """
        Donne des directions au moteur
        """
        self.dart.set_speed(left, right)
        self.cmd = (left, right)

    def stop(self):
        """
        Arrête le robot
        """
        self.set_motor_cmd(0, 0)
        sleep(0.4)  #
        self.update_odometer()
