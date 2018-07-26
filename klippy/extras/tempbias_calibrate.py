# Calibration of heater tempbias settings
#

#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import heater

AMBIENT_TEMP = 25.

class tempbias_calibrate:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'TEMPBIAS_CALIBRATE', self.cmd_tempbias_calibrate,
            desc="Run Temp bias calibration test")
        logging.info("tempbias")
        self.old_control = None
        print(self.cmd_tempbias_calibrate)

    def cmd_tempbias_calibrate(self, params):
        heater_name = self.gcode.get_str('HEATER', params)
        target = self.gcode.get_float('TARGET', params)
        write_file = self.gcode.get_int('WRITE_FILE', params, 0)
        pheater = self.printer.lookup_object('heater')
        try:
            heater = pheater.lookup_heater(heater_name)
        except self.printer.config_error as e:
            raise self.gcode.error(str(e))
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        calibrate = tempbiasautotune(heater,self, self.printer)
        self.old_control = heater.set_control(calibrate)
        try:
            heater.set_temp(print_time, target)
        except heater.error as e:
            heater.set_control(old_control)
            raise self.gcode.error(str(e))
        self.gcode.bg_temp(heater)
        heater.set_control(self.old_control)
        if write_file:
            calibrate.write_file('/tmp/heattest.txt')
        temp_bias = calibrate.calc_final_bias()
        logging.info("Autotune: final: temp bias =%f", temp_bias)
        self.gcode.respond_info(
            "Temp bias=%.3f\n"
            "To use this parameters, update the printer config file with\n"
            "the above and then issue a RESTART command" % (temp_bias))


#tune safety factor

class tempbiasautotune:
    def __init__(self, heater,routine, printer):
        self.reactor = printer.get_reactor()
        self.routine = routine
        self.heater = heater
        # Heating control
        self.heating = False
        self.pwm_samples = []
        self.lasttemp = 0
        self.ready = False
        self.ready_time = 0

    # Heater control passed back to heater but with tempbias =0 
    def temperature_callback(self, read_time, temp):
        self.routine.old_control.temp_bias = 0
        self.routine.old_control.temperature_callback(read_time, temp)
        if abs(temp - self.heater.target_temp) <2:
            if not self.ready:
                self.ready_time = self.reactor.monotonic()
            self.ready = True
            self.pwm_samples.append(self.routine.old_control.lastpwm)
        else:
            self.ready = False
            self.pwm_samples = []			
        
    def calc_final_bias(self):
        if len(self.pwm_samples) > 0: 
	        pwm_mean = math.fsum(self.pwm_samples)/(len(self.pwm_samples))
        else:
		    pwm_mean = 0
        temp_bias = 0.8*pwm_mean/(self.heater.target_temp - AMBIENT_TEMP) 
        return temp_bias

    def check_busy(self, eventtime):
        if self.reactor.monotonic() - self.read_time > 5:
            return True
        return False
    
    def write_file(self, filename):
        pwm = ["pwm: %.3f %.3f" % (time, value)
               for time, value in self.pwm_samples]
        out = ["%.3f %.3f" % (time, temp) for time, temp in self.temp_samples]
        f = open(filename, "wb")
        f.write('\n'.join(pwm + out))
        f.close()

def load_config(config):
    return tempbias_calibrate(config)
