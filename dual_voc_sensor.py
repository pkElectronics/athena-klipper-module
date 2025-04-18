import json
import logging
import os.path
import time
from json import JSONDecodeError

class SensorDataHandler:

    def __init__(self,data):
        self.calibration_data = data
        self.cached_measurements = list()
        self.normalized_data = dict()

        self.lower_threshold = 700

        logging.info(f"Loaded Calibration Data")

        for k,v in self.calibration_data.items():
            logging.info(f"Temp: {k} | Value: {v}")

    @classmethod
    def empty(cls):
        return cls(dict())

    def update_data(self, measurement, temperature_map):
        calibration_changed = False
        data_changed = False

        if measurement == self.cached_measurements:
            return  data_changed,calibration_changed
        else:
            data_changed = True


        for i in range(0,len(measurement)):
            meas = round(measurement[i],2)
            temp = temperature_map[i]

            calib = 0
            if temp in self.calibration_data.keys():
                calib = self.calibration_data[temp]

            if calib < meas:
                calib = meas
                self.calibration_data[temp] = calib
                calibration_changed = True

            logging.info(f"Raw: {meas}, Calib: {calib}, Temp: {temp}")

            f = 100 / (calib - self.lower_threshold)
            meas = (meas - self.lower_threshold) * f

            logging.info(f"Factor: {f}, Normalized: {meas}")

            self.normalized_data[temp] = round( meas,2)

        self.cached_measurements = measurement.copy()

        return data_changed, calibration_changed

    def get_air_quality_indicator(self):

        m = 0.0
        for k,v in self.normalized_data.items():
            m+= v

        m /= len(self.normalized_data)
        m = round(m,2)
        aq = 3
        if m < 20:
            aq = 0
        elif m < 50:
            aq = 1
        elif m < 70:
            aq = 2

        return aq,m

    def get_measurement_data_for_export(self):
        export = dict()
        aq,vi = self.get_air_quality_indicator()
        export["airquality"] = aq
        export["vocindex"] = vi
        export["rawdata"] = self.normalized_data
        return export

    def get_calibration_data_for_export(self):
        return self.calibration_data

class DualVocSensor:

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        gcode = self.printer.lookup_object('gcode')
        self.dummy_gcode_cmd = gcode.create_gcode_command("", "", {})

        self.name = config.get_name()
        self.calib_storage_path = config.get('calib_storage_path', "./aegis_calib.json")
        self.data_storage_path = config.get('data_storage_path', "./aegis_data.json")

        self.inlet_sensor_name = config.get('inlet_sensor_name', "bme280 bme680_inlet")
        self.outlet_sensor_name = config.get('outlet_sensor_name', "bme280 bme680_outlet")

        self.enable_respond = config.get('enable_respond', True)


        if os.path.exists(self.calib_storage_path):
            try:
                f_calib = open(self.calib_storage_path, "r")
                data = json.load(f_calib)

                if "inlet" in data.keys():
                    self.inlet_calib = SensorDataHandler(data["inlet"])
                else:
                    self.inlet_calib = SensorDataHandler.empty()

                if "outlet" in data.keys():
                    self.outlet_calib = SensorDataHandler(data["outlet"])
                else:
                    self.outlet_calib = SensorDataHandler.empty()

            except JSONDecodeError:
                self.inlet_calib = SensorDataHandler.empty()
                self.outlet_calib = SensorDataHandler.empty()
        else:
            self.inlet_calib = SensorDataHandler.empty()
            self.outlet_calib = SensorDataHandler.empty()

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)


    def _handle_connect(self):

        self.inlet_sensor = self.printer.lookup_object(self.inlet_sensor_name)
        self.outlet_sensor = self.printer.lookup_object(self.outlet_sensor_name)

        self.sample_timer = self.reactor.register_timer(self._sample_voc)
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)


    def _sample_voc(self,eventtime):

        data = self.inlet_sensor.get_status(0)
        data_changed, calib_changed = self.inlet_calib.update_data(data["gas_complete"], data["gas_complete_temperatures"])

        data = self.outlet_sensor.get_status(0)
        changed = self.outlet_calib.update_data(data["gas_complete"], data["gas_complete_temperatures"])
        data_changed &= changed[0]
        calib_changed &= changed[1]

        if data_changed:
            logging.info("Storing Data")
            self.store_data()

            if self.enable_respond:
                self.dummy_gcode_cmd.respond_raw(f"INLETVOC:{self.inlet_calib.get_air_quality_indicator()[1]}")
                self.dummy_gcode_cmd.respond_raw(f"OUTLETVOC:{self.outlet_calib.get_air_quality_indicator()[1]}")


        if calib_changed:
            logging.info("Storing Calibration")
            self.store_calib()


        measured_time = self.reactor.monotonic()
        return measured_time + 1

    def store_data(self):
        inlet_data = self.inlet_calib.get_measurement_data_for_export()
        outlet_data = self.outlet_calib.get_measurement_data_for_export()

        out = dict()
        out["inlet"] = inlet_data
        out["outlet"] = outlet_data

        with open(self.data_storage_path, "w") as f:
            json.dump(out, f)

    def store_calib(self):
        inlet_calibration_values = self.inlet_calib.get_calibration_data_for_export()
        outlet_calibration_values = self.outlet_calib.get_calibration_data_for_export()
        out = dict()
        out["inlet"] = inlet_calibration_values
        out["outlet"] = outlet_calibration_values

        with open(self.calib_storage_path, "w") as f:
            json.dump(out, f)

def load_config(config):
    return DualVocSensor(config)
