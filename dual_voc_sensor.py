import json
import logging
import os.path
import time
from json import JSONDecodeError
import json
import os
import queue
import tempfile
import threading
from typing import Dict, Any, Union


class AsyncJSONFileWriter:
    """
    Offload JSON-encoded writes to a background thread.
    - Use submit('file1', data) / submit('file2', data) to enqueue writes.
    - Files are overwritten atomically (temp file + os.replace).
    - Main thread remains non-blocking.
    """

    def __init__(self, file_map: Dict[str, Union[str, os.PathLike]], flush: bool = True):
        """
        file_map: e.g. {'file1': 'path/to/file1.json', 'file2': 'path/to/file2.json'}
        flush: fsync writes for extra durability
        """
        self._paths = {k: os.fspath(v) for k, v in file_map.items()}
        self._q: "queue.Queue[tuple[str, Any]]" = queue.Queue()
        self._stop = threading.Event()
        self._flush = flush
        self._worker = threading.Thread(target=self._run, name="AsyncJSONFileWriter", daemon=True)
        self._worker.start()

    def submit(self, target: str, data: Any) -> None:
        """Queue a write to the given target key from file_map."""
        if target not in self._paths:
            raise KeyError(f"Unknown target '{target}'. Valid: {list(self._paths)}")
        # put_nowait keeps caller non-blocking (queue is unbounded)
        self._q.put_nowait((target, data))

    def _run(self) -> None:
        while not self._stop.is_set() or not self._q.empty():
            try:
                target, data = self._q.get(timeout=0.1)
            except queue.Empty:
                continue

            path = self._paths[target]
            payload = json.dumps(data, ensure_ascii=False)  # UTF-8 friendly
            dirn = os.path.dirname(path) or "."

            # Write to a temp file then replace -> atomic overwrite
            fd, tmp = tempfile.mkstemp(prefix=".tmp_asyncwrite_", dir=dirn)
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as f:
                    f.write(payload)
                    if self._flush:
                        f.flush()
                        os.fsync(f.fileno())
                os.replace(tmp, path)  # atomic on POSIX/Windows (same filesystem)
            finally:
                # If an exception occurred before replace, ensure temp file is removed
                if os.path.exists(tmp):
                    try:
                        os.remove(tmp)
                    except OSError:
                        pass
                self._q.task_done()

    def stop(self, wait: bool = True) -> None:
        """Signal the worker to stop. If wait=True, drain the queue before joining."""
        self._stop.set()
        if wait:
            self._q.join()
        self._worker.join(timeout=2)


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

            f = 100 / (calib - self.lower_threshold)
            meas = (meas - self.lower_threshold) * f

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

        self.writer = AsyncJSONFileWriter({
            "calib": self.calib_storage_path,
            "data": self.data_storage_path,
        })

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)

        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)


    def _handle_connect(self):

        self.inlet_sensor = self.printer.lookup_object(self.inlet_sensor_name)
        self.outlet_sensor = self.printer.lookup_object(self.outlet_sensor_name)

        self.sample_timer = self.reactor.register_timer(self._sample_voc)
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def _handle_shutdown(self):
        self.store_data()
        self.store_calib()

    def _sample_voc(self,eventtime):

        data = self.inlet_sensor.get_status(0)
        data_changed, calib_changed = self.inlet_calib.update_data(data["gas_complete"], data["gas_complete_temperatures"])

        data = self.outlet_sensor.get_status(0)
        changed = self.outlet_calib.update_data(data["gas_complete"], data["gas_complete_temperatures"])
        data_changed &= changed[0]
        calib_changed &= changed[1]

        if data_changed:
            logging.info("Storing Data")
            #self.store_data()

            if self.enable_respond:
                self.dummy_gcode_cmd.respond_raw(f"VOCINLET:{self.inlet_calib.get_air_quality_indicator()[1]}")
                self.dummy_gcode_cmd.respond_raw(f"VOCOUTLET:{self.outlet_calib.get_air_quality_indicator()[1]}")


        #if calib_changed:
        #    logging.info("Storing Calibration")
        #    self.store_calib()


        measured_time = self.reactor.monotonic()
        return measured_time + 1

    def store_data(self):
        inlet_data = self.inlet_calib.get_measurement_data_for_export()
        outlet_data = self.outlet_calib.get_measurement_data_for_export()

        out = dict()
        out["inlet"] = inlet_data
        out["outlet"] = outlet_data

        self.writer.submit("data", out)

    def store_calib(self):
        inlet_calibration_values = self.inlet_calib.get_calibration_data_for_export()
        outlet_calibration_values = self.outlet_calib.get_calibration_data_for_export()
        out = dict()
        out["inlet"] = inlet_calibration_values
        out["outlet"] = outlet_calibration_values

        self.writer.submit("calib", out)


def load_config(config):
    return DualVocSensor(config)
