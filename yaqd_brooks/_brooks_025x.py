__all__ = ["BrooksMfcGf"]


import asyncio
from typing import Dict, Any, List
import struct
import serial  # type: ignore
import math
import numpy as np

from yaqd_core import (
    HasTransformedPosition,
    HasLimits,
    HasPosition,
    UsesSerial,
    UsesUart,
    IsDaemon,
    aserial
)


class Brooks025x(HasTransformedPosition, HasLimits, HasPosition, UsesUart, UsesSerial, IsDaemon):
    _kind = "brooks-025x"

    aserials: Dict[str, aserial.ASerial] = {}

    def __init__(self, name, config, config_filepath):
        super().__init__(name, config, config_filepath)
        if config["serial_port"] in BrooksMfcGf.hart_dispatchers:
            self._ser = aserial.ASerial[config["serial_port"]]
        else:
            self._ser = aserial.ASerial(
                config["serial_port"],
                baudrate=config["baud_rate"],
                parity=config["parity"],
                stop_bits=config["stop_bits"],
            )
            Brooks025x.aserials[config["serial_port"]] = self._ser
        self._units = "ml/min"
        self._native_units = "ml/min"

    def close(self):
        self._ser.flush()
        self._ser.close()

    def direct_serial_write(self, _bytes):
        self._ser.write(_bytes)

    def get_position(self):
        return self.to_transformed(self._state["position"])

    def _process_response(self, msg):
        if msg.command == 1:
            self._state["position"] = msg.primary_variable
            if self._state["position"] < 0:
                self._state["position"] == 0
        elif msg.command == 14:  # read primary variable information
            # the values I get here are off by a factor I do not understand
            # still, they scale---faster MFCs give larger limits
            # I will keep this for now ---Blaise 2022-09-28
            self._state["hw_limits"][0] = msg.lower_limit
            self._state["hw_limits"][1] = msg.upper_limit

    async def _read_hw_limits(self):
        while True:
            command = hart_protocol.universal.read_primary_variable_information(
                self._config["address"]
            )
            self._ser.write(command)
            await asyncio.sleep(1)
            if all([not math.isnan(v) for v in self._state["hw_limits"]]):
                break

    def _relative_to_transformed(self, relative_position):
        xp = [p["setpoint"] for p in self._config["calibration"]]
        fp = [p["measured"] for p in self._config["calibration"]]
        out = np.interp(relative_position, xp, fp)
        return out

    def _set_position(self, position):
        if position == 0:
            # this is a "hack" to FORCE the MFC closed
            position = -100
        units_code = 171
        data = struct.pack(">Bf", units_code, position)
        command = hart_protocol.tools.pack_command(
            address=self._config["address"], command_id=236, data=data
        )
        self._ser.write(command)

    def _transformed_to_relative(self, transformed_position):
        xp = [p["measured"] for p in self._config["calibration"]]
        fp = [p["setpoint"] for p in self._config["calibration"]]
        return np.interp(transformed_position, xp, fp)

    async def update_state(self):
        while True:
            self._ser.write(hart_protocol.universal.read_primary_variable(self._config["address"]))
            if abs(self._state["position"] - self._state["destination"]) < 1.0:
                self._busy = False
            if self._state["destination"] == 0.0:
                if self._state["position"] < 1.0:
                    self._busy = False
            await asyncio.sleep(0.25)
