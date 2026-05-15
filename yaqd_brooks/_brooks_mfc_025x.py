__all__ = ["BrooksMfc025x"]


import asyncio
from dataclasses import dataclass
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
    aserial,
)

parameters = {"SP Rate": 1, "SP Full Scale": 9}


def construct_write(address: int, port: int, parameter: int, value: float) -> bytes:
    command = "AZ"
    if address:
        command += f"{address:05}"
    command += f".{port:02}P{parameter:02}={value:.2f}\r\n"
    return command.encode()


def construct_query(address: int, port: int, parameter: int) -> bytes:
    command = "AZ"
    if address:
        command += f"{address:05}"
    command += f".{port:02}P{parameter:02}?\r\n"
    return command.encode()


@dataclass
class Response:
    predelimiter: str
    address: int
    port: int
    response_type: int
    parameter: int
    value: float
    checksum: bytes
    checksum_valid: bool


def parse_response(raw: bytes) -> Response:
    string = raw.decode().strip()
    predelimiter, addport, response_type, parameter, value, checksum = string.split(",")
    address, port = addport.split(".")
    # TODO CHECKSUM
    return Response(
        predelimiter=predelimiter,
        address=int(address),
        port=int(port),
        response_type=int(response_type),
        parameter=int(parameter[1:]),
        value=float(value),
        checksum=checksum.encode(),
        checksum_valid=True,
    )


parity_options = {"even": "E", "odd": "O"}

stop_bit_options = {"one": 1, "one_and_half": 1.5, "two": 2}


class BrooksMfc025x(
    HasTransformedPosition, HasLimits, HasPosition, UsesUart, UsesSerial, IsDaemon
):
    _kind = "brooks-mfc-025x"

    def __init__(self, name, config, config_filepath):
        super().__init__(name, config, config_filepath)
        self._ser = aserial.get_aserial(
            config["serial_port"],  # magically ensures single instance per port
            baudrate=config["baud_rate"],
            parity=parity_options[config["parity"]],
            stop_bits=stop_bit_options[config["stop_bits"]],
        )
        self._units = "ml/min"
        self._native_units = "ml/min"

    def close(self):
        self._ser.flush()
        self._ser.close()

    def direct_serial_write(self, _bytes):
        self._ser.write(_bytes)

    def get_position(self):
        return self.to_transformed(self._state["position"])

    def _relative_to_transformed(self, relative_position):
        xp = [p["setpoint"] for p in self._config["calibration"]]
        fp = [p["measured"] for p in self._config["calibration"]]
        out = np.interp(relative_position, xp, fp)
        return out

    def _set_position(self, position):
        command = construct_write(
            self._config["address"], self._config["physical_port"], parameters["SP Rate"], position
        )
        response = self._ser.awrite_then_readline(command)

    def _transformed_to_relative(self, transformed_position):
        xp = [p["measured"] for p in self._config["calibration"]]
        fp = [p["setpoint"] for p in self._config["calibration"]]
        return np.interp(transformed_position, xp, fp)

    async def update_state(self):
        while True:
            command = construct_query(
                self._config["address"], self._config["physical_port"], parameters["SP Rate"]
            )
            raw = self._ser.awrite_then_readline(command)
            response = parse_response(raw)
            if response.parameter == parameters["SP Rate"]:
                self._state["position"] = response.value
            if abs(self._state["position"] - self._state["destination"]) < 1.0:
                self._busy = False
            if self._state["destination"] == 0.0:
                if self._state["position"] < 1.0:
                    self._busy = False
            await asyncio.sleep(0.25)
