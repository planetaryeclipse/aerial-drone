from evdev import InputDevice, ecodes, list_devices
from pprint import pprint
import curses
import time

from threading import Lock, Thread
from typing import Optional

from contextlib import nullcontext

from dataclasses import dataclass

import struct

import serial

from os import PathLike

# Sets up external communication
RADIO_FILE = '/dev/ttyUSB0'

#region Controller

@dataclass
class ControllerRead:
    is_connected: bool = False

    button_a: bool = False
    button_b: bool = False
    button_x: bool = False
    button_y: bool = False
    button_right_trigger: bool = False
    button_left_trigger: bool = False
    button_right_thumb: bool = False
    button_left_thumb: bool = False
    button_start: bool = False
    button_select: bool = False
    button_mode: bool = False

    left_joy_horiz: float = 0
    left_joy_vert: float = 0
    right_joy_horiz: float = 0
    right_joy_vert: float = 0

    dpad_horiz: float = 0
    dpad_vert: float = 0

    right_bumper: float = 0
    left_bumper: float = 0

RAW_AXIS_DEADZONE = 30
RADIO_BAUD_RATE = 57600

def read_controller(
        curr_controller_read: ControllerRead,
        curr_controller_read_lock: Optional[Lock] = None):
    
    controller_mapping = {
        # Buttons
        ecodes.BTN_A: 'button_a',
        ecodes.BTN_B: 'button_b',
        ecodes.BTN_X: 'button_x',
        ecodes.BTN_Y: 'button_y',
        ecodes.BTN_TR: 'button_right_trigger',
        ecodes.BTN_TL: 'button_left_trigger',
        ecodes.BTN_THUMBR: 'button_right_thumb',
        ecodes.BTN_THUMBL: 'button_left_thumb',
        ecodes.BTN_START: 'button_start',
        ecodes.BTN_SELECT: 'button_select',
        ecodes.BTN_MODE: 'button_mode',

        # Axes
        ecodes.ABS_HAT0X: 'dpad_horiz',
        ecodes.ABS_HAT0Y: 'dpad_vert',
        ecodes.ABS_X: 'left_joy_horiz',
        ecodes.ABS_Y: 'left_joy_vert',
        ecodes.ABS_RX: 'right_joy_horiz',
        ecodes.ABS_RY: 'right_joy_vert',
        ecodes.ABS_Z: 'left_bumper',
        ecodes.ABS_RZ: 'right_bumper'
    }

    while True:
        # Sets is_connected to False if the connection fails
        with curr_controller_read_lock if curr_controller_read_lock is not None else nullcontext():
            curr_controller_read.is_connected = False
        try:
            # Finds the path of the X-Box controller
            cntrllr_path = None
            devices = [InputDevice(path) for path in list_devices()]
            for device in devices:
                # print(f'path={device.path}, name={device.name}, phys={device.phys}\n')
                if device.name == 'Generic X-Box pad':
                    cntrllr_path = device.path

            if cntrllr_path is None:
                continue

            # For some reason a simple time delay is necessary to properly read the
            # initial state of pressed buttons. However, time delays are insufficient
            # to be able to properly read initial joystick states and seems to require
            # a full open and close again.
            
            device = InputDevice(cntrllr_path)
            device.close()

            device = InputDevice(cntrllr_path)
        except:
            continue  # Does not attempt to continue with controller read if it fails
        
        # Sets connection as successful as the device has successfully setup
        with curr_controller_read_lock if curr_controller_read_lock is not None else nullcontext():
            curr_controller_read.is_connected = True

        print('IS CONNECTED!!')

        try:
            # Reads the initial buttons and axes of the controller
            axes_min_max = dict()
            with curr_controller_read_lock if curr_controller_read_lock is not None else nullcontext():
                button_reads = device.active_keys()
                for button_id in button_reads:
                    setattr(curr_controller_read, controller_mapping[button_id], True)

                axes_capabilities = device.capabilities()[ecodes.EV_ABS]
                for axis_id, info in axes_capabilities:
                    axes_min_max[axis_id] = (info.min, info.max)
                    
                    if abs(info.value) < RAW_AXIS_DEADZONE:
                        continue
                    scaled_value = info.value / info.max if info.value >= 0 else info.value / abs(info.min)
                    setattr(curr_controller_read, controller_mapping[axis_id], scaled_value)
            
            # Reads updates made to the buttons and axes of the controller thorugh a series of events
            for event in device.read_loop():
                with curr_controller_read_lock if curr_controller_read_lock is not None else nullcontext():
                    if event.type == ecodes.EV_KEY:
                        setattr(curr_controller_read, controller_mapping[event.code], event.value == 1)  # Forces boolean
                    elif event.type == ecodes.EV_ABS:
                        if abs(event.value) < RAW_AXIS_DEADZONE:
                            scaled_value = 0
                        else:
                            scaled_value = event.value / axes_min_max[event.code][1] if event.value >= 0 else event.value / abs(axes_min_max[event.code][0])
                        setattr(curr_controller_read, controller_mapping[event.code], scaled_value)
        except:
            # If this fails at any time it means the controller has become disconnected
            # therefore want to simply continue to attempt reconnection again
            pass

#endregion

#region Plane control

MAX_THRUSTER_PWR_CHNG_RATE = 10
MAX_FLAP_ANG_CHNG_RATE = 10
MAX_AILERON_ANG_CHNG_RATE = 10
MAX_ELEVATOR_ANG_CHNG_RATE = 10
MAX_RUDDER_ANG_CHNG_RATE = 10

MAX_ALLOWABLE_ABS_ANG = 45

@dataclass
class PlaneActuators:
    # Thruster power (percentage of max power)
    left_thruster_power: float = 0
    right_thruster_power: float = 0

    # Wing control surfaces (degrees)
    left_flap_ang: float = 0
    left_aileron_ang: float = 0
    right_flap_ang: float = 0
    right_aileron_ang: float = 0

    # Tail control surfaces (degrees)
    left_elevator_ang: float = 0
    right_elevator_ang: float = 0
    rudder_ang: float = 0

def _clamp(val: float, min: float, max: float) -> float:
    if val < min:
        return min
    elif val > max:
        return max
    return val

def update_actuators(actuators: PlaneActuators, cntrllr_read: ControllerRead):
    # Update the thrust based on the bumpers
    actuators.left_thruster_power -= MAX_THRUSTER_PWR_CHNG_RATE * cntrllr_read.left_bumper
    actuators.left_thruster_power += MAX_THRUSTER_PWR_CHNG_RATE * cntrllr_read.right_bumper
    actuators.left_thruster_power = _clamp(actuators.left_thruster_power, 0, 100)
    actuators.right_thruster_power = actuators.left_thruster_power  # Shared

    # NOTE: in the following setup of control angles, pulling down on the joystick
    # is a positive value and is not negated

    # Wing control surfaces

    # NOTE: pull up on left joystick -> flap angles decrease -> plane lift increases 
    actuators.left_flap_ang += MAX_FLAP_ANG_CHNG_RATE * cntrllr_read.left_joy_vert
    actuators.left_flap_ang = _clamp(actuators.left_flap_ang, -MAX_ALLOWABLE_ABS_ANG, MAX_ALLOWABLE_ABS_ANG)
    actuators.right_flap_ang = actuators.left_flap_ang

    # NOTE: pull right on left joystick -> right aileron decreases, left aileron increases -> plane begins to roll
    actuators.left_aileron_ang += MAX_AILERON_ANG_CHNG_RATE * cntrllr_read.left_joy_horiz
    actuators.left_aileron_ang = _clamp(actuators.left_aileron_ang, -MAX_ALLOWABLE_ABS_ANG, MAX_ALLOWABLE_ABS_ANG)
    actuators.right_aileron_ang = -actuators.left_aileron_ang

    # Tail control surfaces
    
    # NOTE: pull down on right joystick -> elevator angles increase -> plane pitches up
    actuators.left_elevator_ang += MAX_ELEVATOR_ANG_CHNG_RATE * cntrllr_read.right_joy_vert
    actuators.left_elevator_ang = _clamp(actuators.left_elevator_ang, -MAX_ALLOWABLE_ABS_ANG, MAX_ALLOWABLE_ABS_ANG)
    actuators.right_elevator_ang = actuators.left_elevator_ang  # Shared

    # NOTE: pull right on joystick -> rudder angle increases -> plane yaw increases
    actuators.rudder_ang += MAX_RUDDER_ANG_CHNG_RATE * cntrllr_read.right_joy_horiz
    actuators.rudder_ang = _clamp(actuators.rudder_ang, -MAX_ALLOWABLE_ABS_ANG, MAX_ALLOWABLE_ABS_ANG)

def encode_plane_cmd_pkt(actuators: PlaneActuators) -> bytes:
    encoded_cmd = struct.pack(
        '<4sfffffffff2s',
        b'SCMD',
        actuators.left_thruster_power,
        actuators.right_thruster_power,
        actuators.left_flap_ang,
        actuators.left_aileron_ang,
        actuators.right_flap_ang,
        actuators.right_aileron_ang,
        actuators.left_elevator_ang,
        actuators.right_elevator_ang,
        actuators.rudder_ang,
        b'\r\n')
    return encoded_cmd

#endregion

#region Plane sensing

@dataclass
class PlaneSensors:
    timestamp_ms: int = 0

    # GOOD DATA

    pressure: float = 0
    airspeed: float = 0  # Computed value derived on microcontroller from pressure

    # TODO: write firmware to support GPS (are therefore omitted from packet)
    # latitude: float = 0
    # longitude: float = 0
    # altitude: float = 0
    
    # Values aligned with world axis (magnetic north)
    # units of g's
    world_accel_x: float = 0
    world_accel_y: float = 0
    world_accel_z: float = 0
    # units of deg/s
    world_ang_vel_x: float = 0
    world_ang_vel_y: float = 0
    world_ang_vel_z: float = 0
    # units of mgauss
    world_mag_x: float = 0
    world_mag_y: float = 0
    world_mag_z: float = 0

    range_1: float = 0
    range_2: float = 0

    # RAW/PRELIM DATA

    # Values aligned with axes of IMU
    # units of g's
    accel_x: float = 0
    accel_y: float = 0
    accel_z: float = 0
    # units of deg/s
    ang_vel_x: float = 0
    ang_vel_y: float = 0
    ang_vel_z: float = 0
    # units of mgauss
    mag_x: float = 0
    mag_y: float = 0
    mag_z: float = 0

    pressure_adc: int = 0

def decode_telemetry_pkt(data: bytes) -> PlaneSensors:
    # NOTE: 4 preceding bytes are STLM and last 2 bytes are \r\n
    vals = struct.unpack('>4sLffffffffffffffffffffffI2s', data)

    return PlaneSensors(
        timestamp_ms=vals[0],
        pressure=vals[1],
        airspeed=vals[2],
        world_accel_x=vals[3],
        world_accel_y=vals[4],
        world_accel_z=vals[5],
        world_ang_vel_x=vals[6],
        world_ang_vel_y=vals[7],
        world_ang_vel_z=vals[8],
        world_mag_x=vals[9],
        world_mag_y=vals[10],
        world_mag_z=vals[11],
        range_1=vals[12],
        range_2=vals[13],
        accel_x=vals[14],
        accel_y=vals[15],
        accel_z=vals[16],
        ang_vel_x=vals[17],
        ang_vel_y=vals[18],
        ang_vel_z=vals[19],
        mag_x=vals[20],
        mag_y=vals[21],
        mag_z=vals[22],
        pressure_adc=vals[23])

#endregion

def main(stdscr):    
    # Sets up communication with the radio
    radio = None
    try:
        radio = serial.Serial(RADIO_FILE, RADIO_BAUD_RATE)
    except:
        pass

    # Sets up the thread to read from the controller to ensure that the
    # most recent updates have been polled

    curr_controller_read: ControllerRead = ControllerRead()
    curr_controller_read_lock: Lock = Lock()

    actuators: PlaneActuators = PlaneActuators()

    controller_thread = Thread(
        target=read_controller,
        daemon=True, 
        args=[
            curr_controller_read,
            curr_controller_read_lock])
    controller_thread.start()

    stdscr.nodelay(True)

    while True:
        stdscr.clear()

        # rows, cols = stdscr.getmaxyx()
        # stdscr.addstr(f'screen width={cols}\n')
        # stdscr.addstr(f'screen height={rows}\n')

        stdscr.addstr(f'Fixed-Wing Plane Manager v0.1\n')
        stdscr.addstr(f'Copyright 2023 Samuel Street\n\n')

        with curr_controller_read_lock:
            if curr_controller_read.is_connected:
                stdscr.addstr(
                    f'Controller input: \tbtn_a={curr_controller_read.button_a!s:5}, '
                    f'btn_b={curr_controller_read.button_b!s:5}, '
                    f'btn_x={curr_controller_read.button_x!s:5}, '
                    f'btn_y={curr_controller_read.button_y!s:5}, '
                    f'btn_left_trigger={curr_controller_read.button_left_trigger!s:5}, '
                    f'btn_right_trigger={curr_controller_read.button_right_trigger!s:5}, '
                    f'\n\t\t\tleft_x={curr_controller_read.left_joy_horiz:+.03f}, '
                    f'left_y={curr_controller_read.left_joy_vert:+.03f}, '
                    f'right_x={curr_controller_read.right_joy_horiz:+.03f}, '
                    f'right_y={curr_controller_read.right_joy_vert:+.03f}, '
                    f'lbumper={curr_controller_read.left_bumper:+.03f}, '
                    f'rbumper={curr_controller_read.right_bumper:+.03f}, '
                    f'dpad_x={curr_controller_read.dpad_horiz:+.03f}, '
                    f'dpad_y={curr_controller_read.dpad_vert:+.03f}\n')
                update_actuators(actuators, curr_controller_read) 
            else:
                stdscr.addstr(f'Unable to find X-Box controller, verify the connection\n')

        stdscr.addch('\n')     

        stdscr.addstr(f'Target actuators: \tleft_thruster={actuators.left_thruster_power:.03f}%, right_thruster={actuators.right_thruster_power:.03f}%'
                      f'\n\t\t\tleft_flap_ang={actuators.left_flap_ang:.03f}deg, right_flap_ang={actuators.right_flap_ang:.03f}deg, left_aileron_ang={actuators.left_aileron_ang:.03f}deg, right_aileron_ang={actuators.right_aileron_ang:.03f}'
                      f'\n\t\t\tleft_elevator_ang={actuators.left_elevator_ang:.03f}deg, right_elevator_ang={actuators.right_elevator_ang:.03f}deg, rudder_ang={actuators.rudder_ang:.03f}deg\n')
        
        stdscr.addch('\n')

        # Detects whether the radio is disconnected and attempts reconnection
        if radio is None:
            try: 
                radio = serial.Serial(RADIO_FILE, RADIO_BAUD_RATE)
            except:
                stdscr.addstr('Radio is not connected, cannot retreive telemetry!\nRetrying connection to radio again!')
       
        if radio is not None:
            try:
                cmd_bytes = encode_plane_cmd_pkt(actuators)
                stdscr.addstr(f'Writing command packet: {" ".join([f"{byte:02x}" for byte in cmd_bytes])}')

                radio.write(cmd_bytes)
            except:
                radio = None
   

            # TODO: add read of telemetry packet
       
        stdscr.refresh()

        time.sleep(0.02) # 50 Hz update to plane and UI

curses.wrapper(main)
