from typing import Union, List, Dict
from pymodaq.control_modules.move_utility_classes import (DAQ_Move_base, comon_parameters_fun,
                                                          main, DataActuatorType, DataActuator)

from pymodaq_utils.utils import ThreadCommand  # object used to send info back to the main thread
from pymodaq_gui.parameter import Parameter

from pylablib.devices.Thorlabs import KinesisMotor, list_kinesis_devices


def _gen_params():
    """Generate parameter list for this device."""

    # Get available devices and serial numbers
    devices = list_kinesis_devices()
    serials = [x[0] for x in devices]

    # Create parameter with these values
    serial_no = {"title": "Kinesis controller serial number",
                 "name": "serial_no",
                 "type": "list",
                 "value": serials[0],
                 "limits": serials,
                }
    return [serial_no]


class DAQ_Move_KinesisMotor(DAQ_Move_base):
    """ Instrument plugin class for an actuator.
    
    This object inherits all functionalities to communicate with PyMoDAQ’s DAQ_Move module through inheritance via
    DAQ_Move_base. It makes a bridge between the DAQ_Move module and the Python wrapper of a particular instrument.

    TODO Complete the docstring of your plugin with:
        * The set of controllers and actuators that should be compatible with this instrument plugin.
        * With which instrument and controller it has been tested.
        * The version of PyMoDAQ during the test.
        * The version of the operating system.
        * Installation instructions: what manufacturer’s drivers should be installed to make it run?

    Attributes:
    -----------
    controller: object
        The particular object that allow the communication with the hardware, in general a python wrapper around the
         hardware library.
    """

    # General parameters
    is_multiaxes = False
    _axis_names: Union[List[str], Dict[str, int]] = ["Axis1"]
    _controller_units: Union[str, List[str]] = "mm"
    _epsilon: Union[float, List[float]] = 0.01
    data_actuator_type = DataActuatorType.DataActuator

    # Parameter tree
    params = _gen_params() + comon_parameters_fun(is_multiaxes, axis_names=_axis_names, epsilon=_epsilon)


    def ini_attributes(self):
        # For automcompletion
        self.controller: KinesisMotor = None


    def get_actuator_value(self):
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        pos = DataActuator(data=self.controller.get_position(), units="m")
        pos = self.get_position_with_scaling(pos)
        return pos


    def user_condition_to_reach_target(self) -> bool:
        """ Implement a condition for exiting the polling mechanism and specifying that the
        target value has been reached

       Returns
        -------
        bool: if True, PyMoDAQ considers the target value has been reached
        """
        return not self.controller.is_moving()


    def close(self):
        """Terminate the communication protocol"""
        if self.is_master:
            self.controller.close()


    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        name = param.name()
        val = param.value()

        # Serial number change: no action
        if name == "serial_no":
            pass

        else:
            pass


    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """
        if self.is_master:  # is needed when controller is master
            sn = self.settings["serial_no"]
            self.controller = KinesisMotor(sn, scale="stage")
            initialized = self.controller.is_opened()
        else:
            self.controller = controller
            initialized = True

        status = self.controller.get_full_info()
        sn = status["device_info"].serial_no
        stage = status["stage"]
        info = f"Initialized KinesisMotor with S/N {sn} on stage {stage}"
        return info, initialized


    def move_abs(self, value: DataActuator):
        """ Move the actuator to the absolute target defined by value

        Parameters
        ----------
        value: (float) value of the absolute target positioning
        """
        value = self.check_bound(value)  #if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one
        self.controller.move_to(value.value("m"))
        self.emit_status(ThreadCommand('Update_Status', [f"Moving abs to {value}"]))


    def move_rel(self, value: DataActuator):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: (float) value of the relative target positioning
        """
        value = self.check_bound(self.current_position + value) - self.current_position
        self.target_value = value + self.current_position
        value = self.set_position_relative_with_scaling(value)
        self.controller.move_by(value.value("m"))
        self.emit_status(ThreadCommand('Update_Status', [f"Moving ref by {value}"]))


    def move_home(self):
        """Call the reference method of the controller"""
        self.controller.home()


    def stop_motion(self):
        """Stop the actuator and emits move_done signal"""
        self.controller.stop(immediate=False)
        self.emit_status(ThreadCommand('Update_Status', [f"Stopping movement"]))


if __name__ == '__main__':
    main(__file__)
