import numpy as np
from jiminy_py.simulator import Simulator
from jiminy_py.robot import BaseJiminyRobot
from jiminy_py.core import ControllerFunctor

# Global parameters
SIMULATION_END_TIME = 10.0
urdf_path = "simple_pendulum.urdf"
hardware_path = "simple_pendulum_hardware.toml"


robot = BaseJiminyRobot()
robot.initialize(urdf_path, hardware_path, has_freeflyer=False)

def controller_function(t: np.array,
                        q: np.array,
                        v: np.array,
                        sensor_data: "jiminy_py.core.sensorsData",
                        u: np.array):
    # PD on true true simulation state
    Kp = np.array([5000])
    Kd = np.array([0.02])
    desired_torque = - Kp * (q + Kd * v)

    # PD on encoder measurement
    # q_enc, v_enc = sensor_data["EncoderSensor"]
    # desired_torque = - Kp * (q_enc + Kd * v_enc)

    u[:] = desired_torque


controller = ControllerFunctor(controller_function)
controller.initialize(robot)

simulator = Simulator(robot, controller)
simulator.import_options("simulation_config.toml")

q0 = 0.1 * np.random.rand(1)
v0 = np.random.rand(1)
simulator.simulate(SIMULATION_END_TIME, q0, v0)
simulator.replay()
