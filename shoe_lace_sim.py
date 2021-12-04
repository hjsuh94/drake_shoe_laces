import yaml
import numpy as np
import matplotlib.pyplot as plt

from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator

from manipulation_diagram.manipulation_diagram import ManipulationDiagram

builder = DiagramBuilder()
# Set up manipulation diagram.
config = yaml.safe_load(open("config.yaml", 'r'))

station = builder.AddSystem(ManipulationDiagram(config))
station.add_arms_from_config(config)
station.process_model_directives()
station.connect_to_drake_visualizer()
station.finalize()

diagram = builder.Build()

simulator = Simulator(diagram)
simulator.set_publish_every_time_step(False)
simulator.set_target_realtime_rate(1.0)

simulator_context = simulator.get_mutable_context()
station_context = station.GetMyMutableContextFromRoot(simulator_context)
station.GetInputPort("iiwa_position").FixValue(station_context, np.array([-1.57, 0.1, 0, 1.2, 0, -1.6, 0]))
station.GetInputPort("iiwa_feedforward_torque").FixValue(station_context, np.zeros(7))
station.GetInputPort("iiwa_gripper_position").FixValue(station_context, np.array([0.05]))
station.GetInputPort("iiwa_gripper_force_limit").FixValue(station_context, np.array([80]))
simulator.AdvanceTo(100)

