from future.utils import iteritems
import numpy as np

from pydrake.geometry import DrakeVisualizer
from pydrake.math import RigidTransform
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import BodyIndex
from pydrake.systems.framework import DiagramBuilder, Diagram
from pydrake.systems.meshcat_visualizer import ConnectMeshcatVisualizer
from pydrake.systems.sensors import CameraInfo, RgbdSensor

class DrakeSimDiagram(Diagram):
    def __init__(self, config):
        Diagram.__init__(self)
        
        dt = config["mbp_dt"]
        self._builder = DiagramBuilder()
        self._mbp, self._sg = AddMultibodyPlantSceneGraph(self._builder, dt)
        
        self._finalize_functions = []
        self._finalized = False
        self._rgbd_sensors = dict()
        self._renderer_name = None
        
    # === Property accessors ========================================
    @property
    def mbp(self):
        return self._mbp

    @property
    def sg(self):
        return self._sg

    @property
    def builder(self):
        return self._builder

    @property
    def finalize_functions(self):
        return self._finalize_functions

    # === Add visualizers ===========================================
    def connect_to_meshcat(self, zmq_url="tcp://127.0.0.1:6000", frames_to_draw=[]):
        self._meshcat = ConnectMeshcatVisualizer(
            self._builder, scene_graph=self._sg, zmq_url=zmq_url,
            draw_period=1, frames_to_draw=frames_to_draw)
        return self._meshcat

    def connect_to_drake_visualizer(self):
        self._drake_viz = DrakeVisualizer.AddToBuilder(
            builder=self._builder, scene_graph=self._sg)
        return self._drake_viz

    # === Finalize the completed diagram ============================
    def finalize(self):
        self._mbp.Finalize()
        self._finalized = True

        for func in self._finalize_functions:
            func()

        self._builder.ExportOutput(self._sg.get_pose_bundle_output_port(),
                                   "pose_bundle")
        self._builder.ExportOutput(self._mbp.get_contact_results_output_port(),
                                   "contact_results")
        self._builder.ExportOutput(self._mbp.get_state_output_port(),
                                   "plant_continuous_state")
        self._builder.ExportOutput(self._mbp.get_geometry_poses_output_port(),
                                   "geometry_poses")
        self._builder.ExportInput(self._mbp.get_applied_spatial_force_input_port(),
                                   "spatial_input")
        self._builder.ExportOutput(self._mbp.get_body_poses_output_port(),
                                   "body_poses")
        self._builder.ExportOutput(self._mbp.get_reaction_forces_output_port(),
                                   "reaction_forces")

        self._builder.BuildInto(self)

    def is_finalized(self):
        return self._finalized
