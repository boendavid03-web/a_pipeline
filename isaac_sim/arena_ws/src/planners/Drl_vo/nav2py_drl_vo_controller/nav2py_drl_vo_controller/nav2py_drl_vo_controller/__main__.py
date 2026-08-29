import io
import itertools
import logging
import os
import typing
import zipfile

import nav2py
import nav2py.interfaces
import yaml

from . import LaserScan, Path, Pose
from .drl_vo_controller.controllerv2 import ControllerV2


def directory_to_zip(directory) -> io.BytesIO:
    zip_buffer = io.BytesIO()
    with zipfile.ZipFile(zip_buffer, "w") as zip_file:
        for root, directories, files in os.walk(directory):
            for child in itertools.chain(directories, files):
                file_path = os.path.join(root, child)
                arcname = os.path.relpath(file_path, directory)
                zip_file.write(file_path, arcname=arcname)
    zip_buffer.seek(0)
    return zip_buffer


class nav2py_drl_vo_controller(nav2py.interfaces.nav2py_costmap_controller):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._register_callback('scan', self._scan_callback)
        self._register_callback('odom', self._odom_callback)
        self._register_callback('path', self._path_callback)
        self._register_callback('speed_limit', self._speed_limit_callback)

        self.logger = logging.getLogger('nav2py_drl_vo_controller')
        self.frame_count = 0
        self.path = None

        self.logger.info("nav2py_drl_vo_controller initialized")

        package_share_dir = os.path.dirname(__file__)
        model_path = os.path.join(package_share_dir, '../../../..', 'share', 'model', 'drl_vo')
        model_zip = directory_to_zip(model_path)
        self.controller = ControllerV2(model_zip, self.logger)

    def _scan_callback(self, scan_: typing.List[bytes]):
        """
        Process scan data from C++ controller
        """
        self.logger.info('Process scan data from C++ controller')
        if scan_:
            scan = yaml.safe_load(scan_[0].decode())
            self.controller.scan_callback(LaserScan.parse(scan))

    def _path_callback(self, path_: typing.List[bytes]):
        """
        Process path data from C++ controller
        """
        self.logger.info('Process path data from C++ controller')
        if path_:
            path = yaml.safe_load(path_[0].decode())
            self.controller.setPath(Path.parse(path))

    def _odom_callback(self, data: typing.List[bytes]):
        """
        Process data from C++ controller
        """
        self.logger.warning('Process odom data from C++ controller')

        parsed_data = yaml.safe_load(data[0].decode())

        pose = Pose.parse(parsed_data["pose"])
        velocity = parsed_data["velocity"]

        linear_x, angular_z = self.controller.computeVelocityCommands(pose)

        self.logger.info(f"Sending control commands: linear_x={linear_x:.2f}, angular_z={angular_z:.2f}")

        self._send_cmd_vel(linear_x, angular_z)

    def _speed_limit_callback(self, speed_limit: typing.List[bytes]):
        """
        Process speed limit data from C++ controller
        """
        self.logger.info('Process speed limit data from C++ controller')

        if len(speed_limit) == 2:
            speed_limit = yaml.safe_load(speed_limit[0].decode())
            is_percentage = yaml.safe_load(speed_limit[1].decode())
            self.controller.setSpeedLimit(speed_limit, is_percentage)


if __name__ == "__main__":
    nav2py.main(nav2py_drl_vo_controller)
