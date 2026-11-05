#!/usr/bin/env python3
"""Start a ROS 2 executable with parameters without invoking a shell."""
import subprocess
import yaml

class RosNode:
    def __init__(self, package, executable):
        self._package = package
        self._executable = executable

    def _parameters(self, parameters, namespace=""):
        result = []
        for key, value in parameters.items():
            name = f"{namespace}.{key}" if namespace else key
            if isinstance(value, dict):
                result.extend(self._parameters(value, name))
            else:
                text = yaml.safe_dump(value, default_flow_style=True).split("\n...")[0].strip()
                result.extend(["-p", f"{name}:={text}"])
        return result

    def run(self, parameter_dictionary, namespace=""):
        command = ["ros2", "run", self._package, self._executable, "--ros-args"]
        command.extend(self._parameters(parameter_dictionary, namespace.strip("./")))
        return subprocess.run(command, check=True)
