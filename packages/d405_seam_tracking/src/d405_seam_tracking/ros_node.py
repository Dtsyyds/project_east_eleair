"""Compatibility import for the separately installed ROS adapter."""
def main(argv=None):
    from d405_seam_ros.node import main as run
    return run(argv)

def image_payload(view):
    from d405_seam_ros.node import image_payload as convert
    return convert(view)
