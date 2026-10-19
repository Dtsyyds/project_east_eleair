"""TF validation helpers."""

import math


def clean_frame_id(frame_id):
    return str(frame_id).strip().lstrip("/")


def transform_values(transform_stamped):
    parent = clean_frame_id(transform_stamped.header.frame_id)
    child = clean_frame_id(transform_stamped.child_frame_id)
    if not parent or not child:
        raise ValueError("TF parent and child frame IDs must be non-empty")
    translation = transform_stamped.transform.translation
    rotation = transform_stamped.transform.rotation
    xyz = [float(translation.x), float(translation.y), float(translation.z)]
    xyzw = [float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)]
    norm = math.sqrt(sum(value * value for value in xyzw))
    if not all(math.isfinite(value) for value in xyz + xyzw) or norm <= 1.0e-12:
        raise ValueError(f"invalid TF transform {parent} -> {child}")
    return parent, child, xyz, [value / norm for value in xyzw]
