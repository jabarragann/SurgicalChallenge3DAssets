def ambf_rigid_body_set_scale(object_handle, scale):
    print(object_handle.name)
    if object_handle.type == "MESH":
        if object_handle.ambf_object_type == "RIGID_BODY":
            object_handle.ambf_rigid_body_linear_inertial_offset = (
                object_handle.ambf_rigid_body_linear_inertial_offset * scale
            )
            for (
                prop_tuple
            ) in object_handle.ambf_collision_shape_prop_collection.items():
                shape_prop_group = prop_tuple[1]
                collision_shape_set_scale(shape_prop_group, scale)


def collision_shape_set_scale(shape_prop, scale):
    coll_shape_obj_handle = shape_prop.ambf_collision_shape_pointer
    if coll_shape_obj_handle is None:
        return
    if shape_prop.ambf_collision_shape == "BOX":
        print("Setting BOX scale", scale)
        shape_prop.ambf_collision_shape_xyz_dims = (
            shape_prop.ambf_collision_shape_xyz_dims * scale
        )
    elif shape_prop.ambf_collision_shape in ["CONE", "CYLINDER", "CAPSULE", "SPHERE"]:
        shape_prop.ambf_collision_shape_radius = (
            shape_prop.ambf_collision_shape_radius * scale
        )
        shape_prop.ambf_collision_shape_height = (
            shape_prop.ambf_collision_shape_height * scale
        )
    shape_prop.ambf_collision_shape_linear_offset = (
        shape_prop.ambf_collision_shape_linear_offset * scale
    )


for o in D.objects:
    if "Needle" in o.name:
        ambf_rigid_body_set_scale(o, 0.1)
