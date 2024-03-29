#!/usr/bin/env python3

import sys
from os import listdir, path

import trimesh
from pcg_gazebo.parsers.sdf import SDF, create_sdf_element

# Note: Both `trimesh` and `pcg_gazebo` can be installed via `pip`
# `trimesh` is used to estimate volume and inertial properties from meshes of links
# `pcg_gazebo` is used for its SDF parser


def main():

    # Total mass taken from datasheet (given as ~18.0kg)
    # You can also use your own estimate of total mass if you managed to weigh Summit XL-GEN yourself :)
    total_mass = 50.0
    if len(sys.argv) > 1:
        if float(sys.argv[1]) > 0.0:
            total_mass = float(sys.argv[1])
        else:
            print(
                "Error: Total mass of Summit XL-GEN (first argument) must be positive."
            )
            exit(1)
    print(
        "Estimating inertial properties for each link to add up to %f kg" % total_mass
    )

    # ## Get path to all visual meshes (automatic mode)
    # visual_mesh_dir = path.join(
    #     path.dirname(path.dirname(path.realpath(__file__))),
    #     "lunalab_summit_xl_gen",
    #     "meshes",
    #     "summit_xl",
    #     "visual",
    # )
    # visual_mesh_basenames = listdir(visual_mesh_dir)
    # visual_mesh_basenames.sort()

    ## Get path to specific visual meshes (manual mode)
    visual_mesh_dir = path.join(
        path.dirname(path.dirname(path.realpath(__file__))),
        "lunalab_summit_xl_gen",
        "meshes",
        "summit_xl",
        "visual",
    )
    visual_mesh_basenames = ["summit_xl_chassis.dae"]
    visual_mesh_basenames.sort()

    # Load all meshes
    meshes = {}
    for mesh_basename in visual_mesh_basenames:
        link_name = path.splitext(mesh_basename)[0]
        mesh_path = path.join(visual_mesh_dir, mesh_basename)
        meshes[link_name] = trimesh.load(mesh_path, force="mesh", ignore_materials=True)

    # Compute the total volume of the robot in order to estimate the required density
    total_volume = 0.0
    for link_name in meshes:
        mesh = meshes[link_name]
        total_volume += mesh.volume
        print("Volume estimate of %s: %f m^3" % (link_name, mesh.volume))

    # Compute average density
    average_density = total_mass / total_volume
    print("Average density estimate: %f kg/m^3" % average_density)

    # Estimate inertial properties for each link
    mass = {}
    inertia = {}
    centre_of_mass = {}
    for link_name in meshes:
        mesh = meshes[link_name]
        mesh.density = average_density
        mass[link_name] = mesh.mass
        inertia[link_name] = mesh.moment_inertia
        centre_of_mass[link_name] = mesh.center_mass

    # Create a new SDF with one model
    sdf = SDF()
    sdf.add_model(name="lunalab_summit_xl_gen")
    model = sdf.models[0]

    # Set inertial properties for each link into the SDF
    for link_name in meshes:
        link = create_sdf_element("link")
        link.mass = mass[link_name]
        link.inertia.ixx = inertia[link_name][0][0]
        link.inertia.iyy = inertia[link_name][1][1]
        link.inertia.izz = inertia[link_name][2][2]
        link.inertia.ixy = inertia[link_name][0][1]
        link.inertia.ixz = inertia[link_name][0][2]
        link.inertia.iyz = inertia[link_name][1][2]
        link.inertial.pose = [
            centre_of_mass[link_name][0],
            centre_of_mass[link_name][1],
            centre_of_mass[link_name][2],
            0.0,
            0.0,
            0.0,
        ]
        model.add_link(link_name, link)

    # Write into output file
    output_file = "lunalab_summit_xl_gen_inertial_out.sdf"
    sdf.export_xml(output_file)
    print('Results written into "%s"' % output_file)


if __name__ == "__main__":
    main()
