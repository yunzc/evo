"""
Common functions for evo_ape and evo_rpe, internal only.
author: Michael Grupp

This file is part of evo (github.com/MichaelGrupp/evo).

evo is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

evo is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with evo.  If not, see <http://www.gnu.org/licenses/>.
"""

import logging

logger = logging.getLogger(__name__)

SEP = "-" * 80  # separator line

multirobot_reference_color = ['gray', 'black', 'red', 'blue', 'green']

def load_trajectories(args):
    from evo.core import sync
    from evo.tools import file_interface

    if args.subcommand == "tum":
        traj_ref = file_interface.read_tum_trajectory_file(args.ref_file)
        traj_est = file_interface.read_tum_trajectory_file(args.est_file)
        ref_name, est_name = args.ref_file, args.est_file
    elif args.subcommand == "kitti":
        traj_ref = file_interface.read_kitti_poses_file(args.ref_file)
        traj_est = file_interface.read_kitti_poses_file(args.est_file)
        ref_name, est_name = args.ref_file, args.est_file
    elif args.subcommand == "euroc":
        traj_ref = file_interface.read_euroc_csv_trajectory(args.state_gt_csv)
        traj_est = file_interface.read_tum_trajectory_file(args.est_file)
        ref_name, est_name = args.state_gt_csv, args.est_file
    elif args.subcommand == "bag":
        import os
        logger.debug("Opening bag file " + args.bag)
        if not os.path.exists(args.bag):
            raise file_interface.FileInterfaceException(
                "File doesn't exist: {}".format(args.bag))
        import rosbag
        bag = rosbag.Bag(args.bag, 'r')
        try:
            traj_ref = file_interface.read_bag_trajectory(bag, args.ref_topic)
            traj_est = file_interface.read_bag_trajectory(bag, args.est_topic)
            ref_name, est_name = args.ref_topic, args.est_topic
        finally:
            bag.close()
    else:
        raise KeyError("unknown sub-command: {}".format(args.subcommand))

    return traj_ref, traj_est, ref_name, est_name

def load_trajectories_multi(args):
    from evo.core import sync
    from evo.tools import file_interface

    if args.subcommand == "tum":
        traj_ref = file_interface.read_tum_trajectory_file(args.ref_file)
        traj_est = file_interface.read_tum_trajectory_file(args.est_file)
        ref_name, est_name = args.ref_file, args.est_file
    elif args.subcommand == "kitti":
        traj_ref = file_interface.read_kitti_poses_file(args.ref_file)
        traj_est = file_interface.read_kitti_poses_file(args.est_file)
        ref_name, est_name = args.ref_file, args.est_file
    elif args.subcommand == "euroc":
        traj_ref = file_interface.read_euroc_csv_trajectory(args.state_gt_csv)
        traj_est = file_interface.read_tum_trajectory_file(args.est_file)
        ref_name, est_name = args.state_gt_csv, args.est_file
    elif args.subcommand == "bag":
        import os
        logger.debug("Opening bag file " + args.bag)
        if not os.path.exists(args.bag):
            raise file_interface.FileInterfaceException(
                "File doesn't exist: {}".format(args.bag))
        import rosbag
        bag = rosbag.Bag(args.bag, 'r')
        try:
            ref_topics = args.ref_topic.split(' ')
            est_topics = args.est_topic.split(' ')
            traj_ref_list = [file_interface.read_bag_trajectory(bag, topic) for topic in ref_topics]
            traj_est_list = [file_interface.read_bag_trajectory(bag, topic) for topic in est_topics]
            ref_name, est_name = ref_topics, est_topics
        finally:
            bag.close()
    else:
        raise KeyError("unknown sub-command: {}".format(args.subcommand))

    return traj_ref_list, traj_est_list, ref_name, est_name


def get_pose_relation(args):
    from evo.core.metrics import PoseRelation
    pose_relation = None
    if args.pose_relation == "full":
        pose_relation = PoseRelation.full_transformation
    elif args.pose_relation == "rot_part":
        pose_relation = PoseRelation.rotation_part
    elif args.pose_relation == "trans_part":
        pose_relation = PoseRelation.translation_part
    elif args.pose_relation == "angle_deg":
        pose_relation = PoseRelation.rotation_angle_deg
    elif args.pose_relation == "angle_rad":
        pose_relation = PoseRelation.rotation_angle_rad
    return pose_relation


def get_delta_unit(args):
    from evo.core.metrics import Unit
    delta_unit = Unit.none
    if args.delta_unit == "f":
        delta_unit = Unit.frames
    elif args.delta_unit == "d":
        delta_unit = Unit.degrees
    elif args.delta_unit == "r":
        delta_unit = Unit.radians
    elif args.delta_unit == "m":
        delta_unit = Unit.meters
    return delta_unit


def plot(args, result, traj_ref, traj_est):
    from evo.tools import plot
    from evo.tools.settings import SETTINGS

    import matplotlib.pyplot as plt
    import numpy as np

    logger.debug(SEP)
    logger.debug("Plotting results... ")
    plot_mode = plot.PlotMode(args.plot_mode)

    # Plot the raw metric values.
    fig1 = plt.figure(figsize=SETTINGS.plot_figsize)
    if "seconds_from_start" in result.np_arrays:
        seconds_from_start = result.np_arrays["seconds_from_start"]
    else:
        seconds_from_start = None

    plot.error_array(
        fig1, result.np_arrays["error_array"], x_array=seconds_from_start,
        statistics={
            s: result.stats[s]
            for s in SETTINGS.plot_statistics if s not in ("min", "max")
        }, name=result.info["label"], title=result.info["title"],
        xlabel="$t$ (s)" if seconds_from_start else "index")

    # Plot the values color-mapped onto the trajectory.
    fig2 = plt.figure(figsize=SETTINGS.plot_figsize)
    ax = plot.prepare_axis(fig2, plot_mode)
    if args.ros_map_yaml:
        plot.ros_map(ax, args.ros_map_yaml, plot_mode)

    plot.traj(ax, plot_mode, traj_ref, style=SETTINGS.plot_reference_linestyle,
              color=SETTINGS.plot_reference_color, label='reference',
              alpha=SETTINGS.plot_reference_alpha)
    plot.draw_coordinate_axes(ax, traj_ref, plot_mode,
                              SETTINGS.plot_axis_marker_scale)

    if args.plot_colormap_min is None:
        args.plot_colormap_min = result.stats["min"]
    if args.plot_colormap_max is None:
        args.plot_colormap_max = result.stats["max"]
    if args.plot_colormap_max_percentile is not None:
        args.plot_colormap_max = np.percentile(
            result.np_arrays["error_array"], args.plot_colormap_max_percentile)

    plot.traj_colormap(ax, traj_est, result.np_arrays["error_array"],
                       plot_mode, min_map=args.plot_colormap_min,
                       max_map=args.plot_colormap_max,
                       title="Error mapped onto trajectory")
    plot.draw_coordinate_axes(ax, traj_est, plot_mode,
                              SETTINGS.plot_axis_marker_scale)
    fig2.axes.append(ax)

    plot_collection = plot.PlotCollection(result.info["title"])
    plot_collection.add_figure("raw", fig1)
    plot_collection.add_figure("map", fig2)
    if args.plot:
        plot_collection.show()
    if args.save_plot:
        plot_collection.export(args.save_plot,
                               confirm_overwrite=not args.no_warnings)
    if args.serialize_plot:
        logger.debug(SEP)
        plot_collection.serialize(args.serialize_plot,
                                  confirm_overwrite=not args.no_warnings)

def plot_multi(args, result, traj_ref_list, traj_est_list):
    from evo.tools import plot
    from evo.tools.settings import SETTINGS

    import matplotlib.pyplot as plt
    import numpy as np

    logger.debug(SEP)
    logger.debug("Plotting results... ")
    plot_mode = plot.PlotMode(args.plot_mode)

    figs = []
    # Plot the raw metric values.
    error_array_comb = np.array([])
    for i in range(len(result)):
        figs.append(plt.figure(figsize=SETTINGS.plot_figsize))
        if "seconds_from_start" in result[i].np_arrays:
            seconds_from_start = result[i].np_arrays["seconds_from_start"]
        else:
            seconds_from_start = None

        plot.error_array(
            figs[i], result[i].np_arrays["error_array"], x_array=seconds_from_start,
            statistics={
                s: result[i].stats[s]
                for s in SETTINGS.plot_statistics if s not in ("min", "max")
            }, name=result[i].info["label"], title=result[i].info["title"],
            xlabel="$t$ (s)" if seconds_from_start else "index")

    # Plot the values color-mapped onto the trajectory.
    figs.append(plt.figure(figsize=SETTINGS.plot_figsize))
    ax = plot.prepare_axis(figs[-1], plot_mode)
    if args.ros_map_yaml:
        plot.ros_map(ax, args.ros_map_yaml, plot_mode)

    if args.plot_colormap_min is None:
        args.plot_colormap_min = min([result[i].stats["min"] for i in range(len(result))])
    if args.plot_colormap_max is None:
        args.plot_colormap_max = max([result[i].stats["max"] for i in range(len(result))])
    if args.plot_colormap_max_percentile is not None:
        args.plot_colormap_max = np.percentile(
            error_array_comb, args.plot_colormap_max_percentile)

    traj_est_list_comb = np.array([])
    for i in range(len(result)):
        plot.traj(ax, plot_mode, traj_ref_list[i], style=SETTINGS.plot_reference_linestyle,
                  color=multirobot_reference_color[i], label='reference'+str(i),
                  alpha=0.8)
        plot.draw_coordinate_axes(ax, traj_ref_list[i], plot_mode,
                                  SETTINGS.plot_axis_marker_scale)

        plot.draw_coordinate_axes(ax, traj_est_list[i], plot_mode,
                                  SETTINGS.plot_axis_marker_scale)
        figs[-1].axes.append(ax)
    
    plot.traj_colormap_multi(ax, traj_est_list, [r.np_arrays["error_array"] for r in result],
                       plot_mode, min_map=args.plot_colormap_min,
                       max_map=args.plot_colormap_max,
                       title="Error mapped onto trajectory")

    plot_collection = plot.PlotCollection("Multi-robot APE analysis")
    for i in range(len(result)):
        plot_collection.add_figure("raw" + str(i), figs[i])
    plot_collection.add_figure("map", figs[-1])
    if args.plot:
        plot_collection.show()
    if args.save_plot:
        plot_collection.export(args.save_plot,
                               confirm_overwrite=not args.no_warnings)
    if args.serialize_plot:
        logger.debug(SEP)
        plot_collection.serialize(args.serialize_plot,
                                  confirm_overwrite=not args.no_warnings)
