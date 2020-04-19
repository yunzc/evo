#!/usr/bin/env python
# -*- coding: UTF8 -*-
# PYTHON_ARGCOMPLETE_OK
"""
Extending ape to multirobot 
Author: Yun Chang 
"""

from __future__ import print_function

import logging

logger = logging.getLogger(__name__)

SEP = "-" * 80  # separator line


def parser():
    import argparse
    basic_desc = "Absolute pose error (APE) metric app for multirobot"
    lic = "(c) evo authors and yunchang"

    shared_parser = argparse.ArgumentParser(add_help=False)
    algo_opts = shared_parser.add_argument_group("algorithm options")
    output_opts = shared_parser.add_argument_group("output options")
    usability_opts = shared_parser.add_argument_group("usability options")

    algo_opts.add_argument(
        "-r", "--pose_relation", default="trans_part",
        help="pose relation on which the APE is based",
        choices=["full", "trans_part", "rot_part", "angle_deg", "angle_rad"])
    algo_opts.add_argument("-a", "--align",
                           help="alignment with Umeyama's method (no scale)",
                           action="store_true")
    algo_opts.add_argument("-s", "--correct_scale", action="store_true",
                           help="correct scale with Umeyama's method")
    algo_opts.add_argument(
        "--align_origin",
        help="align the trajectory origin to the origin of the reference "
        "trajectory", action="store_true")

    output_opts.add_argument(
        "-p",
        "--plot",
        action="store_true",
        help="show plot window",
    )
    output_opts.add_argument(
        "--plot_mode", default="xyz", help="the axes for plot projection",
        choices=["xy", "xz", "yx", "yz", "zx", "zy", "xyz"])
    output_opts.add_argument(
        "--plot_colormap_max", type=float,
        help="the upper bound used for the color map plot "
        "(default: maximum error value)")
    output_opts.add_argument(
        "--plot_colormap_min", type=float,
        help="the lower bound used for the color map plot "
        "(default: minimum error value)")
    output_opts.add_argument(
        "--plot_colormap_max_percentile", type=float,
        help="percentile of the error distribution to be used "
        "as the upper bound of the color map plot "
        "(in %%, overrides --plot_colormap_max)")
    output_opts.add_argument(
        "--plot_full_ref",
        action="store_true",
        help="plot the full, unsynchronized reference trajectory",
    )
    output_opts.add_argument(
        "--ros_map_yaml", help="yaml file of an ROS 2D map image (.pgm/.png)"
        " that will be drawn into the plot", default=None)
    output_opts.add_argument("--save_plot", default=None,
                             help="path to save plot")
    output_opts.add_argument("--serialize_plot", default=None,
                             help="path to serialize plot (experimental)")
    output_opts.add_argument("--save_results",
                             help=".zip file path to store results")
    output_opts.add_argument("--logfile", help="Local logfile path.",
                             default=None)
    usability_opts.add_argument("--no_warnings", action="store_true",
                                help="no warnings requiring user confirmation")
    usability_opts.add_argument("-v", "--verbose", action="store_true",
                                help="verbose output")
    usability_opts.add_argument("--silent", action="store_true",
                                help="don't print any output")
    usability_opts.add_argument(
        "--debug", action="store_true",
        help="verbose output with additional debug info")
    usability_opts.add_argument(
        "-c", "--config",
        help=".json file with parameters (priority over command line args)")

    main_parser = argparse.ArgumentParser(
        description="{} {}".format(basic_desc, lic))
    sub_parsers = main_parser.add_subparsers(dest="subcommand")
    sub_parsers.required = True

    kitti_parser = sub_parsers.add_parser(
        "kitti", parents=[shared_parser],
        description="{} for KITTI pose files - {}".format(basic_desc, lic))
    kitti_parser.add_argument("ref_file",
                              help="reference pose file (ground truth)")
    kitti_parser.add_argument("est_file", help="estimated pose file")

    tum_parser = sub_parsers.add_parser(
        "tum", parents=[shared_parser],
        description="{} for TUM trajectory files - {}".format(basic_desc, lic))
    tum_parser.add_argument("ref_file", help="reference trajectory file")
    tum_parser.add_argument("est_file", help="estimated trajectory file")

    euroc_parser = sub_parsers.add_parser(
        "euroc", parents=[shared_parser],
        description="{} for EuRoC MAV files - {}".format(basic_desc, lic))
    euroc_parser.add_argument(
        "state_gt_csv",
        help="ground truth: <seq>/mav0/state_groundtruth_estimate0/data.csv")
    euroc_parser.add_argument("est_file",
                              help="estimated trajectory file in TUM format")

    bag_parser = sub_parsers.add_parser(
        "bag", parents=[shared_parser],
        description="{} for ROS bag files - {}".format(basic_desc, lic))
    bag_parser.add_argument("bag", help="ROS bag file")
    bag_parser.add_argument("ref_topic", help="string of a list of reference trajectory topics separated by spaces")
    bag_parser.add_argument("est_topic", help="string of a list of estimated trajectory topics separated by spaces")

    # Add time-sync options to parser of trajectory formats.
    for trajectory_parser in {bag_parser, euroc_parser, tum_parser}:
        trajectory_parser.add_argument(
            "--t_max_diff", type=float, default=0.01,
            help="maximum timestamp difference for data association")
        trajectory_parser.add_argument(
            "--t_offset", type=float, default=0.0,
            help="constant timestamp offset for data association")

    return main_parser


def ape(traj_ref_list, traj_est_list, pose_relation, align=False, correct_scale=False,
        align_origin=False, ref_name=["reference"], est_name=["estimate"]):
    from evo.core import metrics
    from evo.core import trajectory

    # Align the trajectories.
    only_scale = correct_scale and not align
    if align or correct_scale:
        logger.debug(SEP)
        for i in range(len(traj_ref_list)):
            print(traj_est_list[i], traj_ref_list[i])
            traj_est_list[i] = trajectory.align_trajectory(traj_est_list[i], traj_ref_list[i],
                                               correct_scale, only_scale)
    elif align_origin:
        logger.debug(SEP)
        for i in range(len(traj_ref_list)):
            traj_est_list[i] = trajectory.align_trajectory_origin(traj_est_list[i], traj_ref_list[i])

    # Calculate APE.
    logger.debug(SEP)
    data = [(traj_ref_list[i], traj_est_list[i]) for i in range(len(traj_ref_list))]
    ape_metric = [metrics.APE(pose_relation) for i in range(len(traj_ref_list))]
    for i in range(len(traj_ref_list)):
        ape_metric[i].process_data(data[i])

    ape_result = [ape_metric[i].get_result(ref_name[i], est_name[i]) for i in range(len(traj_ref_list))]
    for i in range(len(traj_ref_list)):
        title = str(ape_metric[i])
        if align and not correct_scale:
            title += "\n(with SE(3) Umeyama alignment)"
        elif align and correct_scale:
            title += "\n(with Sim(3) Umeyama alignment)"
        elif only_scale:
            title += "\n(scale corrected)"
        elif align_origin:
            title += "\n(with origin alignment)"
        else:
            title += "\n(not aligned)"
        ape_result[i].info["title"] = title

        logger.debug(SEP)
        logger.info(ape_result[i].pretty_str())

        ape_result[i].add_trajectory(ref_name[i], traj_ref_list[i])
        ape_result[i].add_trajectory(est_name[i], traj_est_list[i])
        if isinstance(traj_est_list[i], trajectory.PoseTrajectory3D):
            seconds_from_start = [
                t - traj_est_list[i].timestamps[0] for t in traj_est_list[i].timestamps
            ]
            ape_result[i].add_np_array("seconds_from_start", seconds_from_start)
            ape_result[i].add_np_array("timestamps", traj_est_list[i].timestamps)

    return ape_result


def run(args):
    import evo.common_ape_rpe as common
    from evo.core import sync
    from evo.tools import file_interface, log
    from evo.tools.settings import SETTINGS

    log.configure_logging(args.verbose, args.silent, args.debug,
                          local_logfile=args.logfile)
    if args.debug:
        from pprint import pformat
        parser_str = pformat({arg: getattr(args, arg) for arg in vars(args)})
        logger.debug("main_parser config:\n{}".format(parser_str))
    logger.debug(SEP)

    traj_ref_list, traj_est_list, ref_name, est_name = common.load_trajectories_multi(args)

    for i in range(len(traj_ref_list)):
        traj_ref_full = None
        if args.plot_full_ref:
            import copy
            traj_ref_full = copy.deepcopy(traj_ref_list[i])

        if args.subcommand != "kitti":
            logger.debug("Synchronizing trajectories...")
            traj_ref_list[i], traj_est_list[i] = sync.associate_trajectories(
                traj_ref_list[i], traj_est_list[i], args.t_max_diff, args.t_offset,
                first_name=ref_name[i], snd_name=est_name[i])

    pose_relation = common.get_pose_relation(args)

    result = ape(
        traj_ref_list=traj_ref_list,
        traj_est_list=traj_est_list,
        pose_relation=pose_relation,
        align=args.align,
        correct_scale=args.correct_scale,
        align_origin=args.align_origin,
        ref_name=ref_name,
        est_name=est_name,
    )

    if args.plot or args.save_plot or args.serialize_plot:
        common.plot_multi(args, result,
                    traj_ref_list,
                    traj_est_list)

    # if args.save_results:
    #     logger.debug(SEP)
    #     if not SETTINGS.save_traj_in_zip:
    #         del result.trajectories[ref_name]
    #         del result.trajectories[est_name]
    #     file_interface.save_res_file(args.save_results, result,
    #                                  confirm_overwrite=not args.no_warnings)


if __name__ == '__main__':
    from evo import entry_points
    entry_points.multi_ape()
