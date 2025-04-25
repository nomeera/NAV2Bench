# NAV2Bench
> Navigation Benchmarking Package

The **NAV2Bench** is a ROS2 package for benchmarking navigation algorithms. It provides a set of tools to evaluate the performance of different navigation algorithms. The package includes a set of planners and controllers, as well as criteria for performance analysis, such as time taken, CPU usage, memory usage, path length, and safety. The package provides a simple interface via RViz2 for users to easily test and compare different navigation algorithms.

## Planners

- GridBased
- NavFn
- smac_planner
- ThetaStar
- Lattice

## Controllers

- FollowPath
- DWB
- RPP
- DWB_RSC
- RPP_RSC
- MPPI

## Criteria for performance analysis

- Time
- CPU
- Memory
- Path Length
- Safety

## Development

The package is currently under development and is not yet ready for production use. 


---
### Working Flow

- Modify package to work with sdf file (gz sim standard)
- Spawn robot in gz sim to reset it position
- Fix the issues that prevent package from launching
- Test amcl-localization and Tune parameters for best performance
- Test each planner separately and Tune parameters for best performance
- Test each controller separately and Tune parameters for best performance
- Benchmark planner-controller combinations.


### Finished
---

- Modify package to work with sdf file (gz sim standard)

    > [modify_sdf_file](/src/NAV2Bench/launch/petra_urdf_v7.launch.py#L15-L30)

    > [spawn_robot_node](/src/NAV2Bench/launch/petra_urdf_v7.launch.py#L139-L162)

- Fix the issues that prevent package from launching
- Test amcl-localization and Tune parameters for best performance
    > [amcl](/src/NAV2Bench/config/amcl_localization.yaml)

![](/src/NAV2Bench/resources/frames.png)

### In Progress
---

- Test DWA controller with GridBased planner and Tune parameters for best performance
    * Issues:
        - Robot rotates around itself and fails to reach the goal

> [video](/src/NAV2Bench/resources/Screencast%20from%202025-04-25%2007-05-57.mp4)


### Planning
---
- Develop a GUI for experiment management.
- Continuous improvements.

