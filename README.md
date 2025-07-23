# ROS Pick‑and‑Place → SLAM Navigation (Franka Panda + 4WD UGV)

**Built and tested on ROS Noetic (Ubuntu 18.04).**

---

## Quick Start

Launch everything:

    roslaunch panda_moveit_config demo_gazebo.launch

Run the pick‑and‑place script:

    rosrun franka_example_controllers pick_operation.py

Extras:
- `nav.rviz` is an RViz config for the navigation robot.
---

## Video Demo

[![YouTube Demo](https://img.youtube.com/vi/XC-ISpkyecU/hqdefault.jpg)](https://youtu.be/XC-ISpkyecU)

---

## Requirements

- ROS Noetic (Ubuntu 18.04)
- Gazebo (from `ros-noetic-desktop-full`)
- MoveIt
- Navigation stack packages (`gmapping`, `move_base`, etc.)

Install missing deps:

    rosdep install --from-paths src --ignore-src -r -y

---

## Project Focus

End‑of‑course project integrating a **Franka Panda pick & place task** with **SLAM‑based autonomous navigation** of a 4‑wheel ground robot.  
Most effort went into modifying `panda_moveit_config/launch/demo_gazebo.launch` so both robots run seamlessly (namespacing, topic remaps, controllers, sensors).

---

## How It Works

1. **Gazebo bring-up** (`panda_moveit_config/launch/demo_gazebo.launch`): spawns the Panda arm + gripper (MoveIt‑controlled) and a 4WD UGV with laser + differential drive.
2. **Pick & Place** (`pick_operation.py`): executes approach → grasp → lift → place; sends gripper commands via MoveIt/action topics.
3. **SLAM & Navigation**: the UGV runs `gmapping` for mapping and `move_base` for global/local planning; YAML files tune costmaps and planners.
4. **Visualization**: `nav.rviz` displays the SLAM map, TF tree, robot model, and navigation paths.

---

## Sources / Credits

- Franka arm ROS packages: https://github.com/frankaemika/franka_ros  
- Panda MoveIt config: https://github.com/moveit/panda_moveit_config

See `LICENSE` and `THIRD_PARTY_LICENSES.md` for upstream license details.
