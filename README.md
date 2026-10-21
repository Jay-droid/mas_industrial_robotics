# mas_industrial_robotics
this package is being used by the 
## Status

### Working modules

- mir_bringup
- mir_perception
  - mir_object_recognition
  - mir_ppt_detection

### Modules under development

- mir_navigation
- mir_manipulation
- mir_actions


## Setup

- Create a workspace
  ```bash
  mkdir -p ~/mir/src
  ```

- Clone the rolling branch.
  ```bash
  cd ~/mir/src

  git@github.com:Jay-droid/mas_industrial_robotics.git
  ```

- Clone the dependency packages
  ```bash
  cd ~/mir/src

  vcs import < mas_industrial_robotics/mir.repos
  ```

- Build the workspace
  ```bash
  cd ~/mir

  colcon build --symlink-install
  ```


ros2 launch mir_bringup robot.launch.py
```

