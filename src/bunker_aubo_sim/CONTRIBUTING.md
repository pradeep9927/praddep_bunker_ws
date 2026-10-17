# Contributing

Thanks for taking an interest. Bug reports, questions and pull requests are all welcome.

## Reporting a problem

Open an issue with the bug report template. Please include your Ubuntu / ROS 2 / Gazebo
versions, whether you run natively or in WSL, the exact launch command, and the terminal output.
For simulation problems, the output of `ros2 run bunker_aubo_bringup smoke_check` helps a lot.

## Making a change

1. Fork the repository and create a branch from `main`.
2. Build and test in a ROS 2 Jazzy workspace:

   ```bash
   colcon build --symlink-install
   colcon test && colcon test-result --verbose
   ```

3. Keep the style of the surrounding code (PEP 8, 120 characters per line, parameters in YAML
   rather than in code). Add or update a test when you change behaviour.
4. Add a line to `CHANGELOG.md` and open a pull request describing what changed and how you
   checked it.

By contributing you agree that your contribution is licensed under the Apache-2.0 licence of
this repository.
