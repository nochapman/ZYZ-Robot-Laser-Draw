# ZYZ Robot Laser Draw

Overview
- A MATLAB project that simulates a 3-DOF robot arm with a laser end-effector which "draws" shapes on a virtual wall. The code computes inverse kinematics, generates smooth interpolated joint trajectories, computes dynamics (joint torques), and animates the arm while plotting the laser trace on the wall.
![Heart Drawing](/data/Drawn-Heart-Looping.gif)

Key Features
- Inverse kinematics solver to compute joint targets for Cartesian waypoints.
- Constant-acceleration interpolation between waypoints for smooth motion profiles.
- Forward kinematics, Jacobian, and Newton–Euler dynamics for torque estimation.
- 3D animation of the arm and 2D plotting of the laser trace on a wall.
- Optionally convert image edges to waypoint traces and draw them.

Requirements
- MATLAB (R2018b or later recommended).
- No special toolboxes are required by the provided code; if you use extra tooling for image processing you may need the Image Processing Toolbox.

Quick Start
1. Open the project folder in MATLAB: the top-level script is `main.m`.
2. Choose the shape to draw by editing the `shape` variable near the top of `main.m` (for example `shape = 'heart';`) or set `use_image = true` and provide a path to an image inside the `data/` folder.
3. Run `main.m` in MATLAB. The animation window will show the arm, the wall, and the evolving laser trace.

Available shapes
- 'line', 'square', 'triangle', 'circle', 'cosine', 'heart', 'spiral' (default in some runs). You can add more by editing `get_shape.m`.

Using an image
- Set `use_image = true` in `main.m` and ensure the image path passed to `get_image_shape` is correct (e.g. `data\\apple_image.jpg`). The pipeline will extract edge waypoints and the arm will attempt to trace them.

Project layout (important files)
- `main.m`: top-level script that prepares targets, runs IK, interpolates motion, computes dynamics, and calls the animator.
- `create_linklist.m`, `createLink.m`: define robot link parameters and assemble the DH table.
- `dhTable.m`: central DH parameters used across FK/IK.
- `dhTransform.m`: build individual homogeneous transforms from DH params.
- `dhFwdKine.m`: forward kinematics for requested joint index.
- `dhInvKine.m`: inverse kinematics solver used to compute joint targets for Cartesian poses.
- `constAccelInterp.m`, `calCubicInterp.m`, `calCubicCoeffs.m`: interpolation helpers to produce smooth joint trajectories.
- `velocityJacobian.m`: velocity Jacobian and its derivative.
- `newtonEuler.m`: dynamics (recursive Newton–Euler) for torque estimation.
- `animateArm.m`: animation loop, wall plotting, and torque plotting.
- `drawArm.m`: draws the arm in 3D and computes laser tip position.
- `get_shape.m`, `get_image_shape.m`, `get_displacement.m`: utilities for generating target poses and geometry helpers.
- `data/`: example images and `Test_Cases.mat` used by the project.
- `Helper Functions/`: assorted small utilities (quaternions, rotations, dual quaternions, etc.).

Notes & Tips
- To change the interpolation granularity or timing, edit `time_between_points` and `total_samples` in `main.m`.
- The inverse-kinematics routine uses a numerical solver; if it fails for some waypoints try changing the `initial_guess` in `main.m` or adjust the desired Cartesian pose slightly.
- The plotting axes and wall position are defined in `animateArm.m` and `drawArm.m` (wall at x = 10 by default) — adjust those if you change link lengths.

Examples
- Draw the heart shape: set `shape = 'heart'` in `main.m` and run.
- Trace an image: set `use_image = true` and provide a valid image path in the `if use_image` branch of `main.m`.
![Figure showing result of the arm having drawn the Apple logo](/data/Drawn-Apple.png)
- Results are influenced by the link lengths specifiied in the DH table, the distance from the wrist to the wall, the input image, and the scaling factor
    - Sometimes, parameters result in computations that are outside of the dexterous workspace, and in these cases, proper interpolation is impossible. Resulting trajectories may be inaccurate as the arm rapidly switches between configurations.
    - Because the laser is awlays on, the arm will do its best to draw the final shape as accurately as possible given some extra connections between what would otherwise be disconnected points.
    - An example of how the system can misdraw:
![](/data/Drawn-WonkyFace.png)
- An example of the torque graph, showing the results of Newton-Euler Inverse Dynamics at each time step, after having drawn the spiral shape:
![](/data/Spiral-Torques.png)

Collaborators & Attribution
- This project was developed as part of a ROBO544 team project at the Colorado School of Mines. I worked alongside:
    - Zachary Royal
    - Oluwatosin Oseni

