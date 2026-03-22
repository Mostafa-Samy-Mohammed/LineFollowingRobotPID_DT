# LineFollowingRobotPID_DT

This repository contains a digital-twin based line-following robot project that uses PID control with a differential-drive setup.

The implementation is centered in `diffDriveLineFollower_v8/lineFollowerTwin/`, which includes the simulation twin definition, communication gateways, server code, and Python client modules.

## Repository Contents

- `diffDriveLineFollower_v8/vsiBuildCommands`: Build command helper/script used for compiling or launching the VSi-based twin environment.
- `diffDriveLineFollower_v8/lineFollowerTwin/`: Main digital twin project folder, including Makefiles, `.dt` and `.vsi` definitions, and integration sources.
- `diffDriveLineFollower_v8/lineFollowerTwin/src/`: Runtime source modules.
	- `controller/`: PID-based line-following control logic client.
	- `simulator/`: Robot/environment simulation client.
	- `visualizer/`: Visualization client for observing robot behavior and signals.
	- `server/`: Fabric server implementation used by the clients for communication.
- `Report/capstoneProject_DT_report (1).pdf`: Project report document describing the capstone implementation and results.

## Notes

The three main Python-side clients are the controller, simulator, and visualizer modules in `src/`, while the server module provides the backend communication layer in the twin runtime.

## How to Run the Simulation

1. Open a terminal in `diffDriveLineFollower_v8/lineFollowerTwin/`.
2. Run:

	```bash
	make build && make sim
	```

	This command builds the project files and opens the server (Innexis VSI Simulator Control). The server controls communication between the simulation clients through the Python2DtEthernet gateway as part of the VSI backplane, and launches the 3 simulation clients.

3. In the Innexis VSI window, run:

	```text
	run
	```

Running the simulation this way keeps it active continuously, and the visualizer generates plots for path tracking and error versus time.

Simulation controls in the same Innexis VSI window:

- Stop continuous simulation: press `Ctrl + C`.
- Advance one simulation time step:

	```text
	step
	```

- Terminate the simulation:

	```text
	exit
	```

##Demo
Demo run of the S-curved path tracking with low-level applied noise disturbance (std dev. = 0.01 m) on the robot's real-time position feedback of the simulator:

https://github.com/user-attachments/assets/22e1fbf4-66fd-4e8f-aaa5-12a6d123bd31

