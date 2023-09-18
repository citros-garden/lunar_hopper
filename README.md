## Example overview 🌐 

![jpg](img/hopper0.jpg "Did you find a easter egg?)")

The Lunar Hopper project is a lunar exploration planning project aimed at solving the intricate problem of lunar hopper missions. It focuses on optimizing the spacecraft's trajectory across the whole flight. Leveraging state-of-the-art optimal problem-solving algorithms, specifically the MPOPT Python library, this project seeks to determine the most efficient path for maximizing the distance traveled during the mission. It empowers users to customize essential parameters such as spacecraft mass, fuel quantity, thrust, and specific impulse. 

## Mathematical explanation 🎓
This example provides an optimal trajectory solver for Lunar Hopper. The result computes by solving non-linear optimal control problems(OCP) in the standard Bolza form using pseudo-spectral collocation methods and adjusted using an additional real dynamic function. The OCP solver used in this example is MPOPT (based on IPOPT) library modified by Lulav Space team. You can find more information about MPOPT optimal control solving library on the MPOPT [GitHub](https://github.com/mpopt/mpopt) or [website](https://mpopt.readthedocs.io/en/latest/).

The main goal of the example is to find the optimal way to "hop" on the Moon as far as possible with given vessel parameters. The dynamic function for MPOPT is:

$$ F_{thrust} = \frac{F_{thrust}^{max} f_1}{m_{dry} + m_{fuel}}  
$$

$$ r_{inv} = \frac{1}{h + R_{L}}
$$

$$ g = \mu_{L} r_{inv}^2
$$

$$ v_h = \dot{h}
$$

$$ v_d = \dot{d}
$$

$$ \dot{v_h} = F\cos(f_0) -g + v_d^2 r_{inv} 
$$ 

$$ \dot{v_d} = F\sin(f_0) - v_h v_d r_{inv} 
$$

$$
\dot{m} = \frac {-F_{thrust}}{g_0*I_{sp}}
$$


$$
\begin{array}{|c|c|}
\hline
\text{Variable} & \text{Description} \\
\hline
v_h & \text{horizontal velocity} \\
v_d & \text{vertical velocity} \\
f_1, f_2 & \text{control values} \\
R_{L} & \text{Lunar radius} \\
h & \text{altitude above lunar surface level} \\
\mu_L & \text{standard gravitational parameter} \\
m & \text{mass} \\
F_{\text{thrust}} & \text{The amount of thrust} \\
g_0 & \text{gravity parameter} \\
I_{\text{sp}} & \text{Specific impulse} \\
\hline
\end{array}
$$


For this example terminal cost function, path constraints and terminal constraints functions used as well: 

1. Terminal contraints (we want to start from the given initial conditions and finish with the given final conditions):

$$
b_{min}^{(g)} \le b\big[x^{(1)}(t_0^{(1)}),...,x^{(P)}(t_0^{(P)}),t_0^{(1)},...,t_0^{(P)},x^{(1)}(t_f^{(1)}),...,x^{(P)}(t_f^{(1)}),...,t_f^{(P)},q^{(1)},...,q^{(P)},s \big] \le b_{max}^{(g)}
$$

2. Path constraints (we need to limit controls values within the maximum possible thrust):

$$
c_{min}^{(p)} \le c^{(p)} \Big[x^{(p)}, y^{(p)}, t^{(p)} \Big] \le c_{max}^{(p)},
$$

3. Cost function:

$$
J = \phi \big[x^{(1)}(t^{(1)}_0),..., x^{(P)}(t^{(P )}_0), t^{(1)}_0, . . . , t^{(P)}_0, x^{(1)}(t^{(1)}_f), . . . , x^{(P )}(t^{(P )}_f), t^{(1)}_f, . . . , t^{(P )}_f, q^{(1)}, . . . , q^{(P )}, s \big]
$$ 


## Code overview

The project consists of two main files: 
1. ```lunar_hopper.py``` - the main executable file of the project. This file defines a ROS 2 node named "lunar_hopper" for simulating the dynamics of a lunar hopper spacecraft. The node publishes the spacecraft's state, control commands, and time information.

    Key functionalities and components of the code include:
    * It inherits from the Node class provided by the RCLPy (ROS 2 Python client library) for creating a ROS node.

    * The node publishes data on the spacecraft's state, control inputs, and time via ROS topics.

    * Various parameters related to the lunar hopper and its mission are declared and initialized, such as fuel mass, dry mass, thrust, specific impulse, and more.

    * Dynamics functions, terminal constraints, and cost functions are defined to describe the behavior of the lunar hopper during different phases of its mission (takeoff, flight, landing).

    * The lopt.solve function is called to solve the optimal control problem for the lunar hopper's trajectory, taking into account the defined parameters and functions.

    * A timer callback function is implemented to periodically publish the simulation results on ROS topics.

    * The main function initializes the ROS 2 node, runs the simulation, and handles the node's lifecycle.

    Overall, this code serves as a ROS 2 node for simulating and controlling a lunar hopper's trajectory using an optimal control approach, making it a valuable tool for lunar mission planning and analysis.

2. ```lopt_hopper.py``` - OCP solver wrapper file. This Python code is designed to solve optimal control problems using the MPOPT Python library. It provides a single-phase optimal control simulation, focusing on spacecraft dynamics. Here is an overview of the code's functionality:

    * The code begins by importing necessary libraries, including MPOPT, Matplotlib, NumPy, and CasADi. It also defines functions for interpolating results and solving optimal control problems.

    * The solve function is the main entry point for solving an optimal control problem. It takes various parameters, including dynamics functions, cost functions, initial and target conditions, bounds, and more.

    * Inside the solve function, an optimal control problem (OCP) is initialized using the MPOPT library. Parameters such as the initial guess for simulation duration (ocp_tf0), initial and target state conditions (ocp_x00 and ocp_xf0), and bounds on states and controls are set.

    * The OCP's dynamics, terminal constraints, and terminal costs are defined based on the provided functions (dyn_func, term_constr, and term_cost).

    * The code proceeds to validate the OCP, ensuring that all parameters are correctly defined.

    * The simulation is then performed using the MPOPT library with adaptive settings. The solver iteratively refines the solution, and the results are processed.

    * Finally, the code performs interpolation on the simulation results to provide a smoother representation of states and controls over time.

    In summary, this code serves as a tool for solving optimal control problems related to spacecraft dynamics. It uses the MPOPT library and provides a flexible interface for specifying problem parameters and functions while delivering interpolated results for further analysis or visualization.


## Local Usage 💻

All project installation and usage information also available in the project [GitHub page](https://github.com/citros-garden/lunar_hopper).

### Installation 🔨
1. Docker engine. This project runs inside Docker container, and requires Docker Engine/Docker Desktop. Follow the instructions on [Docker official website](https://www.docker.com/get-started/).
2. To use Docker inside VS Code several extensions are required. Install [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) and [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) extensions from Extensions tab on your left control panel.
3. Clone the repository:
```bash 
git git@github.com:citros-garden/lunar_hopper.git
```

### Build 🏠
1. Open project root folder in VS Code.
2. Navigate to the lower-left corner of VS Code window and click on green mark.
3. Select "Reopen in container" option in the list on the top of the VS Code window. Wait a minute while Docker container is starting.
2. Open ```/src/lunar_hopper/config/params.xml``` file to set parameters for simulation or just keep it default. Don't forget to save your changes!
3. Build ROS 2 environment:
```bash 
colcon build
```
4. Source the environment:
```bash 
source install/local_setup.bash
```

### Preparing FoxGlove Studio 🪄
FoxGlove Studio is a robotics visualization and debugging tool, which can connect to ROS topic and get the data publishing through it. We will use it to visualizate the results of our simulations.

First of all, you need to download it from the [official website](https://foxglove.dev/) and install following the instructions. 

Next step is connecting to your ROS node. To perform it, open FoxGlove Studio and select *__Open connection__* option, then select *__Rosbridge__* option. Check the *__WebSocket URL__* field on the right of the window, it should contain ```ws://localhost:9090```. Now we are almost ready to go!

Last step is configuring the layout of FoxGlove. There are two ways to do it: using prepared layout file or manualy.

Manual plot layout configuration: we have 5 states in the output messages, so we need 5 plots. Add 5 plots using third tab on the left FoxGlove control panel. Then click on the fourth tab and set 'Message Path' for each plot: the path should be ``` /lunar_hopper/state.data[n] ```, where n - number of the state. Use ``` /lunar_hopper/control.data[m] ``` as a Message Path for control vector, where m - number of the control (0 or 1).

OR

:::tip

You can use prepared layout: Go to the ```Layout``` tab on the top panel, then click on import_layout button and select the file from foxglove_layouts folder.

:::

$$
\begin{array}{|c|c|c|}
\hline
\text{State number} & \text{Value} & \text{Describtion} \\
\hline
0 & h & \text{altitude above lunar surface level} \\
1 & d & \text{ground distance} \\
2 & v_h & \text{horizontal velocity} \\
3 & v_d & \text{vertical velocity} \\
4 & m_{fuel} & \text{fuel mass} \\
\hline
\end{array}
$$


:::tip

The best way to process simulation results is CITROS notebook 🍋 :)

:::


### Run 🚀
1. Go back to the VS Code.
2. Prepare your FoxGlove studio (previous step, if you haven't done it yet).
3. Launch ROS 2 package:
```bash 
ros2 launch lunar_hopper lunar_hopper.launch.py
```
4. Watch the FoxGlove plot built from results!

OR

:::tip

You can use Visual Code Tasks: simply press ```Alt+T``` and select ```Launch``` task to build, source and launch the project automaticly.

:::

<!-- ![gif](img/gif0.gif "FoxGlove example") -->

## CITROS usage 🛸
Although you can get simulation results using FoxGlove, the best way to work with such simulations and process the results is CITROS! With its power, it is possible to create complex data processing scenarios, including the construction of more complex graphs, mathematical analysis and other high-level processing methods.

### CITROS installation 🛫

First of all, to use all the powerfull CITROS features usage requires CITROS installation: follow the instructions on the CITROS CLI [GitHub page](https://github.com/lulav/citros_cli).

### Configuring the project ⚙️
After all the prerequisites done, we can start configuring our project. The starting point is the Lunar_Starship devcontainer loaded and running, CITROS CLI is installed and ready.
1. Initialize CITROS:
```bash 
citros init
```
Now you can see ```.citros``` folder in the explorer.

2. Configuring the setup. We need to set up the maximum perfomance available: timeout, CPU, GPU and Memory. To perform it, we need to define it in the ```.citros/simulations/simulation_lunar_hopper.json```. The recommended setup is minimum 600 seconds timeout, 4 CPU, 4 GPU and 4096 MB of Memory. Don't forget to save the file!

3. Configuring the params setup. You can find default setup in ```.citros/parameter_setups/default_param_setup.json```. Variables with '_0' are initial conditions, and variables with '_f' are final conditions respectively.

$$
\begin{array}{|c|c|}
\hline
\text{Parameter} & \text{Description} \\
\hline
m_{\text{fuel 0}} & \text{initial fuel mass} \\
m_{\text{fuel f}}& \text{final fuel mass} \\
dry~mass & \text{dry mass} \\
F_{\text{thrustmax}} & \text{The maximum amount of thrust} \\
I_{\text{sp}} & \text{Specific impulse} \\
publish~freq & \text{frequency of publishing} \\
\hline
\end{array}
$$

Don't forget to save the file!

### Syncing the project's setup 📡
Now we can sync our project settings with CITROS server:
```bash 
citros commit
citros push
```
### Running locally 🛋️
Since all the preparations done, we can launch it locally (your project should be built and sourced before that, check the instructions above):
```bash 
citros run -n 'Lunar_hopper' -m 'local test run'
```
Select the launch file (should be the only one here) by pressing ```Enter``` button and wait for the output in the terminal. To plot the local run results you can use FoxGlove.

### Uploading Docker image to the CITROS database and running in the cloud 🛰️
1. We need to build and push Docker container image to the CITROS server:
```bash 
citros docker-build-push
```

2. Finally, we can run it in the cloud! Simply add ```-r``` to the terminal command: 
```bash 
citros run -n 'Lunar_hopper' -m 'cloud test run' -r
```
Select the launch file (should be the only one here) by pressing ```Enter``` button. Now the simulation is running in the CITROS server, and it will upload results to the CITROS database automaticly.

### CITROS Web usage and data analysis 🌌
TODO
## Extras
### FoxGlove examples

<!-- ![png](img/img0.png "FoxGlove example") -->
<!-- ![png](img/img1.png "FoxGlove example") -->
