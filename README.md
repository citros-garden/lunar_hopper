# General Information 🌍
This example provides an optimal trajectory solver for Lunar Hopper.The result computes by solving non-linear optimal control problems(OCP) in the standard Bolza form using pseudo-spectral collocation methods and adjusted using an additional real dynamic function. The OCP solver used in this example is MPOPT (based on IPOPT) library modified by Lulav Space team. 

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


You can define a "real" dynamic function to test the control values computed by solving non-linear optimal control problems(OCP). This function should have the same number of outputs. 

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


# Installation 🛫
1. Docker engine. This project runs inside Docker container, and requires Docker Engine/Docker Desktop. Follow the instructions on [Docker official website](https://www.docker.com/get-started/).
2. To use Docker inside VS Code several extensions are required. Install [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) and [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) extensions from Extensions tab on your left control panel.
3. Clone the repository:
```bash 
git clone git@github.com:citros-garden/lunar_hopper.git
```

# Build 🛰
1. Open project root folder in VS Code.
2. Navigate to the lower-left corner of VS Code window and click on green mark.
3. Select "Reopen in container" option in the list on the top of the VS Code window. Wait a minute while Docker container is starting.
2. Open ```/src/lunar_hopper/config/params.xml``` file to set parameters for simulation or just keep it default. Don't forget to save your changes!
3. Build ROS2 environment:
```bash 
colcon build
```
4. Source the environment:
```bash 
source install/local_setup.bash
```

# Preparing FoxGlove Studio 🪄
FoxGlove Studio is a robotics visualization and debugging tool, which can connect to ROS topic and get the data publishing through it. We will use it to visualizate the results of our simulations.

First of all, you need to download it from the [official website](https://foxglove.dev/) and install following the instructions. 

Next step is connecting to your ROS node. To perform it, open FoxGlove Studio and select *__Open connection__* option, then select *__Rosbridge__* option. Check the *__WebSocket URL__* field on the right of the window, it should contain ```ws://localhost:9090```. Now we are almost ready to go!

Last step is configuring the layout of FoxGlove. There are two ways to do it: using prepared layout file or manualy.


To use prepared layout: Go to the second tab on the left panel, then click on *__import_layout__* button and select the file from [foxglove_layouts](/foxglove_layouts/) folder. There are two files - for state vector and for control vector.

OR 

Manual plot layout configuration: we have 5 states in the output messages, so we need 5 plots. Add 5 plots using third tab on the left FoxGlove control panel. Then click on the fourth tab and set 'Message Path' for each plot: the path should be ``` /lunar_hopper/state.data[n] ```, where n - number of the state. Use ``` /lunar_hopper/control.data[m] ``` as a Message Path for control vector, where m - number of the control (0 or 1).

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

Although the best way to process simulation results is Citros notebook 🍋 :)

# Run 🚀
1. Go back to the VS Code.
2. Prepare your FoxGlove studio (previous step, if you haven't done it yet).
3. Launch ROS2 package:
```bash 
ros2 launch lunar_hopper lunar_hopper.launch.py
```
1. Watch the FoxGlove plot built from results!

# Citros usage 🛸
Although you can get simulation results using FoxGlove, the best way to work with such simulations and process the results is Citros! With its power, it is possible to create complex data processing scenarios, including the construction of more complex graphs, mathematical analysis and other high-level processing methods.


# Extras
## FoxGlove examples
![png](/docs/img/img0.png "FoxGlove example")
![png](/docs/img/img1.png "FoxGlove example")