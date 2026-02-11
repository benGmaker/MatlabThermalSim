\# MATLAB Thermal Simulation Environment



A simulation framework for thermal control systems with advanced model-based and data-driven predictive control algorithms.



\## Overview



This repository implements a modular thermal control simulation environment designed for research and education in advanced process control. The framework provides a complete workflow from data collection and system identification through controller design and comparative performance evaluation.



The simulation environment models a thermal system inspired by the Temperature Control Laboratory (TCLab), featuring nonlinear thermal dynamics including convection and radiation heat transfer mechanisms.



\## Features



\### Control Algorithms

\- \*\*Model Predictive Control (MPC)\*\* - Uses MATLAB's MPC Toolbox for optimal control with constraints

\- \*\*Dynamic Matrix Control (DMC)\*\* - Step response-based predictive control implementation

\- \*\*Data-enabled Predictive Control (DeePC)\*\* - Direct data-driven predictive control without explicit model identification



\### System Identification

\- Multiple excitation signals: step response, impulse response, and multi-sine

\- Transfer function and state-space model estimation

\- Model validation with independent test datasets

\- Configurable measurement noise simulation



\### Thermal Model

\- First-principles energy balance equations

\- Configurable linear or nonlinear dynamics

\- Convective and radiative heat transfer

\- Parametric physical properties (heat capacity, transfer coefficients, surface area)



\### Experimental Design

\- Automated data collection workflows

\- Multi-sine excitation for frequency response characterization

\- Doublet test signals

\- Configurable setpoint tracking scenarios



\## Repository Structure



```

MatlabThermalSim/

├── config\_simulation.m          # Centralized configuration file

├── run\_closed\_loop\_test.m       # Main comparison script

├── src/

│   ├── controllers/             # Control algorithm implementations

│   │   ├── MPC\_controller.m    # Model Predictive Control

│   │   ├── DMC\_controller.m    # Dynamic Matrix Control

│   │   └── DeePC\_controller.m  # Data-enabled Predictive Control

│   ├── models/

│   │   └── thermal\_model.m     # Thermal dynamics simulation

│   ├── experiment/

│   │   ├── experiment\_data\_collection.m

│   │   └── system\_identification.m

│   ├── evaluation/

│   │   └── compare\_all\_controllers.m

│   └── utils/                   # Helper functions

└── example\_tcl/                 # TCLab reference implementations

```



\## Installation



\### Prerequisites

\- MATLAB R2020a or later

\- Required Toolboxes:

&nbsp; - Control System Toolbox

&nbsp; - System Identification Toolbox

&nbsp; - Model Predictive Control Toolbox

&nbsp; - Optimization Toolbox



\### Setup

Clone the repository and add it to your MATLAB path:



```matlab

addpath(genpath('path/to/MatlabThermalSim'))

```



\## Usage



\### Basic Workflow



1\. \*\*Configure Simulation Parameters\*\*

&nbsp;  

&nbsp;  Edit `config\_simulation.m` to set controller parameters, constraints, and simulation settings.



2\. \*\*Collect Experimental Data\*\*

&nbsp;  

&nbsp;  ```matlab

&nbsp;  config = config\_simulation();

&nbsp;  experiment\_data\_collection(config);

&nbsp;  ```



3\. \*\*Perform System Identification\*\*

&nbsp;  

&nbsp;  ```matlab

&nbsp;  system\_identification(config);

&nbsp;  ```



4\. \*\*Run Individual Controllers\*\*

&nbsp;  

&nbsp;  ```matlab

&nbsp;  MPC\_controller(config);

&nbsp;  DMC\_controller(config);

&nbsp;  DeePC\_controller(config);

&nbsp;  ```



5\. \*\*Compare Controller Performance\*\*

&nbsp;  

&nbsp;  ```matlab

&nbsp;  run\_closed\_loop\_test;

&nbsp;  ```



\### Configuration



All simulation parameters are centralized in `config\_simulation.m`:



```matlab

config.simulation.t\_sim = 600;           % Simulation time \[s]

config.simulation.dt = 1;                % Sampling time \[s]

config.setpoint.times = \[0, 200, 400];   % Setpoint changes \[s]

config.setpoint.values = \[40, 50, 35];   % Target temperatures \[°C]

```



Controller-specific parameters can be tuned for each algorithm:



```matlab

% MPC Parameters

config.MPC.P = 30;          % Prediction horizon

config.MPC.M = 10;          % Control horizon

config.MPC.Q\_weight = 1.0;  % Output tracking weight

config.MPC.R\_weight = 0.1;  % Control move suppression

```



\## Methodology



\### Thermal System Model



The system is governed by an energy balance equation incorporating convective and radiative heat transfer:



\*\*Nonlinear Model:\*\*

```

dT/dt = (αQ - UA(T - Ta) - εσA(T⁴ - Ta⁴)) / (mCp)

```



where:

\- T: heater temperature \[K]

\- Ta: ambient temperature \[K]

\- Q: heater input power \[%]

\- α: heater efficiency \[W/%]

\- U: heat transfer coefficient \[W/m²·K]

\- A: surface area \[m²]

\- m: mass \[kg]

\- Cp: heat capacity \[J/kg·K]

\- ε: emissivity coefficient

\- σ: Stefan-Boltzmann constant \[W/m²·K⁴]



\### Control Objectives



\- Track setpoint temperature changes with minimal overshoot

\- Satisfy input constraints (0-100% heater power)

\- Minimize control effort and temperature deviation

\- Maintain robust performance under model uncertainty



\### Performance Metrics



Results are evaluated using:

\- Integral Absolute Error (IAE)

\- Integral Square Error (ISE)

\- Total Variation (TV) of control signal

\- Settling time and overshoot characteristics



\## Results



The comparison framework generates:

\- Time-domain response plots for all controllers

\- Performance metric tables

\- Control effort visualization

\- Configuration documentation for reproducibility



Results are saved in the `results/` directory with timestamped filenames.



\## References



This implementation is based on the TCLab (Temperature Control Laboratory) platform developed at Brigham Young University:



\- Hedengren, J. D., \& Martin, R. A. (2020). Temperature Control Lab for Dynamics and Control. \*IFAC-PapersOnLine\*, 53(2), 17503-17508.



\## License



This project is available under standard academic use terms. Please contact the author for commercial applications.



\## Contact



For questions or collaboration inquiries, please open an issue on the GitHub repository.



\## Acknowledgments



The thermal model parameters and validation data are derived from the open-source TCLab project. Example implementations in `example\_tcl/` are adapted from APMonitor training materials.

