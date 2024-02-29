# Regulatory Controller

<p align="justify">
This repository is a collection of process control codes based on first principles.
  
## P-only controller
<p align="center">
<img src="https://github.com/MuhammadRiyanMadya/Regulatory-Controller/blob/main/png/p_only.png">
</p>
<p align="justify">
The proportional controller can be excellent for integrating processes but when there are setpoint changes it will not follow the setpoint.
  
## PI controller
<p align="center">
<img src="https://github.com/MuhammadRiyanMadya/Regulatory-Controller/blob/main/png/PI_Controller_1.png">
<img src="https://github.com/MuhammadRiyanMadya/Regulatory-Controller/blob/main/png/PI_controller_2.png">
</p>
<p align="justify">
The Proportional-Integral is commonly used in industry, it is reliable due to the ability to follow the setpoint

## PID controller for FOPTD
<p align="center">
<img src="https://github.com/MuhammadRiyanMadya/Regulatory-Controller/blob/main/png/pid_1.png">
<img src="https://github.com/MuhammadRiyanMadya/Regulatory-Controller/blob/main/png/pid_2.png">
</p>
<p align="justify">
The derivative controller could reduce the winding response caused by the integral component, but it has limitations and it is mainly not used for a system with high noise.
<p align="center">
<img src="https://github.com/MuhammadRiyanMadya/Regulatory-Controller/blob/main/png/pid_3.png">
</p>
<p align="justify">
Time delay is a very critical problem for process control, it reduces the performance of the PID controller and has to be considered when tuning the controller
<img src="https://github.com/MuhammadRiyanMadya/Regulatory-Controller/blob/main/png/pid_4.png">
</p>
<p align="justify">
Ultimately, when the system is time-delay-dominated, simulation for the controller response to get the best PID constants is necessary.


