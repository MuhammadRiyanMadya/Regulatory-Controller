# Regulatory Controller

<p align="justify">
This repository is a collection of process control codes with the *first principle* method. Some of the most important codes are System Identification, Cascade Controller, PID, and P-only for an integrating process. Particularly, system identification and cascade controller help me a lot when facing real-world problems. System identification provides me a way to get the controller parameters such as process gain (Kp), process time (Ƭp), and dead time (Ѳp) from a system with acceptable accuracy. The cascade code offers me the ability to simulate complex controller configurations like cascade and gives me a chance to see disturbance influence or dead-time variation effects so I can have a better controller setting for the real-world one.
  
## System Identification - Extracting System's Kp, Ƭp, and Ѳp.
<p align="center">
<img src="https://github.com/MuhammadRiyanMadya/Regulatory-Controller/blob/main/png/sys_fit.png">
<img src="https://github.com/MuhammadRiyanMadya/Regulatory-Controller/blob/main/png/sys_fit2.png">
</p>
<p align="justify">
When we want to tune a closed-loop control system, the first thing we would like to know is the possible value of process gain, process time, and dead time. Getting those values may be a time-consuming task and most of the step tests cannot be conducted seamlessly for many reasons such as process interruption limitations and disturbance elimination difficulties. Therefore, extracting values from limited and low-quality data is very common in industry (particularly in Indonesia) but how to get maximum potential from scratch is what we are talking about.
  
## Cascde - Disturbance and Dead-Time Influence.
<p align="center">
<img src="https://github.com/MuhammadRiyanMadya/Regulatory_Controller/blob/main/responseselfdrive.png">
</p>



















### First Principle
<!-- ![alt text]()-->
<p align="center">
<img src="https://github.com/MuhammadRiyanMadya/Regulatory_Controller/blob/main/responseselfdrive.png">
</p>

### Machine Learning

<table border="1">
<td><img alt="image" src="https://github.com/MuhammadRiyanMadya/Regulatory_Controller/blob/main/responseselfdrive.png"></td>
</table>

<a href="https://github.com/MuhammadRiyanMadya/Regulatory_Controller/blob/main/responseselfdrive.png">
<img alt="image" src="https://github.com/MuhammadRiyanMadya/Regulatory_Controller/blob/main/responseselfdrive.png" width=13%></a>
<!>
