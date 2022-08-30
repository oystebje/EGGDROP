# EGGDROP
Cybernetics egg drop project for learning system identification

CHALLENGE:
Lower an Egg from 20cm height to the ground without breaking the egg. Fastest time wins. 

COMPETITION RULES:
Use Sparkfun DC motor ROB-11696 (https://www.sparkfun.com/products/11696)
You can break as many eggs as you like while training.
You have three attempts in the competition.

SETUP:

![image](https://user-images.githubusercontent.com/78421762/131328160-02fde334-3e22-4bac-aa96-b582d948aa07.png)


The task is to estimate system parameters for a DC motor using an Arduino Uno microcontroller. The project consists of the following steps:
1) Set up the circuit with components from provided schematic.
2) Write Arduino program code for controlling the DC motor with Pololu TB67H420FTG motor driver, and record shaft angle using rotary AMT 102V encoder.
2) Perform experiments for generating input (voltage) and outputs (shaft angle). Save in- and outputs to file that can later be imported to Matlab (we should provide an independent validation dataset).
3) Import data in Matlab, and perform system identification.
4) Tune PID-controller with identified system, and try it out on the DC-motor.
5) Present findings in final presentation, and compete in egg drop challenge!

Model structure:

![sisID](https://user-images.githubusercontent.com/78421762/186128136-da18a0d2-cd17-4c4b-813f-32da16243a2e.PNG)
