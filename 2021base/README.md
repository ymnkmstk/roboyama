This is a program provided as a basis for ET Robocon 2021 competition and is intended to be compiled for and executed on EV3RT/TOPPERS (HRP3) platform. 

In order to better balance modelling and programming, behavior tree is incorporated into this base program.
A behavior tree is a mathmatical model of plan execution frequently used in robotic and video games.  A number of game development platforms such as UE4 (Unreal Engine 4) is equipped with behavior trees, which are used to create artificial intelligence for NPC (non-playter characters).
This program uses a behavior tree to give AI capabilities to an EV3 robot.

As a basis for implementing a behavior tree, an existing c++ single header library by Par Arvidsson has been selected.  It is a light-weight implementation of a behavior tree suitable to be used on a platform like EV3RT/TOPPERS where computing resources are limited.  Note that the BrainTree header file included in this repository has been modified to avoid the use of shared_ptr, being executable on TOPPERS/EV3RT (HRP3) with Athrill.
You may retrieve the original verion of the library from Arvidsson's GitHub repository at the following URL:
  https://github.com/arvidsson/BrainTree

Enjoy your modelling and programming in ET Robocon 2021 competition and good luck!


Usage:

1. Place 2021base directory in your ETrobo workspace.
2. Start ETrobo environment.
3. In terminal, execute "make [right] app=2021base sim up" command.
4. When LOG_ON_CONSOL directive is defined, various application messages are written to consol;
  otherwise, they are written to the pseudo Bluetooth device.
  They can be observed by another terminal by "btcat [right] app=2021base" command.
  Look at _log() macro in "appusr.hpp" to see how it works.

Further information on ETrobo environment:
  https://github.com/ETrobocon/etrobo


ToDo:

1. update_task() is currently executed in every 4ms, set by PERIOD_UPD_TSK in app.h file.
  This seems too often for the simulation environment.  The guideline from the ET Robocon secretariat is to set it to 10ms at least; otherwise, various side effects might be observed during simulated running such as instable data feed from sensor devices.
2. When PERIOD_UPD_TSK is changed, PID values, i.e., P_CONST, I_CONST, and D_CONST in appusr.h,
  also have to be fine-tuned to make the robot properly trace the line.