This is a program provided as a basis for ET Robocon 2021 competition and is intended to be compiled for and executed on EV3RT/TOPPERS (HRP3) platform. 

In order to better balance modelling and programming, behavior tree is incorporated into this base program.
A behavior tree is a mathmatical model of plan execution frequently used in robotic and video games.  A number of game development platforms such as UE4 (Unreal Engine 4) is equipped with behavior trees, which are used to create artificial intelligence for NPC (non-playter characters).
This program uses a behavior tree to give AI capabilities to an EV3 robot.

As a basis for implementing a behavior tree, an existing c++ single header library by Par Arvidsson has been selected.  It is a light-weight implementation of a behavior tree suitable to be used on a platform like EV3RT/TOPPERS where computing resources are limited.  Note that the BrainTree header file included in this repository has been modified to avoid the use of shared_ptr, being executable on TOPPERS/EV3RT (HRP3) with Athrill.
You may retrieve the original verion of the library from Arvidsson's GitHub repository at the following URL:
  https://github.com/arvidsson/BrainTree

Enjoy your modelling and programming in ET Robocon 2021 competition and good luck!
