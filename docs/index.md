# MOdular MObile (MOMObot) Robot Documentation
[![Documentation Status](https://readthedocs.org/projects/momobot/badge/?version=latest)](https://momobot.readthedocs.io/en/latest/?badge=latest)

> Version 1.1
## Credits
This documentation has been put together with the combined efforts of members of the **SUTD Organisation of Autonomous Robotics**

### V1.0: 2018 - 2019

- [methylDragon](https://github.com/methylDragon)
- [Shine16](https://github.com/shine16)
- [Fasermaler](https://github.com/fasermaler)
- [imossim](https://github.com/imossim)
- Bryan Kong
- Low En
- [Senrli](https://github.com/senrli)

### V1.1: 2019 - 2020

- [Photon](https://github.com/1487quantum)
- [robobdo](https://github.com/robobdo)
- [Jia Hwee](https://github.com/SigmarusValkyrja)
- Jerremy
- [darthnoward](https://github.com/darthnoward)

## Introduction

MOMObot is a service AGV built for extensibility and to roam autonomously using ROS!

## Table Of Contents <a name="top"></a>

1. [Hardware](#1)    
   1.1 [Dimensions](#1.1)    
   1.2 [Specifications](#1.2)    
   1.3 [Gotchas, Hacky Stuff and Things to Take Note Of](#1.3)    
   1.4 [To Dos](#1.4)    
2. [Electronics](#2)    
   2.1 [Electronic BOM](#2.1)    
   2.2 [Start-Up, Shut-Down Procedure](#2.2)    
   2.3 [Gotchas, Hacky Stuff and Things to Take Note Of](#2.3)    
   2.4 [Circuit Diagrams](#2.4)    
   2.5 [Motor Tuning](#2.5)    
   2.6 [PID Tuning from the MOMObot side](#2.6)    
3. [Software](#3)    
   3.1 [Pre-Requisites](#3.1)    
   3.2 [Setting Up MOMObot](#3.2)    
   3.3 [Running MOMObot Capabilities](#3.3)    
   3.4 [Tuning MOMObot](#3.4)    
   3.5 [Console Commands](#3.5)    
   3.6 [Mapping](#3.6)    
   3.7 [Exporting Display to MOMObot from Computer](#3.7)    
   3.8 [Semantic Pose and Semantic Pose Sounder Packages](#3.8)    
   3.9 [Software To-Dos](#3.9)
