# <img src="https://github.com/bethesirius/ChosunTruck/blob/master/README/Logo.png" width="64">ChosunTruck

## Introduction
ChosunTruck is an autonomous driving solution for [Euro Truck Simulator 2](https://eurotrucksimulator2.com/).
Recently, autonomous driving technology has become a big issue and as a result we have been studying technology that incorporates this.
It is being developed in a simulated environment called Euro Truck Simulator 2 to allow us to study it using vehicles.
We chose Euro Truck Simulator 2 because this simulator provides a good test environment that is similar to the real road.

## Features
* You can drive a vehicle without handling it yourself.
* You can understand the principles of autonomous driving.
* (Experimental/Linux only) You can detect where other vehicles are.

## How To Run It
### Windows

#### Dependencies
- IDE: Visual Studio 2017
- [Cuda Toolkit 7.5](https://developer.nvidia.com/cuda-75-downloads-archive) 
  - Do an ADVANCED INSTALLATION. ONLY install the Toolkit + Integration to Visual Studio. Do NOT install the drivers + other stuff it would normally give you. Once installed, your project properties should look like this: https://i.imgur.com/e7IRtjy.png)

#### Required to allow input to work in Windows:
- **Go to C:\Users\YOURUSERNAME\Documents\Euro Truck Simulator 2\profiles and edit controls.sii from** 
```
config_lines[0]: "device keyboard `di8.keyboard`"
config_lines[1]: "device mouse `fusion.mouse`"
```
to 
```
config_lines[0]: "device keyboard `sys.keyboard`"
config_lines[1]: "device mouse `sys.mouse`"
```
(thanks Komat!)
- **While you are in controls.sii, make sure your sensitivity is set to:**
```
 config_lines[33]: "constant c_rsteersens 0.775000"
 config_lines[34]: "constant c_asteersens 4.650000"
```
#### Then:
- Set controls.sii to read-only
- Open the visual studio project and build it. 
- Run ETS2 in windowed mode and set resolution to 1024 * 768.(It will work properly with 1920 * 1080 screen resolution and 1024 * 768 window mode ETS2.)

### Linux
- This fork does not support the linux version of ChosunTruck. See [Parent Repository](https://github.com/bethesirius/ChosunTruck/)

## Troubleshooting
- See our [Troubleshooting wiki page](https://github.com/bethesirius/ChosunTruck/wiki/Troubleshooting).
- If you have a problem during installation, look at our [Windows Installation wiki page](https://github.com/bethesirius/ChosunTruck/wiki/Windows-Installation)
- If you have some problems running this project, [open a issue to contact our team](https://github.com/bethesirius/ChosunTruck/issues).

## Demo Video
Lane Detection (Youtube link)

[![youtube link](http://img.youtube.com/vi/vF7J_uC045Q/0.jpg)](http://www.youtube.com/watch?v=vF7J_uC045Q)
[![youtube link](http://img.youtube.com/vi/qb99czlIklA/0.jpg)](http://www.youtube.com/watch?v=qb99czlIklA)

Lane Detection + Vehicle Detection (Youtube link)

[![youtube link](http://img.youtube.com/vi/w6H2eGEvzvw/0.jpg)](http://www.youtube.com/watch?v=w6H2eGEvzvw)

## Founders
- Chiwan Song, chi3236@gmail.com

- JaeCheol Sim, simjaecheol@naver.com

- Seongjoon Chu, hs4393@gmail.com

## Contributors
- [zappybiby](https://github.com/zappybiby)

## How To Contribute
Anyone who is interested in this project is welcome! Just fork it and pull requests!

## License
ChosunTruck, Euro Truck Simulator 2 auto driving solution
Copyright (C) 2017 chi3236, bethesirius, uoyssim

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
