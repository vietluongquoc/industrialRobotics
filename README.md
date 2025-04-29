# Industrial Robotics
This is the python application to control a robot arm in course "Industrial Robotics"

## Infor
Author: vietlq@huit.edu.vn
CoAuthor: vuonhiendai@gmail.com
Start date: 19/04/2025

## Requirement
- python 3.09
- i2cpy
- Qt5
- pca 9685
- usbi2c 341T

## How to run
- install requirement

```
    <!-- create environment -->
    python -m venv venv
    <!-- create active -->
    call venv/Scripts/activate
    <!-- install package -->
    pip install pyqt5 pyqt5-tools
    <!-- use tools -->
    pyqt5-tools designer
    <!-- covert ui to tool -->
    pyuic5 -o ver01.py ver250420.ui 
    <!-- i2cpy -->
    pip install i2cpy
    <!-- numpy -->
    pip install numpy

```

## Update
### ver. 1.250429
- fixed bug and improve the performance
- list to do: add to the GUI
- fixed the i2cpy error
- Add displace in GUI
- join anlge is finished

### ver. 1.250424
- create Project
- create GUI  --- FINISHED
- Test connection __ FINISHED
- Upload the Kinematic RObot -- FInished
list to do: add to the GUI
### ver. 1.250419
- create Project
- create GUI  --- FINISHED
- Test connection __ FINISHED
- create the DH table and control

