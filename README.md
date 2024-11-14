# Optical Flow Vector Calculation

Costa Rica 

[![GitHub](https://img.shields.io/badge/--181717?logo=github&logoColor=ffffff)](https://github.com/)
[brown9804](https://github.com/brown9804)

Last updated: 2020-04-14

------------------------------------------

<div align="center">
  <h3 style="color: #4CAF50;">Total Visitors</h3>
  <img src="https://profile-counter.glitch.me/brown9804/count.svg" alt="Visitor Count" style="border: 2px solid #4CAF50; border-radius: 5px; padding: 5px;"/>
</div>

> This repository contains a program that calculates two-dimensional positions and the optical flow vector. The program is written in C.

## Files

- `header.h`: Contains the header files and function prototypes.
- `prototipo_func.h`: Contains the function prototypes.
- `structs.h`: Contains the structure definitions.
- `main.c`: Contains the main function of the program.

## Program Description

The program calculates the position of a point with respect to a reference coordinate system using a pose transformation for two instances (zero and one). It also calculates the projections of the point on the camera plane at both instances and the optical flow vector.

The program reads the control flow parameters of the program from a text file and stores them in a container. It calculates the rotation matrix for both instances and the three-dimensional position at both instances. The program also calculates the two-dimensional position of the projection of the point on the camera plane at both instances and the optical flow vector.

## Usage

Compile the program using a C compiler and run the executable. The program will display the results in the terminal.
