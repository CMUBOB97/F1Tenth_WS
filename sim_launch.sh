#!/bin/bash

rocker --nvidia --x11 --name f1tenth_sim --volume .:/sim_ws/ -- f1tenth_gym_ros
