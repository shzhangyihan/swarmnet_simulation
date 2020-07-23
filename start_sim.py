#!/usr/bin/env python3

import os
import argparse

def prRed(skk): print("\n\033[91m{}\033[00m\n" .format(skk))
def prGreen(skk): print("\n\033[92m{}\033[00m\n" .format(skk))
def prYellow(skk): print("\n\033[93m{}\033[00m\n" .format(skk)) 


def clean_build():
    error_flag = 0
    prYellow("Cleaning the current build ...")
    if os.system("rm -rf build") != 0: error_flag = 1
    if error_flag:
        prRed("Cleaning failed!")
    else:
        prGreen("Cleaning done!")

def cmake_and_build():
    error_flag = 0
    prYellow("Start building the simulation ...")
    os.system("mkdir build")
    os.chdir("./build")
    if os.system("cmake ..") != 0: error_flag = 1
    if os.system("make") != 0: error_flag = 1
    os.chdir("..")
    if error_flag:
        prRed("Building failed!")
    else:
        prGreen("Building done!")

def run_simulation(config_file):
    error_flag = 0
    prYellow("Starting up the simulation with config file \"" + config_file + "\" ...")
    if os.system("./build/src/core/simulation " + config_file) != 0: error_flag = 1
    if error_flag:
        prRed("Simulation failed!")
    else:
        prGreen("Simulation finished!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Default entry for simulation')
    parser.add_argument("-c", "--clean", action="store_true", help="Clean the build")
    parser.add_argument("-b", "--build", action="store_true", help="Build simulation")
    parser.add_argument("-r", "--run", action="store", help="Run simulation")
    args = parser.parse_args()

    if args.clean:
        clean_build()

    if args.build:
        cmake_and_build()
    
    if args.run:
        run_simulation(args.run)


