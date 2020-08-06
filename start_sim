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
    prGreen("Cleaning done!")
    return 0

def cmake_and_build(target_name = ""):
    error_flag = 0
    if target_name == "":
        prYellow("Start building the simulation ...")
    else:
        prYellow("Start building target " + target_name + " ...")
    os.system("mkdir build")
    os.chdir("./build")
    if os.system("cmake ..") != 0: error_flag = 1
    if target_name == "":
        if os.system("make") != 0: error_flag = 1
    else:
        if os.system("make " + target_name) != 0: error_flag = 1
    os.chdir("..")
    if error_flag:
        prRed("Building failed!")
        return -1
    else:
        prGreen("Building done!")
        return 0

def run_simulation(config_file):
    error_flag = 0
    prYellow("Starting up the simulation with config file \"" + config_file + "\" ...")
    if os.system("./build/src/core/simulation " + config_file) != 0: error_flag = 1
    if error_flag:
        prRed("Simulation failed!")
        return -1
    else:
        prGreen("Simulation finished!")
        return 0

def run_unit_test(test_name):
    error_flag = 0
    prYellow("Starting up unit test " + test_name + " ...")
    if os.system("./build/src/unit_test/" + test_name + "/" + test_name) != 0: error_flag = 1
    if error_flag:
        prRed("Unit test " + test_name + " failed!")
    else:
        prGreen("Unit test " + test_name + " finished!")

def main():
    parser = argparse.ArgumentParser(description='Default entry for simulation')
    parser.add_argument("-c", "--clean", action="store_true", help="Clean the build")
    parser.add_argument("-b", "--build", action="store_true", help="Build simulation")
    parser.add_argument("-r", "--run", action="store", help="Run simulation")
    parser.add_argument("-u", "--unit_test", action="store", help="Run unit test")

    args = parser.parse_args()

    if args.clean:
        clean_build()

    if args.build:
        if cmake_and_build() != 0:
            return
    
    if args.run:
        if run_simulation(args.run) != 0:
            return

    if args.unit_test:
        if cmake_and_build(args.unit_test) != 0:
            return
        if run_unit_test(args.unit_test) != 0:
            return

if __name__ == "__main__":
    main()



