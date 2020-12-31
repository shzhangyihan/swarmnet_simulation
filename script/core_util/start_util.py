import os
from . import video_renderer
from . import batch_scheduler

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

def run_visualization(speed, motion_file, video_file):
    error_flag = 0
    prYellow("Rendering video from motion file \"" + motion_file + "\" with x" + speed + " speed to \"" + video_file + "\" ...")
    # if os.system("python3 ./src/visualization/video_renderer.py " + motion_file + " " + speed + " " + video_file) != 0: error_flag = 1
    if video_renderer.renderer(motion_file, float(speed), video_file) != 0: error_flag = 1
    if error_flag:
        prRed("Visualization failed!")
        return -1
    else:
        prGreen("Visualization finished!")
        return 0

def start_scheduler(config_file):
    error_flag = 0
    prYellow("Starting scheduler with config file \"" + config_file + "\"...")
    scheduler = batch_scheduler.Scheduler(config_file)
    if scheduler.start() != 0: error_flag = 1
    if error_flag:
        prRed("Scheduler failed!")
        return -1
    else:
        prGreen("Scheduler finished!")
        return 0