from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np
import pandas as pd
import sys
import pathlib
from PIL import Image
from PIL import ImageOps
import subprocess
import traceback
import math

log_file = ""
events = {}
robot_list = []
arena_width = 0
arena_height = 0
gl_width = 0
gl_height = 0
num_robots = 0
speed = 1
max_time = 0
RADIUS = 7
fps = 24
frame_folder = "./tmp_frames/"

gl_margin_bot = 30
gl_margin_others = 10
arena_margin_size = 3

class Robot:
    def set_last_updated_time(self, time):
        self.last_updated_time = time
    
    def set_attributes(self, x, y, theta, v, r, g, b, log):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.r = r
        self.g = g
        self.b = b
        self.log = log
    
    def __init__(self, id):
        self.id = id

def init():
    global num_robots
    global events
    global robot_list
    global log_file
    global arena_width
    global arena_height
    global gl_width
    global gl_height
    global max_time
    global gl_margin_bot
    global gl_margin_others

    metadata = pd.read_csv(log_file, sep='\t', nrows=1, header=0)
    arena_width = metadata["arena_max_x"][0]
    arena_height = metadata["arena_max_y"][0]
    num_robots = int(metadata["num_robots"][0])
    max_time = metadata["duration"][0]

    gl_width = int((math.ceil(arena_width) + 1) / 2) * 2 + gl_margin_others * 2
    gl_height = int((math.ceil(arena_height) + 1) / 2) * 2 + gl_margin_bot + gl_margin_others

    robot_list = []
    for i in range(num_robots):
        robot_list += [Robot(i)]

    events = {}
    logs = pd.read_csv(log_file, sep='\t', skiprows=2, header=0, keep_default_na=False)
    for index, row in logs.iterrows():
        log = [e for e in row]
        #log_time = int(log[0])
        log_time = round(log[0], 2)
        id = int(log[1])
        if log_time not in events:
            events[log_time] = {}
        events[log_time][id] = log
    
    # print(len(events))
    
    init_event = events.pop(0)
    for id in init_event:
        log = init_event[id]
        robot_list[id].set_attributes(log[2], log[3], log[4], log[5], log[6], log[7], log[8], log[9])
        robot_list[id].set_last_updated_time(0)
    
    sim_time = 0

    glutInit()
    glutInitDisplayMode(GLUT_RGBA)
    glutInitWindowSize(gl_width, gl_height)
    glutCreateWindow("Swarmnet Simulation")
    # glutHideWindow()

def draw_rect(x, y, width, height, R, G, B):
    glBegin(GL_QUADS)                                  # start drawing a rectangle
    glColor3f(R,G,B)
    glVertex2f(x, y)                                   # bottom left point
    glVertex2f(x + width, y)                           # bottom right point
    glVertex2f(x + width, y + height)                  # top right point
    glVertex2f(x, y + height)                          # top left point
    glEnd()   

def drawSingleRobot(x, y, R, G, B, radius):
    glBegin(GL_POLYGON)
    glColor3f(R,G,B)
    # print("draw")
    for vertex in range(0, 8):
        angle  = float(vertex) * 2.0 * np.pi / 8
        glVertex2f(x + np.cos(angle) * radius, y + np.sin(angle) * radius)
    glEnd()

def calcUnitMovement(theta, v):
    xmov = v * np.cos(np.deg2rad(theta))
    ymov = v * np.sin(np.deg2rad(theta))
    return (xmov, ymov)

def glut_print(x, y, font, text, r, g, b, a):
    blending = False 
    if glIsEnabled(GL_BLEND) :
        blending = True

    #glEnable(GL_BLEND)
    glColor3f(1,1,1)
    glRasterPos2f(x,y)
    for ch in text :
        glutBitmapCharacter(font, ctypes.c_int(ord(ch)))

    if not blending :
        glDisable(GL_BLEND)

def createFrames():
    global max_time
    global frame_folder
    global fps
    global events
    global speed
    global robot_list
    global arena_width
    global arena_height
    global gl_width
    global gl_height
    global RADIUS
    global gl_margin_bot
    global gl_margin_others
    global arena_margin_size

    pathlib.Path(frame_folder).mkdir(parents=True, exist_ok=True)

    sim_time = 0
    counter = 0
    while (sim_time < max_time):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glViewport(0, 0, gl_width, gl_height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0.0, gl_width, 0.0, gl_height, 0.0, 1.0)
        glMatrixMode (GL_MODELVIEW)
        glLoadIdentity()
        glColor3f(1.0, 0.0, 3.0)
        glEnable(GL_LINE_SMOOTH)

        draw_rect(gl_margin_others - arena_margin_size, gl_margin_bot - arena_margin_size,
                  arena_width + 2 * arena_margin_size, arena_height + 2 * arena_margin_size,
                  255, 255, 255)
        draw_rect(gl_margin_others, gl_margin_bot,
                  arena_width, arena_height, 0, 0, 0)
        
        while len(events) > 0:
            next_event_time = min(events.keys())
            if sim_time >= next_event_time:
                next_event = events.pop(next_event_time)
                for id in next_event:
                    log = next_event[id]
                    robot_list[id].set_attributes(log[2], log[3], log[4], log[5], log[6], log[7], log[8], log[9])
                    robot_list[id].set_last_updated_time(log[0])
            else:
                break

        for robot in robot_list:
            xmov, ymov = calcUnitMovement(robot.theta, robot.v)
            time_change = sim_time - robot.last_updated_time

            robot.x = robot.x + xmov * time_change
            robot.y = robot.y + ymov * time_change

            if robot.x < 0:
                robot.x = 0
            elif robot.x > arena_width:
                robot.x = arena_width

            if robot.y < 0:
                robot.y = 0
            elif robot.y > arena_height:
                robot.y = arena_height
            
            draw_x = robot.x + gl_margin_others
            draw_y = robot.y + gl_margin_bot
            drawSingleRobot(draw_x, draw_y, robot.r, robot.g, robot.b, RADIUS)
            glut_print(draw_x - len(str(robot.id)) * 8 / 2, draw_y - RADIUS / 2, GLUT_BITMAP_8_BY_13, str(robot.id), 1.0, 1.0, 1.0, 1.0)
            log_txt_x = draw_x - len(str(robot.log)) * 4 / 2
            log_txt_y = draw_y - 2.2 * RADIUS
            if log_txt_x < 0:
                log_txt_x = 0
            if log_txt_y < 0:
                log_txt_y = 0
            glut_print(log_txt_x, log_txt_y, GLUT_BITMAP_TIMES_ROMAN_10, str(robot.log), 1.0, 1.0, 1.0, 1.0)
            robot.set_last_updated_time(sim_time)
        
        # glFlush()
        glut_print(10, 10, GLUT_BITMAP_9_BY_15, format(sim_time, '.2f'), 1.0, 1.0, 1.0, 1.0)
        glPixelStorei(GL_PACK_ALIGNMENT, 1)
        data = glReadPixels(0, 0, gl_width, gl_height, GL_RGBA, GL_UNSIGNED_BYTE)
        image = Image.frombytes("RGBA", (gl_width, gl_height), data)
        image = ImageOps.flip(image) # in my case image is flipped top-bottom for some reason
        image.save(frame_folder + "img{}.png".format(counter), 'PNG')
        sim_time += speed / fps
        counter += 1

def makeVideo(output_file):
    global fps
    global frame_folder
    pathlib.Path("./video/").mkdir(parents=True, exist_ok=True)
    ffmpeg_command = "ffmpeg -y -r {} -i {}img%d.png -c:v libx264 -pix_fmt yuv420p ./video/".format(fps, frame_folder) + output_file
    subprocess.call(ffmpeg_command, shell=True)
    rm_command = "rm -rf {}".format(frame_folder)
    subprocess.call(rm_command, shell=True)


def renderer(in_log_file="./motion_log/default_log.txt", in_speed=1, in_outfile="out.mp4"):
    global speed
    global log_file

    log_file = in_log_file
    speed = in_speed

    try:
        init()
        createFrames()
        makeVideo(in_outfile)
    except Exception as e:
        traceback.print_tb(e.__traceback__)
        print(e)
        return -1
    
    return 0

if __name__ == "__main__":
    renderer(sys.argv[1], float(sys.argv[2]), sys.argv[3])
