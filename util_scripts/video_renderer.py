from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np
import sys
import pathlib
from PIL import Image
from PIL import ImageOps
import subprocess

log_file = ""
events = {}
robot_list = []
arena_width = 0
arena_height = 0
num_robots = 0
speed = 1
max_time = 0
RADIUS = 7
fps = 24
frame_folder = "./tmp_frames/"

class Robot:
    def set_last_updated_time(self, time):
        self.last_updated_time = time
    
    def set_attributes(self, x, y, theta, v, r, g, b):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.r = r
        self.g = g
        self.b = b
    
    def __init__(self, id):
        self.id = id

def init():
    global num_robots
    global events
    global robot_list
    global log_file
    global arena_width
    global arena_height
    global max_time

    metadata = np.loadtxt(log_file, delimiter=' ', max_rows=1, dtype=np.int)
    arena_width = metadata[0]
    arena_height = metadata[1]
    num_robots = metadata[2]
    max_time = metadata[3]

    robot_list = []
    for i in range(num_robots):
        robot_list += [Robot(i)]

    events = {}
    logs = np.loadtxt(log_file, delimiter=' ', dtype=np.float, skiprows=1)
    for log in logs:
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
        robot_list[id].set_attributes(log[2], log[3], log[4], log[5], log[6], log[7], log[8])
        robot_list[id].set_last_updated_time(0)
    
    sim_time = 0

    glutInit()
    glutInitDisplayMode(GLUT_RGBA)
    glutInitWindowSize(arena_width, arena_height)
    glutCreateWindow("Swarmnet Simulation")
    # glutHideWindow()

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
    global RADIUS

    pathlib.Path(frame_folder).mkdir(parents=True, exist_ok=True)

    sim_time = 0
    counter = 0
    while (sim_time < max_time):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glViewport(0, 0, arena_width, arena_height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0.0, arena_width, 0.0, arena_height, 0.0, 1.0)
        glMatrixMode (GL_MODELVIEW)
        glLoadIdentity()
        glColor3f(1.0, 0.0, 3.0)
        glEnable(GL_LINE_SMOOTH)

        while len(events) > 0:
            next_event_time = min(events.keys())
            if sim_time >= next_event_time:
                next_event = events.pop(next_event_time)
                for id in next_event:
                    log = next_event[id]
                    robot_list[id].set_attributes(log[2], log[3], log[4], log[5], log[6], log[7], log[8])
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
            
            drawSingleRobot(robot.x, robot.y, robot.r, robot.g, robot.b, RADIUS)
            glut_print(robot.x - RADIUS / 2, robot.y - RADIUS / 2, GLUT_BITMAP_8_BY_13, str(robot.id), 1.0, 1.0, 1.0, 1.0)
            robot.set_last_updated_time(sim_time)
        
        # glFlush()
        glut_print(10, 10, GLUT_BITMAP_9_BY_15, format(sim_time, '.2f'), 1.0, 1.0, 1.0, 1.0)
        glPixelStorei(GL_PACK_ALIGNMENT, 1)
        data = glReadPixels(0, 0, arena_width, arena_height, GL_RGBA, GL_UNSIGNED_BYTE)
        image = Image.frombytes("RGBA", (arena_width, arena_height), data)
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
    except:
        return -1
    
    return 0

if __name__ == "__main__":
    renderer(sys.argv[1], float(sys.argv[2]), sys.argv[3])
