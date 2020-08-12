from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np
import sys
import time

speed_options = [0.1, 0.25, 0.5, 1, 2, 10, 100, 500]
log_file = ""
events = {}
robot_list = []
arena_width = 0
arena_height = 0
num_robots = 0
tick_per_second = 0
speed_index = 0
paused = False
finished = False
prev_time = 0
cur_tick = 0
max_tick = 0
RADIUS = 7

class Robot:
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

def setWindowTitle(speed):
    glutSetWindowTitle("Swarmnet Simulation - " + str(speed) + "x speed")

def drawPlay():
	glBegin(GL_TRIANGLES)
	glColor3f(1,1,1)
	glVertex2f(50, 50)
	glVertex2f(50, 80)
	glVertex2f(75, 65)
	glEnd()

def drawPause():
	glBegin(GL_QUADS)
	glColor3f(1,1,1)
	glVertex2f(50, 50)
	glVertex2f(50, 80)
	glVertex2f(59, 80)
	glVertex2f(59, 50)
	glEnd()

	glBegin(GL_QUADS)
	glColor3f(1,1,1)
	glVertex2f(66, 50)
	glVertex2f(66, 80)
	glVertex2f(75, 80)
	glVertex2f(75, 50)
	glEnd()

def drawFastForward(arenawidth):
	glColor3f(1,1,1)

	glBegin(GL_TRIANGLES)
	glVertex2f(arenawidth-69, 57)
	glVertex2f(arenawidth-69, 73)
	glVertex2f(arenawidth-62, 65)
	glEnd()

	glBegin(GL_TRIANGLES)
	glVertex2f(arenawidth-62, 57)
	glVertex2f(arenawidth-62, 73)
	glVertex2f(arenawidth-55, 65)
	glEnd()

def drawSlowDown(arenawidth):
	glColor3f(1,1,1)

	glBegin(GL_TRIANGLES)
	glVertex2f(arenawidth-90, 57)
	glVertex2f(arenawidth-90, 73)
	glVertex2f(arenawidth-97, 65)
	glEnd()

	glBegin(GL_TRIANGLES)
	glVertex2f(arenawidth-97, 57)
	glVertex2f(arenawidth-97, 73)
	glVertex2f(arenawidth-104, 65)
	glEnd()

### Calculate how much the robot moves in a tick with a given velocity and direction
### v = coordinate units/tick
### HACKY PAUSE PLAY CLICK DETECTION FUNCTION IN HERE
def clickUtilities(clicktype, onoff, x, y):
    global arena_width
    global arena_height
    global paused
    global finished
    global speed_options
    global speed_index

    if x > 50 and x < 75 and y < (glutGet(GLUT_WINDOW_HEIGHT) - 50) and y > (glutGet(GLUT_WINDOW_HEIGHT) - 80) and onoff:
        paused = not paused
        if finished:
            init()
            finished = False

    elif x > arena_width - 69 and x < arena_width - 55 and (glutGet(GLUT_WINDOW_HEIGHT) - 57) and y > (glutGet(GLUT_WINDOW_HEIGHT) - 73) and onoff:
        if speed_index == (len(speed_options) - 1):
            return
        speed_index += 1
        setWindowTitle(speed_options[speed_index])

    elif x > arena_width - 104 and x < arena_width - 90 and (glutGet(GLUT_WINDOW_HEIGHT) - 57) and y > (glutGet(GLUT_WINDOW_HEIGHT) - 73) and onoff:
        if speed_index == 0:
            return
        speed_index -= 1
        setWindowTitle(speed_options[speed_index])

def calcUnitMovement(theta, v):
    xmov = v * np.cos(np.deg2rad(theta))
    ymov = v * np.sin(np.deg2rad(theta))
    return (xmov, ymov)

def drawSingleRobot(x, y, R, G, B, radius):
    glBegin(GL_POLYGON)
    glColor3f(R,G,B)  
    for vertex in range(0, 8):
        angle  = float(vertex) * 2.0 * np.pi / 8
        glVertex2f(x + np.cos(angle) * radius, y + np.sin(angle) * radius)
    glEnd()

def drawRobots():
    global tick_per_second
    global speed_options
    global speed_index
    global events
    global robot_list
    global cur_tick
    global max_tick
    global prev_time
    global paused
    global finished
    global arena_width
    global arena_height
    global RADIUS

    cur_time = time.time()
    time_diff = (cur_time - prev_time) * speed_options[speed_index]
    prev_time = time.time()

    drawFastForward(arena_width)
    drawSlowDown(arena_width)

    if paused:
        # paused
        drawPlay()
        time_diff = 0
    else:
        # playing
        drawPause()
    
    cur_tick = cur_tick + time_diff * tick_per_second

    if cur_tick >= max_tick:
        paused = True
        finished = True
        return

    for robot in robot_list:
        xmov, ymov = calcUnitMovement(robot.theta, robot.v)
        xpos = robot.x
        ypos = robot.y
        drawSingleRobot(xpos, ypos, robot.r, robot.g, robot.b, RADIUS)

        robot.x = xpos + xmov * time_diff
        robot.y = ypos + ymov * time_diff

        if robot.x < 0:
            robot.x = 0
        elif robot.x > arena_width:
            robot.x = arena_width

        if robot.y < 0:
            robot.y = 0
        elif robot.y > arena_height:
            robot.y = arena_height

    while len(events) > 0:
        next_event_tick = min(events.keys())
        if cur_tick >= next_event_tick:
            next_event = events.pop(next_event_tick)
            for id in next_event:
                log = next_event[id]
                robot_list[id].set_attributes(log[2], log[3], log[4], log[5], log[6], log[7], log[8])
        else:
            break
    
    sleep_time = 0.01 - (time.time() - cur_time)
    if sleep_time < 0:
        print("slow")
        sleep_time = 0
    time.sleep(sleep_time)

def iterate():
    glViewport(0, 0, arena_width, arena_height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glOrtho(0.0, arena_width, 0.0, arena_height, 0.0, 1.0)
    glMatrixMode (GL_MODELVIEW)
    glLoadIdentity()

def showScreen():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    iterate()
    glColor3f(1.0, 0.0, 3.0)
    glEnable(GL_LINE_SMOOTH)
    drawRobots()
    glutSwapBuffers()

def onIdle():
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glutPostRedisplay()

def init():
    global num_robots
    global paused
    global finished
    global events
    global robot_list
    global prev_time
    global cur_tick
    global log_file

    paused = True
    finished = False
    robot_list = []
    for i in range(num_robots):
        robot_list += [Robot(i)]

    events = {}
    logs = np.loadtxt(log_file, delimiter=' ', dtype=np.float, skiprows=1)
    for log in logs:
        tick = int(log[0])
        id = int(log[1])
        if tick not in events:
            events[tick] = {}
        events[tick][id] = log
    
    init_event = events.pop(0)
    for id in init_event:
        log = init_event[id]
        robot_list[id].set_attributes(log[2], log[3], log[4], log[5], log[6], log[7], log[8])
    
    prev_time = time.time()
    cur_tick = 0

def main():
    global arena_width
    global arena_height
    global num_robots
    global tick_per_second
    global speed_index
    global paused
    global finished
    global events
    global robot_list
    global prev_time
    global cur_tick
    global max_tick
    global log_file

    log_file = sys.argv[1]
    metadata = np.loadtxt(log_file, delimiter=' ', max_rows=1, dtype=np.int)
    arena_width = metadata[0]
    arena_height = metadata[1]
    num_robots = metadata[2]
    tick_per_second = metadata[3]
    max_tick = metadata[4] * tick_per_second
    speed_index = 3

    init()

    glutInit()
    glutInitDisplayMode(GLUT_RGBA)
    glutInitWindowSize(arena_width, arena_height)
    glutInitWindowPosition(0, 0)
    glutCreateWindow("Swarmnet Simulation")
    setWindowTitle(speed_options[speed_index])
    glutDisplayFunc(showScreen)
    glutIdleFunc(onIdle)
    glutMouseFunc(clickUtilities)
    glutMainLoop()

if __name__ == "__main__":
    main()

