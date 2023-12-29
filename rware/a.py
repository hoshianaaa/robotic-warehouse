import math
import pyglet
from pyglet.gl import *

# Create a window
window = pyglet.window.Window(500, 500)

# Draw a 10x10 grid
def draw_grid():
    glBegin(GL_LINES)
    for i in range(11):
        # Vertical lines
        glVertex2i(int(i * window.width / 10), 0)
        glVertex2i(int(i * window.width / 10), window.height)

        # Horizontal lines
        glVertex2i(0, int(i * window.height / 10))
        glVertex2i(window.width, int(i * window.height / 10))
    glEnd()

# Draw a circle
def draw_circle(x, y, radius):
    iterations = 20
    s = math.sin(2 * math.pi / iterations)
    c = math.cos(2 * math.pi / iterations)

    dx, dy = radius, 0

    glBegin(GL_TRIANGLE_FAN)
    glVertex2f(x, y)
    for _ in range(iterations + 1):
        glVertex2f(x + dx, y + dy)
        dx, dy = (dx * c - dy * s), (dy * c + dx * s)
    glEnd()

@window.event
def on_draw():
    window.clear()
    draw_grid()
    draw_circle(250, 250, 50)  # Draw circle at the center with radius 50

# Run the application
pyglet.app.run()

