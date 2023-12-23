from PyQt6.QtOpenGL import *
from PyQt6.QtOpenGLWidgets import QOpenGLWidget

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *


class OpenGLWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

    def initializeGL(self):
        glClearColor(0, 0, 0, 1)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHT0)
        glEnable(GL_LIGHTING)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glEnable(GL_COLOR_MATERIAL)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glBegin(GL_TRIANGLES)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(-0.5, -0.5, 0)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0.5, -0.5, 0)
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0.0, 0.5, 0)

        glEnd()

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w / h, 0.01, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0)
