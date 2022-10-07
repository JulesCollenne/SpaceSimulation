import sys

import numpy as np
import pygame
from pygame.locals import *


# def algo(ps, inerties, nb_iter=1, lr=1.):
#     for iter in range(nb_iter):
#         distances = np.zeros((len(ps), len(ps), 2))
#         for i in range(len(ps) - 1):
#             for j in range(i + 1, len(ps)):
#                 dist = np.linalg.norm(ps[j] - ps[i])
#                 distances[i][j] = (ps[j] - ps[i]) / (dist * dist)
#                 distances[j][i] = (ps[i] - ps[j]) / (dist * dist)
#                 # ps[i] += (ps[j] - ps[i]) * lr
#                 # ps[j] += (ps[i] - ps[j]) * lr
#         for i in range(len(ps)):
#             ps[i] += (inerties[i] + np.mean(distances[i], axis=0) * lr)
#     return ps


class Point:
    def __init__(self, pos, inertia, weight=1., density=1., color=None, nb_absorbed=1, size=1.):
        if color is None:
            color = [100, 100, 255]
        self.color = color
        self.pos = np.asarray(pos, dtype=np.float64)
        self.inertia = np.asarray(inertia, dtype=np.float64)
        self.weight = weight
        self.density = density
        self.size = size
        # self.size = weight * self.density * SCALE
        self.speed = 0.1
        self.nb_absorbed = nb_absorbed
        self.lr = 1

    def nextStep(self, points):
        forces = []
        for i, point in enumerate(points):
            if point == self:
                continue
            direction = point.pos - self.pos
            dist = np.linalg.norm(direction) / SCALE
            direction /= dist
            if dist > ((point.size + self.size) * SCALE):
                forces.append(self.weight * point.weight / (dist ** 2) * G * direction)
            else:
                heavy_pos = self.pos if self.weight >= point.weight else point.pos
                sommasse = (self.weight+point.weight)
                points.append(Point(heavy_pos,
                                    np.mean((self.inertia, point.inertia), axis=0),
                                    self.weight+point.weight,
                                    density=np.mean((self.density * self.weight / sommasse, point.density * point.weight / sommasse)),
                                    color=np.round(np.mean((self.color, point.color),
                                                           axis=0)).astype(int),
                                    nb_absorbed=self.nb_absorbed+point.nb_absorbed,
                              size=self.size+point.size))
                points.pop(i)
                points.remove(self)
                return
            # elif dist > 0:
            #     forces[i] = 0
            #     self.inertia = 0., 0.
            # ps = self.pos, point.pos
            # dist = np.linalg.norm(ps[1] - ps[0])
            # if dist > 1:
            #     distances[i] = (ps[1] - ps[0]) / (dist * dist)

        # print(forces)
        # sys.exit()
        forces = np.asarray(forces)
        # sys.exit()
        if len(forces):
            self.pos += ((self.inertia + np.mean(forces, axis=0)) * self.lr) / self.weight
            self.inertia = ((self.inertia + np.mean(forces, axis=0)) * self.lr)

    def draw(self, surface):
        pygame.draw.circle(surface, self.color, self.pos[:2], self.size * SCALE)


if __name__ == "__main__":
    pygame.init()
    displaysurface = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("Simulation")
    BG_COLOR = [50, 50, 50]
    FPS = 60
    G = 3.
    SCALE = 2.

    nb_points = 10
    # points = [Point(np.random.uniform(100, 600, 2), np.random.uniform(-1, 1, 2)*0.01) for i in range(nb_points)]

    # points = [Point(np.random.uniform(50, 550, 2),
    #                 np.random.uniform(-1, 1, 2) * 15.,
    #                 np.random.randint(1, 20),
    #                 color=np.random.randint(0, 255, 3),
    #                 density=1.)
    #           for i in range(nb_points)]

    # points = [Point(np.random.uniform(100, 600, 2),
    #                 np.zeros(2), np.random.randint(1, 4))
    #           for i in range(nb_points)]

    points = [
        Point((400, 200), (0.1, 0), 100, density=5.5, size=6.4),
        Point((400, 100), (3., 0), 1, color=[150, 150, 150], density=3.4, size=1.7),
        Point((100, 0), (0., 2), 1, color=[150, 150, 200], density=3.4, size=1.7),
        # Point((0, 200), (5., 0), 4, color=[200, 100, 100])
    ]

    FramePerSec = pygame.time.Clock()
    getTicksLastFrame = pygame.time.get_ticks()

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

        t = pygame.time.get_ticks()
        deltaTime = (t - getTicksLastFrame) / 1000.0
        getTicksLastFrame = t

        displaysurface.fill(BG_COLOR)

        for point in points:
            point.nextStep(points)
            point.draw(displaysurface)

        pygame.display.update()
        FramePerSec.tick(FPS)
