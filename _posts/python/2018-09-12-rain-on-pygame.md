---
layout: post
title: PyGame 기반 비오는 장면 연출
category: Python
tag: [Python, pygame]
---
# PyGame 기반 비오는 장면 연출

<pre class="prettyprint">
import pygame
import sys
import random

SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480

white = (255, 255, 255)
black = (0, 0, 0)

pygame.init()
pygame.display.set_caption("Rain")
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
clock = pygame.time.Clock()


class Raindrop(object):
    def __init__(self):
        self.height = random.randint(4, 7)
        self.speed = random.randint(5, 10)
        self.x = random.randint(0, SCREEN_WIDTH)
        self.y = -self.height

    def move(self):
        self.y += self.speed

    def draw(self):
        pygame.draw.line(screen, white, (self.x, self.y), (self.x, self.y + self.height), 1)


def main():
    raindrops = []

    while True:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()

        raindrops.append(Raindrop())
        screen.fill(black)

        for drop in raindrops:
            drop.move()
            drop.draw()

            if drop.y > SCREEN_HEIGHT:
                raindrops.remove(drop)

        pygame.display.update()


if __name__ == "__main__":
    main()
</pre>