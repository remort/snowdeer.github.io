---
layout: post
title: PyGame 기반 PNG 이미지 로드하는 방법
category: Python
tag: [Python, pygame]
---
# PyGame 기반 PNG 이미지 로드하는 방법

<pre class="prettyprint">
import pygame
import sys

SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480

white = (255, 255, 255)
black = (0, 0, 0)

pygame.init()
pygame.display.set_caption("Rain")
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
clock = pygame.time.Clock()


class Player(object):
    def __init__(self):
        self.x = 100
        self.y = 300
        self.image = pygame.image.load("images/player.png").convert_alpha()

    def draw(self):
        screen.blit(self.image, (self.x, self.y))


def main():
    player = Player()

    while True:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()

        screen.fill(black)
        player.draw()

        pygame.display.update()


if __name__ == "__main__":
    main()
</pre>