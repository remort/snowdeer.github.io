---
layout: post
title: PyGame 스페이스 인베이더(Space Invader) - 좌우로 움직이는 적(Enemy)
category: Python
tag: [Python, pygame]
---
# PyGame 스페이스 인베이더(Space Invader) - 좌우로 움직이는 적(Enemy)

아래는 사용한 이미지입니다. 구글링으로 찾은 무료 이미지이며 크기를 좀 줄였습니다. (40x40 사이즈입니다.)

![image](/assets/python/space_invader_enemy.png)

<br>

## 소스 코드

적은 좌우로 움직이며, 양쪽 끝에 도달할 경우 아래로 이동합니다.

<pre class="prettyprint">
import pygame
import sys

SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480

white = (255, 255, 255)
black = (0, 0, 0)

pygame.init()
pygame.display.set_caption("Space Invaders")
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
clock = pygame.time.Clock()


class Enemy(object):
    def __init__(self):
        self.image = pygame.image.load("images/space_invader_enemy.png").convert_alpha()
        self.x = 100
        self.y = 20
        self.speed = 3
        self.direction = 1

    def move(self):
        self.x += self.speed * self.direction

        if self.x <= 0:
            self.y += 20
            self.direction = 1

        if self.x + 40 >= SCREEN_WIDTH:
            self.y += 20
            self.direction = -1

    def draw(self):
        screen.blit(self.image, (self.x, self.y))


def main():
    enemy = Enemy()

    while True:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()

        screen.fill(white)

        enemy.move()
        enemy.draw()

        pygame.display.update()


if __name__ == "__main__":
    main()
</pre>
