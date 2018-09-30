---
layout: post
title: PyGame 빗방울과 캐릭터간 충돌 체크하기
category: Python
tag: [Python, pygame]
---
# PyGame 빗방울과 캐릭터간 충돌 체크하기

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


class Player(object):
    def __init__(self):
        self.x = 100
        self.y = 300
        self.image = pygame.image.load("images/player.png").convert_alpha()

    def draw(self):
        screen.blit(self.image, (self.x, self.y))

    def is_collide(self, x, y):
        return pygame.Rect(self.x, self.y, 65, 120).collidepoint(x, y)


def main():
    raindrops = []
    player = Player()

    while True:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()

        screen.fill(black)
        raindrops.append(Raindrop())

        player.draw()
        for drop in raindrops:
            drop.move()
            drop.draw()

            if drop.y > SCREEN_HEIGHT or player.is_collide(drop.x, drop.y):
                raindrops.remove(drop)

        pygame.display.update()


if __name__ == "__main__":
    main()
</pre>

<br>

## 키보드 이벤트로 캐릭터 이동시키기

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


class Raindrop:
    def __init__(self):
        self.height = random.randint(4, 7)
        self.speed = random.randint(5, 10)
        self.x = random.randint(0, SCREEN_WIDTH)
        self.y = -self.height

    def move(self):
        self.y += self.speed

    def draw(self):
        pygame.draw.line(screen, white, (self.x, self.y), (self.x, self.y + self.height), 1)


class Player:
    def __init__(self):
        self.x = 100
        self.y = 300
        self.image = pygame.image.load("images/player.png").convert_alpha()

    def draw(self):
        screen.blit(self.image, (self.x, self.y))

    def is_collide(self, x, y):
        return pygame.Rect(self.x, self.y, 65, 120).collidepoint(x, y)


def main():
    raindrops = []
    player = Player()

    while True:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()

        screen.fill(black)
        raindrops.append(Raindrop())

        player.draw()
        for drop in raindrops:
            drop.move()
            drop.draw()

            if drop.y > SCREEN_HEIGHT or player.is_collide(drop.x, drop.y):
                raindrops.remove(drop)

        pygame.display.update()


if __name__ == "__main__":
    main()
</pre>
