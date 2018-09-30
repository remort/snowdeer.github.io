---
layout: post
title: PyGame 스페이스 인베이더(Space Invader) - 조작 가능한 플레이어 추가
category: Python
tag: [Python, pygame]
---
# PyGame 스페이스 인베이더(Space Invader) - 조작 가능한 플레이어 추가

아래는 사용한 이미지입니다. 구글링으로 찾은 무료 이미지이며 크기를 좀 줄였습니다.

![image](/assets/python/space_invader_enemy.png)

![image](/assets/python/space_invader_player.png)

<br>

## 소스 코드

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

        if self.x <= 40:
            self.y += 20
            self.direction = 1

        if self.x + 80 >= SCREEN_WIDTH:
            self.y += 20
            self.direction = -1

    def draw(self):
        screen.blit(self.image, (self.x, self.y))


class Player(object):
    def __init__(self):
        self.x = 300
        self.y = 400
        self.speed = 5
        self.image = pygame.image.load("images/space_invader_player.png").convert_alpha()

    def move(self, key_event):
        if key_event[pygame.K_LEFT]:
            self.x -= self.speed

        if key_event[pygame.K_RIGHT]:
            self.x += self.speed

        if self.x <= 40:
            self.x = 40

        if self.x + 80 >= SCREEN_WIDTH:
            self.x = SCREEN_WIDTH - 80

    def draw(self):
        screen.blit(self.image, (self.x, self.y))


def main():
    player = Player()
    enemy = Enemy()

    while True:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()

        key_event = pygame.key.get_pressed()

        screen.fill(white)

        enemy.move()
        enemy.draw()

        player.move(key_event)
        player.draw()

        pygame.display.update()


if __name__ == "__main__":
    main()
</pre>