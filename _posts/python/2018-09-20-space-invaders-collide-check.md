---
layout: post
title: PyGame 스페이스 인베이더(Space Invader) - 총알 충돌 판정
category: Python
tag: [Python, pygame]
---
# PyGame 스페이스 인베이더(Space Invader) - 총알 충돌 판정

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

        if self.y >= 400:
            self.y = 20

    def draw(self):
        screen.blit(self.image, (self.x, self.y))

    def collide_check(self, bullets):
        for b in bullets:
            if pygame.Rect(self.x, self.y, 40, 40).colliderect((b.x, b.y, 0, b.height)):
                self.x = 100
                self.y = 20

                bullets.remove(b)
                break


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


class Bullet(object):
    def __init__(self, x, y):
        self.height = 4
        self.speed = 7
        self.x = x
        self.y = y

    def move(self):
        self.y -= self.speed

    def is_screen_out(self):
        return self.y - self.height <= 0

    def draw(self):
        pygame.draw.line(screen, black, (self.x, self.y), (self.x, self.y + self.height), 1)


def main():
    player = Player()
    enemy = Enemy()
    bullets = []

    while True:
        clock.tick(60)
        screen.fill(white)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                bullets.append(Bullet(player.x + 20, player.y - 5))

        key_event = pygame.key.get_pressed()

        enemy.move()
        enemy.draw()
        enemy.collide_check(bullets)

        player.move(key_event)
        player.draw()

        for b in bullets:
            b.move()
            b.draw()

            if b.y > SCREEN_HEIGHT:
                bullets.remove(b)

        pygame.display.update()


if __name__ == "__main__":
    main()
</pre>