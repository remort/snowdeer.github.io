---
layout: post
title: PyGame 기반 비오는 장면 속의 캐릭터 연출
category: Python
tag: [Python, pygame]
---
# PyGame 기반 비오는 장면 속의 캐릭터 연출

다음과 같은 화면을 만들어보는 예제입니다.

![image](/assets/python/rain_image.png)

<br>

아래는 캐릭터에 사용한 이미지입니다. 구글링으로 찾은 무료 이미지이며 크기를 좀 줄였습니다. (65x120 사이즈입니다.)

![image](/assets/python/player.png)

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