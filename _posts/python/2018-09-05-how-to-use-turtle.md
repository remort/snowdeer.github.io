---
layout: post
title: Turtle 라이브러리 사용해서 거북이 움직여보기
category: Python
tag: [Python, turtle]
---
# Turtle 라이브러리 사용해서 거북이 움직여보기

<br>

## 기본 예제

<pre class="prettyprint">
import turtle

t = turtle.Turtle()
t.shape("turtle")
t.shapesize(3, 3, 3)

t.speed(3)
# 'fastest': 0
# 'fast': 10
# 'normal': 6
# 'slow': 3
# 'slowest': 1

t.forward(300)
t.right(90)
t.forward(300)
t.right(90)
t.forward(300)
t.right(90)
t.forward(300)

while True:
    pass
</pre>

<br>

## 거북이 좌표 출력하기

<pre class="prettyprint">
import turtle

t = turtle.Turtle()
t.shape("turtle")
t.shapesize(3, 3, 3)
t.speed(3)

print(t.position())

t.forward(300)
print(t.position())

t.right(90)
t.forward(300)
print(t.position())

t.right(90)
t.forward(300)
print(t.position())

t.right(90)
t.forward(300)
print(t.position())

t.setpos(300, 300)
print(t.position())

while True:
    pass
</pre>

