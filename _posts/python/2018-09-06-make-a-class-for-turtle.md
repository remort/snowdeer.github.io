---
layout: post
title: 마우스 클릭 이벤트에 따라 움직이는 거북이 클래스 만들어보기
category: Python
tag: [Python, turtle]
---
# 마우스 클릭 이벤트에 따라 움직이는 거북이 클래스 만들어보기

<br>

## 거북이 클래스 만들기

<pre class="prettyprint">
import turtle


class MyTurtle(turtle.Turtle):
    def __init__(self):
        super(MyTurtle, self).__init__()
        self.shape("turtle")
        self.shapesize(2, 2)
        self.getscreen().bgcolor("yellow")


t = MyTurtle()

while True:
    pass
</pre>

<br>

## 마우스 클릭 이벤트에 반응하기

거북이가 이동하고 있는 도중에 마우스 클릭 이벤트가 온 경우를 처리하기 위한 코드가 포함되어 있습니다.

<pre class="prettyprint">
import turtle


class MyTurtle(turtle.Turtle):
    def __init__(self):
        super(MyTurtle, self).__init__()
        self.shape("turtle")
        self.shapesize(2, 2)
        self.color("purple")

        self.window = turtle.Screen()
        self.window.onclick(self.on_mouse_clicked)

        self.is_moving = False

    def loop(self):
        self.window.mainloop()

    def on_mouse_clicked(self, x, y):
        print("clicked ({0}, {1})".format(x, y))
        if self.is_moving is True:
            return

        self.is_moving = True
        self.goto(x, y)
        self.is_moving = False


t = MyTurtle()
t.loop()
</pre>

<br>

## Lock 설정/해제 방식으로 마우스 클릭 이벤트 처리해보기

<pre class="prettyprint">
import turtle


class MyTurtle(turtle.Turtle):
    def __init__(self):
        super(MyTurtle, self).__init__()
        self.shape("turtle")
        self.shapesize(2, 2)
        self.color("purple")

        self.window = turtle.Screen()
        self.window.onclick(self.on_mouse_clicked)

        self.is_moving = False

    def loop(self):
        self.window.mainloop()

    def on_mouse_clicked(self, x, y):
        if self.acquire_lock():
            self.goto(x, y)
            self.release_lock()

    def acquire_lock(self):
        if self.is_moving is True:
            return False

        self.is_moving = True
        return True

    def release_lock(self):
        self.is_moving = False


t = MyTurtle()
t.loop()
</pre>
