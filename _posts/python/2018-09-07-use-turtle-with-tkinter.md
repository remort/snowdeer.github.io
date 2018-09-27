---
layout: post
title: tkinter와 turtle 동시에 사용하기
category: Python
tag: [Python, turtle, tkinter]
---
# tkinter와 turtle 동시에 사용하기

<br>

## 기본 예제

<pre class="prettyprint">
import turtle
import tkinter as tk

root = tk.Tk()
canvas = tk.Canvas(master=root, width=500, height=500)
canvas.pack()

t = turtle.RawTurtle(canvas)

tk.Button(master=root, text="Forward", command=lambda: t.forward(100)).pack(side=tk.TOP)
tk.Button(master=root, text="Back", command=lambda: t.back(100)).pack(side=tk.BOTTOM)
tk.Button(master=root, text="Left", command=lambda: t.left(90)).pack(side=tk.LEFT)
tk.Button(master=root, text="Right", command=lambda: t.right(90)).pack(side=tk.RIGHT)

root.mainloop()
</pre>

<br>

## 거북이 클래스 만들어서 tkinter와 같이 사용하기

<pre class="prettyprint">
import turtle
import tkinter as tk

WIDTH = 500
HEIGHT = 500


class MyTurtle(turtle.RawTurtle):
    def __init__(self, canvas):
        super(MyTurtle, self).__init__(canvas)
        self.shape("turtle")
        self.shapesize(2, 2)
        self.getscreen().bgcolor("yellow")

        self.center_offset_x = WIDTH / 2
        self.center_offset_y = HEIGHT / 2

        canvas.bind("<Button-1>", self.on_mouse_clicked)

        self.is_moving = False

    def on_mouse_clicked(self, event):
        if self.acquire_lock():
            x = event.x - self.center_offset_x
            y = -(event.y - self.center_offset_y)

            print("clicked ({0}, {1})".format(x, y))
            self.goto(x, y)

            self.release_lock()

    def acquire_lock(self):
        if self.is_moving is True:
            return False

        self.is_moving = True
        return True

    def release_lock(self):
        self.is_moving = False


root = tk.Tk()
canvas = tk.Canvas(master=root, width=WIDTH, height=HEIGHT)
canvas.pack()

t = MyTurtle(canvas)

tk.Button(master=root, text="Forward", command=lambda: t.forward(100)).pack(side=tk.TOP)
tk.Button(master=root, text="Back", command=lambda: t.back(100)).pack(side=tk.BOTTOM)
tk.Button(master=root, text="Left", command=lambda: t.left(90)).pack(side=tk.LEFT)
tk.Button(master=root, text="Right", command=lambda: t.right(90)).pack(side=tk.RIGHT)

root.mainloop()
</pre>