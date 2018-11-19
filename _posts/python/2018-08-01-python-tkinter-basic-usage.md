---
layout: post
title: Tkinter 기본 Sample Window
category: Python
tag: [Python, tkinter]
---
# Tkinter를 이용해서 Empty Window 만들어보기

`tkinter`를 이용해서 빈 Window를 만들어보는 예제입니다. 

만약 `tkinter`가 설치되어 있지 않으면 다음 명령어를 이용해서 설치할 수 있습니다. (Ubuntu 기준)

<pre class="prettyprint">
sudo apt install python-tk
</pre>

<br>

## 예제

<pre class="prettyprint">
import tkinter as tk


def initWindow():
    window = tk.Tk()
    window.title("Simple Window")
    window.resizable(False, False)
    window.geometry("1024x768")

    window.mainloop()


def main(args=None):
    initWindow()


if __name__ == '__main__':
    main()
</pre>

<br>

# 버튼 추가해보기

<pre class="prettyprint">
import tkinter as tk


def initWindow():
    window = tk.Tk()
    window.title("Simple Window")
    window.resizable(False, False)
    window.geometry("1024x768")

    button = tk.Button(master=window, text="Test Button", command=callback_button_clicked)
    button.pack()

    window.mainloop()


def callback_button_clicked():
    print("Button Clicked !!")


def main(args=None):
    initWindow()


if __name__ == '__main__':
    main()
</pre>

<br>

# 간단한 이미지 출력해보기

<pre class="prettyprint">
import tkinter as tk


window = tk.Tk()
window.title("Simple Window")
window.resizable(False, False)
window.geometry("1024x768")

canvas = tk.Canvas(window, width=1024, height=768)
canvas.pack(expand=tk.YES, fill=tk.BOTH)

img = tk.PhotoImage(file="01.png")
canvas.create_image(20, 20, anchor=tk.NW, image=img)

window.mainloop()
</pre>