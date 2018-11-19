---
layout: post
title: Tkinter - 레이아웃에 Weight 적용하기
category: Python
tag: [Python, tkinter]
---
# Tkinter - 레이아웃 Row, Column에 Weight 적용하기

<pre class="prettyprint">
root = tk.Tk()
root.title("SnowDeer's Tkinter Example")
root.resizable(False, False)
root.geometry("640x480")

for col in range(0, 4):
    root.grid_columnconfigure(col, weight=1)
</pre>