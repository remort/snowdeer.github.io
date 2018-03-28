---
layout: post
title: Markdown Test
category: 블로그
permalink: /blog/:year/:month/:day/:title/

tag: [markdown]
---

GitHub의 Markdown에서 이미지 경로를 바꿨을 때 화면에 어떻게 렌더링되는지 테스트해봅니다.

## /assets/go/001.png

![Image](/assets/go/001.png)

## 001.png

![Image](001.png)

## ./001.png

![Image](./001.png)

## ../blog/001.png

![Image](../blog/001.png)