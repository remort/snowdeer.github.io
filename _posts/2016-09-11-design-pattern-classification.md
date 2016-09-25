---
layout: post
title: 디자인 패턴 특성별 분류
category: 디자인패턴
tag: [design pattern, classification]
---

디자인 패턴을 Creational/Structural/Behavioral 특성으로 분류하면 다음과 같습니다.

![Image]({{ site.baseurl }}/assets/2016-09-11-design-pattern-classification/JavaDesignPatterns_Fig1.png)

<br>

## Organizing Design Patterns by Purpose

목적 | 설명
------ | ------
Creation   | Concerns object creation
------ | ------
Structural | Deals with composition of classes or object
------ | ------
Behavioral | Characterizes ways in which classes or objects interact and distribute responsibility

<br>

## Organizing Design Patterns by Scope

### Class Patterns

* Deal with relatinships between classes and their subclasses
* Relationships established through inheritance, so they are fixed at compile time(static)
* inheritance : white-box reuse

### Object Patterns

* Deal with object relatinships
* Relationships can be changed at runtime(dynamic)
* Composition : Black-box reuse, to avoid reaking encapsulation, and implementation dependency