---
layout: post
title: 디자인 패턴 특성별 분류
category: 디자인패턴
permalink: /designpattern/:year/:month/:day/:title/

tag: [디자인패턴]
---

디자인 패턴을 Creational/Structural/Behavioral 특성으로 분류하면 다음과 같습니다.

<br>

![image]({{ site.baseurl }}/assets/2016-05-20-design-pattern-classification/01.png)

<br>
## Organizing Design Patterns by Purpose
### Creation Patterns
<ul>
 	<li>Deal with initializing and configuring classes and objects</li>
</ul>
<h5>Structural Patterns</h5>
<ul>
 	<li>Deal with decoupling interface and implementation of classes and objects</li>
</ul>
### Behavioral Patterns
<ul>
 	<li>Deal with dynamic interactions among societies of classes and objects</li>
</ul>
<br>

## Organizing Design Patterns by Scope
### Class Patterns
<ul>
 	<li>Deal with relatinships between classes and their subclasses</li>
 	<li>Relationships established through inheritance, so they are fixed at compile time(static)</li>
 	<li>inheritance : white-box reuse</li>
</ul>
### Object Patterns
<ul>
 	<li>Deal with object relatinships</li>
 	<li>Relationships can be changed at runtime(dynamic)</li>
 	<li>Composition : Black-box reuse, to avoid reaking encapsulation, and implementation dependency</li>
</ul>
