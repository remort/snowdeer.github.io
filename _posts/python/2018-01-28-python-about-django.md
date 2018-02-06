---
layout: post
title: Django 웹 프레임워크
category: Python
tag: [Python, Django]
---
# Django

Django는 파이썬으로 웹 서버를 쉽게 개발할 수 있는 기능을 제공하는 프레임워크(Framework)입니다. 2003년 로렌스 저널-월드 신문사의 내부 프로젝트로 시작했으며, 2005년 오픈소스화되었습니다.

<br>

## Django 설치 방법

Django는 `pip`를 이용해서 간단히 설치할 수 있습니다.

~~~
$ pip install django
~~~

<br>

## Django 버전 확인

다음 명령어로 현재 Django의 버전을 확인할 수 있습니다.

~~~
$ python -m django --version
~~~

<br>

## Django 특징

### MTV 패턴

Django는 MTV 패턴 구조로 되어 있습니다. MTV 패턴은 Modle-Template-View로 이루어져 있으며, MVC 패턴의 Model-View-Controller와 일치합니다. 주의할 점은 MTV 패턴에서 Template는 MVC 패턴에서의 View와 대응되며, MTV 패턴의 View는 MVC 패턴의 Controller와 대응됩니다.

<br>

### ORM 지원

ORM(Object-Relational Mapping)은 데이터베이스 시스템과 데이터 모델을 쉽게 연결시켜주는 기능입니다. 직접 SQL문으로 쿼리를 작성하지 않더라도 데이터베이스로부터 값을 쉽게 가져올 수 있고, 데이터를 마치 객체를 다루듯이 사용할 수 있습니다.

<br>

## Admin 화면 제공

Django로 제작한 웹 서버는 Admin 화면을 자동으로 제공합니다. Admin 화면을 통해 데이터들을 쉽게 관리할 수 있습니다.

<br>

## Elegant URL 방식

웹 프로그래밍에서 URL 디자인은 필수적인 작업인데, Django에서는 Elegant URL 방식을 채택하여 각 URL 형태를 파이썬 함수나 클래스에 직접 연결할 수 있도록 하여 쉽게 개발할 수 있는 환경을 제공합니다.

<br>

## 자체 템플릿 시스템

Django는 내부적으로 확장이 가능하고 디자인이 쉬운 템플릿 시스템을 제공합니다. 이를 통해 디자인과 로직을 분리해서 개발할 수 있습니다.

<br>

## Cache 시스템

동적인 페이지는 일반적으로 데이터베이스 쿼리를 실행하고 템플릿을 해석하여 관련 페이지를 렌더링해주는 작업이 필요합니다. 이는 서버에 큰 부하를 주게 되는데 Django에서는 Cache 시스템을 제공하여 서버의 부하를 줄여줄 수 있습니다.

<br>

## 자체 웹 서버 제공

Django는 자체적으로 웹 서버 기능을 제공하고 있어 개발 과정에서 손쉽게 테스트할 수 있습니다. 또한 디버깅 모드를 제공하여 에러를 쉽게 파악하고 수정할 수 있도록 상세한 메시지를 제공합니다.

<br>

## 소스 변경사항 자동 반영

Django는 소스에 변경사항이 있을 경우 자동으로 반영해줍니다. 따라서 웹 서버를 구동시킨 후 소스 수정을 하더라도 웹 서버 재실행을 할 필요가 없습니다.