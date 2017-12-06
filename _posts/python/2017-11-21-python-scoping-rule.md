---
layout: post
title: Scoping Rule
category: Python
tag: [Python]
---
# Python에서의 변수의 범위

다른 언어들을 사용하다가 Python을 다루게 되면 변수에 대해서 헷갈리는 경우가 생기기 쉽습니다. 전역 변수, 로컬 변수 등의 개념이 Python에서는 어떻게 표현되고 있는지 살펴보겠습니다.

<pre class="prettyprint">
a = [1, 2, 3]
print(a)


def func():
    a = [4, 5, 6]
    print(a)


if __name__ == '__main__':
    func()
    print(a)
</pre>

위 코드의 실행 결과는

~~~
[1, 2, 3]
[4, 5, 6]
[1, 2, 3]
~~~

입니다. `a`라는 이름의 전역 변수가 정의되었고, `func()` 함수 안에서 `a`라는 이름의 지역 변수가 정의되었습니다. 함수 내부에 있는 코드가 배열 `a`에 새로운 값을 대입하라는 의미가 아니라 지역 변수를 선언하는 코드입니다.

만약 함수 `func()` 내에서 전역 변수 `a`를 사용하고 싶은 경우에는 다음과 같이 작성해야 합니다.

<pre class="prettyprint">
def func():
    global a
    a = [4, 5, 6]
    print(a)
</pre>