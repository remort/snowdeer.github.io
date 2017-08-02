---
layout: post
title: Python 2.7.x vs Python 3.x
category: Python
tag: [Python]
---

Python 2.7.x 버전과 Python 3.x 버전의 차이점을 간단하게 적어봅니다. 생각보다 많은 것들이 바뀌어서 혼란을 가져올 수 있는 부분들이 꽤 있습니다. '두 버전 중에 어느 버전이 더 좋은건가'라고 하면 아무래도 좀 더 많은 부분이 개선된 3.x 버전을 사용하는 것이 더 좋습니다. 하지만, 2.7.x 버전도 많이 사용하고 있고 시중에 나와 있는 많은 교재들도 2.7.x 버전 위주로 설명하고 있는 부분이 많아서 버전별 차이점은 조금 알고 있어야 할 것 같습니다.

[Python Wiki](https://wiki.python.org/moin/Python2orPython3)에 따르면 다음과 같은 경우에는 2.7.x 버전을 사용하는 것이 더 좋다고 설명하고 있습니다.

<ul>
 	<li>개발한 내용을 어딘가에 배포할 때, 그 환경을 제어할 수 없는 경우(예를 들어 2.7.x 버전만 사용가능한 환경인 경우)</li>
 	<li>특정 라이브러리나 유틸리티 등을 사용할 때, 해당 라이브러리가 3.x 버전을 지원하지 않는 경우</li>
</ul>
즉, 대부분은 그냥 3.x 버전을 사용하는 편이 더 나을 것 같습니다.


자, 그럼 Python 2.7.x 버전과 Python 3.x 버전의 차이점을 살펴보도록 하겠습니다. 더 많은 정보는 [여기를 참조](http://sebastianraschka.com/Articles/2014_python_2_3_key_diff.html)하시길 바랍니다.

<br>

# Python 2.7.x과 Python 3.x의 차이점

## print

Python 3.x 버전은 print 문에 대해 괄호를 필요로 합니다.

예를 들어, 2.7.x 버전에서는 다음과 같은 코드를 사용할 수 있었습니다.

<pre class="prettyprint">print 'Hello, Python!'
print('Hello, Python!')
print "Hello",;
print 'snowdeer'</pre>

하지만, 3.x 버전에는 무조건 괄호가 있어야 하며, 다음과 같이 작성해야 합니다.

<pre class="prettyprint">print('Hello, Python!')
print("Hello ", end="")
print 'snowdeer'</pre>

<br>

## 자동 형 변환

Python 2.7.x 버전에서는 자동으로 형 변환을 하지 않지만, 3.x 버전에서는 자동 형 변환을 지원합니다.

예를 들어, 다음 코드의 실행 결과는

<pre class="prettyprint">print ('3 / 2 =', 3 / 2)</pre>

2.7.x 버전에서는 '1' 이 출력되지만, 3.x 버전에서는 '1.5'가 출력됩니다.

<br>

## 인코딩

Python 3.x 버전 부터는 소스 코드의 인코딩이 기본적으로 'utf-8'이기 때문에 소스 코드 첫 줄에 다음과 같은 라인이 생략되어져도 됩니다.
<pre class="prettyprint"># -*- coding: utf-8 -*-</pre>

<br>

## input

Python 2.7.x에서 사용자 입력은 다음과 같은 함수를 사용했었습니다.

<pre class="prettyprint">name = raw_input("input name:")</pre>

하지만 3.x 버전에서는 다음 함수를 사용하면 됩니다.

<pre class="prettyprint">name = input("input name:")</pre>

<br>

## Exception Handling

Python 2.7.x 버전에서는 예외 처리 코드를 다음과 같이 사용했다면,

<pre class="prettyprint">print 'Exception Handling'
try:
    let_us_cause_a_NameError
except NameError, err:
    print err, '--&gt; our error message'</pre>
&nbsp;

3.x 버전에는 다음과 같은 코드로 사용해야 합니다.

<pre class="prettyprint">print 'Exception Handling'
try:
    let_us_cause_a_NameError
except NameError as err:
    print(err, '--&gt; our error message')</pre>
