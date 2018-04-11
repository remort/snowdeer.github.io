---
layout: post
title: Lambda 함수 사용 예제
category: Python
tag: [Python]
---
# Lambda 함수 사용 예제

## 함수 파라메터로 함수 사용하는 방법

아래 예시 코드는 `add_with_lambda`라는 함수의 파라메터로 함수를 전달하는 예제 코드입니다. `add_one`, `add_two`, `add_three`의 함수를 전달하고 있습니다.

<pre class="prettyprtin">
def add_with_lambda(a, b, lambda_func):
  sum = lambda_func(a) + lambda_func(b)
  return lambda_func(sum)


def add_one(a):
  return a + 1


def add_two(a):
  return a + 2


def add_three(a):
  return a + 3


if __name__ == "__main__":
  print("Lambda Example")

  print(add_with_lambda(3, 5, add_one))
  print(add_with_lambda(3, 5, add_two))
  print(add_with_lambda(3, 5, add_three))
</pre>

함수를 파라메터로 전달하는 기능은 상당히 유용하며, 함수의 재활용성을 높이고 가독성을 높이는데 도움을 줍니다. 하지만, 인자로 사용할 함수를 매번 다른 이름으로 하나씩 만드는 것은 번거로운 일입니다. 이런 경우 람다 함수를 사용하면 코드를 훨씬 더 깔끔하게 만들 수 있습니다.

참고로 람다 함수(Lambda Function)는 '익명 함수'라는 의미입니다. 별도로 이름이 없더라도 함수를 사용할 수 있다는 의미입니다.

<br>

## 람다 함수 사용 예제

위의 예제 코드를 람다 함수를 사용해서 변환한 모습입니다. 코드가 훨씬 더 깔끔해졌습니다.

<pre class="prettyprtin">
def add_with_lambda(a, b, lambda_func):
  sum = lambda_func(a) + lambda_func(b)
  return lambda_func(sum)


if __name__ == "__main__":
  print("Lambda Example")

  print(add_with_lambda(3, 5, lambda a: a + 1))
  print(add_with_lambda(3, 5, lambda a: a + 2))
  print(add_with_lambda(3, 5, lambda a: a + 3))
</pre>