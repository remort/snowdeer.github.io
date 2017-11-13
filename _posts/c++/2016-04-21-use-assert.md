---
layout: post
title: assert 활용하기
category: C++
tag: [C++, CleanCode, Debug]
---

`assert`를 잘 쓰면 개발에 꽤 유용할 때가 많습니다.

`assert`는 디버그 모드(Debug Mode)에서만 작동하고 릴리즈 모드(Release Mode)에서는 컴파일 단계에서 무효화되어 걸러지기 때문에 디버깅 용도로 쓰기에 참 좋습니다. (보통 디버깅 환경에서는 로그 메세지를 남발하며 뿌리다가 릴리즈 모드에서는 그 로그 메세지들을 일일이 막는 것도 번거로운 작업들입니다. 그래서 Log 함수들을 별도의 클래스로 Wrapper하는 경우도 많이 있습니다.)

`assert`는 괄호안에 들어가는 조건식이 `false`인 경우 오류 메세지를 출력하면서 프로그램을 강제 종료시킵니다. 프로그램을 강제 종료함으로 인해, 나중에 발생할지도 모르는 논리적인 오류들을 사전에 막아주는 역할을 해서 디버깅 환경에 큰 도움이 됩니다.

`assert` 활용 예제는 다음과 같습니다.

<br>

# assert 활용 예제

<pre class="prettyprint">
static const char* weeks[] = {
    "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday",
    "Friday", "Saturday",
};
void displayWeek(int day) {
  assert((day >= 0) && (day <=6));
  cout<<weeks[day];
}
</pre>

위 코드에서 `displayWeek` 함수의 인자로 0보다 작거나 6보다 큰 값을 입력하게 되면 다음과 같은 오류 메세지를 출력합니다.

~~~
assertion "(day >= 0) && (day <=6)" failed: file "/cygdrive/c/Workspace/Clion/SnowDeer/main.cpp", line 11, function: void displayWeek(int)
~~~

<br>

# 주의할 점

`assert`의 매개변수에는 조건식만 들어가야 합니다. 만약 조건식이 아닌 실행 가능한 구문이 들어가게 되면 큰 오류를 범할 수 있습니다. `assert`는 디버그 모드에서만 활성화되며, 릴리즈 모드에서는 활성화되지 않기 때문에 다음과 같은 코드는 문제가 발생할 수 있습니다.

<pre class="prettyprint">
void openFile() {
  FILE* fp;
  assert((fp = fopen(file_name, "r")) != nullptr));
}
</pre>

`assert'는 잘 쓰면 좋은 효과를 가져올 수 있지만, 입력해야 하는 매개변수 또한 주의하지 않으면 또다른 문제를 발생하기도 합니다. 따라서 매개변수에 들어가는 조건문을 최대한 간결하게해서 필요한 부분에만 적절히 사용한다면 개발에 큰 도움이 될 수 있을 것 같습니다.