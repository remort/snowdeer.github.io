---
layout: post
title: Eclipse에서 C++11 이상 지원하도록 설정하는 방법
category: C++
tag: [C++11, Eclipse]
---

## C++11

C언어는 지금도 계속해서 발전하고 있습니다. 2011년 C++11 규격이 정해졌으며,
그 이후에도 계속하여 발전하고 있습니다. 지금은 현재 C++14, C++17 까지 나온 상태입니다.
개인적인 생각으로는 C++11에서 아주 큰 변화(스마트 포인터 지원 및 Thread 표준화)가 있었던 것 같습니다.
그래서 신규 프로젝트를 진행한다면 적어도 C++11 이상은 사용하는 것이 좋다고 생각합니다.
그 중에서도 스마트 포인터(Smart Pointer)는 메모리 릭(Memory Leak) 방지에 아주 큰 도움이 되며,
표준화가 된 쓰레드(Thread)는 멀티 플랫폼 지원에 유리할 것 같습니다.

<br>

## Eclipse for C++11
Visual Studio나 CLion 등 좋은 C++ IDE가 많이 있지만, 회사에서는 아무래도 Eclipse를
많이 사용하게 되네요. 아마도 라이센스를 크게 신경쓸 필요가 없어서이지 않을까 싶습니다.
그래서 Eclipse에서 C++11 이상을 지원하는 방법을 포스팅해봅니다.
저는 현재 PC에 MinGW를 설치했기 때문에 포스팅을 MinGW 기준으로 설명을 하고 있지만,
MinGW가 아니더라도 Cygwin 등 기타 다른 컴파일러에서도 거의 동일한 설정을 하면 됩니다.

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2017-07-14-eclipse-setting-for-cpp11/01.png)

일단 새로운 프로젝트를 시작하고 컴파일러를 MinGW를 선택합니다.
(굳이 새로운 프로젝트를 시작할 필요없이 세팅에서 바로 설정을 할 수도 있지만,
여기서는 실제로 C++11 문법으로 잘 빌드가 되는지 확인하기 위해서 새로운 프로젝트를
만들어서 테스트를 합니다.)

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2017-07-14-eclipse-setting-for-cpp11/02.png)

~~~
Project > Properties > C/C++ Build / Settings > Tool Settings에서 [GCC C++ Compiler] 항목과 [GCC C Compiler] 항목에서 [Miscellaneous] 항목을 찾습니다. 그리고 [Other flags] 뒷 부분에 다음 옵션을 추가합니다.

-std=c++0x
~~~


<br>

이제 간단한 코드를 작성하여 테스트를 해봅니다. 코드 예제는 다음과 같습니다.
코드 작성 후 빌드를 실행해봅니다.

<br>

<pre class="prettyprint">#include &lt;cstdio&gt;
#include &lt;memory&gt;
#include &lt;string&gt;
#include &lt;thread&gt;

using namespace std;

int main(void) {
  printf("Hello SnowDeer.\n");

  shared_ptr&lt;string&gt; a = make_shared&lt;string&gt;();

  return 0;
}</pre>

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2017-07-14-eclipse-setting-for-cpp11/03.png)

<br>

위 이미지와 같이 빌드가 잘 되는 것을 확인할 수 있습니다.

하지만 다음 이미지에 나온 것처럼 문제점이 하나 있습니다.

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2017-07-14-eclipse-setting-for-cpp11/04.png)

컴파일 및 빌드는 잘되지만, Eclipse의 에디터에서는 오류가 난 것처럼 표시되고 있습니다.
그냥 무시하고 개발을 진행해도 되지만, 시각적으로 상당히 불편합니다. 이럴거면 IDE를 사용하는
의미가 없을 것 같네요. 이를 해결하기 위해서는 IDE 내의 에디터키가 C++11 이상을 지원하도록
추가 설정을 해주어야 합니다. 앞서 설정한 내용은 컴파일러의 옵션만 설정한 것이기 때문에
추가 설정이 필요합니다.

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2017-07-14-eclipse-setting-for-cpp11/05.png)

~~~
Project > Properties > C/C++ General > Preprocessor Include Paths, Macros etc에서 [Providers] 탭을 선택한 다음 [CDT GCC Built-in Compiler Settings MinGW] 항목을 선택합니다. 그리고 아래쪽에 있는 Flag 설정 칸에 다음 옵션을 추가해줍니다.

-std=c++0x
~~~

<br>


![image -fullwidth]({{ site.baseurl }}/assets/2017-07-14-eclipse-setting-for-cpp11/06.png)

그 이후 `C/C++ Index Rebuild` 를 수행해주면 Eclipse IDE가 C++11 문법을 정상적으로 인식하기 시작합니다.

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2017-07-14-eclipse-setting-for-cpp11/07.png)

이제 깔끔하게 Eclipse에서 C++11 이상의 문법들로 프로그램을 작성할 수 있습니다.
