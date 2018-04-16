---
layout: post
title: 함수 템플릿
category: C++
tag: [C++]
---
# 함수 템플릿

함수를 파라메터별로 다중으로 정의하는 경우는 편의성과 확장성을 얻을 수 있기 때문입니다. 다만, 파라메터별로 일일이 함수를 정의하는 것은 코드 길이도 길어지고 유지보수 측면에서도 좋지않은 방법입니다. 함수 템플릿을 이용하면 이를 보다 쉽고 효율적으로 사용할 수 있습니다.

<br>

## 예제 코드

<pre class="prettyprint">
#include &lt;iostream&gt;

using namespace std;

template&lt;typename T&gt;
T add(T a, T b) {
    return a + b;
}

int main() {
    cout << "hello, snowdeer!" << endl;
    cout << add(3, 4) << endl;
    cout << add(1.1, 2.2) << endl;
    //cout << add(string("abc"), string("def")) << endl;

    return 0;
}
</pre>

함수 템플릿은 사용하는 사람이 어떤 인자를 이용해서 함수를 호출하는가에 따라 컴파일러가 다중 정의 코드를 생성합니다.