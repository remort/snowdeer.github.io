---
layout: post
title: 람다식
category: C++
tag: [C++]
---
# 람다식

람다(Lambda)는 C++11부터 지원하기 시작했으며, `이름없는 익명 함수`라는 뜻을 갖고 있습니다. 재사용성이 떨어지고 한 번 쓰고 버릴 용도의 함수가 필요한 경우, 그 때마다 함수를 매번 정의해서 작성하는 것은 번거롭습니다. 이런 경우 람다 함수를 사용하면 함수의 정의없이 간편하게 함수를 사용할 수 있습니다.

<br>

## 함수를 변수에 지정하는 예제 코드

<pre class="prettyprint">
#include &lt;iostream&gt;

using namespace std;

int add(int a, int b) {
    return a + b;
}

int main() {
    cout << "hello, snowdeer!" << endl;

    auto lambda = add;

    cout << lambda(3, 4) << endl;

    return 0;
}
</pre>

<br>

## 함수 내부에 람다식 선언

<pre class="prettyprint">
#include &ltiostream&gt;

using namespace std;

int add(int a, int b) {
    return a + b;
}

int main() {
    cout << "hello, snowdeer!" << endl;

    auto func = [](int a, int b) -> void {
        cout << a << " + " << b << " = " << add(a, b) << endl;
    };

    func(5, 6);

    auto lambda = add;

    cout << lambda(3, 4) << endl;

    return 0;
}
</pre>

위에서 보듯이 `[](int a, intb) -> void` 부분이 람다식입니다. 매개변수가 `int a, int b`이며 리턴 타입이 `void`인 람다 함수를 선언했습니다.

<br>

## 본격적인 람다 함수 사용 예제

<pre class="prettyprint">
#include &lt;iostream&gt;
#include &lt;functional&gt;
#include &lt;memory&gt;

using namespace std;

class LambdaTest {
public:
    LambdaTest(function&lt;int(int, int)&gt; lambda) : mLambdaFunc(lambda) {}

    int launch(int a, int b) {
        return mLambdaFunc(a, b);
    }

private:
    function&lt;int(int, int)&gt; mLambdaFunc;
};

int main() {
    cout << "hello, snowdeer!" << endl;

    shared_ptr&lt;LambdaTest&gt; test1 = make_shared&lt;LambdaTest&gt;([](int a, int b) -> int {
        return a + b;
    });

    shared_ptr&lt;LambdaTest&gt; test2 = make_shared&lt;LambdaTest&gt;([](int a, int b) -> int {
        return a - b;
    });

    shared_ptr&lt;LambdaTest&gt; test3 = make_shared&lt;LambdaTest&gt;([](int a, int b) -> int {
        return a * b;
    });

    cout << test1->launch(3, 4) << endl;
    cout << test2->launch(10, 5) << endl;
    cout << test3->launch(2, 7) << endl;

    return 0;
}
</pre>

위의 예제에서 `LambdaTest`라는 클래스는 하나만 선언했지만, 람다식을 이용해서 그 내용을 동적으로 정의하여 다양한 용도로 사용할 수 있는 것을 볼 수 있습니다.