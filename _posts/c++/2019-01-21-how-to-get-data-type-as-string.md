---
layout: post
title: DataType 출력하기
category: C++
tag: [C++]
---
# C++ DataType 출력하기

## typeid 명령어 이용

리눅스에서 gcc 기반으로 실행했을 때 Windows에서 Visual Studio로 실행했을 때의 결과가 조금 다릅니다.

<pre class="prettyprint">
void test01() {
    int a = 10;
    double b = 10.0;
    string c = "abc";
    bool d = true;

    cout << typeid(a).name() << ", " << typeid(b).name() << ", " << typeid(c).name() << ", " << typeid(d).name()
         << endl;

}
</pre>

~~~
i, d, NSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE, b
~~~


## std::is_same 명령어 이용

<pre class="prettyprint">
template&lt;typename T&gt;
std::string getType(T) {
    std::string type = "unknown";
    if (std::is_same&lt;T, int&gt;::value) type = "int";
    if (std::is_same&lt;T, double&gt;::value) type = "double";
    if (std::is_same&lt;T, float&gt;::value) type = "float";
    if (std::is_same&lt;T, bool&gt;::value) type = "bool";
    if (std::is_same&lt;T, string&gt;::value) type = "string";


    return type;
}

void test02() {
    int a = 10;
    double b = 10.0;
    string c = "abc";
    bool d = true;

    cout << getType(a) << ", " << getType(b) << ", " << getType(c) << ", " << getType(d) << endl;
}
</pre>

~~~
int, double, string, bool
~~~
