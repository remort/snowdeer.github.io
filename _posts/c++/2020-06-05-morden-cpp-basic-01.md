---
layout: post
title: 모던 C++ 입문 책 요약 - (1)
category: C++
tag: [C++]
---
[모던 C++ 입문](http://www.yes24.com/Product/Goods/57615943) 책을 읽고 실수하기 쉽거나 유용한 팁, 더 나은 방법 등을 정리해보았습니다.

## LVvalue

주소를 지정할 수 있는 항목을 `Lvalue`라고 합니다.

<pre class="prettyprint">
int i = 3;
i++;
const int j = 5;
j++;
(3 + 5)++;
</pre>

위의 코드에서 오직 `i`만 `Lvalue`입니다.

> 증감 연산자(++, --) 등은 가독성 등에서 좋지 않습니다. `j++` 보다는 `j+1`이 가독성이 더 좋으며, 컴파일러 최적화도 더 쉽습니다.

<br>

## 할당 연산자

할당 연산자는 오른쪽에서 왼쪽 순서로 결합됩니다. 하지만 가독성에서 그렇게 좋은 편은 아닙니다. 
할당 연산자를 쓰지 말고 전부 개별 라인으로 작성하기를 강제화하는 언어도 있습니다.

<pre class="prettyprint">
o3 = o2 = o1 = expr;
</pre>

<br>

## 인라인(inline)

함수를 호출하면 내부적으로 레지스터(Register)에 저장하고 스택(Stack)에 인수를 복잡하는 작업을 수행합니다.
제법 무거운 비용이 발생하며 컴파일러는 최적화 과정을 통해 함수 호출을 `inline`하기도 합니다. 

`inline`을 하면 해당 코드를 함수에 포함된 연산으로 대체합니다. `inline` 키워드를 이용해 직접 컴파일러에게
인라인 여부를 요청할 수 있습니다.

<pre class="prettyprint">
inline double square(double x) { return x * x; }
</pre>

물론 컴파일러가 `inline`을 무조건 수행할 의무는 없으며, 최적화 과정에서 `inline` 선언되지 않은 함수를 `inline` 하기도 합니다.

<br>

## main 함수

`main()` 함수는 주로 다음과 같은 형태로 작성됩니다.

<pre class="prettyprint">
int main() {
  // ...
}

int main(int argc, char* argv[]) {
  // ...
}

int main(int argc, char** argv[]) {
  for (int i = 0; i < argc i++) {
    cout << argv[i] << endl;
  }

  return 0;
}
</pre>

> main 함수릐 리턴 값은 표준을 준수할 경우 `0`을 리턴합니다. (`<cstdlib>`의 `EXIT_SUCCESS` 매크로에 정의되어 있습니다.)
`return` 문을 생략하더라도 컴파일러가 자동으로 `return 0;`을 삽입하는 경우도 있습니다.

<br>

## assert

`assert`는 C에서 상속받은 매크로이지만 여전히 유용하게 사용되고 있습니다. 괄호 부분이 `false`이면 프로그램을 종료합니다.

`assert`의 가장 큰 장점은 개발시 마음껏 사용하다가 릴리즈(release) 모드로 빌드하면 실행 파일에서 아무런 동작을 하지 않도록
할 수 있다는 점입니다.

<br>

## Exception

`throw`를 이용해서 예외를 던질 수도 있습니다. 이 함수를 호출하는 부분에서 예외 처리를 하지 않으면 App Crash가 발생합니다. 

<pre class="prettyprint">
matrix read_matrix_file(const char* fname, ...) {
  fstream f(fname);
  if (!f.is_open()) {
    throw "Cannot open file.";
  }
  // ...
}
</pre>

C++에서 문자열, 숫자, 사용자 정의 타입 등 모든 타입을 예외로 처리할 수 있지만, 적절하게 처리하게 위해서는 예외 타입을 별도로
정의하거나 표준 라이브러리의 메소드를 사용하는게 좋습니다.

예외 처리는 `try ~ catch` 구문으로 수행할 수 있습니다. `catch(...)` 블록으로 모든 예외를 처리할 수도 있습니다.

<br>

## `endl`과 `\n`

둘 다 다음 라인으로 넘어가는 개행 문자를 생성합니다. 하지만 두 방식은 동작에서 조금 차이가 있습니다.

효율적인 출력을 위해 버퍼링(Buffering)을 사용하는데, `endl`은 버퍼를 비우지만, `\n`은 버퍼를 비우지 않습니다.
버퍼를 비우는 작업은 디버거(Debugger) 없이 디버깅 할 때 프로그램의 Crash가 발생하는 출력을 찾는데 도움이 됩니다.
대신 I/O가 느려질 수 있는 단점이 있습니다.

<br>

## stringstream

> 표준 라이브러리에서 제공하는 `stringstream`은 출력 가능한 모든 타입의 문자열을 만드는데 사용할 수 있습니다.
`str()` 메소드를 이용해서 스트림의 내부 문자열을 반환할 수 있습니다.

<br>

## C++의 I/O 오류 처리

C++의 I/O에는 안전 장치가 없습니다. 예를 들어 다음과 같은 코드에서

<pre class="prettyprint">
int main() {
  std::ifstream infile("some_missing_file.txt";
  int i = 0;
  infile >> i;

  infile.close();

  return 0;
}
</pre>

파일이 만약 존재하지 않더라도 파일을 여는 작업은 실패하지 않습니다. 

기본적으로 스트림은 예외를 발생하지 않습니다. 역사적으로 스트림이 예외 처리보다 먼저 등장했기에 그동안의 코드 호환성을 위해
어쩔 수 없이 이렇게 되었으며, 그래서 보통 별도 플래그 체크를 통해 오류를 확인합니다.

<pre class="prettyprint">
int main() {
  std::ifstream infile("some_missing_file.txt";
  if(infile.good()) {
    int i = 0;
    infile >> i;
  }

  infile.close();

  return 0;
}
</pre>

<br>

## 배열

배열의 크기는 컴파일 타임(Compile-time)에 결정되어져야 합니다. 런타임(Runtime)에 동적으로 바꾸고 싶을 경우에는
`vector` 등을 이용해야 합니다.