---
layout: post
title: C++ Google Test 샘플
category: C++
tag: [C++]
---
# Google Test
Google Test 샘플 코드입니다.

## CMakeLists.txt

`FetchContent` 명령어를 이용해서 Google Test 라이브러리를 연동합니다.

<pre class="prettyprint">
cmake_minimum_required(VERSION 3.21)
project(gtest_sample)

set(CMAKE_CXX_STANDARD 17)


include(FetchContent)
FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.11.0)

FetchContent_MakeAvailable(googletest)
enable_testing()

add_executable(gtest_sample Calculator.cpp CalculatorTests.cpp)

target_link_libraries(
        gtest_sample
        gtest_main
)
</pre>


## Calculator.hpp

<pre class="prettyprint">
#ifndef GTEST_SAMPLE__CALCULATOR_H_
#define GTEST_SAMPLE__CALCULATOR_H_

class Calculator {
 public:
  Calculator();

 public:
  int add(int x, int y);
  int sub(int x, int y);
};


#endif //GTEST_SAMPLE__CALCULATOR_H_
</pre>

## Calculator.cpp

<pre class="prettyprint">
#include "Calculator.hpp"
Calculator::Calculator() {

}
int Calculator::add(int x, int y) {
  return x + y;
}
int Calculator::sub(int x, int y) {
  return x - y;
}
</pre>

## CalculatorTests.cpp 

<pre class="prettyprint">
#include &lt;gtest/gtest.h&gt;
#include "Calculator.h"

TEST(Calculator_Add_Test, test_name) {
  Calculator c;
  EXPECT_EQ(8, c.add(3, 5));
}

TEST(Calculator_Sub_Test, test_name) {
  Calculator c;
  EXPECT_EQ(7, c.sub(12, 5));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
</pre>

## 실행결과

<pre class="prettyprint">
/Users/snowdeer/Workspace/cpp/gtest/cmake-build-debug/gtest_sample
[==========] Running 2 tests from 2 test suites.
[----------] Global test environment set-up.
[----------] 1 test from Calculator_Add_Test
[ RUN      ] Calculator_Add_Test.test_name
[       OK ] Calculator_Add_Test.test_name (0 ms)
[----------] 1 test from Calculator_Add_Test (0 ms total)

[----------] 1 test from Calculator_Sub_Test
[ RUN      ] Calculator_Sub_Test.test_name
[       OK ] Calculator_Sub_Test.test_name (0 ms)
[----------] 1 test from Calculator_Sub_Test (0 ms total)

[----------] Global test environment tear-down
[==========] 2 tests from 2 test suites ran. (0 ms total)
[  PASSED  ] 2 tests.

Process finished with exit code 0
</pre>