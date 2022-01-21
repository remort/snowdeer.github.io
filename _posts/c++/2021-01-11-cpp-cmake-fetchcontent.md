---
layout: post
title: CMake FetchContent 명령어
category: C++
tag: [C++]
---
# FetchContent

`FetchContent`는 `CMake 3.11`에 새로 추가된 명령어입니다.
대부분의 언어들은 외부 라이브러리들을 사용하기 위해 라이브러리 설치 또는 종속성 추가를 쉽게 할 수 있는
기능을 제공하고 있습니다. 예를 들면, Java의 `maven`이나 `gradle`, Python의 `pip` 등입니다. 하지만, C++에는
그런 기능이 없었기 때문에 외부 라이브러리를 사용하기 위해서는 아주 까다로운 설정이 필요했었습니다.

이제 `FetchContent` 명령어를 통해 CMake에서도 외부 라이브러리들을 쉽게 설치하고 사용할 수 있게 되었습니다.

## ExternalProject과 차이

CMake 3.11 버전 이전에도 `ExternalProject` 명령어를 이용해서 외부 라이브러리를 사용할 수 있었습니다. 
하지만, `ExternalProject` 명령어는 빌드 타임에 외부 라이브러리를 가져오지만, `FetchContent`는 CMake 실행 시점에
외부 라이브러리를 가져온다는 점에서 차이가 있습니다.

## FetchContent 사용 예제

`CMakeLists.txt` 파일에 다음 명령어를 이용해서 외부 라이브러리를 가져올 수 있습니다.
여기서는 [nlohnmann json](https://github.com/nlohmann/json)를 예시로 작성했습니다.

<pre class="prettyprint">
include(FetchContent)
FetchContent_Declare(json
        GIT_REPOSITORY https://github.com/nlohmann/json
        GIT_TAG v3.10.5
        GIT_PROGRESS TRUE
        GIT_SHALLOW TRUE)

FetchContent_MakeAvailable(json)
</pre>

## CMakeLists.txt 전체 코드

<pre class="prettyprint">
cmake_minimum_required(VERSION 3.21)
project(nhlomann_json_example)

set(CMAKE_CXX_STANDARD 14)

include(FetchContent)
FetchContent_Declare(json
        GIT_REPOSITORY https://github.com/nlohmann/json
        GIT_PROGRESS TRUE
        GIT_SHALLOW TRUE
        GIT_TAG v3.10.5)

FetchContent_MakeAvailable(json)

add_executable(nhlomann_json_example main.cpp)

target_link_libraries(nhlomann_json_example
        PRIVATE nlohmann_json::nlohmann_json)
</pre>

## main.cpp

그 이후에는 아래와 같이 `nlohmann/json.hpp` 파일 등을 `include`해서 사용할 수 있습니다.

<pre class="prettyprint">
#include &lt;iostream&gt;
#include &lt;nlohmann/json.hpp&gt;

int main() {
    std::cout << "Hello, World!" << std::endl;

    // ...

    return 0;
}
</pre>