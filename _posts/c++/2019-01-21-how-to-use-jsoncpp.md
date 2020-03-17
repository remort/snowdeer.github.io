---
layout: post
title: JsonCpp 라이브러리 사용하기
category: C++
tag: [C++]
---
# C++ JsonCpp 사용하기

JsonCpp의 repository는 [여기](https://github.com/open-source-parsers/jsoncpp)입니다.

<br>

## Amalgamated source

JsonCpp는 `Amalgamated source`이라는 기능을 제공합니다. 이 기능은 간단한 간단한 `python` 스크립트를 이용해서 몇 개의 헤더 파일과 소스 파일을 만들어서 다른 프로젝트에서 쉽게 사용할 수 있게 해주는 기능입니다. JsonCpp 소스 파일 전체를 참조하지 않고 몇 개의 파일 추가만으로 기능을 사용할 수 있기 때문에 사용성이 많이 간편해집니다.

보다 자세한 내용은 [여기](https://github.com/open-source-parsers/jsoncpp/wiki/Amalgamated-(Possibly-outdated))에서 확인할 수 있습니다.

명령어는 다음과 같습니다.

<pre class="prettyprint">
python amalgamate.py
</pre>

이 명령어를 수행하면 해당 디렉토리에 `dist`라는 디렉토리가 생성되며 그 안에

~~~
.
├── json
│   ├── json-forwards.h
│   └── json.h
└── jsoncpp.cpp
~~~

구조로 파일들이 생성됩니다. 이 파일들을 복사해서 사용하려는 프로젝트에 넣고 사용하면 됩니다.

<br>

## CMakeLists.txt

사용하려는 프로젝트의 `CMakeLists.txt` 사용 예제는 다음과 같습니다.

<pre class="prettyprint">
cmake_minimum_required(VERSION 3.15)
project(JsonExample)

set(CMAKE_CXX_STANDARD 14)

add_executable(JsonExample main.cpp jsonCpp/jsoncpp.cpp jsonCpp/json/json.h jsonCpp/json/json-forwards.h)
</pre>

<br>

## Json 파싱 예제

<pre class="prettyprint">
#include &lt;iostream&gt;
#include &lt;fstream&gt;

#include "jsonCpp/json/json.h"

using namespace std;

int main() {
    Json::Value root;
    Json::CharReaderBuilder reader;
    ifstream is("MoveToPoint.json", ifstream::binary);
    auto bret = Json::parseFromStream(reader, is, &root, &errorMessage);

    if (bret == false) {
        cout << "Error to parse JSON file !!!" << endl;
    }

    cout << "root: " << root << endl;
    cout << "root.size(): " << root.size() << endl;
    cout << "name: " << root["name"] << endl;

    auto nodeList = root["nodeList"];
    cout << "nodeList.size(): " << nodeList.size() << endl;

    int left = 0;
    for(auto n : nodeList) {
        cout<<"- node: " << n["className"] << endl;
        cout<<"-      id: " << n["id"] << endl;
        left = n["left"].asInt();
        cout<<"-      left: " << left << endl;
        cout<<"-      paramList: " << n["paramList"] << endl;
    }

    return 0;
}
</pre>

<br>

## Json 생성 예제

<pre class="prettyprint">

Json::Value compInfo;
compInfo["category"] = it->second;
compInfo["className"] = classInfo.name();
compInfo["type"] = getType();

auto seq = 0;
for (auto it = iter.begin(); it != iter.end(); it++) {
  Json::Value paramInfo;
  paramInfo["seq"] = seq++;
  paramInfo["type"] = getTypeFromKind(it->second->kind());
  paramInfo["name"] = it->first;

  compInfo["paramList"].append(paramInfo);
}

root["componentList"].append(compInfo);

auto result = root.toStyledString();
}
</pre>