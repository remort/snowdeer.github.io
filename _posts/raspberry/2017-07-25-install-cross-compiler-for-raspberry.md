---
layout: post
title: 라즈베리파이 C++ 크로스 컴파일(Cross Compile) 환경 설정
category: 라즈베리파이
permalink: /raspberry/:year/:month/:day/:title/

tag: [라즈베리파이, 컴파일러]
---

라즈베리파이용 프로그램을 Windows PC에서 개발할 수 있도록 크로스 컴파일(Cross Compile) 환경을 설정하는 방법입니다.

<br>

# 툴체인(Toolchain) 다운로드 및 설치

먼저 [Windows toolchain for Raspberry/PI](http://gnutoolchains.com/raspberry/)을 다운로드 하고 설치를 합니다. 

![image -fullwidth](/assets/2017-07-25-install-cross-compiler-for-raspberry/01.png)

<br>

# Eclipse 설치

[Visual GDB](https://visualgdb.com/) 같은 상용 IDE도 존재하지만, 여기서는 무료로 사용할 수 있는 Eclipse를 활용하도록 하겠습니다.

[Eclipse Download 사이트](https://www.eclipse.org/downloads/eclipse-packages/)로 가서 Eclipse IDE for C/C++ Developers를 다운로드합니다.

그리고 환경 설정으로 가거나 새로운 프로젝트를 하나 만들어서 Cross GCC 설정을 해줍니다.

![image](/assets/2017-07-25-install-cross-compiler-for-raspberry/02.png)


![image](/assets/2017-07-25-install-cross-compiler-for-raspberry/03.png)

크로스 컴파일러 prefix를 `arm-linux-gnueabihf-`로 설정합니다.

이제 Hello, World 등의 간단한 코드를 실행해서 빌드가 잘되는지 확인하면 됩니다.

<br>

# C++11 환경 설정

라즈베리파이용 툴체인도 C++11 이상을 지원합니다. 따라서 이왕 개발할거면 C++11 이상으로 개발하는 것이 좋을 것 같습니다. 

![image](/assets/2017-07-25-install-cross-compiler-for-raspberry/04.png)

~~~
Project > Properties > C/C++ Build / Settings > Tool Settings에서 [GCC C++ Compiler] 항목과 [GCC C Compiler] 항목에서 [Miscellaneous] 항목을 찾습니다. 그리고 [Other flags] 뒷 부분에 다음 옵션을 추가합니다.

-std=c++0x
~~~

<br>

위와 같이 설정하면 C++11 문법으로 빌드는 되지만, Eclipse IDE 상에서는 문법 오류로 표시됩니다. 아래의 설정을 추가로 해주시면 됩니다.

![image](/assets/2017-07-25-install-cross-compiler-for-raspberry/05.png)

~~~
Project > Properties > C/C++ General > Preprocessor Include Paths, Macros etc에서 [Providers] 탭을 선택한 다음 [CDT GCC Built-in Compiler Settings MinGW] 항목을 선택합니다. 그리고 아래쪽에 있는 Flag 설정 칸에 다음 옵션을 추가해줍니다.

-std=c++0x
~~~

<br>

그 이후 `C/C++ Index Rebuild` 를 수행해주면 Eclipse IDE가 C++11 문법을 정상적으로 인식합니다.

![image](/assets/2017-07-25-install-cross-compiler-for-raspberry/06.png)

