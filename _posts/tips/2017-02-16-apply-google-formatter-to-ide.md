---
layout: post
title: IntelliJ에 Google Style Formatter 적용하기(Android Studio, WebStrom 등)
category: Tips
tag: [IDE, Formatter]
---

IntelliJ에 Google Style의 Formatter를 적용하는 방법입니다.

<br>

# 다운로드

`Google Style`은 [여기](https://github.com/google/styleguide)서 받을 수 있습니다.
아래의 이미지처럼 IntelliJ 외에도 Eclipse나 XML, Java, C++ 등 다양한 IDE 및
언어용 스타일을 지원하고 있습니다.

![image -fullwidth](/assets/2017-02-16-apply-google-formatter-to-ide/01.png)

<br>

# IntelliJ에 스타일 설정

다음 메뉴를 통해서 Code Style 설정 창을 띄웁니다.
~~~
IntelliJ → Preference
~~~

![image](/assets/2017-02-16-apply-google-formatter-to-ide/02.png)
여기서 'Scheme'의 'Manage...' 버튼을 누릅니다.

![image](/assets/2017-02-16-apply-google-formatter-to-ide/03.png)
그리고 'Import' 버튼을 누르면 아래와 같은 창이 뜹니다.

![image](/assets/2017-02-16-apply-google-formatter-to-ide/04.png)

여기서 `IntelliJ IDEA code style XML`을 선택합니다.
그리고 아까 다운로드한 파일 내에서 IntelliJ 관련 xml 파일을 선택해줍니다.
![image](/assets/2017-02-16-apply-google-formatter-to-ide/05.png)

<br>

이제 완료되었습니다. IntelliJ에 Google Code Style이 적용되었을 것입니다.

![image](/assets/2017-02-16-apply-google-formatter-to-ide/06.png)

![image](/assets/2017-02-16-apply-google-formatter-to-ide/07.png)
