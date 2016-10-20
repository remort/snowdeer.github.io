---
layout: post
title: Windows 10 설치 후 할 일
category: Windows10
tag: [windows]
---

윈도우 10을 사용하고 나서 초기화를 벌써 수 차례 진행하다 보니
자꾸만 잊어버리는 것들이 있어서 메모 차원에서 간단히 할 일들을 적어보았습니다.  
참고로 이 글은 지극히 개인적인 용도로 작성한 글입니다.

<br>

## Windows & 드라이버 업데이트

너무나 당연한 일입니다. 최우선적으로 하도록 합시다.  
화면 우측 슬라이딩 메뉴에서 

~~~
모든 설정 → 업데이트 및 복구
~~~

를 선택하면 됩니다.


<br>

## 불필요한 어플 삭제

윈도우 10을 설치해보면 불필요한 어플들이 기본적으로 많이 설치되어 있습니다.  
(대기업 노트북이라 그런 줄 알았는데, Surface Pro도 마찬가지더군요.)  
목록에서 불필요한 어플들은 삭제를 합시다.

화면 우측 슬라이딩 메뉴에서 

~~~
모든 설정 → 시스템 → 앱 및 기능
~~~

을 선택하면 아래 이미지와 같이 각 어플들을 선택해서 삭제할 수 있습니다.

![image -fullwidth]({{ site.baseurl }}/assets/2016-07-21-windows10-setup/20160806_082038.png)

참고로 저는 3D Builder, Skype, Office 365나 소개 및 게임 등의 어플들은 모두 삭제합니다.


<br>

## 삭제되지 않는 어플은?

위의 과정에서 삭제되지 않는 어플들도 있습니다. (ex. 원노트)

![image -fullwidth]({{ site.baseurl }}/assets/2016-07-21-windows10-setup/20160806_082218.png)

이런 경우에는 관리자 권한으로 Command 명령어를 이용하여 삭제를 해줘야 합니다.


관리자 권한으로 Command 명령어를 내리기 위해서

![image -fullwidth]({{ site.baseurl }}/assets/2016-07-21-windows10-setup/20160806_124830.png)

일단, PowerShell을 마우스 오른 버튼을 눌러 관리자 권한으로 실행합니다.

그런 다음 다음 명령어로 원노트의 패키지 이름을 확인합니다.  
(필터링 옵션을 줄 수 있습니다.)

~~~
Get-AppxPackage -AllUsers

Get-AppxPackage -AllUsers *OneNote*
~~~

어플의 이름을 확인한 다음 다음 명령어를 이용하여 삭제를 하도록 합시다.

~~~
Get-AppxPackage -AllUsers Microsoft.Office.OneNote | Remove-AppxPackage
~~~

<br>

## 키보드 재입력 시간 조절

지극히 개인적인 설정입니다.  
키보드 재입력 시간에 딜레이가 있으면 답답함을 많이 느껴서  
(아무래도 코딩이나 글쓰기 작업을 많이 하다 보니...)  
저는 키보드 재입력 시간을 최소로 합니다.

~~~
제어판 → 키보드
~~~

에서 조절할 수 있습니다.


<br>

## JDK 설치

역시나 개인적인 설정입니다.  
Java 기반의 어플들을 설치하거나 실행할 때 JDK가 필요한 경우가
많아서 (대표적인 예로 Eclipse) 아예 미리 설치해 버리는게 편하더군요.
개발용 PC에는 보통 필수적인 설치라고 볼 수 있습니다!!

<br>

## 필요한 어플 설치

* [반디집](https://www.bandisoft.co.kr/bandizip/) : 압축 프로그램
* [꿀뷰](https://www.bandisoft.co.kr/honeyview/) : 이미지 뷰어
* [크롬](https://www.google.co.kr/chrome/browser/desktop/)
* [DropBox](https://www.dropbox.com/ko/downloading) : 1password나 기타 작업 파일들을 위한 클라우드 저장소
* [1Password](https://agilebits.com/downloads) : 비밀번호 관리 어플(유료)
* [Sublime Text](https://www.sublimetext.com/) : 텍스터 에디터기(유료)
* MS Office 2016 : 오피스(유료)
* [SetPoint](http://support.logitech.com/en_us/software/setpoint) : Logitech 키보드 & 마우스 설정 어플
* [Visual Studio Community](https://www.visualstudio.com/ko-kr/products/visual-studio-community-vs.aspx)
* [Eclipse](https://eclipse.org/downloads/)
* [GitHub Desktop](https://desktop.github.com/)
* [Everything](https://www.voidtools.com/) : 파일 검색툴 
