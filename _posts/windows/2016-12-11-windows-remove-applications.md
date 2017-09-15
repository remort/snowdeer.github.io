---
layout: post
title: Windows 10 - 불필요한 프로그램 삭제하기

category: Windows
permalink: /windows/:year/:month/:day/:title/
tag: [Windows]
---
# 불필요한 프로그램 삭제

Windows 10을 설치하면 기본적으로 설치되는 프로그램들이 많이 있습니다. 삼성이나 LG 등 대기업 노트북이라서 제조사 프로그램들이 많아서 그런 줄 알았는데, Microsoft에서 직접 만든 Surface에서도 마찬가지였습니다.

## 설정에서 프로그램 삭제

사용하지 않는 어플들을 삭제하기 위해서는

~~~
모든 설정 → 시스템 → 앱 및 기능
~~~
으로 들어가면 다음 화면이 나오며, 여기에서 삭제를 할 수 있습니다.

![image](/assets/tips-windows/001.png)


하지만, 이 방식으로 삭제되지 않는 프로그램들이 많이 있습입니다. 대표적인 것으로 ‘원노트(OneNote)’ 등이 있습니다. 제 경우는 MS Office를 별도 버전으로 구매해서 쓰고 있기 때문에 기본 탑재된 원노트는 삭제하고 있습니다.

<br>

## 터미널을 이용한 프로그램 삭제

제어판에서 삭제되지 않는 프로그램을 지우기 위해서는 관리자 권한으로 Command(또는 PowerShell)를 실행하여 삭제 할 수 있습니다. 관리자 권한으로 Command를 실행하기 위해서는 다음과 같이 PowerShell 아이콘 위에서 마우스 오른 버튼을 눌러서 실행할 수 있습니다.

![image](/assets/tips-windows/002.png)

프로그램을 삭제하기 위해서는 각 프로그램의 패키지 이름(Package Name)을 알아야 하는데, 패키지 이름은 다음 명령어를 통해서 알 수 있습니다.

~~~
Get-AppxPackage -AllUsers

또는

Get-AppxPackage -AllUsers *OneNote*
~~~

<br>

패키지 이름을 확인한 후, 다음 명령어를 통해 어플 삭제를 할 수 있습니다.

~~~
Get-AppxPackage -AllUsers Microsoft.Office.OneNote | Remove-AppxPackage
~~~