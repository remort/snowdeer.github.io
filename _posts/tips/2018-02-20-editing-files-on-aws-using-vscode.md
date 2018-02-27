---
layout: post
title: Visual Studio Code를 이용해서 AWS의 EC2 인스턴스내 파일 수정하기
category: Tips
tag: [IDE, vscode, AWS]
---
## Visual Studio Code와 AWS EC2 인스턴스와 연결

Visual Studio Code(이하 vscode)를 이용해서 AWS의 EC2 인스턴스내의 파일을 수정하는 방법입니다. 사실 [WinSCP](https://winscp.net/eng/download.php)를 사용해도 되고 다양한 방법들이 있지만 여기서는 간단하게 vscode를 이용해서 접근하는 방법을 소개합니다.

<br>

## ftp-simple 설치

먼저 [vscode 마켓](https://marketplace.visualstudio.com/items?itemName=humy2833.ftp-simple)에서 `ftp-simple`를 설치합니다.

<br>

## ftp-simple 설정

설치를 한다음 '다시로드(Reload)'를 한 번 해주고, <kbd>F1</kbd> 키를 눌러서 `ftp-simple : Config`을 검색합니다.

![image](/assets/tips/001.png)

그러면 아래와 같은 템플릿이 뜨는데, 

<pre class="prettyprint">
[
	{
		"name": "localhost",
		"host": "",
		"port": 21,
		"type": "ftp",
		"username": "",
		"password": "",
		"path": "/",
		"autosave": true,
		"confirm": true
	}
]
</pre>

여기에 서버 접속 정보를 작성하면 됩니다. 예를 들어 저는 다음과 같이 작성했습니다.

<pre class="prettyprint">
[
	{
		"name": "AWS 개발 서버",
		"host": "111.222.333.444",
		"port": 22,
		"type": "sftp",
		"username": "centos",
		"path": "/",
		"autosave": true,
		"confirm": true
        "privateKeyPath": "D:\\aws-key\\snowdeerAWS.pem"
	}
]
</pre>

<br>

## 접속 확인

이제 <kbd>F1</kbd> 키를 눌러서 `ftp-simple : Remote directory open to workspace`를 선택합니다.

![image](/assets/tips/002.png)

그러면 위와같이 조금 전에 작성한 접속 설정 리스트가 뜨고, 원하는 항목을 선택해서 접속합니다. 그 이후 접속할 디렉토리를 지정해주면 해당 디렉토리 내부의 파일들을 vscode를 이용해서 수정할 수 있게 됩니다.