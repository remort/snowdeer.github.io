---
layout: post
title: Git 설명서 - (7) 브랜치의 개념
category: Git
tag: [git]
---
# 브랜치

Git을 사용하는 가장 큰 이유 중 하나는 '브랜치(Branch)' 때문이라고 할 수 있습니다. 사실 저도 브랜치 사용하는 법은 익숙하지 않아서 잘 사용하지 못하는데, Git에서는 브랜치 활용을 적극적으로 추천하고 있습니다. 

심지어 혼자서 개발을 하더라도, 하루에 수십 번씩 브랜치를 생성하더라도 브랜치 활용을 적극적으로 하기를 추천하고 있습니다.

<br>

# Git의 브랜치

Git은 소스 관리를 이전 버전과 현재 버전과의 차이점(Diff)으로 저장하는 것이 아닌, 해당 시점의 스냅샷(Snapshot)으로 저장하고 있습니다. 

그리고 소스를 commit 하게 되면 이전 commit과의 연결 포인터를 저장하고 있습니다. 따라서 다음 그림처럼 각 commit 간의 이동이 자유롭습니다.

![image](/assets/git-reference/02.png)

Git에서의 브랜치는 각 commit 간 이동을 쉽게 해줄 수 있는 포인터라고 생각하면 됩니다. Git은 기본적으로 `master` 브랜치를 만들어줍니다.

![image](/assets/git-reference/03.png)

`master` 브랜치는 가장 마지막 commit을 가리키고 있습니다.

<br>

## 브랜치 생성

새로운 브랜치를 생성해봅니다.

<pre class="prettyprint">
$ git branch testing
</pre>

![image](/assets/git-reference/04.png)

위 그림처럼 새로 만든 브랜치도 기본적으로 가장 마지막의 commit을 가리키게 됩니다.

<br>

## HEAD 브랜치

Git에서 `HEAD` 브랜치는 현재 작업중인(checkout 상태인) 브랜치를 가리키는 특수한 포인터입니다. 

위에서 'testing' 이라는 브랜치를 새로 생성하긴 했지만, 'checkout'을 하지 않았기 때문에 `HEAD` 브랜치는 여전히 `master` 브랜치를 가리키고 있습니다.

![image](/assets/git-reference/05.png)

여기서 'testing' 브랜치로 checkout을 하게 되면 다음과 같이 `HEAD` 브랜치의 위치가 옮겨가게 됩니다.

![image](/assets/git-reference/06.png)

여기서 'testing' 브랜치에 새로운 commit을 하면 소스 트리는 다음과 같은 형태가 됩니다.

![image](/assets/git-reference/07.png)

이 상태에서 다시 `master` 브랜치로 checkout을 하면 `HEAD` 브랜치의 위치가 옮겨집니다.

![image](/assets/git-reference/08.png)

즉, 이렇게 `checkout`을 이용해서 각 브랜치간 이동을 자유롭고 간편하게 할 수 있습니다.

만약, 이 상태에서 `master` 브랜치에서 새로운 작업을 commit 하게 되면 소스 트리는 다음과 같은 형태가 될 것입니다.

![image](/assets/git-reference/09.png)

<br>

# Git 브랜치의 장점

보통 다른 버전 관리 시스템에서는 브랜치를 생성하게 되면 해당 버전의 파일들을 통째로 복사하기 때문에 속도도 느리고 용량도 많이 차지하게 됩니다.

하지만, Git에서 브랜치는 각 commit을 가리키는 40 글자의 SHA-1 체크섬 파일에 불과합니다. 즉, 아주 가볍습니다. 그래서 브랜치 생성이나 브랜치간 이동도 아주 자유롭고 빠릅니다. 또한 Git의 브랜치는 이전 버전의 commit 포인터를 갖고 있기 때문에 나중에 소스 정합(Merge)를 할 때도 수월하고 편하게 작업할 수 있는 장점이 있습니다.