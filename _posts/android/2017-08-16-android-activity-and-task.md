---
layout: post
title: Activity와 Task, 실행 Flag
category: Android

tag: [Android]
---
# Stack

여기서 스택(Stack)은 각 Activity가 실행될 때 그 순서가 저장되는 공간입니다.
'뒤로 가기(Back)' 버튼을 누르면 최상단의 Activity가 종료되고 그 다음 Activity가
화면에 출력됩니다.

<br>

# Task

Activity는 그룹으로 묶여서 관리됩니다. 여러 Activity를 하나의 그룹으로 묶은 것이
태스크(Task)입니다. 안드로이드 공식 사이트에서는 'Task는 사용자 관점에서 보는 
App 단위'라고 말하고 있습니다.

Task 역시 스택에 저장됩니다.

<br>

## 복수의 Activity와 복수의 Task간 관계

예를 들어 다음과 같은 App이 있다고 가정합니다.

* 메일 App
* 갤러리 App

시나리오는 다음과 같습니다.

1. 메일 작성 App을 실행한다. 
2. 첨부하기 버튼을 눌러 **갤러리 App의 사진 선택 화면**을 실행한다.
3. 메일 작성칸에 사진이 첨부된다.

이 경우, **갤러리 App에 있는 사진 선택 화면 Activity**는 **메일 App의 Task**에
포함됩니다. 갤러리 App의 Activity이지만, 사용자 관점에서는 메일 App에 포함된 것으로
보이기 때문입니다.

<br>

## 새로운 Task로 시작하기

위의 시나리오처럼 메일 App과 사진 선택 Activity가 밀접하게 연동되는 경우가 아니라
완전히 다른 App을 실행하는 경우는 서로 다른 Task로 시작되어야 합니다.

<pre class="prettyprint">
    Intent intent = new Intent(MediaStore.ACTION_IMAGE_CAPTURE);
    intent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
    startActivity(intent);
</pre>

<br>

# Affinity

일반적으로 같은 Task에서 실행하는 Activity는 같은 Affinity를 갖고 있습니다.
Affinity는 `AndroidManifest.xml`에서 `taskAffinity` 속성에서 지정할 수 있습니다.
기본적으로 패키지 이름을 갖습니다.

보통은 크게 신경쓰지 않아도 되는 속성이지만, 만약 여러 개의 App을 하나의 패키지로 묶어서
배포하는 경우 각각의 App에 해당하는 Activity 들은 별도의 Affinity를 지정하는 것이 
좋습니다. 그렇게 하지 앟은 경우, 예를 들어 하나의 패키지내에 2개의 Activity가 있으며
각 Activity는 별개의 App 처럼 동작할 때 Affinity가 같다면, 실행 아이콘을 클릭시 전혀
의도하지 않은 Activity가 실행되는 등의 예상하지 못한 문제가 발생할 수 있습니다.

<br>

# Activity 이중 실행 방지

`AndroidManifest.xml`의 `activity`의 속성 중 `launchMode`라는 속성으로 설정할 수 있습니다.

옵션 | 설명
--- | ---
standard | 기본값이며 여러 인스턴스 생성이 가능
singleTop | Task 상단에 해당 Activity 인스턴스가 있는 경우, 새로운 인스턴스 생성을 하지 않고 기존 인스턴스의 `onNewIntent()`를 호출
singleTask | Activity가 새로운 Task의 루트 인스턴스로 실행됨. 다른 Task에 속할 수 없음. Activity가 이미 존재하는 경우는 기존 Activity의 `onNewIntent()`를 호출
singleInstance | `singleTask`와 기본적으로 동일하지만, 시스템은 인스턴스를 보유하고 있는 것 외에 다른 Activity를 실행하지 않음. Task와 Activity는 항상 하나만 있음

일반적으로 `standard`와 `singleTop`를 많이 사용하며, `singleTask`와 `singleInstance`는 개발 사이트에서도 '일반적인 경우에는 권장하지 않는다'라고 되어 있습니다.