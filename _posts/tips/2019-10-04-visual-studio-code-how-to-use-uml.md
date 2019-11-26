---
layout: post
title: Visual Studio Code 에서 UML 사용하기
category: Tips
tag: [IDE, vscode]
---
# PlantUML 설치

`VSCode`의 `EXTENSIONS: MARKETPLACE`에서 `PlantUML` 플러그인을 설치합니다.

![image](/assets/tips/vscode-uml-001.png)

<br>

## Sequence Diagram

빈 텍스트를 만들어서 아래와 같은 내용을 입력합니다.

<pre class="prettyprint">
@startuml

scale 2
title My first Diagram

A -> B : Hello
B -> C : Good Morning
C --> A : Bye Bye

@enduml
</pre>

그 이후 <kbd>Shift</kbd> + <kbd>Command</kbd> + <kbd>P</kbd> 키를 눌러서 `PlantUML: Preview Current Diagram`을 선택하면 아래 이미지와 같이 UML 미리보기를 할 수 있습니다.

![image -fullwidth](/assets/tips/vscode-uml-002.png)

<br>

## GraphViz 설치

PlantUML은 기본적으로 Sequence Diagram을 지원합니다. Class Diagram을 그리기 위해서는 `GraphViz`를 설치해줘야 합니다.

MacOS 기준으로 

<pre class="prettyprint">
brew install libtool
brew link libtool
brew install graphviz
brew link --overwrite graphviz
</pre>

그 이후 다음과 같은 코드를 작성해서 제대로 렌더링 되는지 확인해봅니다.

<pre class="prettyprint">
@startuml

scale 2
class Event {
    +startTime: DateTime
    +venue: string
    +registrationClosed: boolean
    -notifyAttendes()
}

class ApplicationUser {
    -userName: string
    -password: string
    +isLocked: boolean
    -suggestRandomPasswod()
    +changeProfilePic()
}

class Speaker {
    +openForSpeaking: boolean
    -callForAction()
    +applyAsSpokePerson()
}

class Topic {
    +title: string
    +sourceCodeUrl: string
    +downloadMaterials()
}

class Attendee {
    -eventAttended: number
    -suggestEventBasedOnPreference()
    +registerForTicket()
}

ApplicationUser <|-- Speaker
ApplicationUser <|-- Attendee
Speaker "1" *-- "*" Topic
Event "1" o-- "*" Speaker
Event "1" o-- "*" Attendee

@enduml
</pre>

![image -fullwidth](/assets/tips/vscode-uml-003.png)

<br>

## PlatUML 장점

PlantUML은 텍스트 스크립트 기반 UML이라서 진입 장벽이 조금 있습니다. 물론, 그 문법이 어렵지는 않아서 금방 사용할 수 있습니다. 

텍스트 기반이라 가장 큰 장점은 협업이 쉽다는 점입니다. `git`을 이용해서 서로 공유 및 편집이 쉽습니다.

물론 무료라서 라이센스 걱정 없는 것도 큰 장점입니다.