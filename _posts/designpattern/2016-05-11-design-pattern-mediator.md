---
layout: post
title: 중재자(Mediator) 패턴
category: 디자인패턴
permalink: /designpattern/:year/:month/:day/:title/

tag: [디자인패턴]
---

중재자(Mediator) 패턴은 Colleague들의 분쟁을 중간에서 중재해주는 클래스를 두고,
각 Colleague들끼리는 서로를 직접적으로 참조하지 못하도록 하여 낮은 결합도(Low Coupling)를
가지게 해주는 패턴입니다.

Colleague들끼리는 서로를 알지 못하기 때문에 특정 작업을 요청 하려면 무조건 Mediator에게
요청을 해야 합니다. 따라서 Mediator의 함수가 많아지고 코드량이 길어진다는 단점이 있습니다.
하지만, 결합도를 낮추어주기 때문에 향후 Colleague가 변경되거나 추가,
삭제될 때 유지 보수가 유리해진다는 장점이 있습니다.

Mediator 패턴의 UML은 다음과 같습니다.

![Image]({{ site.baseurl }}/assets/design-patterns/mediator.png)

Mediator 패턴을 쓸 수 있는 경우는 다음과 같은 시나리오를 들 수 있습니다.

>어떤 GUI 화면이 있다.
특정 체크박스의 값이 바뀌면 화면에 있는 각 버튼의 Enabled/Disabled 속성이 변경 된다.
또한, 텍스트 박스의 값에 따라 각 컴포넌트의 속성이 변경된다.
이와같이 각 컴포넌트들이 서로가 서로에게 영향을 미치는 경우이다.

<br>

위와 같은 경우 Mediator를 두지 않고, 각 버튼이나 체크 버튼, 텍스트 박스 등의 이벤트 처리 부분에서
각 컴포넌트의 속성을 직접 바꾸는 경우를 생각해 봅시다. 이 경우, 복수의 컴포넌트가 하나의 리소스에
동시에 작업 요청을 한다던지, 두 컴포넌트가
 서로에게 작업 요청을 하여 무한 루프와 같은 데드락(Deadlock) 상황이 발생할 수도 있습니다.
 또한 각 컴포넌트간이 참조가 많기 때문에 향후 특정 컴포넌트가 다른 컴포넌트로 교체하는 등의
 유지보수가 쉽지 않다는 문제가 있습니다.

이런 경우 Mediator를 두고 각 상태에 따라 Mediator가 각 컴포넌트의 상태들을 변경시켜주게 되면
코드도 깔끔해지고 유지 보수도 수월할 수 있습니다.

<br>

## 예제 코드
Mediator 패턴은 크게 Mediator 인터페이스와 Colleague 인터페이스로 구성됩니다.
<pre class="prettyprint">public abstract class Mediator {

  private ArrayList&lt;Colleague&gt; mColleagueList = new ArrayList&lt;Colleague&gt;();

  public void addColleague(Colleague colleague) {
    mColleagueList.add(colleague);
  }

  public void removeColleague(Colleague colleague) {
    mColleagueList.remove(colleague);
  }

  public void init() {
    for(Colleague colleague : mColleagueList) {
      colleague.init();
    }
  }

  public void fin() {
    for(Colleague colleague : mColleagueList) {
      colleague.fin();
    }
  }
}</pre>
<br>
<pre class="prettyprint">public abstract class Colleague {

  private Mediator mMediator;

  public void setMediator(Mediator mediator) { mMediator = mediator; }

  public Mediator getMediator() { return mMediator; }

  public abstract void init();

  public abstract void fin();
}</pre>

<br>
그리고 Mediator와 Colleague를 상속(구현)받는 클래스들을 구현하면 됩니다.
