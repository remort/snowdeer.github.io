---
layout: post
title: 중재자(Mediator) 패턴
category: 디자인패턴
tag: [pattern, mediator]
---

중재자(Mediator) 패턴입니다.  

Colleague들의 분쟁을 중간에서 중재해주는 클래스를 두고, 
각 Colleague들끼리는 서로를 모르게 하여 낮은 결합도(Low Coupling)를 가지게 해주는
패턴입니다. 

Colleague들끼리는 서로를 알지 못하기 때문에 특정 작업을 요청 하려면 무조건
Mediator에게 요청을 해야 하기 떄문에, Mediator의 함수가 많아지고 코드량이 
길어진다는 단점이 있긴 한데, 유지 보수 측면에서는 유리한 패턴입니다.

Mediator 패턴의 UML을 살펴보면 다음과 같습니다.

![Image]({{ site.baseurl }}/assets/design-patterns/mediator.png) 

Mediator 패턴을 쓸 수 있는 경우를 들면 다음과 같은 시나리오를 들 수 있습니다.

> 특정 GUI 화면에서 특정 체크박스의 값이 바뀌면 화면에 있는 각 버튼의 Enabled/Disabled 속성이
변경 되고, 텍스트 박스의 값에 따라 각 컴포넌트의 속성이 변경되어 각 컴포넌트들이 서로가 서로에게 영향을 미치는 경우를 예로 들 수 있습니다.

위와 같은 경우 Mediator를 두지 않고, 각 버튼이나 체크 버튼, 텍스트 박스 등의 이벤트 처리 부분에서
각 컴포넌트의 속성을 직접 바꾸게 될 경우, 나중에 디버깅을 하거나 컴포넌트가 추가/삭제될 경우 
큰 비용이 발생할 수도 있습니다. 

이런 경우 Mediator를 두고 각 상태에 따라 Mediator가 각 컴포넌트의 상태들을 변경시켜주게 되면 
코드도 깔끔해지고 유지 보수도 수월하게 됩니다.


Mediator 패턴은 크게 Mediator 인터페이스와 Colleague 인터페이스로 구성됩니다.
<pre class="prettyprint lang-java">
public abstract class Mediator {
	private ArrayList<Colleague> mColleagueList = new ArrayList<Colleague>();
	
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
}
</pre>

<pre class="prettyprint lang-java">
public abstract class Colleague {
	private Mediator mMediator;
	
	public void setMediator(Mediator mediator) {
		mMediator = mediator;
	}
	
	public Mediator getMediator() {
		return mMediator;
	}
	
	public abstract void init();
	public abstract void fin();
}
</pre>

그리고 Mediator아 Colleague를 상속(구현)받는 클래스들을 구현하면 됩니다.