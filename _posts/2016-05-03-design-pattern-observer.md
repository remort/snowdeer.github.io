---
layout: post
title: 옵저버(Observer) 패턴
category: 디자인패턴
tag: [pattern, observer]
---

가장 많이 사용되는 패턴 중 하나가 아닐까 싶습니다.
안드로이드 개발 등을 할 때 Button에 OnClickEventListener를 등록하는 것들이
옵저버(Observer) 패턴에 해당됩니다.

특정 인스턴스에 이벤트 리스너(EventListener)를 달고 대기하고 있다가 
그 인스턴스에 이벤트가 발생하면 그 결과를 통보(Notify)받는 방식입니다.

옵저버 패턴의 UML은 다음과 같습니다.

![Image]({{ site.baseurl }}/assets/2016-05-03-design-pattern-observer/observer.png) 

예제 코드도 다음과 같이 작성할 수 있습니다.

<pre class="prettyprint">
public interface Observer {
	public void update(Subject subject, int event);
}
</pre>

<pre class="prettyprint">
public abstract class Subject {
	private ArrayList<Observer> mObserverList = new ArrayList<Observer>();
	
	public void add(Observer observer) {
		mObserverList.add(observer);
	}
	
	public void Remove(Observer observer) {
		mObserverList.remove(observer);
	}
	
	public void notify(int event) {
		for(Observer observer : mObserverList) {
			observer.update(this, event);
		}
	}
}
</pre>