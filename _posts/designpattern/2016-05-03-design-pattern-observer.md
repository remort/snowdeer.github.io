---
layout: post
title: 옵저버(Observer) 패턴
category: 디자인패턴
permalink: /designpattern/:year/:month/:day/:title/

tag: [디자인패턴]
---

가장 많이 사용 되는 패턴 중 하나가 옵저버 패턴(Observer Pattern)입니다.
아마 디자인 패턴을 잘 모르더라도 자신도 모르게 이미 옵저버 패턴을 사용하고 있는 경우가
대부분일 것 같습니다. 예를 들어, 안드로이드 개발을 할 때 Button에 OnClickEventListener를
등록하는 것들이 옵저버(Observer) 패턴에 해당됩니다. 평소엔 가만히 있다가 해당 버튼이
클릭되었을 때 그 이벤트를 알려달라고 리스너를 등록하는 것입니다.

옵저버 패턴은 특정 인스턴스에 이벤트 리스너(EventListener)를 달고 대기하고 있다가
그 인스턴스에 이벤트가 발생하면 그 결과를 통보(Notify)받는 방식이며 UML로 표현하면 다음과 같습니다.

![Image](/assets/design-patterns/observer.png)

<br>

# 예제 코드

예제 코드는 다음과 같이 간단히 작성할 수 있습니다.
<pre class="prettyprint">public interface Observer {

  public void update(Subject subject, int event);
}
</pre>
<pre class="prettyprint">public abstract class Subject {

  private ArrayList&lt;Observer&gt; mObserverList = new ArrayList&lt;Observer&gt;();

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
}</pre>
