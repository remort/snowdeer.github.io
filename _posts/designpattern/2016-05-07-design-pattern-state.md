---
layout: post
title: 스테이트(State) 패턴
category: 디자인패턴
permalink: /designpattern/:year/:month/:day/:title/

tag: [디자인패턴]
---

스테이트(Command) 패턴은 오브젝트의 상태(State)를 클래스화한 패턴입니다.

스테이트 패턴을 사용하지 않았을 경우 만약 각 상태마나 다른 동작을 하게 하기 위해서 if 문을 사용했다면,
<pre class="prettyprint">public void work() {
  if(현재 상태 == 낮){
    낮에 할 일();
  }
  else if(현재 상태 == 밤){
    밤에 할 일();
  }
}</pre>
와 같은 형태의 코드가 됩니다. 각 함수마다 현재 상태에 따라 분기문을 타게 되면,
향후 함수의 개수가 증가할 때마다(또는 상태가 증가할 때마다) 유지 보수가 힘들어지는 경우가
발생할 수 있습니다.

이런 경우 State 패턴을 사용하면 코드를 깔끔하게 관리할 수 있습니다.
State 패턴의 UML은 다음과 같습니다.

![Image](/assets/design-patterns/state.gif)

<br>

# 예제 코드

예제를 살펴 보도록 하겠습니다. 여기서는 아까의 예제와 같이 낮/밤이라는 '상태'를
각각 클래스화하도록 할 것입니다. 그리고 낮에는 이미지 갤러리,
밤에는 뮤직 프로그램이 실행되는 프로그램을 생각해보도록 하겠습니다.

일단, 다음과 같은 인터페이스를 만들어보겠습니다.
<pre class="prettyprint">public interface State {

  public void setTime(ContextManager cm, int hour);

  public void setItem(Item item);

  public void show();

  public void close();
}</pre>
그리고, 각 상태는 다음과 같이 구현할 수 있습니다.
(여기서는 각 State를 Singleton으로 구현을 했는데, 이 부분은 자유롭게 구현해도 됩니다.
	다만 잦은 State 변경에 의해서 State를 재생성해야할 경우가 많다면, State를
매번 생성해야 할 필요가 있는지는 고민해보는 것이 좋을 것 같습니다.)
<pre class="prettyprint">public class DayState implements State {

  private static DayState mInstance = new DayState();
  private ImageGallery mImageGallery = new ImageGallery();

  private DayState() {
  }

  public static State getInstance() {
    return mInstance;
  }

  @Override
  public void setTime(ContextManager cm, int hour) {
    if((hour &lt; 9) || (hour &gt;= 17)) {
      cm.setState(NightState.getInstance());
    }
    cm.setTime(hour);
  }

  @Override
  public void setItem(Item item) {
    mImageGallery.setItem(item);
  }

  @Override
  public void show() {
    mImageGallery.show();
  }

  @Override
  public void close() {
    mImageGallery.close();
  }

}</pre>
<br>
<pre class="prettyprint">public class NightState implements State {

  private static NightState mInstance = new NightState();
  private MusicLibrary mMusicLibrary = new MusicLibrary();

  private NightState() {
  }

  public static State getInstance() {
    return mInstance;
  }

  @Override
  public void setTime(ContextManager cm, int hour) {
    if((hour &gt;= 9) &amp;&amp; (hour &lt; 17)) {
      cm.setState(DayState.getInstance());
    }
    cm.setTime(hour);
  }

  @Override
  public void setItem(Item item) {
    mMusicLibrary.setItem(item);
  }

  @Override
  public void show() {
    mMusicLibrary.show();
  }

  @Override
  public void close() {
    mMusicLibrary.close();
  }
}</pre>
