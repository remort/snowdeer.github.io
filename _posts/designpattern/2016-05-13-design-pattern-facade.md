---
layout: post
title: 퍼샤드(Facade) 패턴
category: 디자인패턴
permalink: /designpattern/:year/:month/:day/:title/

tag: [디자인패턴]
---

퍼샤드(Facade)는 ‘정면’, ‘표면’ 이라는 뜻입니다.
그리고 카메라로 사진을 찍을 때 보는 ‘바늘 구멍’ 이라고 하기도 합니다.
모든 시야는 그 바늘 구멍을 통해서 보게 되듯이 특정 모듈의 ‘창구’ 역할을 하는 클래스를 두는 패턴을
Facade 패턴이라고 합니다.

Facade 패턴의 UML을 살펴보면 다음과 같습니다.

![Image]({{ site.baseurl }}/assets/design-patterns/facade.png)

Facade 패턴은 클래스의 은닉화, 캡슐화와 아주 깊은 관련이 있습니다.
어떤 클래스의 구조가 아주 다양하고 복잡하다고 하더라도 그걸 사용하는 개발자들은
그 내부 구조를 일일이 알 필요없이 특정 인터페이스 몇 개만 알아도 사용이 가능하도록
해주는 것이 바로 Facade 패턴입니다.

그러다보니, 앞서 포스팅했던 Mediator 패턴과 비슷한 부분이 있습니다.

한 군데로 모아서 관리한다는 특징이 비슷한데, 시스템 내부적으로 볼 때는 Mediator,
외부에서 볼 때는 Facade가 되는 경우가 많습니다. 따라서 보통 두 패턴이 동시에
사용되는 경우가 많습니다.

<br>

## 예제 코드
<pre class="prettyprint">public class FacadeExample {

  private MusicPlayer mMusicPlayer = new MusicPlayer();
  private VideoPlayer mVideoPlayer = new VideoPlayer();
  private Gallery mGallery = new Gallery();

  public void playMusic() {
    mMusicPlayer.play();
  }

  public void playVideo() {
    mVideoPlayer.play();
  }

  public void showImage() {
    Gallery.show();
  }
}</pre>

<br>

위의 예제는 간단합니다. 음악 플레이어, 비디오 플레이어, 갤러리가 있을 때
각 컴포넌트를 감싸는 Facade 클래스를 만들고, 외부에서는 Facade를 통해
각 컴포넌트의 기능을 호출하는 예제입니다. 외부에서는 Facade 내부에 어떤 클래스가 있는지
알 필요가 없기 때문에 어떤 기능들을 SDK나 라이브러리 API 형태로 제공을 할 때
Facade 패턴을 많이 사용합니다.
