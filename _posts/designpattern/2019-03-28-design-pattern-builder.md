---
layout: post
title: 생성자에 매개변수가 많으면 Builder를 사용하자
category: 디자인패턴
permalink: /designpattern/:year/:month/:day/:title/

tag: [디자인패턴]
---

보통 하나의 메소드가 가지는 매개변수의 개수를 최대 4개 정도까지가 사용하기 무난한 메소드라고 합니다.
그 이상이 될 경우, 해당 메소드를 사용하는 사용자에게 혼란이나 실수를 발생시키기 쉽기 때문에
매개변수를 줄이는 것을 권장합니다.

매개변수를 줄이는 방법으로는 보통 다음 3가지 방법이 있습니다.

* 매개 변수를 쪼갤 수 있도록 메소드를 여러 개로 분할한다.
* 매개 변수를 포함하는 헬퍼 클래스를 하나 생성해서 매소드의 파라메터로 헬퍼 클래스를 사용한다.
* Builder를 사용한다.

여기서는 세 번째 방법인 Builder 사용법에 대한 예시를 알아봅니다.

<br>

## 매개변수가 많은 클래스 예시

<pre class="prettyprint">
public class Robot {

  private final String name;
  private final String nickName;
  private final String controlServerUrl;
  private final int width;
  private final int height;
  private final int x;
  private final int y;
  private final double orientation;

  public Robot(String name, String nickName, String controlServerUrl, int width, int height, int x,
      int y, double orientation) {
    this.name = name;
    this.nickName = nickName;
    this.controlServerUrl = controlServerUrl;
    this.width = width;
    this.height = height;
    this.x = x;
    this.y = y;
    this.orientation = orientation;
  }

  public Robot(String name, String nickName, String controlServerUrl, int width, int height, int x,
      int y) {
    this(name, nickName, controlServerUrl, width, height, x, y, 0.0f);
  }

  public Robot(String name, String nickName, String controlServerUrl, int width, int height) {
    this(name, nickName, controlServerUrl, width, height, 0, 0);
  }
}
</pre>

이 경우 해당 클래스를 생성할 때 코드를 작성하거나 읽기가 어려워지며 버그가 발생하기 쉬워집니다.

<br>

## JavaBeans 패턴 적용

고전적으로 사용하던 JavaBeans 패턴을 이용해서 각 매개변수의 setter를 생성해주는 방법도 있습니다.

<pre class="prettyprint">
package builder;

public class Robot {

  private String name;
  private String nickName;
  private String controlServerUrl;
  private int width;
  private int height;
  private int x;
  private int y;
  private double orientation;

  public void setName(String name) {
    this.name = name;
  }

  public void setNickName(String nickName) {
    this.nickName = nickName;
  }

  public void setControlServerUrl(String controlServerUrl) {
    this.controlServerUrl = controlServerUrl;
  }

  public void setWidth(int width) {
    this.width = width;
  }

  public void setHeight(int height) {
    this.height = height;
  }

  public void setX(int x) {
    this.x = x;
  }

  public void setY(int y) {
    this.y = y;
  }

  public void setOrientation(double orientation) {
    this.orientation = orientation;
  }
}
</pre>

이 경우 인스턴스를 생성하기 위해서는 다음과 같은 코드를 사용합니다.

<pre class="prettyprint">
Robot robot = new Robot();
robot.setName("SuperBot");
robot.setNickName("HiBot");
robot.setHeight(50);
robot.setWidth(30);
robot.setX(10);
robot.setY(20);
robot.setOrientation(250.0f);
</pre>

하나의 인스턴스 생성을 위해 메소드를 여러 번 호출해야 하기 때문에 객체가 완전히 생성하기 전까지 일관성(Consistency)을 보장받지 못하게 되었습니다.
또한 더 이상 `final` 키워드를 사용할 수 없기 때문에, 불변성을 갖지 못해 Thread Safety를 보장할 수가 없습니다.

<br>

## Builder 패턴 적용

Builder 패턴을 적용하면 다음과 같습니다.

<pre class="prettyprint">
public class Robot {

  private final String name;
  private final int width;
  private final int height;
  private final int x;
  private final int y;
  private final double orientation;

  public static class Builder {

    private final String name;  // 필수 매개변수

    private int width = 0;
    private int height = 0;
    private int x = 0;
    private int y = 0;
    private double orientaiton = 0.0f;

    public Builder(String name) {
      this.name = name;
    }

    public Builder width(int value) {
      width = value;
      return this;
    }

    public Builder height(int value) {
      height = value;
      return this;
    }

    public Builder x(int value) {
      x = value;
      return this;
    }

    public Builder y(int value) {
      y = value;
      return this;
    }

    public Builder orientation(double value) {
      orientaiton = value;
      return this;
    }

    public Robot build() {
      return new Robot(this);
    }
  }

  private Robot(Builder builder) {
    name = builder.name;
    width = builder.width;
    height = builder.height;
    x = builder.x;
    y = builder.y;
    orientation = builder.orientaiton;
  }
}
</pre>

<br>

사용 예제는 다음과 같습니다.

<pre class="prettyprint">
Robot robot = new Robot.Builder("SuperBot").height(30).width(20).x(100).y(100)
        .orientation(250.0f).build();
</pre>