---
layout: post
title: Single Thread Execution 패턴
category: 디자인패턴
permalink: /designpattern/:year/:month/:day/:title/

tag: [디자인패턴]
---

# Single Thread Execution 패턴

Single Thread Execution 패턴은 공통의 오브젝트에 대해 다수의 Thread가 작업을 할 때, 동시에 작업을 하는 것이 아니라 한 번에 하나의 Thread가 대상 오브젝트를 사용할 수 있도록 하는 패턴입니다.

하나의 Thread만 처리가능한 영역을 Critical Section 또는 Critical Region이라고 부릅니다.

Java에서는 `synchronized` 키워드를 이용해서 쉽게 Critical Section을 만들 수 있습니다.

<br>

## 예제 

<pre class="prettyprint">
public class CommonObject {

  private int id = 0;
  private String name = "name";
  private String address = "address";

  public synchronized void setAttribute(int id, String name, String address) {
    this.id = id;
    this.name = name;
    this.address = address;
  }

  public synchronized String toString() {
    return "(id: " + id + ", name: " + name + ", address: " + address + ")";
  }
}
</pre>

위와 같은 `CommonObject` 클래스의 인스턴스는 동시에 여러 개의 Thread가 접근하더라도 각 함수에 대해 오직 하나의 Thread만 접근을 허용합니다.

<br>

## synchronized와 Lock

만약 특정 Thread가 `synchronized` 메소드를 실행하게 되면 해당 Thread는 Lock을 획득(Aquire)하게 됩니다. 그리고 해당 메소드를 빠져나가게 되면 Lock을 해제(Release)합니다.

Lock는 각 인스턴스마다 존재합니다. 만약 여러 개의 인스턴스를 생성했을 경우 Lock은 각각의 인스턴스마다 존재하게 됩니다.

<br>

## 주의점

Single Thread Execution 패턴은 다음과 같은 주의점이 있습니다.

* 복수의 Thread간 데드락(Dead Lock)이 발생할 가능성이 높습니다.
* 해당 클래스를 상속할 경우 서브 클래스에 의해 안정성이 무너질 가능성이 존재합니다.
* Lock은 Cost가 많이 드는 작업입니다. 여러 Thread에 의해 충돌이 날 문제가 없는 경우는 Lock을 사용하지 않는 것이 더 좋습니다.
* Thread간 충돌이 발생해서 성능 저하가 됩니다. 하나의 Thread가 Lock을 점유해서 다른 Thread들이 작업을 대기하고 있는 것을 충돌(Conflict)라고 합니다. Lock을 너무 많이 사용하게 될 경우 Multi-Thread 환경의 장점이 사라지고 단일 Thread를 사용한 것보다도 더 안 좋은 성능을 낼 수도 있습니다.