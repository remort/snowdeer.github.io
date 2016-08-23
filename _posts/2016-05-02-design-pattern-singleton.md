---
layout: post
title: 싱글톤(Singleton) 패턴
category: 디자인패턴
tag: [pattern, singleton]
---

프로그램에서 단 하나의 인스턴스(Instance)만 존재하게 하고 싶을 때 사용하는 패턴입니다.
어디서든지 그 인스턴스에 접근할 수 있기 때문에 전역 변수 등을 관리할 때 상당히 편리합니다.

하지만, 싱글톤(Singleton)은 실제로는 전역 변수와 거의 같은 용도로 많이 쓰이며 
객체 지향과는 거리가 있는 패턴입니다. 왠만하면 사용하지 않는 것을 추천드리지만
적절하게 사용하면 편리하긴 합니다. 

싱글톤 패턴의 UML은 다음과 같습니다. 달랑 클래스 하나 뿐입니다.

![Image]({{ site.baseurl }}/assets/201design-patterns/singleton.gif) 

싱글톤 패턴을 코드로 구현하면 다음과 같습니다. (다만, 아래의 코드는 멀티 쓰레드 환경에서 문제가 발생합니다.)

<pre class="prettyprint">
public class Singleton {
	private static Singleton mInstance = null;

	private Singleton() {
	}

	public static Singleton getInstance() {
		if (mInstance == null) {
			mInstance = new Singleton();
		}

		return mInstance;
	}
}
</pre>

위와 같은 경우는 멀티 쓰레드(Multi-Thread) 환경에서는 getInstace() 메소드가 끝나기 전에
각각의 쓰레드에서 접근이 가능하기 때문에 운이 없으면 인스턴스가 여러 개 생성될 수도 있습니다.

이 경우 해결 방법은 getInstance() 메소드를 synchronized로 동기화시키는 방법이 있습니다.

<pre class="prettyprint">
	public synchronized static Singleton getInstance() {
		if (mInstance == null) {
			mInstance = new Singleton();
		}

		return mInstance;
	}
</pre>

하지만, 함수 전체에 synchronized는 동기화 과정에서 속도 문제가 발생할 수 있습니다.
(사실 이 경우에서는 그렇게 치명적인 속도 문제가 발생할 것 같지는 않습니다.)

그래서 함수 내부에 최소 구간에만 synchronized를 거는 방법도 있습니다.

<pre class="prettyprint">
	public static Singleton getInstance() {
		if (mInstance == null) {
			synchronized (Singleton.class) {
				if (mInstance == null) {
					mInstance = new Singleton();
				}
			}
		}

		return mInstance;
	}
</pre>


그리고 코드를 좀 더 깔끔하고 쉽게 가져가기 위해서는 아예 처음부터 인스턴스를 생성해버리는 방법입니다.
저는 이 방법을 가장 많이 사용합니다. 코드도 간단하고 버그가 발생할 가능성도 낮거든요.

<pre class="prettyprint">
public class Singleton {
	private static Singleton mInstance = new Singleton();

	private Singleton() {
	}

	public static Singleton getInstance() {
		return mInstance;
	}
}
</pre>

물론 이 경우는 프로그램이 처음 실행되면서 바로 인스턴스가 생겨버리기 때문에 불필요한 부분에서 인스턴스가
메모리를 차지해버린다는 단점이 있습니다. 하지만, 멀티 쓰레드 동기화 문제에서 자유로울 수 있고 코드가 간결해진다는
장점이 있어서 저는 이 방법을 제일 애용합니다. 

하지만 최선은 싱글톤 패턴을 사용하지 않는 방법이 아닐까 생각되네요. 