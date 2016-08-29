---
layout: post
title: 컴포지트(Composite) 패턴
category: 디자인패턴
tag: [pattern, composite]
---

컴포지트(Composite) 패턴의 UML은 다음과 같습니다.

![Image]({{ site.baseurl }}/assets/design-patterns/composite.gif) 

컴포지트 패턴의 개념은 각 객체들을 동일화시키겠다는 것입니다.

조금 다르게 표현하자면

~~~
추상적인 상위 클래스 하나를 만들고, 그 클래스를 상속받는 다양한 자식 클래스들을 만듭니다.
그리고 그 자식 클래스들을 마치 같은 종류의 클래스 다루듯이 동일시해서 사용하겠다는 패턴입니다.
~~~

이 컴포지트 패턴은 커맨드(Command) 패턴이나 방문자(Visitor) 패턴, 데코레이터(Decorator) 패턴 등에 
응용되서 사용되어질 수 있습니다. 상당히 널리 쓰이는 패턴입니다.

그리고 여기서 더 나아가 트리 구조와 같은 재귀적인(Recursive) 구조를 만들기 위해서도 유용하게 쓰입니다.

트리 구조의 가장 대표적인 예제로는 '파일 구조'가 있습니다.
폴더와 파일로 이루어져 있으며, 폴더 아래에는 폴더들이 있을 수 있고, 또한 파일들이 있을 수 있습니다.


컴포지트 패턴은 다음과 같이 Component, Leaf, Composite 로 구성되어집니다.

<pre class="prettyprint lang-java">
public abstract class Component {
	public abstract Component add(Component cp);
	public abstract void remove(Component cp);
	// ... 
} 

public class Leaf extends Component {
	// ...
} 

public class Composite extends Component {
	private ArrayList arrChildren = new ArrayList();
	public Component add(Component cp);
	public abstract void remove(Component cp); 
	// ...
}  
</pre>

위에서 언급했던 '파일 구조' 예제를 들어보도록 하겠습니다.

<pre class="prettyprint lang-java">
public abstract class Component {
	public abstract String getName();
	public abstract int getSize();

	public Component add(Component component) throws Exception {
		throw new IOException();
	}

	public void printList() {
		printList("");
	}

	protected abstract void printList(String strPrefix);

	public String toString() {
		return getName() + "(" + getSize() + ")";
	}
}
</pre>

<pre class="prettyprint lang-java">
public class File extends Component {
	private String mName;
	private int mSize;

	public File(String name, int size) {
		mName = name;
		mSize = size;
	}

	@Override
	public String getName() {
		return mName;
	}

	@Override
	public int getSize() {
		return mSize;
	}

	@Override
	protected void printList(String strPrefix) {
		System.out.println(strPrefix + "/" + getName());
	}
}
</pre>

<pre class="prettyprint lang-java">
public class Folder extends Component {
	private String mName;
	private ArrayList&lt;Component&gt; mChildList = new ArrayList&lt;Component&gt;();

	public Folder(String strName) {
		mName = strName;
	}

	@Override
	public String getName() {
		return mName;
	}

	@Override
	public int getSize() {
		int _size = 0;
		Iterator&lt;Component&gt; it = mChildList.iterator();
		while (it.hasNext() == true) {
			Component cp = (Component) it.next();
			_size = _size + cp.getSize();
		}

		return _size;
	}

	@Override
	public Component add(Component component) {
		mChildList.add(component);

		return this;
	}

	@Override
	protected void printList(String strPrefix) {
		System.out.println(strPrefix + "/" + getName());

		Iterator&lt;Component&gt; it = mChildList.iterator();
		while (it.hasNext() == true) {
			Component cp = (Component) it.next();
			cp.printList(strPrefix + "/" + getName());
		}
	}
}
</pre>