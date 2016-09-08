---
layout: post
title: 커맨드(Command) 패턴
category: 디자인패턴
tag: [design pattern, command]
---

커맨드(Command) 패턴입니다.  
나중에 포스팅하겠지만, 먼저 간단히 다른 패턴들과 비교하자면

* 스테이트(State) 패턴은: 상태 그 자체를 클래스화해서 사용
* 스트래터지(Strategy) 패턴 : 알고리즘 자체를 클래스화해서 사용
* 컴포지트(Composite) 패턴 : 각 객체들을 동일시화해서 사용 

여기에 커맨드 패턴은 명령 그 자체를 클래스화해서 사용하는 패턴입니다.

커맨드 패턴의 UML은 다음과 같습니다.

![Image]({{ site.baseurl }}/assets/design-patterns/command.png) 

커맨드 패턴의 예제는 다음과 같습니다.

<pre class="prettyprint">
public interface Command {
	public void execute();
}
</pre>

이와 같은 인터페이스를 하나 구현하고,  
이 인터페이스를 구현(상속)한 클래스들을 정의하면 됩니다.

예를 들면 다음과 같습니다.

<pre class="prettyprint">
public class DrawCommand implements Command {
	protected Drawable mDrawable;
	private Point mPosition;

	public DrawCommand(Drawable drawable, Point position) {
		mDrawable = drawable;
		mPosition = position;
	}

	@Override
	public void execute() {
		mDrawable.draw(mPosition.x, mPosition.y);
	}
}
</pre>

다음과 같이 Macro 형태의 Command도 만들 수 있습니다.
Command들의 리스트를 이용하면 되는데, Stack, Queue, List 등 자유롭게 써도 됩니다.  
(여기서는 Undo, Redo 기능처럼 보이게 하기 위해 Stack을 사용했습니다.)

<pre class="prettyprint lang-java">
public class MacroHandler implements Command {
	private Stack&lt;Command&gt; mCommandStack = new Stack&lt;Command&gt;();

	@Override
	public void execute() {
		Iterator&lt;Command&gt; it = mCommandStack.iterator();
		while (it.hasNext()) {
			((Command) it.next()).execute();
		}
	}

	public boolean append(Command cmd) {
		// 만약 자기 자신이 추가(append)되었을 경우 execute()에서
		// 무한 루프로 빠질 가능성이 있기 때문에 미리 확인함
		if (cmd != this) {
			mCommandStack.push(cmd);

			return true;
		}

		return false;
	}

	public void undo() {
		if (mCommandStack.empty() == false) {
			mCommandStack.pop();
		}
	}

	public void clear() {
		mCommandStack.clear();
	}
}
</pre>