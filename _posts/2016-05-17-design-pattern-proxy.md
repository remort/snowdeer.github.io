---
layout: post
title: 프록시(Proxy) 패턴
category: 디자인패턴
tag: [pattern, proxy]
---

프록시(Proxy) 패턴의 UML은 다음과 같습니다.

![Image]({{ site.baseurl }}/assets/design-patterns/proxy.gif) 

Proxy는 '대리인'이라고 생각하면 됩니다. '대리인'이란 어떤 작업을 대신 해 줄 수 있는 객체입니다.

보통 Proxy라고 하면 흔히 생각할 수 있는 것으로는 HTTP Proxy 같은 것을
떠올릴 수 있습니다. HTTP Proxy는 중간에 있는 Proxy 서버에서 
특정 웹사이트들을 캐싱(Caching)하고 있다가 사용자의 요청이 있으면
캐시에 저장하고 있던 페이지를 보내주고, 필요에 따라 실제 웹페이지에
접속하여 더 많은 정보를 사용자에게 보여주는 일을 합니다.

그 외에도 안드로이드 개발을 해보았으면 한 번쯤은 접해보았을 법한 
'[AIDL](https://developer.android.com/guide/components/aidl.html)'을
떠올려볼 수도 있습니다. AIDL은 'Android Interface Definition Language'입니다. 

실제로 Proxy에서 양쪽간의 인터페이스를 정의하는 언어를 'IDL'이라고 합니다.
안드로이드에서는 여기에 'A'를 붙여서 'AIDL'이라고 하죠.

예제를 들어보도록 하겠습니다.

다음 예제는 프린터라는 클래스에 대한 Proxy를 두는 예제인데, 
이름을 바꾸는 등의 간단한 작업은 실제 프린터가 하지 않고, 프린터의 Proxy가
수행합니다. 하지만, '출력'과 같이 Proxy가 할 수 없는 일은 실제 프린터가
수행하는 예제입니다.

다시 요약하면 

> 실제 클래스가 해야 할 일은 어쩔 수 없지만, 대리인이 대신 처리할 수 있는 
일은 Proxy가 하는 패턴

입니다.

<pre class="prettyprint lang-java">
public interface Printable {
	public abstract void setPrinterName(String strName);
	public abstract String getPrinterName();
	public abstract void print(String strString);
}
</pre>


<pre class="prettyprint lang-java">
public class Printer implements Printable {
	private String mName;

	public Printer() {
		heavyJob("Making an instance of Printer class");
	}

	public Printer(String name) {
		mName = name;
		heavyJob("Making an instance of Printer class + " + name);
	}

	@Override
	public void setPrinterName(String name) {
		mName = name;
	}

	@Override
	public String getPrinterName() {
		return mName;
	}

	@Override
	public void print(String text) {
		System.out.println("[Printer] === " + mName + " ===");
		System.out.println("[Printer] " + text);
	}

	private void heavyJob(String message) {
		System.out.println("[Printer] " + message);

		for (int i = 0; i < 5; i++) {
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			System.out.println("[Printer] .");
		}

		System.out.println("[Printer] Finish..");
	}
}
</pre>


<pre class="prettyprint lang-java">
public class PrinterProxy implements Printable {
	private String mName;
	private Printer mRealPrinter;

	public PrinterProxy() {
	}

	public PrinterProxy(String name) {
		mName = name;
	}

	@Override
	public void setPrinterName(String name) {
		if (mRealPrinter != null) {
			mRealPrinter.setPrinterName(name);
		}
		mName = name;
	}

	@Override
	public String getPrinterName() {
		return mName;
	}

	@Override
	public void print(String text) {
		realize();
		mRealPrinter.print(text);
	}

	private synchronized void realize() {
		if (mRealPrinter == null) {
			mRealPrinter = new Printer(mName);
		}
	}
}
</pre>