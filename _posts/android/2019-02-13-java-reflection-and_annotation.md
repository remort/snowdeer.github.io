---
layout: post
title: Reflection 과 Annotation 같이 사용하기
category: Android
tag: [Android]
---

Reflection을 이용하면 Runtime 환경에서 Java의 Class와 메소드, 파라메터 등의 정보를 얻을 수 있습니다.
파라메터의 경우 데이터 타입과 이름을 얻을 수 있지만, 파라메터의 이름의 경우에는 실제로 선언한 변수명이 아니라
arg1, arg2 등과 같은 변형된 변수명으로 얻게 되어 있습니다. 이는 컴파일되는 도중 컴파일러가 적당한 이름으로 변수명을 변경해버리기 때문인데, Java 버전을 바꾸거나 컴파일 옵션을 수정해서 개발자가 지정한 변수명을 얻을 수는 있습니다. 

하지만 기본적으로는 arg1 과 같이 사람이 알아보기 힘든 이름이 되어 버리는데 Annotation을 적절하게 이용하면 이 문제를 조금은 더 수월(?)하게 해결할 수 있습니다.

<br>

## ParamName1.java

<pre class="prettyprint">
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

@Retention(RetentionPolicy.RUNTIME)
public @interface ParamName1 {

  String name();
}

</pre>

<br>

## ParamName2.java

<pre class="prettyprint">
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

@Retention(RetentionPolicy.RUNTIME)
public @interface ParamName2 {

  String name();
}

</pre>

<br.
>
## Annotation 적용 예제

<pre class="prettyprint">
public class SpeakText extends Action {
    String text;

    @ParamName1(name="text")
    public SpeakText(String text) {
        super("SampleClass(" + text + ")");
        this.text = text;
    }

    @ParamName1(name="name")
    @ParamName2(name="text")
    public SpeakText(String name, String text) {
        super(name);
        this.text = text;
    }
}
</pre>

<br>

## Annotation 정보를 획득하는 예제

<pre class="prettyprint">
    Class c = actionClassMap.get(key);

      Constructor[] cons = c.getDeclaredConstructors();

      for (int i = 0; i < cons.length; i++) {
        Log(cons[i].getName() + ", " + cons[i].toString());

        Annotation[] anns = cons[i].getAnnotations();

        try {
          ParamName1 p = (ParamName1) cons[i].getAnnotation(ParamName1.class);

          if (p != null) {
            Log("   -- Anno : " + p.name());
          }
        } catch (Exception e) {
          e.printStackTrace();
        }

        try {
          ParamName2 p = (ParamName2) cons[i].getAnnotation(ParamName2.class);

          if (p != null) {
            Log("   -- Anno : " + p.name());
          }
        } catch (Exception e) {
          e.printStackTrace();
        }
        
</pre>
