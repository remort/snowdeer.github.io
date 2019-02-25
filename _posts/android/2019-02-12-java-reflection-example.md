---
layout: post
title: Java Reflection 예제
category: Android
tag: [Android]
---

Java 언어에서 제공하는 Reflection 기능을 이용해서 각 클래스의 생성자 정보 등을 출력하는 예제입니다.

<br>

## Main.java

<pre class="prettyprint">
package snowdeer.reflection;

public class Main {

  public static void main(String[] args) {
    ReflectionTest test = new ReflectionTest();
    test.start();
  }
}
</pre>

<br>

## ReflectionTest.java

<pre class="prettyprint">
package snowdeer.reflection;

import java.lang.reflect.Constructor;
import java.lang.reflect.Parameter;

public class ReflectionTest {

  public void start() {
    Class c = SampleClass.class;

    Log("1) Class Info");
    Log("   - " + c.toString());

    Log("2) Class Constructors & Parameters");
    Constructor[] cons = c.getDeclaredConstructors();
    for (int i = 0; i < cons.length; i++) {
      Log("   - " + cons[i].toString());

      Parameter[] params = cons[i].getParameters();
      for (int j = 0; j < params.length; j++) {
        Log("      > " + params[j].getType());
      }
    }
  }

  void Log(String text) {
    System.out.println(text);
  }
}
</pre>

<br>

## SampleClass.java

<pre class="prettyprint">
package snowdeer.reflection;

public class SampleClass {

  int id;
  String name;
  long startPos;
  float speed;

  public SampleClass(int id, String name, long startPos, float speed) {
    init(id, name, startPos, speed);
  }

  public SampleClass(int id, String name, long startPos) {
    init(id, name, startPos, 0);
  }

  public SampleClass(int id, String name) {
    init(id, name, 100, 0);
  }

  public SampleClass(int id) {
    init(id, "SampleClass", 100, 0);
  }

  public SampleClass() {
    init(1, "SampleClass", 100, 0);
  }

  void init(int id, String name, long startPos, float speed) {
    this.id = id;
    this.name = name;
    this.startPos = startPos;
    this.speed = speed;
  }
}
</pre>

<br>

## 결과 화면

<pre class="prettyprint">
1) Class Info
   - class snowdeer.reflection.SampleClass
2) Class Constructors & Parameters
   - public snowdeer.reflection.SampleClass()
   - public snowdeer.reflection.SampleClass(int)
      > int
   - public snowdeer.reflection.SampleClass(int,java.lang.String)
      > int
      > class java.lang.String
   - public snowdeer.reflection.SampleClass(int,java.lang.String,long)
      > int
      > class java.lang.String
      > long
   - public snowdeer.reflection.SampleClass(int,java.lang.String,long,float)
      > int
      > class java.lang.String
      > long
      > float
</pre>
