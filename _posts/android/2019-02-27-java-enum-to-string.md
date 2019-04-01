---
layout: post
title: enum을 문자열로 변경하는 방법
category: Android
tag: [Android]
---

Java에서 `enum`으로 정의한 값을 문자열로 변경하는 방법은 여러가지가 있습니다. 

<br>

## toString 메소드를 오버라이딩 

<pre class="prettyprint">
public enum LogLevel {

  VERB {
    @Override
    public String toString() {
      return "Verbose";
    }
  },
  INFO {
    @Override
    public String toString() {
      return "Info";
    }
  },
  DEBUG {
    @Override
    public String toString() {
      return "Debug";
    }
  },
  WARN {
    @Override
    public String toString() {
      return "Warning";
    }
  },
  ERROR {
    @Override
    public String toString() {
      return "Error";
    }
  }
}

public static void main(String[] args) {
  System.out.println(LogLevel.INFO);
}
</pre>

<br>

## name 메소드 사용하는 방법

<pre class="prettyprint">
public enum LogLevel {
  VERB,
  INFO,
  DEBUG,
  WARN,
  ERROR,
}

public static void main(String[] args) {
  System.out.println(LogLevel.INFO.name());
}
</pre>

<br>

## 반대로 문자열을 이용해서 Enum을 생성하는 방법

<pre class="prettyprint">
public enum LogLevel {
  VERB,
  INFO,
  DEBUG,
  WARN,
  ERROR,
}

public static void main(String[] args) {
  LogLevel ll = Enum.valueOf(LogLevel.class, "ERROR")
  System.out.println(ll.name());
}
</pre>