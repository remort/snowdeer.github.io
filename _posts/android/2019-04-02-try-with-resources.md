---
layout: post
title: try-finally 보다는 try-with-resources를 사용하자.
category: Android
tag: [Android, Java]
---

예외 처리를 위해 사용하는 `try - finally` 구문은 강력하긴 하지만, `try`가 2개 이상 사용되거나 그 외의 이유 등으로
코드가 복잡하게 될 수 있는 단점이 있습니다. 
Java 7.0 부터 지원하는 `try with resources`를 사용하면 코드를 훨씬 깔끔하게 구현할 수 있습니다.

## try-finally 예시

<pre class="prettyprint">
public class FileUtil {

  public static String getFirstLineOfFile(String filepath) throws IOException {
    BufferedReader br = new BufferedReader(new FileReader(filepath));
    try {
      return br.readLine();
    } finally {
      br.close();
    }
  }
}
</pre>

<br>

## 2개 이상의 try가 사용될 경우

<pre class="prettyprint">
public class FileUtil {

  private static final int BUFFER_SIZE = 1024;

  public static void copy(String src, String dest) throws IOException {
    InputStream in = new FileInputStream(src);
    try {
      OutputStream out = new FileOutputStream(dest);
      try {
        byte[] buf = new byte[BUFFER_SIZE];
        int n;
        while ((n = in.read(buf)) >= 0) {
          out.write(buf, 0, n);
        }
      } finally {
        out.close();
      }
    } finally {
      in.close();
    }
  }
}
</pre>

<br>

## try-with-resources 예제

<pre class="prettyprint">
public class FileUtil {

  private static final int BUFFER_SIZE = 1024;

  public static String getFirstLineOfFile(String filepath) throws IOException {
    try (BufferedReader br = new BufferedReader(new FileReader(filepath))) {
      return br.readLine();
    }
  }

  public static void copy(String src, String dest) throws IOException {
    try (InputStream in = new FileInputStream(src);
        OutputStream out = new FileOutputStream(dest);) {
      byte[] buf = new byte[BUFFER_SIZE];
      int n;
      while ((n = in.read(buf)) >= 0) {
        out.write(buf, 0, n);
      }
    }
  }
}
</pre>

<br>

## catch 사용 예제

<pre class="prettyprint">
public static String getFirstLineOfFile(String filepath) {
  try (BufferedReader br = new BufferedReader(new FileReader(filepath))) {
    return br.readLine();
  }
  catch (IOException e) {
    e.printStackTrace();
    return "";
  }
}
</pre>