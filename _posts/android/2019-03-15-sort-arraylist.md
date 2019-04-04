---
layout: post
title: ArrayList 정렬하는 방법
category: Android
tag: [Android, Java]
---

## 오름차순 정렬 예제

다음 예제는 숫자를 오름차순으로 정렬하는 예제입니다.

<pre class="prettyprint">
List&lt;Integer&gt; list = new ArrayList<>();

list.add(5);
list.add(3);
list.add(7);
list.add(6);
list.add(10);
list.add(4);

Collections.sort(list, new Comparator&lt;Integer&gt;() {
  @Override
  public int compare(Integer o1, Integer o2) {
    if (o1 < o2) {
      return -1;
    } else if (o1 > o2) {
      return 1;
    }
    return 0;
  }
});

for (int i = 0; i < list.size(); i++) {
  System.out.println(list.get(i));
}
</pre>

<br>

## 수정된 버전 1

데이터 타입이 단순한 `Integer` 형태이기 때문에 비교하는 구문을 아래와 같이 수정할 수도 있습니다.

<pre class="prettyprint">
List&lt;Integer&gt; list = new ArrayList<>();

list.add(5);
list.add(3);
list.add(7);
list.add(6);
list.add(10);
list.add(4);

Collections.sort(list, new Comparator&lt;Integer&gt;() {
  @Override
  public int compare(Integer o1, Integer o2) {
    return (o1 - o2);
  }
});

for (int i = 0; i < list.size(); i++) {
  System.out.println(list.get(i));
}
</pre>

<br>

## 수정된 버전 2

또한 Java에서 제공하는 대부분의 데이터 타입들은 `compareTo` 인터페이스를 구현해놓았기 때문에 아래와 같은 코드도 사용가능합니다.

<pre class="prettyprint">
List&lt;Integer&gt; list = new ArrayList<>();

list.add(5);
list.add(3);
list.add(7);
list.add(6);
list.add(10);
list.add(4);

Collections.sort(list, new Comparator&lt;Integer&gt;() {
  @Override
  public int compare(Integer o1, Integer o2) {
    return o1.compareTo(o2);
  }
});

for (int i = 0; i < list.size(); i++) {
  System.out.println(list.get(i));
}
</pre>

<br>

## Comparable 인터페이스 구현

가장 간편한건 아래의 예제와 같이 `Comparable` 인터페이스를 구현하는 방법입니다. 

<pre class="prettyprint">
public class Data implements Comparable {

  private final int value;

  public Data(int value) {
    this.value = value;
  }

  public final int getValue() {
    return this.value;
  }

  @Override
  public int compareTo(Object o) {
    return getValue() - ((Data) o).getValue();
  }
}
</pre>

이 경우는 아래와 같이 `Collections.sort()` 메소드에 별도의 매개변수없이 간편하게 정렬을 할 수 있습니다.

<pre class="prettyprint">
List&lt;Data&gt; list = new ArrayList<>();

list.add(new Data(3));
list.add(new Data(5));
list.add(new Data(4));
list.add(new Data(2));

Collections.sort(list);
</pre>

<br>

## 파일명으로 오름 차순 정렬 예제

<pre class="prettyprint">
  public void refresh(String path) {
    mList.clear();
    File directory = new File(path);
    File[] files = directory.listFiles();

    for (int i = 0; i < files.length; i++) {
      mList.add(files[i]);
    }

    Comparator&lt;File&gt; cmpAsc = (o1, o2) -> o1.getName().compareTo(o2.getName());
    Collections.sort(mList, cmpAsc);

    notifyDataSetChanged();
  }
</pre>

<br>

## Child Node 각도에 따라 정렬하는 예제

각도는 12시 방향을 원점으로 해서 반시계 방향으로 구했습니다.

<pre class="prettyprint">
  void sort() {
    Collections.sort(childList, (o1, o2) -> {

      int px = left() + width() / 2;
      int py = top + height() / 2;

      int cx1 = o1.left() + o1.width() / 2;
      int cy1 = o1.top() + o1.height() / 2;

      int cx2 = o2.left() + o2.width() / 2;
      int cy2 = o2.top() + o2.height() / 2;

      int dx1 = cx1 - px;
      int dy1 = cy1 - py;

      int dx2 = cx2 - px;
      int dy2 = cy2 - py;

      double degree1 = Math.atan2(dx1, dy1);
      double degree2 = Math.atan2(dx2, dy2);

      if (degree1 < degree2) {
        return -1;
      } else if (degree1 > degree2) {
        return 1;
      } else {
        return 0;
      }
    });
  }
</pre>