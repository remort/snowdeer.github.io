---
layout: post
title: Map, Set 등 배열 내부 값 순회하기
category: Android
tag: [Android]
---

프로그래밍을 하다보면, 리스트를 쓰는 경우는 피할 수가 없을 것입니다.
가장 많이 쓰는 것들이 일반적인 배열이나 ArrayList 들입니다. 그리고 성능이나 여러가지
장점을 위해 HashMap 등을 사용하게 되는 경우도 많은데 일반적인 배열들과 사용법이
조금 달라 혼란스러운 경우가 종종 있습니다.

물론, 구글링을 하면 바로 나오긴 하지만 나중에 좀 더 찾기 쉽게 하기 위해 여기다가
예제 코드를 작성해봅니다.

<br>
## Iterator 사용
가장 기본적인 건 Iterator를 사용하는 것입니다. Iterator는 디자인 패턴 중 하나이기도 하고,
거의 모든 리스트에 적용가능할 정도로 범용적입니다. 하지만, 보통 개발자들은 Iterator 없이
단순 인덱스만 이용해서 for 문이나 while 반복문으로 접근하는 경향이 있습니다.
둘 다 문제없는 사용법이긴 하지만, 여기서는 먼저 Iterator를 이용한 접근 방법부터 살펴보도록 하겠습니다.
<pre class="prettyprint">Map&lt;String, String&gt; map = new HashMap&lt;&gt;();

map.put("Key1", "Value1");
map.put("Key2", "Value2");
map.put("Key3", "Value3");

Iterator&lt;String&gt; iter = map.keySet().iterator();

System.out.println("#1. Using Iterator");
while(iter.hasNext()) {
    String key = iter.next();
    System.out.println(" -- " + key + ", " + map.get(key));
}</pre>
<br>
<pre class="prettyprint">Set&lt;String&gt; set = new HashSet&lt;&gt;();

set.add("Value4");
set.add("Value5");
set.add("Value6");

Iterator&lt;String&gt; iter = set.iterator();

System.out.println("#1. Using Iterator");
while(iter.hasNext()) {
    String value = iter.next();
    System.out.println(" -- " + value);
}</pre>
<br>
## for 문 사용
for 문을 사용하더라도 간단하게 조회할 수 있습니다. 다음 예제는 각각 Map과 Set에
대한 for 문으로의 접근 방법입니다.
<pre class="prettyprint">System.out.println("#2. Using For");
for(String key : map.keySet()) {
    System.out.println(" -- " + key + ", " + map.get(key));
}</pre>
<br>
<pre class="prettyprint">System.out.println("#2. Using For");
for(String value : set) {
    System.out.println(" -- " + value);
}</pre>
