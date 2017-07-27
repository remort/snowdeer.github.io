---
layout: post
title: 시간 관련 함수 코드
category: Android
tag: [Android, Time]
---

다음 함수들은 시간과 관련된 기능을 제공하는 함수들입니다. 그때그때 필요한 기능들을
일일이 찾아서 사용해도 되지만, 제 경우는 별도의 클래스로 만들어서 입맛에 맞게 사용하는 것이
더 편리하더군요. 그래서 저는 시간 관련 함수들을 TimeUtils Class로 만들어서 사용하고 있습니다.

<br>

# 현재 시간을 Long 값으로 리턴하는 코드

<pre class="prettyprint">public static long getTimeAsLong() {
  Calendar calendar = Calendar.getInstance();
  return calendar.getTimeInMillis();
}</pre>
<br>

# 현재 시간을 String으로 리턴하는 코드

<pre class="prettyprint">public static String getTimeAsString(String format) {
  Date date = new Date(getTimeAsLong());
  SimpleDateFormat sdf = new SimpleDateFormat(format,
      Locale.getDefault());

  return sdf.format(date);
}</pre>
<br>

# 입력받은 시간(String)을 Long으로 리턴하는 코드

<pre class="prettyprint">public static Long getTimeAsLong(String format, String text) {
  try {
    SimpleDateFormat sdf = new SimpleDateFormat(format,
        Locale.getDefault());
    Date date = sdf.parse(text);
    return date.getTime();
  } catch(Exception e) {
    e.printStackTrace();
  }
  return new Long(-1);
}</pre>
<br>

## 입력받은 시간(Long)을 String으로 리턴하는 코드

<pre class="prettyprint">public static String getTimeAsString(String format, long time) {
  Date date = new Date(time);
  SimpleDateFormat sdf = new SimpleDateFormat(format,
      Locale.getDefault());

  return sdf.format(date);
}</pre>
<br>

# 오늘 시작 시간을 Long으로 리턴하는 코드

<pre class="prettyprint">public static Long getTodayFrom() {
  String date = SnowTimeUtil.getTimeAsString("yyyy-MM-dd");
  long time = SnowTimeUtil.getTimeAsLong("yyyy-MM-dd", date);

  Calendar calendar = Calendar.getInstance();
  calendar.setTimeInMillis(time);
  calendar.set(Calendar.HOUR, 0);
  calendar.set(Calendar.MINUTE, 0);
  calendar.set(Calendar.SECOND, 0);

  return calendar.getTimeInMillis();
}</pre>
<br>

# 오늘 종료 시간을 Long으로 리턴하는 코드

<pre class="prettyprint">public static Long getTodayTo() {
  String date = SnowTimeUtil.getTimeAsString("yyyy-MM-dd");
  long time = SnowTimeUtil.getTimeAsLong("yyyy-MM-dd", date);

  Calendar calendar = Calendar.getInstance();
  calendar.setTimeInMillis(time);
  calendar.set(Calendar.HOUR, 0);
  calendar.set(Calendar.MINUTE, 0);
  calendar.set(Calendar.SECOND, 0);
  calendar.add(Calendar.DAY_OF_MONTH, 1);

  return calendar.getTimeInMillis() - 1;
}</pre>
