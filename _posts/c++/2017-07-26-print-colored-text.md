---
layout: post
title: 콘솔창에 Colored Text 출력하기
category: C++
tag: [C++11]
---

[Putty](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html)와 같은
터미널에서는 색상이 입혀진 텍스트를 화면에 출력할 수 있습니다. ANSI Color Code라고 하며,
[ANSI Escape Code](https://en.wikipedia.org/wiki/ANSI_escape_code)의 기능 중 하나입니다.
ANSI Escape Codesms 터미널에서 텍스트 포맷을 제어하기 위해 만든 코드이며,
현재 [ISO/IEC-6429](https://www.iso.org/standard/12782.html) 표준으로 제정되어 있습니다.

대부분의 터미널에서는 이 기능을 지원하는데, Windows의 커맨드 창(Command Shell)에서는
이 기능을 지원하지 않는 것 같습니다.
<br>

# ANSI Escape Code

------ | ------ | ------
Code	 | Effect	 | Note
------ | ------ | ------
0	 | Reset / Normal	 | all attributes off
------ | ------ | ------
1	 | Intensity: Bold |
------ | ------ | ------
2	 | Intensity: Faint	 | not widely supported
------ | ------ | ------
3	 | Italic: on	 | not widely supported. Sometimes treated as inverse.
------ | ------ | ------
4	 | Underline: Single |
------ | ------ | ------
5	 | Blink: Slow	 | less than 150 per minute
------ | ------ | ------
6	 | Blink: Rapid	 | MS-DOS ANSI.SYS; 150 per minute or more
------ | ------ | ------
7	 | Image: Negative	 | inverse or reverse; swap foreground and background
------ | ------ | ------
8	 | Conceal	 | not widely supported
------ | ------ | ------
21 | Underline: Double	 | not widely supported
------ | ------ | ------
22 | 	Intensity: Normal | 	not bold and not faint
------ | ------ | ------
24 | 	Underline: None |
------ | ------ | ------
25 | 	Blink: off |
------ | ------ | ------
27 | 	Image: Positive |
------ | ------ | ------
28 | 	Reveal	conceal off |
------ | ------ | ------

이 외에도 더 많은 ANSI Escape Code 들이 있지만, 이 중에서 텍스트의 색상을 결정하는
코드들은 30부터 39까지, 텍스트의 배경색은 40부터 49까지 존재합니다.


<br>

# ANSI Color

------ | ------
Code	 | Effect
------ | ------
30 | set foreground color to black
------ | ------
31 | set foreground color to red
------ | ------
32 | set foreground color to green
------ | ------
33 | set foreground color to yellow
------ | ------
34 | set foreground color to blue
------ | ------
35 | set foreground color to magenta (purple)
------ | ------
36 | set foreground color to cyan
------ | ------
37 | set foreground color to white
------ | ------
39 | set foreground color to default (white)
------ | ------
40 | set background color to black
------ | ------
41 | set background color to red
------ | ------
42 | set background color to green
------ | ------
43 | set background color to yellow
------ | ------
44 | set background color to blue
------ | ------
45 | set background color to magenta (purple)
------ | ------
46 | set background color to cyan
------ | ------
47 | set background color to white
------ | ------
49 | set background color to default (black)
------ | ------

<br>

# 소스 코드

위와 같은 ANSI Color 색상을 printf 함수나 cout 함수 등을 통해 쉽게 출력할 수 있도록
예제 코드는 다음과 같습니다.

## Log.h

<pre class="prettyprint">
#ifndef LITOSERVICE_UTILS_LOG_H_
#define LITOSERVICE_UTILS_LOG_H_

class Log {
 public:
  static void v(const char* format, ...);
  static void d(const char* format, ...);
  static void i(const char* format, ...);
  static void w(const char* format, ...);
  static void e(const char* format, ...);

 private:
  Log() {}
  virtual ~Log() {}
};

#endif /* LITOSERVICE_UTILS_LOG_H_ */
</pre>

<br>

## Log.cc

<pre class="prettyprint">
#include &lt;Log.h&gt;
#include &lt;cstdio&gt;
#include &lt;stdarg.h&gt;

#ifdef __ANDROID__
#include &lt;android/log.h&gt;
#else
#include &lt;stdarg.h&gt;
#endif

#ifdef __ANDROID__
#define PLOGV(...) __android_log_print(ANDROID_LOG_VERBOSE, gProseLogTag, __VA_ARGS__)
#define PLOGD(...) __android_log_print(ANDROID_LOG_DEBUG, gProseLogTag, __VA_ARGS__)
#define PLOGI(...) __android_log_print(ANDROID_LOG_INFO, gProseLogTag,__VA_ARGS__)
#define PLOGW(...) __android_log_print(ANDROID_LOG_WARN, gProseLogTag,__VA_ARGS__)
#define PLOGE(...) __android_log_print(ANDROID_LOG_ERROR, gProseLogTag, __VA_ARGS__)
#define PLOGF(...) __android_log_print(ANDROID_FATAL_ERROR, gProseLogTag,__VA_ARGS__)
#define PLOGS(...) __android_log_print(ANDROID_SILENT_ERROR, gProseLogTag,__VA_ARGS__)
#else
#define LOGV(...)
#define LOGD(...)
#define LOGI(...)
#define LOGW(...)
#define LOGE(...)
#define LOGF(...)
#define LOGS(...)
#endif

void Log::v(const char* format, ...) {
  char message[1024] = { 0, };

  va_list lpStart;
  va_start(lpStart, format);
  vsprintf(message, format, lpStart);
  va_end(lpStart);

  printf(FCYN("[%s] %s\n"), gLitoServiceTag, message);
  LOGV("%s", message);
}

void Log::d(const char* format, ...) {
  char message[1024] = { 0, };

  va_list lpStart;
  va_start(lpStart, format);
  vsprintf(message, format, lpStart);
  va_end(lpStart);

  printf(FGRN("[%s] %s\n"), gLitoServiceTag, message);
  LOGD("%s", message);
}

void Log::i(const char* format, ...) {
  char message[1024] = { 0, };

  va_list lpStart;
  va_start(lpStart, format);
  vsprintf(message, format, lpStart);
  va_end(lpStart);

  printf(FYEL("[%s] %s\n"), gLitoServiceTag, message);
  LOGI("%s", message);
}

void Log::w(const char* format, ...) {
  char message[1024] = { 0, };

  va_list lpStart;
  va_start(lpStart, format);
  vsprintf(message, format, lpStart);
  va_end(lpStart);

  printf(FMAG("[%s] %s\n"), gLitoServiceTag, message);
  LOGW("%s", message);
}

void Log::e(const char* format, ...) {
  char message[1024] = { 0, };

  va_list lpStart;
  va_start(lpStart, format);
  vsprintf(message, format, lpStart);
  va_end(lpStart);

  printf(BOLD(FRED("[%s] %s\n")), gLitoServiceTag, message);
  LOGE("%s", message);
}
</pre>
