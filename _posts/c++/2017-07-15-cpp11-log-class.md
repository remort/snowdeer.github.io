---
layout: post
title: C++용 Log 클래스
category: C++
tag: [C++11]
---

C++ 에서 사용할 수 있는 Log 클래스입니다. 안드로이드 NDK 환경이나 Windows 환경 등에서도 사용할 수
있습니다. 사용법을 최대한 안드로이드 Java의 Log 클래스와 비슷하게 흉내냈습니다.

<br>

# Log.h

<pre class="prettyprint">#ifndef SNOWDEER_LOG_H
#define SNOWDEER_LOG_H

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

#endif //SNOWDEER_LOG_H</pre>

<br>

# Log.cc

<pre class="prettyprint">#include "Log.h"

#include &lt;cstdio&gt;
#ifdef __ANDROID__
#include &lt;android/log.h&gt;
#else
#include &lt;stdarg.h&gt;
#endif

const char* gLogTag = "SDLog";

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

  printf("[%s] %s\n", gLogTag, message);
  LOGV("%s", message);
}

void Log::d(const char* format, ...) {
  char message[1024] = { 0, };

  va_list lpStart;
  va_start(lpStart, format);
  vsprintf(message, format, lpStart);
  va_end(lpStart);

  printf("[%s] %s\n", gLogTag, message);
  LOGD("%s", message);
}

void Log::i(const char* format, ...) {
  char message[1024] = { 0, };

  va_list lpStart;
  va_start(lpStart, format);
  vsprintf(message, format, lpStart);
  va_end(lpStart);

  printf("[%s] %s\n", gLogTag, message);
  LOGI("%s", message);
}

void Log::w(const char* format, ...) {
  char message[1024] = { 0, };

  va_list lpStart;
  va_start(lpStart, format);
  vsprintf(message, format, lpStart);
  va_end(lpStart);

  printf("[%s] %s\n", gLogTag, message);
  LOGW("%s", message);
}

void Log::e(const char* format, ...) {
  char message[1024] = { 0, };

  va_list lpStart;
  va_start(lpStart, format);
  vsprintf(message, format, lpStart);
  va_end(lpStart);

  printf("[%s] %s\n", gLogTag, message);
  LOGE("%s", message);
}
</pre>
