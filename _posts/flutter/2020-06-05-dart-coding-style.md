---
layout: post
title: Dart Coding Style
category: Flutter

tag: [Flutter, Dart]
---

Dart Coding Style 은 [공식 홈페이지](https://dart.dev/guides/language/effective-dart/style)에서 확인할 수 있습니다.

# Identifiers

클래스명, 변수명, 함수명 등을 표현하는 `Identifiers`에는 다음과 같이 가장 많이 사용되는 3가지 유형이 있습니다. 
`Dart`에서는 이 3가지 유형을 전부 활용하고 있습니다.

* UpperCamelCase
* lowerCamelCase
* lowercase_with_underscores

<br>

## 클래스, enum 이나 typedef, extension 등에는 UpperCamelCase

<pre class="prettyprint">
class SliderMenu { ... }

class HttpRequest { ... }

typedef Predicate&lt;T&gt; = bool Function(T value);

extension MyFancyList&lt;T&gt; on List&lt;T&gt; { ... }

extension SmartIterable&lt;T&gt; on Iterable&lt;T&gt; { ... }
</pre>

<br>

## 라이브러리, 패키지, 디렉토리, 소스 파일 이름 및 import prefix에는 lowercase_with_underscores

<pre class="prettyprint">
library peg_parser.source_scanner;

import 'file_system.dart';
import 'slider_menu.dart';

import 'dart:math' as math;
import 'package:angular_components/angular_components'
    as angular_components;
import 'package:js/js.dart' as js;
</pre>

<br>

## 그 외의 이름에는 lowerCamelCase

클래스 멤버 변수, 최상위(Top-level) 선언, 변수, 파라메터 등은 전부 `lowerCamelCase`를 사용합니다.

<pre class="prettyprint">
var item;

HttpRequest httpRequest;

void align(bool clearItems) {
  // ...
}
</pre>

<br>

## 상수값(Constant)에서도 lowerCamelCase

대부분의 언어에서는 상수 값을 나타내는 변수에는 모두 대문자(`SCREAMING_CAPS`)로 표현하는 경우가 많은데, `Dart`에서는 `lowerCamelCase`를 권장합니다.
(과거에는 `Dart`에서도 `SCREAMING_CAPS` 스타일을 사용했으나 몇 가지 단점으로 인해 `lowerCamelCase` 스타일로 변경했습니다.)

<pre class="prettyprint">
const pi = 3.14;
const defaultTimeout = 1000;
final urlScheme = RegExp('^([a-z]+):');

class Dice {
  static final numberGenerator = Random();
}
</pre>

물론 권장이기 때문에 다음과 같은 경우에는 예외적으로 `SCREAMING_CAPS`를 허용하기도 합니다.

* `SCREAMING_CAPS` 형태 네이밍의 변수를 사용하고 있는 기존 코드나 라이브러리를 사용할 경우
* `Dart` 코드를 `Java` 코드와 병행해서 개발할 경우

<br>

## 약어들의 스타일

약어들을 대문자로만 사용할 경우 가독성에 어려움이 발생할 수 있으며, 뜻이 모호해지기도 합니다. 예를 들어 `HTTPSFTP`와 같은 단어는 `HTTPS FTP`인지 `HTTP SFTP`인지 알아 볼 수 없습니다. 따라서 두 단어 이상의 약어들은 일반 단어 사용하듯이 대소문자를 사용하면 됩니다.

### good 예시

<pre class="prettyprint">
HttpConnectionInfo
uiHandler
IOStream
HttpRequest
Id
DB
</pre>

### bad 예시

<pre class="prettyprint">
HTTPConnection
UiHandler
IoStream
HTTPRequest
ID
Db
</pre>

<br>

## 언더스코어(`_`)를 `prefix`로 사용하지 말 것

언더스코어(`_`)는 `private`를 의미하기 때문에 사용하지 말아야 합니다.

<br>

## 변수 이름 앞에 `prefix` 사용할 필요 없음

[Hungarian Notation](https://en.wikipedia.org/wiki/Hungarian_notation)과 같이 과거에는 변수가 어떤 용도로 사용되는 건지 코드 가독성을 위해 변수 타입에 대한 `prefix`를 붙이는 경우가 많았으나, `Dart`에서는 변수의 타입, 범위(Scope), Mutability 등 요소를 모두 알려주기 떄문에 별도의 `prefix`를 사용할 필요가 없습니다.

<br>

## import 순서

`dart`, `package`, 그 외 코드 순으로 `import` 합니다. 또한 `export`는 `import` 뒤에 배치하며, 각 구문은 알파벳 순으로 정렬합니다.

<pre class="prettyprint">
import 'dart:async';
import 'dart:html';

import 'package:bar/bar.dart';
import 'package:foo/foo.dart';

import 'util.dart';

import 'src/error.dart';
import 'src/foo_bar.dart';

export 'src/error.dart';
</pre>

<br>

## Formating

* [`dartfmt`](https://github.com/dart-lang/dart_style)을 이용해서 formatting 적용
* formatter에 의존하기 전에 먼저 formatter-friendly한 형태로 코드를 정리
* 한 라인에는 80글자까지(다만 URL이나 멀티라인의 경우는 예외)

### if 문 중괄호

<pre class="prettyprint">
if (isWeekDay) {
  print('Bike to work!');
} else {
  print('Go dancing or read a book!');
}

if (overflowChars != other.overflowChars) {
  return overflowChars < other.overflowChars;
}
</pre>

`if` 문에 `else` 구문이 없는 경우는 다음과 같이 한 줄로 표현도 가능합니다.

<pre class="prettyprint">
if (arg == null) return defaultValue;
</pre>