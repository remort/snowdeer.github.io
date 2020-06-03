---
layout: post
title: Flutter Widget 예제(Text, Image, RaisedButton)
category: Flutter

tag: [Flutter]
---

# Text Widget 예제

<pre class="prettyprint">
import 'package:flutter/material.dart';

void main() => runApp(SnowDeerExample());

class SnowDeerExample extends StatelessWidget {
  static const title = 'SnowDeer\'s Text Example';

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
        title: title,
        debugShowCheckedModeBanner: false,
        home: Scaffold(
            appBar: AppBar(title: Text(title)),
            body: Column(children: [
              Text('안녕하세요. SnowDeer 입니다.'),
              Text('이건 Styled Text 입니다.',
                  style: TextStyle(
                      color: Colors.blue,
                      fontSize: 16.0,
                      fontWeight: FontWeight.bold)),
              Text('이건 두번 째 Styled Text 입니다.',
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 20.0,
                    fontWeight: FontWeight.bold,
                    background: Paint()
                      ..color = Colors.blueGrey
                      ..style = PaintingStyle.fill,
                  ))
            ])));
  }
}
</pre>

<br>

# Image Widget 예제

먼저 이미지 파일을 `assets/images` 디렉토리에 복사한 다음, `pubspec.yaml` 파일을 다음과 같이 수정합니다.

<pre class="prettyprint">
flutter:

  uses-material-design: true
  assets:
     - assets/images/

  ...
</pre>

그 이후 예제 코드는 다음과 같습니다.

<pre class="prettyprint">
import 'package:flutter/material.dart';

void main() => runApp(SnowDeerExample());

class SnowDeerExample extends StatelessWidget {
  static const title = 'SnowDeer\'s Image Example';

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
        title: title,
        debugShowCheckedModeBanner: false,
        home: Scaffold(
            appBar: AppBar(title: Text(title)),
            body: Column(children: [
              Image.asset('assets/images/snowdeer.png')
            ])));
  }
}
</pre>

<br>

# RaisedButton Widget 예제

<pre class="prettyprint">
import 'package:flutter/material.dart';

void main() => runApp(SnowDeerExample());

class SnowDeerExample extends StatelessWidget {
  static const title = 'SnowDeer\'s RaisedButton Example';

  void onClicked() {
    print('onClicked !!!');
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
        title: title,
        debugShowCheckedModeBanner: false,
        home: Scaffold(
            appBar: AppBar(title: Text(title)),
            body: Column(
                crossAxisAlignment: CrossAxisAlignment.center,
                children: [
                  RaisedButton(
                    child: Text('Normal Button'),
                    onPressed: onClicked,
                  ),
                  RaisedButton(
                      child: Text('Round Shape Button'),
                      onPressed: onClicked,
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(12),
                      ))
                ])));
  }
}
</pre>

<br>

# TextFormField Widget 예제

<pre class="prettyprint">
import 'package:flutter/material.dart';

void main() => runApp(SnowDeerExample());

class SnowDeerExample extends StatelessWidget {
  static const title = 'SnowDeer\'s TextFormField Example';

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
        title: title,
        debugShowCheckedModeBanner: false,
        home: Scaffold(
            appBar: AppBar(title: Text(title)),
            body: Column(
                crossAxisAlignment: CrossAxisAlignment.center,
                children: [
                  TextFormField(),
                  TextFormField(
                    keyboardType: TextInputType.number,
                  ),
                  TextFormField(
                    keyboardType: TextInputType.emailAddress,
                  )
                ])));
  }
}
</pre>

이 때, TextFormField에서 가질 수 있는 키보드 속성은 다음과 같습니다.

* TextInputType.datetime
* TextInputType.emailaddress
* TextInputType.multiline
* TextInputType.number
* TextInputType.phone
* TextInputType.text
* TextInputType.url