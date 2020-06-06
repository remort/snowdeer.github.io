---
layout: post
title: Flutter 화면 전환(복수 파일 이용)
category: Flutter

tag: [Flutter]
---

Flutter에서의 화면 전환 예제를 여러 개의 파일로 나누어서 호출하도록 변경했습니다.
다른 파일을 가져올 때 다음과 같이 두 가지 방식으로 `import` 할 수 있습니다.

<pre class="prettyprint">
import 'first_screen_widget.dart';
import 'package:snowdeer_hello_flutter/second_screen_widget.dart';
</pre>

<br>

# main.dart

<pre class="prettyprint">
import 'package:flutter/material.dart';
import 'first_screen_widget.dart';

void main() => runApp(SnowDeerApp());

class SnowDeerApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'SnowDeer App',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        primaryColor: Colors.deepPurple,
      ),
      home: FirstScreenWidget(),
    );
  }
}
</pre>

<br>

# first_screen_widget.dart

<pre class="prettyprint">
import 'package:flutter/material.dart';
import 'package:snowdeer_hello_flutter/second_screen_widget.dart';

class FirstScreenWidget extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('First Screen'),
      ),
      body: Container(
          width: double.infinity,
          height: double.infinity,
          color: Colors.amber,
          child: Center(
              child: RaisedButton(
            child: Text('Move to Second Screen'),
            onPressed: () {
              Navigator.push(
                context,
                MaterialPageRoute(builder: (context) => SecondScreenWidget()),
              );
            },
          ))),
    );
  }
}
</pre>

<br>

# second_screen_widget.dart

<pre class="prettyprint">
import 'package:flutter/material.dart';

class SecondScreenWidget extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Second Screen'),
      ),
      body: Container(
          width: double.infinity,
          height: double.infinity,
          color: Colors.green,
          child: Center(
              child: RaisedButton(
              child: Text('Back to First Screen'),
              onPressed: () {
                Navigator.pop(context);
              },
          ))),
    );
  }
}
</pre>