---
layout: post
title: Flutter SnackBar 예제
category: Flutter

tag: [Flutter]
---

## Flutter SnackBar

<pre class="prettyprint">
import 'package:flutter/material.dart';

void main() => runApp(MyApp());

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'SnackBar Example',
      home: Scaffold(
        appBar: AppBar(
          title: Text('SnackBar Example'),
        ),
        body: SnackBarPage(),
      ),
    );
  }
}

class SnackBarPage extends StatelessWidget {
  final sb = SnackBar(
    content: Text('Hello, SnowDeer ~!!'),
    action: SnackBarAction(
      label: 'ok',
      onPressed: () {},
    ),
  );

  @override
  Widget build(BuildContext context) {
    return Center(
      child: RaisedButton(
        child: Text('SnackBar'),
        onPressed: () {
          Scaffold.of(context).showSnackBar(sb);
        },
      ),
    );
  }
}
</pre>