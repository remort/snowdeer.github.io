---
layout: post
title: Flutter Layout Widget 예제(Container, Row, Column)
category: Flutter

tag: [Flutter]
---

# Container Widget 예제

`Container` 위젯은 자식을 1개를 갖는 위젯입니다.

<pre class="prettyprint">
import 'package:flutter/material.dart';

void main() => runApp(SnowDeerExample());

class SnowDeerExample extends StatelessWidget {
  static const title = 'SnowDeer\'s Container Example';

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
                  Container(
                    child: Text('첫 번째 Container'),
                    color: Colors.yellow,
                  ),
                  Container(
                    child: Text('두 번째 Container with padding(EdgeInsets.only)'),
                    padding: EdgeInsets.only(
                        left: 12, top: 12, bottom: 12, right: 12),
                    color: Colors.blueGrey,
                  ),
                  Container(
                    child: Text(
                        '세 번째 Container with padding(EdgeInsets.symmetric)'),
                    padding: EdgeInsets.symmetric(vertical: 12, horizontal: 20),
                    color: Colors.deepOrange,
                  ),
                  Container(
                    child: Text(
                        '네 번째 Container with margin(EdgeInsets.symmetric)'),
                    margin: EdgeInsets.symmetric(vertical: 12, horizontal: 20),
                    color: Colors.cyan,
                  ),
                ])));
  }
}
</pre>

<br>

# Row/Column Widget 예제

`Row` 위젯은 자식 위젯들을 `가로`로 배치할 때 사용합니다. 
반대로 `Column`은 자식 위젯들을 `세로`로 배치할 때 사용합니다.

<pre class="prettyprint">
import 'package:flutter/material.dart';

void main() => runApp(SnowDeerExample());

class SnowDeerExample extends StatelessWidget {
  static const title = 'SnowDeer\'s Row Example';

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
        title: title,
        debugShowCheckedModeBanner: false,
        home: Scaffold(
            appBar: AppBar(title: Text(title)),
            body: Row(
                crossAxisAlignment: CrossAxisAlignment.center,
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Container(
                    child: Text('첫 번째 Container'),
                    color: Colors.yellow,
                  ),
                  Container(
                    child: Text('두 번째 Container with padding(EdgeInsets.only)'),
                    padding: EdgeInsets.only(
                        left: 12, top: 12, bottom: 12, right: 12),
                    color: Colors.blueGrey,
                  ),
                  RaisedButton(
                    child: Text('Hello'),
                  )
                ])));
  }
}
</pre>

<br>

# Row/Column Widget 속성

`Row`나 `Column`은 레이아웃 위젯이다보니 중요한 속성들이 있습니다.
`mainAxisAlignment`는 현재 속성의 축(axis)을 기준으로 정렬을 하는 옵션입니다.
현재 레이아웃이 `Row`이면, 가로 방향을 의미하기 때문에 좌/우 정렬값을 의미합니다.
`MainAxisAlignment.start`, `MainAxisAlignment.center` 등의 값을 가질 수 있습니다.

`crossAxisAlignment`는 현재 속성의 반대 방향으로 정렬을 의미합니다.

예를 들어 아래 속성이면 다음 그림과 같이 위젯이 배치됩니다.

<pre class="prettyprint">
  body: Column(
    mainAxisAlignment: MainAxisAlignment.start,
    crossAxisAlignment: CrossAxisAlignment.end,
    ...
</pre>

![Image](/assets/flutter/001_col_main_start_cross_end.png)

<br>

<pre class="prettyprint">
  body: Column(
    mainAxisAlignment: MainAxisAlignment.center,
    crossAxisAlignment: CrossAxisAlignment.start,
    ...
</pre>

![Image](/assets/flutter/002_col_main_center_cross_start.png)

<br>

<pre class="prettyprint">
  body: Column(
    mainAxisAlignment: MainAxisAlignment.end,
    crossAxisAlignment: CrossAxisAlignment.start,
    ...
</pre>

![Image](/assets/flutter/003_col_main_end_cross_start.png)

<br>

<pre class="prettyprint">
  body: Row(
    mainAxisAlignment: MainAxisAlignment.start,
    crossAxisAlignment: CrossAxisAlignment.end,
    ...
</pre>

![Image](/assets/flutter/004_row_main_start_cross_end.png)

<br>

<pre class="prettyprint">
  body: Row(
    mainAxisAlignment: MainAxisAlignment.start,
    crossAxisAlignment: CrossAxisAlignment.center,
    ...
</pre>

![Image](/assets/flutter/005_row_main_start_cross_center.png)




