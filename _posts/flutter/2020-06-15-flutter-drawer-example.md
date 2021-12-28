---
layout: post
title: Flutter Drawer 예제
category: Flutter

tag: [Flutter]
---

## Flutter Drawer

Flutter에서 Drawer 사용은 아주 간단합니다. 거의 기본적으로 쓰는 위젯은 `Scaffold` 위젯에 `drawer` 속성을 갖고 있습니다.

따라서 다음 코드처럼만 작성해도 Drawer를 사용할 수 있습니다.

<pre class="prettyprint">
import 'package:flutter/material.dart';

void main() => runApp(MyApp());

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Drawer Example',
      home: Scaffold(
        appBar: AppBar(
          title: Text('Drawer Widget'),
        ),
        drawer: Drawer(),
      ),
    );
  }
}
</pre>

<br>

하지만, 일반적인 Drawer라고 하면 Header 부분도 있어야 하고, 리스트도 있어야 하니깐 다음과 같이 꾸며줄 수 있습니다.

<pre class="prettyprint">
import 'package:flutter/material.dart';

void main() => runApp(MyApp());

class MyApp extends StatelessWidget {
  final appTitle = 'Drawer Example';

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: appTitle,
      home: MyHomePage(title: appTitle),
    );
  }
}

class MyHomePage extends StatelessWidget {
  final String title;

  MyHomePage({Key key, this.title}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(title)),
      body: Center(child: Text('Drawer Example')),
      drawer: Drawer(
        // Add a ListView to the drawer. This ensures the user can scroll
        // through the options in the drawer if there isn't enough vertical
        // space to fit everything.
        child: ListView(
          // Important: Remove any padding from the ListView.
          padding: EdgeInsets.zero,
          children: [
            DrawerHeader(
              child: Text('Header'),
              decoration: BoxDecoration(
                color: Colors.blue,
              ),
            ),
            ListTile(
              title: Text('Item 1'),
              onTap: () {
                // Update the state of the app
                // ...
                // Then close the drawer
                Navigator.pop(context);
              },
            ),
            ListTile(
              title: Text('Item 2'),
              onTap: () {
                // Update the state of the app
                // ...
                // Then close the drawer
                Navigator.pop(context);
              },
            ),
          ],
        ),
      ),
    );
  }
}
</pre>

보다 자세한 내용은 [여기](https://flutter.dev/docs/cookbook/design/drawer)를 참고하세요.