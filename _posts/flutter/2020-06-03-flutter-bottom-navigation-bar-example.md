---
layout: post
title: Flutter BottomNavigationBar 예제
category: Flutter

tag: [Flutter]
---

하단 탭을 이용한 네비게이션 예제 코드입니다.

# main.dart

<pre class="prettyprint">
import 'package:flutter/material.dart';

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
      home: MainWidget(),
    );
  }
}

class MainWidget extends StatefulWidget {
  @override
  State createState() => _MainWidgetState();
}

class _MainWidgetState extends State&lt;MainWidget&gt; {
  var _index = 0;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Hello Snowdeer'),
      ),
      body: Center(
        child: Text('Page Index: $_index'),
      ),
      bottomNavigationBar: BottomNavigationBar(
          currentIndex: _index,
          onTap: (index) {
            setState(() {
              _index = index;
            });
          },
          items: [
            BottomNavigationBarItem(
              title: Text('Home'),
              icon: Icon(Icons.home),
            ),
            BottomNavigationBarItem(
              title: Text('DoAction'),
              icon: Icon(Icons.email),
            ),
            BottomNavigationBarItem(
              title: Text('DoBehavior'),
              icon: Icon(Icons.favorite),
            ),
          ]),
    );
  }
}
</pre>

<br>

## 실제 페이지 이동하는 예제

<pre class="prettyprint">
import 'package:flutter/material.dart';

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
      home: MainWidget(),
    );
  }
}

class MainWidget extends StatefulWidget {
  @override
  State createState() => _MainWidgetState();
}

class _MainWidgetState extends State&lt;MainWidget&gt; {
  var _index = 0;
  var _pages = [
    HomePageWidget(),
    DoActionPageWidget(),
    DoBehaviorPageWidget(),
  ];

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Hello Snowdeer'),
        actions: [
          IconButton(
            icon: Icon(
              Icons.add,
              color: Colors.white,
            ),
            onPressed: () {},
          )
        ],
      ),
      body: _pages[_index],
      bottomNavigationBar: BottomNavigationBar(
          currentIndex: _index,
          onTap: (index) {
            setState(() {
              _index = index;
            });
          },
          items: [
            BottomNavigationBarItem(
              title: Text('Home'),
              icon: Icon(Icons.home),
            ),
            BottomNavigationBarItem(
              title: Text('DoAction'),
              icon: Icon(Icons.email),
            ),
            BottomNavigationBarItem(
              title: Text('DoBehavior'),
              icon: Icon(Icons.favorite),
            ),
          ]),
    );
  }
}

class HomePageWidget extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Center(
      child: Text('Home'),
    );
  }
}

class DoActionPageWidget extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Center(
      child: Text('DoAction'),
    );
  }
}

class DoBehaviorPageWidget extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Center(
      child: Text('DoBehavior'),
    );
  }
}
</pre>