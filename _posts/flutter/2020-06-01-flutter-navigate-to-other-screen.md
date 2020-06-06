---
layout: post
title: Flutter 화면 전환
category: Flutter

tag: [Flutter]
---

Flutter에서 화면 전환을 하는 예제입니다. `Navigator`를 이용해서 새로운 화면을 `push` 또는 `pop` 하여 화면간 전환을 합니다.

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
      home: FirstScreenWidget(),
    );
  }
}

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

<br>

# 화면간 데이터 전달하는 방법

아래와 같이 사용자 정의 클래스를 만들고 해당 인스턴스를 다음 화면에 전달하는 예제입니다.

<pre class="prettyprint">
class Item {
  int id;
  String name;

  Item({this.id, this.name});
}
</pre>

<br>

## 전체 코드 

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
      home: FirstScreenWidget(),
    );
  }
}

class Item {
  int id;
  String name;

  Item({this.id, this.name});
}

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
              final item = Item(id: 10, name: 'snowdeer');
              Navigator.push(
                context,
                MaterialPageRoute(
                    builder: (context) => SecondScreenWidget(item: item)),
              );
            },
          ))),
    );
  }
}

class SecondScreenWidget extends StatelessWidget {
  final Item item;

  SecondScreenWidget({this.item});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text("Second Screen"),
      ),
      body: Container(
          width: double.infinity,
          height: double.infinity,
          color: Colors.green,
          child: Center(
            child: Text('id: ${item.id}, name: ${item.name}'),
          )),
    );
  }
}
</pre>

위에서 두 번째 위젯인 `SecondScreenWidget`의 생성자를 통해서 `item` 인스턴스를 속성에 넣어주도록 했습니다.

생성자는 아래와 같이 필수 입력 매개변수로 선언하면 좀 더 안전한 코드가 됩니다.

<pre class="prettyprint">
SecondScreenWidget({@required this.item});
</pre>