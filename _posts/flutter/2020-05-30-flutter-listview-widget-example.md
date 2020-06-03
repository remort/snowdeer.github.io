---
layout: post
title: Flutter ListView Widget 예제
category: Flutter

tag: [Flutter]
---

# ListView Widget 예제

<pre class="prettyprint">
import 'package:flutter/material.dart';

void main() => runApp(SnowDeerExample());

class SnowDeerExample extends StatelessWidget {
  static const title = 'SnowDeer\'s ListView Example';
  static const List list = [
    'snowdeer',
    'ran',
    'song',
    'yang',
    'john',
    'downy',
    'gap',
    'kyu',
    'ho',
    'hoon',
  ];

  Widget buildListView() {
    return ListView.builder(
        itemCount: list.length,
        itemBuilder: (BuildContext context, int i) {
          return ListTile(
            title: Text(
              list[i],
            ),
            trailing: Icon(Icons.favorite_border),
          );
        });
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
        title: title,
        debugShowCheckedModeBanner: false,
        home: Scaffold(
          appBar: AppBar(title: Text(title)),
          body: buildListView(),
        ));
  }
}
</pre>
