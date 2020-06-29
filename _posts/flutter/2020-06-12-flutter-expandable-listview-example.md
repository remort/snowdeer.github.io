---
layout: post
title: Flutter Expandable ListView 예제
category: Flutter

tag: [Flutter]
---

## group.dart

<pre class="prettyprint">
import 'package:flutter/material.dart';

class Group {
  final String name;
  final IconData icon;
  final peopleList = List&lt;String&gt;();

  void addPerson(String name) {
    peopleList.add(name);
  }

  Group({this.name, this.icon});
}
</pre>

<br>

## main.dart

<pre class="prettyprint">
import 'package:flutter/material.dart';

import 'group.dart';

void main() => runApp(MyApp());

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Expandable ListView',
      theme: ThemeData(
        primarySwatch: Colors.pink,
      ),
      home: Scaffold(
        appBar: AppBar(
          title: Text("Expandable ListView Example"),
        ),
        body: MyExample(),
      ),
    );
  }
}

class MyExample extends StatelessWidget {
  final Group group1 = Group(name: 'Teacher', icon: Icons.people);
  final Group group2 = Group(name: 'Student', icon: Icons.people_outline);

  final groupList = List&lt;Group&gt;();

  MyExample() {
    group1.addPerson('snowdeer');
    group1.addPerson('snowcat');
    group1.addPerson('snowlion');

    group2.addPerson('ran');
    group2.addPerson('song');
    group2.addPerson('down');
    group2.addPerson('john');
    group2.addPerson('yang');

    groupList.add(group1);
    groupList.add(group2);
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Container(
        margin: EdgeInsets.all(8.0),
        child: ListView.builder(
          itemCount: groupList.length,
          itemBuilder: (context, i) {
            final group = groupList[i];
            return ExpansionTile(
              title: Text(group.name),
              children: [
                Column(
                  children: _buildExpandableContent(group),
                )
              ],
            );
          },
        ),
      ),
    );
  }

  _buildExpandableContent(Group group) {
    final columnContent = List&lt;Widget&gt;();

    for (String name in group.peopleList)
      columnContent.add(
        ListTile(
          title: Text(
            name,
            style: TextStyle(fontSize: 18.0),
          ),
          leading: Icon(group.icon),
        ),
      );

    return columnContent;
  }
}
</pre>