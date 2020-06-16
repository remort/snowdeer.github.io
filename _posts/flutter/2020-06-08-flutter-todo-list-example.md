---
layout: post
title: Flutter TodoList 예제
category: Flutter

tag: [Flutter]
---

## todo_item.dart

<pre class="prettyprint">
class TodoItem {
  String name = '';
  bool isChecked = false;

  TodoItem({this.name});
}
</pre>

<br>

## main.dart

<pre class="prettyprint">
import 'package:fileio/todo_item.dart';
import 'package:flutter/material.dart';

void main() => runApp(SnowApp());

class SnowApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
        title: 'Todo list',
        theme: ThemeData(
          primaryColor: Colors.deepPurple,
        ),
        home: Scaffold(
          appBar: AppBar(
            title: Text('Todo list'),
          ),
          body: TodoListWidget(),
        ));
  }
}

class TodoListWidget extends StatefulWidget {
  @override
  State createState() => TodoListWidgetState();
}

class TodoListWidgetState extends State&lt;TodoListWidget&gt; {
  final controller = TextEditingController();
  final list = List();

  @override
  void dispose() {
    super.dispose();
    controller.dispose();
  }

  void addTodo(TodoItem item) {
    setState(() {
      list.add(item);
    });
  }

  void removeTodo(TodoItem item) {
    setState(() {
      list.remove(item);
    });
  }

  void setChecked(TodoItem item, bool isChecked) {
    setState(() {
      item.isChecked = isChecked;
    });
  }

  Widget buildListTime(BuildContext context, TodoItem item) {
    return ListTile(
      onTap: () {
        setChecked(item, !item.isChecked);
      },
      leading: item.isChecked == true
          ? Icon(Icons.check_box)
          : Icon(Icons.check_box_outline_blank),
      title: Text(
        item.name,
        style: item.isChecked
            ? TextStyle(
                decoration: TextDecoration.lineThrough,
                fontStyle: FontStyle.italic,
              )
            : null,
      ),
      trailing: IconButton(
        icon: Icon(Icons.delete_forever),
        onPressed: () {
          removeTodo(item);
        },
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        Row(
          children: [
            Expanded(
              child: TextField(
                controller: controller,
              ),
            ),
            RaisedButton(
              child: Text(
                'Add',
              ),
              onPressed: () {
                addTodo(TodoItem(name: controller.text));
                controller.text = '';
              },
            ),
          ],
        ),
        Expanded(
          child: ListView(
            children: list.map((item) => buildListTime(context, item)).toList(),
          ),
        )
      ],
    );
  }
}
</pre>