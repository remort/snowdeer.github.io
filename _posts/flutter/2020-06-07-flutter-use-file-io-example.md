---
layout: post
title: Flutter TextFile 쓰기/읽기
category: Flutter

tag: [Flutter]
---

# TextFile 쓰기/읽기

<pre class="prettyprint">
import 'dart:io';

import 'package:flutter/material.dart';

void main() => runApp(SnowApp());

class SnowApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'File IO Example',
      theme: ThemeData(
        primaryColor: Colors.indigoAccent,
      ),
      home: Scaffold(
        appBar: AppBar(
          title: Text('File IO Example'),
        ),
        body: FileIoTest(),
      ),
    );
  }
}

class FileIoTest extends StatelessWidget {
  void saveToFile(String filepath, String text) {
    final file = File(filepath);
    file.createSync();

    file.writeAsStringSync(text, mode: FileMode.append);
  }

  void loadFromFile(String filepath) {
    final file = File(filepath);
    print('Filepath: ${file.absolute.path}');

    final lines = file.readAsLinesSync();
    for (String line in lines) {
      print(line);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Column(
        children: [
          RaisedButton(
            child: Text('Save to file'),
            onPressed: () {
              final filepath = 'snowdeer.txt';
              final text = "Hello, SnowDeer\n";

              saveToFile(filepath, text);
            },
          ),
          RaisedButton(
            child: Text('Load from file'),
            onPressed: () {
              final filepath = 'snowdeer.txt';
              loadFromFile(filepath);
            },
          ),
        ],
      ),
    );
  }
}
</pre>