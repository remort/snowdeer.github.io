---
layout: post
title: Flutter Stream 예제
category: Flutter

tag: [Flutter]
---

## simple_timer.dart

<pre class="prettyprint">
import 'dart:async';

class SampleTimer {
  int seconds = 0;
  bool isRunning = false;

  late Timer timer;

  void start(int seconds) {
    this.seconds = seconds;
    isRunning = true;
  }

  void stop() {
    isRunning = false;
  }

  Stream&lt;int&gt; stream() async* {
    yield* Stream.periodic(const Duration(seconds: 1), (int a) {
      if (isRunning) {
        seconds = seconds - 1;
        if (seconds <= 0) {
          isRunning = false;
        }
      }

      return seconds;
    });
  }
}
</pre>

## timer_page.dart

<pre class="prettyprint">
import 'package:flutter/material.dart';
import 'package:stream_sample/timer/sample_timer.dart';

class TimerPage extends StatelessWidget {
  TimerPage({Key? key}) : super(key: key);

  final timer = SampleTimer();

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        Row(
          children: [
            const Padding(padding: EdgeInsets.all(8)),
            Expanded(
              child: MaterialButton(
                child: const Text('Start'),
                color: Colors.amber,
                onPressed: () => timer.start(100),
              ),
            ),
            const Padding(padding: EdgeInsets.all(8)),
            Expanded(
              child: MaterialButton(
                child: const Text('Stop'),
                color: Colors.amber,
                onPressed: () => timer.stop(),
              ),
            ),
            const Padding(padding: EdgeInsets.all(8)),
          ],
        ),
        StreamBuilder(
          stream: timer.stream(),
          builder: (context, snapshot) {
            return Text(
              snapshot.data.toString(),
              style: const TextStyle(
                fontSize: 30,
              ),
            );
          },
        )
      ],
    );
  }
}
</pre>

# main.dart

<pre class="prettyprint">
import 'package:flutter/material.dart';
import 'package:stream_sample/timer_page.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      home: Scaffold(
        appBar: AppBar(
          title: const Text('snowdeer\'s timer sample'),
        ),
        body: TimerPage(),
      ),
    );
  }
}
</pre>