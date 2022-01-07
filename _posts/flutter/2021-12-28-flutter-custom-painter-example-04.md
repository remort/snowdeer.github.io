---
layout: post
title: Flutter Canvas 그리기 예제 - (4) Canvas에 Text 그리기
category: Flutter

tag: [Flutter]
---

# Canvas에 Text 그리기

## 실행 화면

화면 특정 좌표에 원과 텍스트를 그리는 예제입니다. 원, 사각형, 라인 등은 좌표만 입력하면 쉽게 그릴 수 있기 때문에 사용법이 조금 다른 텍스트 그리기 위주로 포스팅합니다.

![image](/assets/flutter/011.png)

## Text 그리는 함수 예제

<pre class="prettyprint">
void _drawText(Canvas canvas, centerX, centerY, text, style) {
    final textSpan = TextSpan(
      text: text,
      style: style,
    );

    final textPainter = TextPainter()
      ..text = textSpan
      ..textDirection = TextDirection.ltr
      ..textAlign = TextAlign.center
      ..layout();

    final xCenter = (centerX - textPainter.width / 2);
    final yCenter = (centerY - textPainter.height / 2);
    final offset = Offset(xCenter, yCenter);

    textPainter.paint(canvas, offset);
  }
</pre>

## 전체 코드

<pre class="prettyprint">
import 'package:flutter/material.dart';
import 'package:snowdeer_canvas_example/drag_and_drop/node.dart';

class DraggableObjectPage extends StatefulWidget {
  const DraggableObjectPage({Key key}) : super(key: key);

  @override
  State createState() => DraggableObjectPageState();
}

class DraggableObjectPageState extends State&lt;DraggableObjectPage&gt; {
  final List&lt;Node&gt; nodeList = List.empty(growable: true);

  DraggableObjectPageState() {
    initNodes();
  }

  void initNodes() {
    nodeList.clear();
    nodeList.add(Node("1", "Node\n1", 150.0, 180.0));
    nodeList.add(Node("2", "Node\n2", 220.0, 40.0));
    nodeList.add(Node("3", "Node\n3", 380.0, 240.0));
    nodeList.add(Node("4", "Node\n4", 640.0, 190.0));
    nodeList.add(Node("5", "Node\n5", 480.0, 350.0));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: GestureDetector(
        onTapDown: (detail) {
          print("onTapDown:(${detail.localPosition})");
        },
        onTapUp: (detail) {
          print("onTapUp(${detail.localPosition})");
        },
        onHorizontalDragUpdate: (detail) {
          print("onHorizontalDragUpdate:(${detail.localPosition})");
        },
        child: CustomPaint(
          child: Container(),
          painter: DraggablePainter(nodeList),
        ),
      ),
    );
  }
}

class DraggablePainter extends CustomPainter {
  static const gridWidth = 50.0;
  static const gridHeight = 50.0;

  var _width = 0.0;
  var _height = 0.0;

  final List&lt;Node&gt; nodeList;

  DraggablePainter(this.nodeList);

  void _drawBackground(Canvas canvas) {
    var paint = Paint()
      ..style = PaintingStyle.fill
      ..color = Colors.white70
      ..isAntiAlias = true;

    Rect rect = Rect.fromLTWH(0, 0, _width, _height);
    canvas.drawRect(rect, paint);
  }

  void _drawGrid(Canvas canvas) {
    var paint = Paint()
      ..style = PaintingStyle.fill
      ..color = Colors.grey
      ..isAntiAlias = true;

    final rows = _height / gridHeight;
    final cols = _width / gridWidth;

    for (int r = 0; r < rows; r++) {
      final y = r * gridHeight;
      final p1 = Offset(0, y);
      final p2 = Offset(_width, y);

      canvas.drawLine(p1, p2, paint);
    }

    for (int c = 0; c < cols; c++) {
      final x = c * gridWidth;
      final p1 = Offset(x, 0);
      final p2 = Offset(x, _height);

      canvas.drawLine(p1, p2, paint);
    }
  }

  void _drawNodes(Canvas canvas) {
    var paint = Paint()
      ..style = PaintingStyle.fill
      ..color = Colors.amber
      ..isAntiAlias = true;

    const textStyle = TextStyle(
      color: Colors.black,
      fontSize: 18,
    );

    const radius = 30.0;
    for (int i = 0; i < nodeList.length; i++) {
      final c = Offset(nodeList[i].x, nodeList[i].y);
      canvas.drawCircle(c, radius, paint);
      _drawText(
          canvas, nodeList[i].x, nodeList[i].y, nodeList[i].name, textStyle);
    }
  }

  void _drawText(Canvas canvas, centerX, centerY, text, style) {
    final textSpan = TextSpan(
      text: text,
      style: style,
    );

    final textPainter = TextPainter()
      ..text = textSpan
      ..textDirection = TextDirection.ltr
      ..textAlign = TextAlign.center
      ..layout();

    final xCenter = (centerX - textPainter.width / 2);
    final yCenter = (centerY - textPainter.height / 2);
    final offset = Offset(xCenter, yCenter);

    textPainter.paint(canvas, offset);
  }

  @override
  void paint(Canvas canvas, Size size) {
    _width = size.width;
    _height = size.height;

    _drawBackground(canvas);
    _drawGrid(canvas);
    _drawNodes(canvas);
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => false;
}
</pre>

## 전체 코드

<pre class="prettyprint">
import 'package:flutter/material.dart';
import 'package:snowdeer_canvas_example/drag_and_drop/node.dart';

class DraggableObjectPage extends StatefulWidget {
  const DraggableObjectPage({Key key}) : super(key: key);

  @override
  State createState() => DraggableObjectPageState();
}

class DraggableObjectPageState extends State<DraggableObjectPage> {
  final List<Node> nodeList = List.empty(growable: true);

  DraggableObjectPageState() {
    initNodes();
  }

  void initNodes() {
    nodeList.clear();
    nodeList.add(Node("1", "Node\n1", 150.0, 180.0));
    nodeList.add(Node("2", "Node\n2", 220.0, 40.0));
    nodeList.add(Node("3", "Node\n3", 380.0, 240.0));
    nodeList.add(Node("4", "Node\n4", 640.0, 190.0));
    nodeList.add(Node("5", "Node\n5", 480.0, 350.0));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: GestureDetector(
        onTapDown: (detail) {
          print("onTapDown:(${detail.localPosition})");
        },
        onTapUp: (detail) {
          print("onTapUp(${detail.localPosition})");
        },
        onHorizontalDragUpdate: (detail) {
          print("onHorizontalDragUpdate:(${detail.localPosition})");
        },
        child: CustomPaint(
          child: Container(),
          painter: DraggablePainter(nodeList),
        ),
      ),
    );
  }
}

class DraggablePainter extends CustomPainter {
  static const gridWidth = 50.0;
  static const gridHeight = 50.0;

  var _width = 0.0;
  var _height = 0.0;

  final List<Node> nodeList;

  DraggablePainter(this.nodeList);

  void _drawBackground(Canvas canvas) {
    var paint = Paint()
      ..style = PaintingStyle.fill
      ..color = Colors.white70
      ..isAntiAlias = true;

    Rect rect = Rect.fromLTWH(0, 0, _width, _height);
    canvas.drawRect(rect, paint);
  }

  void _drawGrid(Canvas canvas) {
    var paint = Paint()
      ..style = PaintingStyle.fill
      ..color = Colors.grey
      ..isAntiAlias = true;

    final rows = _height / gridHeight;
    final cols = _width / gridWidth;

    for (int r = 0; r < rows; r++) {
      final y = r * gridHeight;
      final p1 = Offset(0, y);
      final p2 = Offset(_width, y);

      canvas.drawLine(p1, p2, paint);
    }

    for (int c = 0; c < cols; c++) {
      final x = c * gridWidth;
      final p1 = Offset(x, 0);
      final p2 = Offset(x, _height);

      canvas.drawLine(p1, p2, paint);
    }
  }

  void _drawNodes(Canvas canvas) {
    var paint = Paint()
      ..style = PaintingStyle.fill
      ..color = Colors.amber
      ..isAntiAlias = true;

    const textStyle = TextStyle(
      color: Colors.black,
      fontSize: 18,
    );

    const radius = 30.0;
    for (int i = 0; i < nodeList.length; i++) {
      final c = Offset(nodeList[i].x, nodeList[i].y);
      canvas.drawCircle(c, radius, paint);
      _drawText(
          canvas, nodeList[i].x, nodeList[i].y, nodeList[i].name, textStyle);
    }
  }

  void _drawText(Canvas canvas, centerX, centerY, text, style) {
    final textSpan = TextSpan(
      text: text,
      style: style,
    );

    final textPainter = TextPainter()
      ..text = textSpan
      ..textDirection = TextDirection.ltr
      ..textAlign = TextAlign.center
      ..layout();

    final xCenter = (centerX - textPainter.width / 2);
    final yCenter = (centerY - textPainter.height / 2);
    final offset = Offset(xCenter, yCenter);

    textPainter.paint(canvas, offset);
  }

  @override
  void paint(Canvas canvas, Size size) {
    _width = size.width;
    _height = size.height;

    _drawBackground(canvas);
    _drawGrid(canvas);
    _drawNodes(canvas);
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => false;
}
</pre>
