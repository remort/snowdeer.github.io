---
layout: post
title: Flutter Canvas 그리기 예제 - (6) 드래그로 Canvas 내 Object 이동시키기
category: Flutter

tag: [Flutter]
---

# 드래그로 Canvas 내 Object 이동시키기

## node.dart

<pre class="prettyprint">
class Node {
  String id;
  String name;
  double x;
  double y;

  Node(this.id, this.name, this.x, this.y);
}
</pre>

## draggable_painter.dart

<pre class="prettyprint">
import 'package:flutter/material.dart';

import 'node.dart';

class DraggablePainter extends CustomPainter {
  static const gridWidth = 50.0;
  static const gridHeight = 50.0;

  var _width = 0.0;
  var _height = 0.0;

  final double offsetX;
  final double offsetY;
  final List&lt;Node&gt; nodeList;

  DraggablePainter(this.nodeList, this.offsetX, this.offsetY);

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
      ..style = PaintingStyle.stroke
      ..color = Colors.grey
      ..isAntiAlias = true;

    final gridRect = Rect.fromLTWH(
        offsetX % gridWidth - gridWidth,
        offsetY % gridHeight - gridHeight,
        _width + gridWidth,
        _height + gridHeight);

    final rows = _height / gridHeight;
    final cols = _width / gridWidth;

    for (int r = -1; r <= rows; r++) {
      final y = r * gridHeight + gridRect.top;
      final p1 = Offset(gridRect.left, y);
      final p2 = Offset(gridRect.right, y);

      canvas.drawLine(p1, p2, paint);
    }

    for (int c = -1; c <= cols; c++) {
      final x = c * gridWidth + gridRect.left;
      final p1 = Offset(x, gridRect.top);
      final p2 = Offset(x, gridRect.bottom);

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
      fontSize: 14,
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

  void _drawCanvas(Canvas canvas) {
    _drawBackground(canvas);
    _drawGrid(canvas);

    canvas.save();
    canvas.translate(offsetX, offsetY);
    _drawNodes(canvas);

    canvas.restore();
  }

  @override
  void paint(Canvas canvas, Size size) {
    _width = size.width;
    _height = size.height;

    _drawCanvas(canvas);
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;
}
</pre>

## draggable_canvas.dart

<pre class="prettyprint">
import 'package:flutter/material.dart';
import 'node.dart';
import 'draggable_painter.dart';

class DraggableObjectPage extends StatefulWidget {
  const DraggableObjectPage({Key key}) : super(key: key);

  @override
  State createState() => DraggableObjectPageState();
}

class DraggableObjectPageState extends State&lt;DraggableObjectPage&gt; {
  final List&lt;Node&gt; nodeList = List.empty(growable: true);

  var offsetX = 0.0;
  var offsetY = 0.0;

  var preX = 0.0;
  var preY = 0.0;

  Node currentNode;

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

  Node getNode(x, y) {
    const radius = 30.0;
    for (final node in nodeList) {
      final c = Offset(node.x - x + offsetX, node.y - y + offsetY);
      if (c.distance <= radius) {
        return node;
      }
    }
    return null;
  }

  void _handlePanDown(details) {
    final x = details.localPosition.dx;
    final y = details.localPosition.dy;

    final node = getNode(x, y);
    if (node != null) {
      currentNode = node;
    } else {
      currentNode = null;
    }

    preX = x;
    preY = y;
  }

  void _handlePanUpdate(details) {
    final dx = details.localPosition.dx - preX;
    final dy = details.localPosition.dy - preY;

    if (currentNode != null) {
      setState(() {
        currentNode.x = currentNode.x + dx;
        currentNode.y = currentNode.y + dy;
      });
    } else {
      setState(() {
        offsetX = offsetX + dx;
        offsetY = offsetY + dy;
      });
    }

    preX = details.localPosition.dx;
    preY = details.localPosition.dy;
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: GestureDetector(
        onPanDown: _handlePanDown,
        onPanUpdate: _handlePanUpdate,
        child: CustomPaint(
          child: Container(),
          painter: DraggablePainter(nodeList, offsetX, offsetY),
        ),
      ),
    );
  }
}
</pre>