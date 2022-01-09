---
layout: post
title: Flutter Canvas 그리기 예제 - (8) Node, Edge는 Model로 분리
category: Flutter

tag: [Flutter]
---

# Node, Edge는 Model로 분리

## drawing_model.dart

<pre class="prettypriint">
import 'dart:math';

import 'package:snowdeer_canvas_example/drag_and_drop/edge.dart';
import 'package:snowdeer_canvas_example/drag_and_drop/node.dart';

class DrawingModel {
  final List&lt;Node&gt; _nodeList = List.empty(growable: true);
  final List&lt;Edge&gt; _edgeList = List.empty(growable: true);

  DrawingModel() {
    _nodeList.clear();
    _edgeList.clear();
  }

  void addNode(Node node) {
    _nodeList.add(node);
  }

  void addEdge(Edge edge) {
    _edgeList.add(edge);
  }

  Node getNode(x, y) {
    const radius = 30.0;
    for (final node in _nodeList) {
      final distance =
          sqrt((node.x - x) * (node.x - x) + (node.y - y) * (node.y - y));
      if (distance <= radius) {
        return node;
      }
    }
    return null;
  }

  List getNodeList() {
    return _nodeList;
  }

  List getEdgeList() {
    return _edgeList;
  }
}
</pre>

## draggable_painter.dart

<pre class="prettypriint">
import 'dart:math';

import 'package:arrow_path/arrow_path.dart';
import 'package:flutter/material.dart';

import 'drawing_model.dart';

class DraggablePainter extends CustomPainter {
  static const gridWidth = 50.0;
  static const gridHeight = 50.0;
  static const radius = 30.0;

  var _width = 0.0;
  var _height = 0.0;

  final double offsetX;
  final double offsetY;

  final DrawingModel model;

  DraggablePainter(this.model, this.offsetX, this.offsetY);

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

  Offset _getCenterPosOfNode(nodeId) {
    for (final node in model.getNodeList()) {
      if (nodeId == node.id) {
        return Offset(node.x, node.y);
      }
    }
    return null;
  }

  void _drawEdges(Canvas canvas) {
    final paint = Paint()
      ..style = PaintingStyle.stroke
      ..color = Colors.black
      ..strokeWidth = 2
      ..isAntiAlias = true;

    for (final edge in model.getEdgeList()) {
      final fromPos = _getCenterPosOfNode(edge.fromId);
      final toPos = _getCenterPosOfNode(edge.toId);

      if ((fromPos != null) && (toPos != null)) {
        final distance =
            Offset(toPos.dx - fromPos.dx, toPos.dy - fromPos.dy).distance -
                radius;
        final theta = atan2((toPos.dy - fromPos.dy), (toPos.dx - fromPos.dx));
        final targetX = fromPos.dx + distance * cos(theta);
        final targetY = fromPos.dy + distance * sin(theta);

        var path = Path();
        path.moveTo(fromPos.dx, fromPos.dy);
        path.lineTo(targetX, targetY);
        path = ArrowPath.make(path: path);
        canvas.drawPath(path, paint);
      }
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

    for (final node in model.getNodeList()) {
      final c = Offset(node.x, node.y);
      canvas.drawCircle(c, radius, paint);
      _drawText(canvas, node.x, node.y, node.name, textStyle);
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

    _drawEdges(canvas);
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

<pre class="prettypriint">
import 'package:flutter/material.dart';
import 'package:snowdeer_canvas_example/drag_and_drop/drawing_model.dart';

import 'draggable_painter.dart';
import 'edge.dart';
import 'node.dart';

class DraggableObjectPage extends StatefulWidget {
  const DraggableObjectPage({Key key}) : super(key: key);

  @override
  State createState() => DraggableObjectPageState();
}

class DraggableObjectPageState extends State&lt;DraggableObjectPage&gt; {
  final model = DrawingModel();

  var offsetX = 0.0;
  var offsetY = 0.0;

  var preX = 0.0;
  var preY = 0.0;

  Node currentNode;

  DraggableObjectPageState() {
    initNodes();
    initEdges();
  }

  void initNodes() {
    model.addNode(Node(1, "Node\n1", 150.0, 210.0));
    model.addNode(Node(2, "Node\n2", 220.0, 40.0));
    model.addNode(Node(3, "Node\n3", 440.0, 240.0));
    model.addNode(Node(4, "Node\n4", 640.0, 150.0));
    model.addNode(Node(5, "Node\n5", 480.0, 350.0));
  }

  void initEdges() {
    model.addEdge(Edge(1, 2));
    model.addEdge(Edge(3, 2));
    model.addEdge(Edge(5, 3));
    model.addEdge(Edge(5, 4));
  }

  void _handlePanDown(details) {
    final x = details.localPosition.dx;
    final y = details.localPosition.dy;

    final node = model.getNode(x - offsetX, y - offsetY);
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

  void _handleLongPressStart(details) {
    final x = details.localPosition.dx;
    final y = details.localPosition.dy;

    final node = model.getNode(x - offsetX, y - offsetY);

    if (node == null) {
      return;
    }

    showDialog(
      context: context,
      builder: (BuildContext context) {
        // return object of type Dialog
        return AlertDialog(
          title: Text(node.name),
          content: const Text("Edit node."),
          actions: [
            TextButton(
              child: const Text("Ok"),
              onPressed: () {
                setState(() {
                  node.name = node.name + '+';
                });
                Navigator.pop(context);
              },
            ),
          ],
        );
      },
    );
  }

  void _handleLongPressMoveUpdate(details) {}

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: GestureDetector(
        onPanDown: _handlePanDown,
        onPanUpdate: _handlePanUpdate,
        onLongPressStart: _handleLongPressStart,
        onLongPressMoveUpdate: _handleLongPressMoveUpdate,
        child: CustomPaint(
          child: Container(),
          painter: DraggablePainter(model, offsetX, offsetY),
        ),
      ),
    );
  }
}
</pre>