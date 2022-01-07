---
layout: post
title: Flutter Canvas 그리기 예제 - (5) 드래그로 Canvas 이동시키기
category: Flutter

tag: [Flutter]
---

# 드래그로 Canvas 이동시키기

## 실행 화면

화면을 드래그(Drag & Drop)해서 Canvas를 이동시키는 예제입니다.

![image](/assets/flutter/012.png)

Grid 그린 부분이 비어있는 것을 볼 수 있는데, 이 부분은 약간만 수정해주면 되는 부분이라 아래 예제 다음에 다루도록 하겠습니다.

## 예제 코드

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

  var offsetX = 0.0;
  var offsetY = 0.0;

  var preX = 0.0;
  var preY = 0.0;

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
        onPanDown: (detail) {
          preX = detail.localPosition.dx;
          preY = detail.localPosition.dy;
        },
        onPanUpdate: (detail) {
          setState(() {
            final dx = detail.localPosition.dx - preX;
            final dy = detail.localPosition.dy - preY;

            offsetX = offsetX + dx;
            offsetY = offsetY + dy;

            preX = detail.localPosition.dx;
            preY = detail.localPosition.dy;
          });
        },
        child: CustomPaint(
          child: Container(),
          painter: DraggablePainter(nodeList, offsetX, offsetY),
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

  final offsetX;
  final offsetY;
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

  void _drawCanvas(Canvas canvas) {
    canvas.save();
    canvas.translate(offsetX, offsetY);

    _drawBackground(canvas);
    _drawGrid(canvas);
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

## 배경은 유지하고 오브젝트만 이동시키기

아래 예제와 같이 배경 그리는 부분과 Grid 그리는 부분을 `canvas.save()` 바깥 쪽으로 빼주면 됩니다.

<pre class="prettyprint">
void _drawCanvas(Canvas canvas) {
    _drawBackground(canvas);
    _drawGrid(canvas);
    
    canvas.save();
    canvas.translate(offsetX, offsetY);

    _drawNodes(canvas);

    canvas.restore();
  }
</pre>

다만 이럴 경우 오브젝트는 이동되지만, Grid의 눈금은 스크롤 되지 않는 문제가 있습니다. 따라서 Grid는 화면의 `offset`을 이용해서 계산해서 그려주는 편이 낫습니다.

## 수정된 Grid 그리는 함수

<pre class="prettyprint">
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
</pre>

## 수정된 결과 화면

![image](/assets/flutter/013.png)