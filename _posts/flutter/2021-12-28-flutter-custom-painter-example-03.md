---
layout: post
title: Flutter Canvas 그리기 예제 - (3) Canvas내의 터치(마우스) 좌표 획득하는 방법
category: Flutter

tag: [Flutter]
---

# Canvas내의 터치(마우스) 좌표 획득하는 방법

## GestureDetector

`GestureDetector`를 이용해서 터치 좌표를 획득할 수 있습니다.

<pre class="prettyprint">
class GridPage extends StatelessWidget {
  const GridPage({Key key}) : super(key: key);

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
          painter: GridPainter(),
        ),
      ),
    );
  }
}
</pre>

## 전체 코드

<pre class="prettyprint">
import 'package:flutter/material.dart';

class GridPage extends StatelessWidget {
  const GridPage({Key key}) : super(key: key);

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
          painter: GridPainter(),
        ),
      ),
    );
  }
}

class GridPainter extends CustomPainter {
  static const gridWidth = 50.0;
  static const gridHeight = 50.0;

  var _width = 0.0;
  var _height = 0.0;

  void _drawBackground(Canvas canvas) {
    var paint = Paint()
      ..style = PaintingStyle.fill
      ..color = Colors.lime
      ..isAntiAlias = true;

    Rect rect = Rect.fromLTWH(0, 0, _width, _height);
    canvas.drawRect(rect, paint);
  }

  void _drawGrid(Canvas canvas) {
    var paint = Paint()
      ..style = PaintingStyle.fill
      ..color = Colors.black38
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

  @override
  void paint(Canvas canvas, Size size) {
    _width = size.width;
    _height = size.height;

    _drawBackground(canvas);
    _drawGrid(canvas);
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => false;
}
</pre>