---
layout: post
title: Introduction to widgets - (2)
category: Flutter

tag: [Flutter]
---

## 제스처 핸들링

`GestureDetector` 위젯을 이용하면 다양한 사용자 제스처 인터랙션을 핸들링할 수 있습니다.

<pre class="prettyprint">
class MyButton extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: () {
        print('MyButton was tapped!');
      },
      child: Container(
        height: 36.0,
        padding: const EdgeInsets.all(8.0),
        margin: const EdgeInsets.symmetric(horizontal: 8.0),
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(5.0),
          color: Colors.lightGreen[500],
        ),
        child: Center(
          child: Text('Engage'),
        ),
      ),
    );
  }
}
</pre>

`GestureDetector` 위젯은 화면을 렌더링하는 요소는 갖고 있지 않지만, 사용자의 인터랙션을 감지할 수 있는 위젯입니다. `child`의 `Container`를 
터치(`onTap`)하면 미리 정의되어 있는 콜백(callback) 함수를 호출합니다.

<br>

## 사용자 입력에 반응하는 위젯 만들기(StatefulWidget)

이제 사용자 입력에 반응하는 위젯을 만들어보겠습니다. `StatelessWidget`의 경우는 모든 속성 값을 부모로부터 전달받으며 그 값은 `final` 멤버로 
저장하게 됩니다. 즉, 더 이상 상태 변환이 없는 위젯이며 새로 갱신될 필요도 없습니다.

`StatefulWidget`는 상태 값을 가지며, 그 값이 바뀔 때 화면을 갱신할 필요가 있으면 새로 렌더링을 하는 위젯입니다. 
관리해야 할 상태가 있고 내부적으로 `State`를 인스턴스를 가지게 되며, `StatelessWidget`에 비해 구조가 좀 더 복잡합니다. 

<pre class="prettyprint">
class Counter extends StatefulWidget {
  // This class is the configuration for the state. It holds the
  // values (in this case nothing) provided by the parent and used
  // by the build  method of the State. Fields in a Widget
  // subclass are always marked "final".

  @override
  _CounterState createState() => _CounterState();
}

class _CounterState extends State&lt;Counter&gt; {
  int _counter = 0;

  void _increment() {
    setState(() {
      // This call to setState tells the Flutter framework that
      // something has changed in this State, which causes it to rerun
      // the build method below so that the display can reflect the
      // updated values. If you change _counter without calling
      // setState(), then the build method won't be called again,
      // and so nothing would appear to happen.
      _counter++;
    });
  }

  @override
  Widget build(BuildContext context) {
    // This method is rerun every time setState is called,
    // for instance, as done by the _increment method above.
    // The Flutter framework has been optimized to make rerunning
    // build methods fast, so that you can just rebuild anything that
    // needs updating rather than having to individually change
    // instances of widgets.
    return Row(
      children: &lt;Widget&gt;[
        RaisedButton(
          onPressed: _increment,
          child: Text('Increment'),
        ),
        Text('Count: $_counter'),
      ],
    );
  }
}
</pre>

위 예제에서 `RaisedButton`을 누르게 되면 `onPressed()` 메소드의 콜백으로 등록된 `_increment()` 메소드가 호출됩니다. `_increment()` 메소드 구현 내부에 있는
`setState()` 메소드를 통해 상태 변화를 위젯에 전달하게 되고, 해당 위젯은 화면을 새로 렌더링하게 됩니다.

`StatefulWidget`과 `State`는 서로 분리되어 있으며 서로 다른 라이프사이클(Lifecycle)을 가집니다. `Widget`은 화면에 뿌리기 위한 일시적인 오브젝트이며, `State`는 
그 상태 값을 유지하고 있는 영구적인 오브젝트라고 볼 수 있습니다.

위 예제는 사용자의 입력에 따라 해당 클래스내 `build()` 메소드로 결과를 바로 렌더링하도록 되어 있는 간단한 예제이지만, 좀 더 복잡한 프로그램에서는 
좀 더 구조적인 구성이 필요하게 됩니다. Flutter에서는 콜백 형태로 전달받는 이벤트의 경우 위젯 트리의 위쪽 방향으로 전달되며, 현재 상태는 아래쪽의 `StatelessWidget`으로 전달되어 화면에 출력하는 형태로 되어 있습니다. 

이러한 흐름은 다음 예제에서 볼 수 있습니다.

<pre class="prettyprint">
class CounterDisplay extends StatelessWidget {
  CounterDisplay({this.count});

  final int count;

  @override
  Widget build(BuildContext context) {
    return Text('Count: $count');
  }
}

class CounterIncrementor extends StatelessWidget {
  CounterIncrementor({this.onPressed});

  final VoidCallback onPressed;

  @override
  Widget build(BuildContext context) {
    return RaisedButton(
      onPressed: onPressed,
      child: Text('Increment'),
    );
  }
}

class Counter extends StatefulWidget {
  @override
  _CounterState createState() => _CounterState();
}

class _CounterState extends State&lt;Counter&gt; {
  int _counter = 0;

  void _increment() {
    setState(() {
      ++_counter;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Row(children: &lt;Widget&gt;[
      CounterIncrementor(onPressed: _increment),
      CounterDisplay(count: _counter),
    ]);
  }
}
</pre>

위 예제에서 `_CounterState` 클래스는 `build()` 메소드를 이용해서 2개의 `StatelessWidget`를 출력하고 있습니다. 각 위젯들은 `CounterIncrementor`와 `CounterDisplay` 이며, `CounterIncrementor`는 `onPressed()` 메소드의 콜백 함수로 `_increment()` 메소드를 연결시켜 놓았고, `CounterDisplay` 위젯에 `_counter` 속성 값을 전달하고 있습니다.

