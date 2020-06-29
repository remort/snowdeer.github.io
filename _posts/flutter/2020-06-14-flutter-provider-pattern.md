---
layout: post
title: Flutter Provider 패턴
category: Flutter

tag: [Flutter]
---

## Provider Pattern

여기서 `Provider`는 생산자/소비자 패턴에서의 생산자를 의미합니다. `React`의 `Redux`와 비슷한 개념이며 
Flutter 기본 샘플 프로그램인 카운터 프로그램에 Provider 패턴을 적용해보도록 하겠습니다.

<br>

### 원래 코드

<pre class="prettyprint">
import 'package:flutter/material.dart';

void main() => runApp(MyApp());

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Provider Pattern Example',
      theme: ThemeData(
        primarySwatch: Colors.indigo,
        visualDensity: VisualDensity.adaptivePlatformDensity,
      ),
      home: MyHomePage(title: 'Provider Pattern Example'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  MyHomePage({Key key, this.title}) : super(key: key);

  final String title;

  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class _MyHomePageState extends State&lt;MyHomePage&gt; {
  int _counter = 0;

  void _incrementCounter() {
    setState(() {
      _counter++;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(widget.title),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: &lt;Widget&gt;[
            Text(
              'You have pushed the button this many times:',
            ),
            Text(
              '$_counter',
              style: Theme.of(context).textTheme.headline4,
            ),
          ],
        ),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: _incrementCounter,
        tooltip: 'Increment',
        child: Icon(Icons.add),
      ),
    );
  }
}
</pre>

<br>

## Provider 패턴 적용 맛보기

### pubspec.yaml

<pre class="prettyprint">
dependencies:
  flutter:
    sdk: flutter
  provider: ^4.1.3
</pre>

<br>

`counter.dart` 파일을 다음과 같이 작성합니다.

### counter.dart

<pre class="prettyprint">
import 'package:flutter/material.dart';

class Counter with ChangeNotifier {
  int _count = 0;

  int get getCount => _count;

  void incrementCounter() {
    _count = _count + 1;
    notifyListeners();
  }
}
</pre>

<br>

### main.dart

여기서 눈여겨 볼 부분은 `MyApp` 위젯에서 `ChangeNotifierProivder`를 사용한 것과 `_MyHomePageState` 클래스에서
Provider를 연동한 부분입니다.

그리고 `MyHomePage` 위젯이 `StatefulWidget`에서 `StatelessWidget`가 되었습니다. 더 이상 `_count`라는 변수를 
가질 필요가 없기 때문에 State를 가질 필요가 없어졌습니다. 기존에는 화면을 갱신하기 위해서 `setState()` 메소드를 호출했었지만, 
Provider를 사용하면 `notifyListeners()` 메소드를 통해 해당 데이터를 사용하는 모든 위젯들을 갱신할 수 있습니다.

<pre class="prettyprint">
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import 'counter.dart';

void main() => runApp(MyApp());

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Provider Pattern Example',
      theme: ThemeData(
        primarySwatch: Colors.indigo,
        visualDensity: VisualDensity.adaptivePlatformDensity,
      ),
      home: ChangeNotifierProvider&lt;Counter&gt;(
        builder: (context) => Counter(),
        child: MyHomePage(title: 'Provider Pattern Example'),
      ),
    );
  }
}

class MyHomePage extends StatelessWidget {
  final String title;

  MyHomePage({this.title});

  @override
  Widget build(BuildContext context) {
    final consumer = Provider.of&lt;Counter&gt;(context);

    return Scaffold(
      appBar: AppBar(
        title: Text(title),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text(
              'You have pushed the button this many times:',
            ),
            Text(
              '${consumer.getCount.toString()}',
              style: Theme.of(context).textTheme.headline4,
            ),
          ],
        ),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () {
          Provider.of&lt;Counter&gt;(context, listen: false).increment();
        },
        tooltip: 'Increment',
        child: Icon(Icons.add),
      ),
    );
  }
}
</pre>

<br>

## Provider 패턴의 장점

* Provider 패턴은 데이터 공유와 로직의 분리를 좀 더 쉽게 할 수 있게 해줍니다. 로직이 분리되면 코드가 좀 더 깔끔해줍니다.
* Provider를 이용하면 각 위젯을 `StatefulWidget` 대신 `StatelessWidget`로 변경할 수 있어서 위젯의 부담이 줄어듭니다. 각 위젯이 상태값을 가지지 않는 다는 것은
그만큼 함수형 구현(동일한 입력값에 대해서는 항상 동일한 결과가 출력됨)에 더 근접하는 것이며, 보다 더 안정적인 코드를 작성할 수 있습니다. 
* 앱을 구성하는 여러 화면에서 참조해야 하는 공통의 데이터를 관리할 때 유용합니다.

<br>

## Provider 패턴의 구조

위에서 작성했던 예제를 조금 더 구조화해서 설명하도록 하겠습니다.

Provider는 기본적으로 `child`로 위젯(Widget)을 가집니다.
위에서 작성했던 코드에는 다음과 같은 형태로 사용하고 있습니다.

<pre class="prettyprint">
ChangeNotifierProvider&lt;Counter&gt;(
  builder: (context) => Counter(),
  child: MyHomePage(title: 'Provider Pattern Example'),
)
</pre>

그리고 공유하는 데이터는 `ChangeNotifier`를 `with`로 구현해줍니다.
`notifyListeners()` 메소드를 통해 해당 데이터를 사용하는 위젯들에게 변경 이벤트를 전송해줍니다.

<pre class="prettyprint">
import 'package:flutter/material.dart';

class Counter with ChangeNotifier {
  int _count = 0;

  int get getCount => _count;

  void incrementCounter() {
    _count = _count + 1;
    notifyListeners();
  }
}
</pre>

위의 예제에서는 여러 개의 Provider를 사용했지만, 아래의 예제처럼 여러 개의 Proivder를 사용할 수도 있습니다.
(위의 예제보다는 좀 더 정돈된 코드입니다.)

<pre class="prettyprint">
class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MultiProvider(
      providers: [
        ChangeNotifierProvider(
          create: (context) => Counter(),
        ),
      ],
      child: MaterialApp(
        title: 'Provider Pattern Example',
        theme: ThemeData(
          primarySwatch: Colors.indigo,
          visualDensity: VisualDensity.adaptivePlatformDensity,
        ),
        home: MyHomePage(title: 'Provider Pattern Example'),
      ),
    );
  }
}
</pre>

Provider의 데이터는 아래와 같이 `Provider.of`를 이용해서 가져올 수 있습니다.

<pre class="prettyprint">
final consumer = Provider.of&lt;Counter&gt;(context);
</pre>

다만, 위의 코드처럼 구현하면 위젯의 재빌드가 자주 발생하게 되는데 이 경우는 `Provider.of` 대신 
`Consumer`나 `Selector`를 사용하고 해당 데이터를 사용하는 부분으로 호출 코드를 이동시켜주면 됩니다.

<pre class="prettyprint">
Consumer&lt;Counter&gt;(
  builder: (context, counter, child) {
    return Text(
      '${counter.getCount.toString()}',
      style: Theme.of(context).textTheme.headline4,
    );
  },
),
</pre>

전체 위젯 코드는 다음과 같습니다.

<pre class="prettyprint">
class MyHomePage extends StatelessWidget {
  final String title;

  MyHomePage({this.title});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(title),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text(
              'You have pushed the button this many times:',
            ),

            Consumer&lt;Counter&gt;(
              builder: (context, counter, child) {
                return Text(
                  '${counter.getCount.toString()}',
                  style: Theme.of(context).textTheme.headline4,
                );
              },
            ),
          ],
        ),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () {
          Provider.of&lt;Counter&gt;(context, listen: false).increment();
        },
        tooltip: 'Increment',
        child: Icon(Icons.add),
      ),
    );
  }
}
</pre>
