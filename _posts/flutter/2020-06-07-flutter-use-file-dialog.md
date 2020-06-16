---
layout: post
title: Flutter openFile, saveFile Dialog 사용
category: Flutter

tag: [Flutter]
---

# openFile, saveFile Dialog 사용 예제

먼저 `pubspec.yaml` 파일에 다음 항목을 추가해줍니다.

<br>

## pubspec.yaml

<pre class="prettyprint">
dependencies:
  file_chooser: ^0.1.2
  path_provider: ^1.6.10
  path_provider_macos: ^0.0.4+3
</pre>

여기서 `file_chooser`는 구글에서 만든 파일을 선택할 수 있는 인터페이스를 제공해주는 라이브러리이며, 
`path_provider`는 `Documents`와 같은 특정 디렉토리를 쉽게 찾을 수 있도록 함수를 제공해주는 라이브러리입니다.

여기서 테스트하는 App은 MacOS 버전이기 때문에 `macos/Runner/DebugProfile.entitlements` 파일에 다음 Permission도 추가해줍니다.

<br>

## macos/Runner/DebugProfile.entitlements

<pre class="prettyprint">
&lt;dict&gt;
  ...
	&lt;key&gt;com.apple.security.files.user-selected.read-write&lt;/key&gt;
    &lt;true/&gt;
&lt;/dict&gt;
</pre>

<br>

## main.dart

<pre class="prettyprint">
import 'dart:io';

import 'package:file_chooser/file_chooser.dart';
import 'package:flutter/material.dart';
import 'package:path_provider/path_provider.dart';

void main() {
  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      theme: ThemeData(
        primarySwatch: Colors.blue,
        visualDensity: VisualDensity.adaptivePlatformDensity,
      ),
      home: Scaffold(
        appBar: AppBar(
          title: Text('File Dialog Example'),
        ),
        body: SnowDeerExampleWidget(),
      ),
    );
  }
}

class SnowDeerExampleWidget extends StatefulWidget {
  @override
  State createState() => SnowDeerExampleWidgetState();
}

class SnowDeerExampleWidgetState extends State&lt;SnowDeerExampleWidget&gt; {
  void showOpenDialog() async {
    String initDirectory;
    if (Platform.isMacOS || Platform.isWindows) {
      initDirectory = (await getApplicationDocumentsDirectory()).path;
    }

    showOpenPanel(
      allowsMultipleSelection: true,
      initialDirectory: initDirectory,
    ).then((value) {
      final paths = value.paths;

      for (int i = 0; i < paths.length; i++) {
        final path = paths[i];
        print('- path: $path');
      }
    });
  }

  void showSaveDialog() {
    showSavePanel().then((value) {
      final paths = value.paths;

      for (int i = 0; i < paths.length; i++) {
        final path = paths[i];
        print('- path: $path');
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      child: Center(
        child: Column(
          children: [
            RaisedButton(
              child: Text('Show open dialog'),
              onPressed: () {
                showOpenDialog();
              },
            ),
            RaisedButton(
              child: Text('Show save dialog'),
              onPressed: () {
                showSaveDialog();
              },
            ),
          ],
        ),
      ),
    );
  }
}
</pre>