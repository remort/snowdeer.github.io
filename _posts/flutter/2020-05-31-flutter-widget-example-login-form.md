---
layout: post
title: Flutter Widget을 이용해 로그인 화면 배치해보기
category: Flutter

tag: [Flutter]
---

# 로그인 화면 배치

![Image](/assets/flutter/006_login_form.png)

# main.dart

<pre class="prettyprint">
import 'package:flutter/material.dart';

void main() => runApp(SnowDeerExample());

class SnowDeerExample extends StatelessWidget {
  static const title = 'SnowDeer\'s Login Example';

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
        title: title,
        debugShowCheckedModeBanner: false,
        home: Scaffold(
//            appBar: AppBar(title: Text(title)),
            body: Container(
              margin: EdgeInsets.symmetric(horizontal: 40),
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                crossAxisAlignment: CrossAxisAlignment.center,
                children: [
                  Container(
                    child: Image.asset(
                      'assets/images/snowdeer.png',
                      width: 160,
                      height: 160,
                    ),
                  ),
                  Container(
                      margin: EdgeInsets.only(top: 20),
                      child: TextFormField(
                        keyboardType: TextInputType.emailAddress,
                        initialValue: 'snowdeer0314@gmail.com',
                        decoration: InputDecoration(
                          border: OutlineInputBorder(),
                        ),
                      )),
                  Container(
                      margin: EdgeInsets.only(top: 8),
                      child: TextFormField(
                        obscureText: true,
                        decoration: InputDecoration(
                          border: OutlineInputBorder(),
                          hintText: 'Input password',
                        ),
                      )),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      RaisedButton(
                        child: Text("Login"),
                        onPressed: () => {print("LoginButton is Clicked.")},
                      ),
                      SizedBox(
                        width: 8,
                      ),
                      RaisedButton(
                        child: Text("Cancel"),
                        onPressed: () => {print("CancelButton is Clicked.")},
                      )
                    ],
                  )
                ],
              ),
            )));
  }
}
</pre>