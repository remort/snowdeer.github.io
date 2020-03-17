---
layout: post
title: Future에 Timeout 적용하기
category: C++
tag: [C++]
---
# Future에 Timeout 적용

future에 Timeout을 적용하는 예제입니다. 
ROS 2.0 관련 코드라 직접적으로 동작은 되지 않겠지만, `wait_until()`을 이용해서 특정 시간을 기다리는 점에서는 사용방법이 동일합니다.

<pre class="prettyprint">
thread t1([client]() {
    while (true) {
      auto a = 0;
      auto b = 0;

      cout << "Input two number: " << endl;
      cin >> a >> b;

      auto req = make_shared&lt;Add::Request&gt;();
      req->a = a;
      req->b = b;

      auto tp = std::chrono::system_clock::now() + std::chrono::seconds(3);
      auto request = client->async_send_request(req);
      auto status = request.wait_until(tp);

      if (status == future_status::ready) {
        auto resp = request.get();
        cout << "Sum : " << resp->sum << endl;
      } else {
        cout << "Timeout !!!" << endl;
      }
    }
  });
</pre>

