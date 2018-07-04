---
layout: post
title: chrono를 활용한 시간 측정
category: C++
tag: [C++]
---
# chrono 예제

<pre class="prettyprint">
#include <iostream>
#include <chrono>

using namespace std;
using namespace chrono;

const int MAX = 200000000;

long long a[MAX];

int main() {
  cout << "init ... " << endl;
  for (int i = 0; i < MAX; i++) {
    a[i] = i;
  }

  cout << "running... " << endl;

  system_clock::time_point start = system_clock::now();
  
  for (int i = 0; i < MAX; i++) {
    a[i] = a[i] * a[i];
  }
  system_clock::time_point end = system_clock::now();
  duration<double> sec = end - start;

  cout << "Time: " << duration_cast<std::chrono::milliseconds>(sec).count() << endl;

  return 0;
}
</pre>