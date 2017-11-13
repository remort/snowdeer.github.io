---
layout: post
title: 강화 학습 - QLearning 예제(C++)
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝]
---

강화 학습(Reinforcement Learning)은 일련의 행동 후에 보상이나 평가가 주어질 때 사용할 수 있는 학습 방법입니다.

여기서는 강화 학습 중 `Q-Learning` 방법에 대해서 C++ 예제를 살펴보도록 하겠습니다.

`Q Value`는 어떤 상태에서 취해야 할 각각의 행동들에 대한 지표가 되는 수치입니다.

무작위 행동을 하면서 특정 보상에 도달한 행동에 대해서는 적절한 보상을 해주고 보상에 따라 `Q Value`를 업데이트 해주면서 결국 정답에 가까운 행동을 할 수 있도록 자연스럽게 유도하는 학습 방법입니다.

여기서는 Q-Learning 방법을 이용하여 미로 찾기를 하는 강화 학습 코드를 살펴보도록 하겠습니다.

![Image](/assets/2017-11-02-qlearning-using-cpp/01.jpg) 

위 그림과 같은 미로가 있으며, 가장 마지막 노드인 `s14`에 도착하면 보상을 주도록 했습니다.

# C++ 예제 코드

<pre class="prettyprint">
#include &lt;stdio.h&gt;
#include &lt;stdlib.h&gt;
#include &lt;time.h&gt;

static const int NODE_COUNT = 15;
static const double EPSILON = 0.3f;
static const int LEARNING_COUNT = 1000;   // 학습 횟수
static const double GAMMA = 0.9f;         // 할인율
static const double ALPHA = 0.1;          // 학습 계수

int getRandom(int max) {
  return rand() % (max + 1);
}

double getRandom() {
  return (double)rand() / RAND_MAX;
}

void printQTable(int qtable[NODE_COUNT]) {
  for (int i = 1; i < NODE_COUNT; i++) {
    printf("%d\t", qtable[i]);
  }
  printf("\n");
}

void initQTable(int qtable[NODE_COUNT]) {
  for (int i = 0; i < NODE_COUNT; i++) {
    qtable[i] = getRandom(100);
  }
  printQTable(qtable);
}

int getNextNode(int pos, int qtable[NODE_COUNT]) {
  int left = 2 * pos + 1;
  int right = 2 * (pos + 1);

  int nextNode;
  if (getRandom() < EPSILON) {
    if (getRandom(1) == 0) {
      nextNode = left;
    }
    else {
      nextNode = right;
    }
  }
  else {
    if (qtable[left] > qtable[right]) {
      nextNode = left;
    }
    else {
      nextNode = right;
    }
  }

  return nextNode;
}

int updateQTable(int pos, int qtable[NODE_COUNT]) {
  int left = 2 * pos + 1;
  int right = 2 * (pos + 1);

  int qvalue = 0;
  int qmax;

  if (pos > 6) {
    if (pos == 14) {
      qvalue = qtable[pos] + ALPHA * (1000 - qtable[pos]);
    }
    else {
      qvalue = qtable[pos];
    }
  }
  else {
    if (qtable[left] > qtable[right]) {
      qmax = qtable[left];
    }
    else {
      qmax = qtable[right];

    }
    qvalue = qtable[pos] + ALPHA * (GAMMA * qmax - qtable[pos]);
  }

  return qvalue;
}

int main() {
  srand(time(NULL));

  int nodeId;
  int qtable[NODE_COUNT];

  initQTable(qtable);

  for (int i = 0; i < LEARNING_COUNT; i++) {
    nodeId = 0;
    for (int j = 0; j < 3; j++) {
      nodeId = getNextNode(nodeId, qtable);

      qtable[nodeId] = updateQTable(nodeId, qtable);
    }

    printQTable(qtable);
  }
  return 0;
}
</pre>