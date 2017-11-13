---
layout: post
title: 귀납적 학습(Inductive Learning) 예제(C++)
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝]
---

귀납적 학습(Inductive Learning)은 구체적인 데이터 값들을 기반으로 특정 규칙이나 지식을 학습하는 것을 말합니다.

간단한 예제를 살펴보도록 하겠습니다. 다음과 같은 데이터가 있다고 가정합니다.

A | B | C | D | E | F | G | H | I | J | Value
--- | --- | --- | --- | --- | --- | --- | --- | --- | --- | ---
1 | 0 | 0 | 0 | 0 | 0 | 1 | 0 | 0 | 1 | 1
0 | 1 | 0 | 1 | 0 | 1 | 1 | 1 | 0 | 1 | 1
0 | 1 | 0 | 0 | 0 | 1 | 1 | 0 | 1 | 0 | 0
1 | 0 | 0 | 1 | 1 | 0 | 1 | 0 | 0 | 1 | 1
1 | 0 | 0 | 1 | 1 | 0 | 1 | 1 | 1 | 1 | 1

A부터 J까지의 값의 패턴에 따라 `Value` 값이 결정이 된다고 할 때, 과거 기록들을 학습하면 향후 A부터 J까지의 값이 주어지면 `Value` 값을 추측할 수 있습니다.

다음 코드는 `Value` 값을 예측할 수 있는 A부터 J까지의 값들의 패턴을 추출하는 예제입니다.

# C++ 예제 코드

<pre class="prettyprint">
#include &lt;stdio.h&gt;
#include &lt;stdlib.h&gt;
#include &lt;time.h&gt;

static const int DATA_COUNT = 30;
static const int COMPANY_COUNT = 10;

// File에서 데이터를 읽어들임
void loadData(int data[DATA_COUNT][COMPANY_COUNT], int value[DATA_COUNT]) {
  freopen("data.txt", "r", stdin);
  setbuf(stdout, NULL);

  for (int i = 0; i < DATA_COUNT; i++) {
    for (int j = 0; j < COMPANY_COUNT; j++) {
      scanf("%d", &data[i][j]);
    }
    scanf("%d", &value[i]);
  }
}

// 초기 AnswerSheet는 랜덤값으로 설정
void createRandomAnswerSheet(int answer[COMPANY_COUNT]) {
  for (int i = 0; i < COMPANY_COUNT; i++) {
    answer[i] = (rand() % 3);
  }
}

// 점수 계산
int getScore(int data[DATA_COUNT][COMPANY_COUNT], int value[DATA_COUNT], int answer[COMPANY_COUNT]) {
  int score = 0;

  for (int i = 0; i < DATA_COUNT; i++) {
    int point = 0;

    for (int j = 0; j < COMPANY_COUNT; j++) {
      if (answer[j] == 2) {
        point++;
      } else if (answer[j] == data[i][j]) {
        point++;
      }

      if ((point == COMPANY_COUNT) && (value[i] == 1)) {
        score++;
      } else if ((point != COMPANY_COUNT) && (value[i] == 0)) {
        score++;
      }
    }
  }

  return score;
}

int main() {
  srand(time(NULL));

  int score;
  int data[DATA_COUNT][COMPANY_COUNT];
  int value[DATA_COUNT];
  int answer[COMPANY_COUNT];

  int bestScore = 0;
  int bestAnswer[COMPANY_COUNT];

  loadData(data, value);

  for (int i = 0; i < 100000; i++) {

    createRandomAnswerSheet(answer);

    score = getScore(data, value, answer);

    if (score > bestScore) {
      for (int j = 0; j < COMPANY_COUNT; j++) {
        bestAnswer[j] = answer[j];
      }
      bestScore = score;

      for (int j = 0; j < COMPANY_COUNT; j++) {
        printf("%1d ", bestAnswer[j]);
      }
      printf(" --> score: %d\n", bestScore);
    }
  }

  printf("\n<Best Answer>\n");
  for (int j = 0; j < COMPANY_COUNT; j++) {
    printf("%1d ", bestAnswer[j]);
  }
  printf(" --> score: %d\n", bestScore);

  return 0;
}
</pre>

코드를 간략하게 설명하면, 

1. 무작위로 랜덤 패턴을 생성(createRandomAnswerSheet)  
이 때 패턴은 숫자 0, 1, 2를 사용(2는 와일드 카드)
2. 생성된 랜덤 패턴과 과거 데이터 기록과 유사성 비교  
유사성이 있으면 score + 1  
3. 가장 높은 score를 획득한 AnswerSheet 추출

과 같습니다.

과거 데이터 기록의 패턴은 0 또는 1의 값을 가지지만 항상 일정한 규칙을 갖고 있는 것은 아니기 때문에 와일드카드(Wildcard)인 2라는 값을 사용합니다.

랜덤 패턴과 과거 데이터 기록 유사성은 A부터 J까지 각 값들을 비교해서 유사성이 있으면 `point` 변수를 1 씩 증가시킵니다.

그리고 해당 패턴과 일치하면서 `Value` 값이 `1`이 되거나, 해당 패턴과 일치하지 않으면서 `Value` 값이 `0`이 되는 경우에 `score` 값을 1 증가시켜줍니다.