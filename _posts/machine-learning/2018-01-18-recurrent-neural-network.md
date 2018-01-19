---
layout: post
title: RNN(Recurrent Neural Network)
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝]
---
# 순환 신경망

RNN(Recurrent Neural Network)는 시리(Siri), 빅스비(Bixby) 등 언어 인식에 많이 활용되고 있습니다. 사람의 언어를 이해한다는 것은 앞뒤 단어와 같이 과거에 입력된 데이터들의 관계를 고려해서 이해할 수 있다는 뜻입니다. 

순환 신경망은 여러 개의 데이터가 순서대로 입력되었을 때 앞서 입력받은 데이터를 잠시 기억해 놓는 방법입니다. 그리고 기억된 데이터의 중요도를 판별하여 별도의 가중치를 부여한 다음 다음 데이터로 넘어갑니다. 모든 입력 값에 대해 이 작업을 순서대로 실행하는데, 마치 다음 층을 가기 전에 같은 층을 계속해서 맴도는 것처럼 보이기 때문에 순환 신경망이라고 부릅니다.

![Image](/assets/machine-learning/043.png)

<br>

# 순환 신경망의 활용

* 음성 인식기
* 번역기
* 사진을 보고 캡션 추가하기
* 펜으로 필기하는 과정을 RNN으로 학습하여 필기를 텍스트로 인식하거나 텍스트를 필기체로 변환
* 음악 일부를 듣고 이어서 음악 재생하기

<br>

# 순환 신경망 예제

[Google Quick, Draw!](https://quickdraw.withgoogle.com/)

![Image](/assets/machine-learning/045.png)

사람이 그린 그림을 보고 구글에서 무엇을 그린 건지 맞추는 웹사이트입니다. 재미로 해볼만한데 꽤 괜찮은 성능을 보여줍니다.

<br>

# LSTM

LSTM(Long Short Term Memory)은 RNN의 성능을 더욱 개선하기 위해서 여러가지 연구가 계속되다가 나온 방법 중 하나입니다. 

RNN은 한 층 안에서 반복을 계속하기 때문에 경사각소실(Vanishing Gradient)이 발생하기 쉽고, LSTM은 이를 보안한 방법입니다. 반복되기 직전에 다음 층으로 기억된 값을 넘길지 안 넘길지 관리하는 단계를 하나 더 추가하는 방법입니다.

![Image](/assets/machine-learning/044.png)

![Image](/assets/machine-learning/046.png)