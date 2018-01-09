---
layout: post
title: Deep Learning 프레임워크별 특징 및 장단점
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝]
---
# Deep Learning Framework

딥러닝(Deep Learning)이 화두가 되다보니 수 많은 프레임워크(Framework)들이 생겨나거나 재조명받기 시작했습니다. 그리고 각 프레임워크의 선호도들도 트렌드에 따라 계속 바뀌고 있습니다.

![Image](/assets/machine-learning/012.jpg)
 
위 도표는 구글에서 조사한 딥러닝 프레임워크 선호도 순위입니다. Contribution, Issue, forks 횟수에 가중치를 곱해서 계산한 결과라 그냥 참고용으로 보면 될 것 같습니다. 하지만, 순위가 높은건 그만큼 많은 사람들이 관심을 가지고있고 이슈화되고 있다고 생각할 수 있을 것 같습니다.

<br>

## Theano 

[Theano](http://deeplearning.net/software/theano/)는 다차원 배열을 쉽게 다룰 수 있는 라이브러리입니다. 데이터 탐색에 적합해서 다른 라이브러리들과 연동하여 연구용으로 많이 사용되었습니다.

[Keras](https://github.com/keras-team/keras)를 비롯한 많은 딥러닝 라이브러리가 Theano를 기반으로 만들어졌습니다. 

Theano는 초창기 프레임워크다보니 많은 부분에서 단점이 존재합니다. 사용하기 복잡하며, 단일 GPU를 지원합니다. 대형 모델에 대해서는 컴파일 시간도 오래걸립니다.

<br>

## Torch7

[Torch](http://torch.ch/)는 [Lua](http://www.lua.org/) 언어 기반의 API를 제공하는 머신러닝 프레임워크입니다. 1990년대 초에 개발된 오래된 프레임워크 중 하나입니다.

작고 다양한 많은 모듈 조각으로 이루어져 있어 결합해서 자신만의 레이어를 만들고 GPU에서 실행하기 쉬운 장점이 있습니다. 또한 미리 학습되어진 모델들이 많이 존재하는 장점도 있지만 Python이나 Java 언어 개발자에게는 접근이 쉽지 않다는 단점이 있습니다.

2017년 1월에 Facebook에서 Python API 형태의 [Pytorch(https://github.com/pytorch/pytorch)]를 오픈 소스로 공개했습니다.

Pytorch는 가변 길이 입력과 출력을 처리할 수 있는 Dynamic Computation Graph를 지원합니다.

<br>

## Tensorflow

현재 가장 많은 개발자들이 선호하는 프레임워크 중 하나로 Google에서 만든 딥러닝용 프레임워크입니다. Theano를 대체하고자 만들었으며, [Tensorflow](https://www.tensorflow.org/) 개발진에 Theano 개발자들도 일부 포함되어 있었습니다. 

Tensorflow는 기본적으로 C++과 Python을 지원하고 있으며, 내부적으로는 전부 C/C++ 엔진으로 되어 있어 Python으로 개발하더라도 속도에 큰 차이가 없습니다.

Tensorflow는 딥러닝 외에도 강화학습(Reinforcement Learning) 등을 위한 알고리즘도 제공하고 있습니다.  

Theno보다 컴파일 속도가 빠르며, TensorBoard라는 시각화 툴을 제공하고 있습니다. CPU나 GPU를 활용한 계산이 수월하며 데이터나 모델들의 병렬 처리에 적합합니다.

다만, 다른 프레임워크보다 속도면에서 조금 느린 면이 있으며 구글에서 상업적 지원은 하지 않고 있다는 단점이 있습니다. 물론, 오픈 소스화되어 있어 꾸준히 발전하고 버그나 성능 개선은 계속해서 이루어질 것입니다.

초기 진입 장벽이 있는 편입니다.

<br>

## Caffe

[Caffe](http://caffe.berkeleyvision.org/)는 비전(Vision)과 머신러닝을 위한 라이브러리로 Matlab에서 C/C++ 기반으로 고속 CNN(Convolution Neural Network)을 구현해놓은 것입니다. 문자나 음성 등 범용적인 딥러닝을 위한 라이브러리는 아닙니다. C++과 Python을 모두 지원합니다.

범용성이 없다보니 확장성이 떨어지고 상업적 지원은 별도로 없습니다. 또한 회귀망에 적합하지 않고 대규모 네트워크에도 적합하지 않습니다. Caffe의 개발이나 발전 속도가 점점 느려지고 있다는 점도 단점입니다.

<br>

## Keras

[Keras](https://deeplearning4j.org/kr/keras.io)는 내부적으로 Theano와 Tensorflow를 바탕으로 하고 있으며 직관적이며 Python API 형태에 가장 가까운 인터페이스를 제공하고 있습니다.

Google 개발자에 의해 만들어졌으며, 가장 빠르게 발전하고 있는 프레임워크 중 하나라고 볼 수 있습니다. 

직관성이 떨어지고 진입 장벽이 높은 Tensorflow와는 달리 일반 Python API 형태로 제공하다보니 사용하기가 훨씬 쉽고 사용자 친화적인 환경을 제공하고 있습니다. 향후 Python 표준 딥러닝 API 형태로 자리 잡을 가능성이 있습니다.

<br>

## CNTK

[CNTK](https://github.com/Microsoft/CNTK)는 Microsoft의 딥러닝 오픈 소스 프레임워크입니다. DNN, CNN, 회귀 등의 알고리즘이 포함되어 있으며 C++ 기반에 Python API를 제공하고 있습니다. 