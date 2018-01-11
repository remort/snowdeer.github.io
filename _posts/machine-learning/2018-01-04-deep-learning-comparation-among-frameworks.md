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
 
위 도표는 구글에서 조사한 딥러닝 프레임워크 관심도 순위입니다. 

<br>

## Theano 

[Theano](http://deeplearning.net/software/theano/)는 다차원 배열을 쉽게 다룰 수 있는 라이브러리입니다. 데이터 탐색에 적합해서 다른 라이브러리들과 연동하여 연구용으로 많이 사용되었습니다.

[Keras](https://github.com/keras-team/keras)를 비롯한 많은 딥러닝 라이브러리가 Theano를 기반으로 만들어졌습니다. 

Theano는 CNN과 노이즈 제거 알고리즘인 Auto-Encoder, 심층 신경망에 최적화된 프레임워크입니다. 2008년 첫 버전이 공개되고 나서 한동안 가장 인기있는 프레임워크로 알려져왔었습니다.

Theano는 초창기 프레임워크다보니 많은 부분에서 단점이 존재합니다. 사용하기 복잡하며, 단일 GPU를 지원합니다. 또한 에러 메시지가 코드의 잘못된 부분 수정에 크게 도움을 주지 못하는 점과 대형 모델에 대해서는 컴파일 시간도 오래 걸린다는 단점이 있습니다.

<br>

## Torch7

[Torch](http://torch.ch/)는 [Lua](http://www.lua.org/) 언어 기반의 API를 제공하는 머신러닝 프레임워크입니다. 1990년대 초에 개발된 오래된 프레임워크 중 하나입니다.

작고 다양한 많은 모듈 조각으로 이루어져 있어 결합해서 자신만의 레이어를 만들고 GPU에서 실행하기 쉬운 장점이 있습니다. Lua 언어의 사용은 장단점을 모두 갖고 있는데, 자바 스크립트(Java Script)와 사용법이 비슷하여 직관적이며 배우기 쉽고, 타인의 코드 이해를 쉽게 할 수 있는 장점이 있지만, Lua 언어 자체가 Java/Python보다 폐쇄적이고 사용자 그룹이 활성화되어 있지 않아 생태계 조성에 불리한 점이 있습니다.

2017년 1월에 Facebook에서 Python API 형태의 [Pytorch(https://github.com/pytorch/pytorch)]를 오픈 소스로 공개했습니다.

Pytorch는 가변 길이 입력과 출력을 처리할 수 있는 Dynamic Computation Graph를 지원합니다.

<br>

## Tensorflow

현재 가장 많은 개발자들이 선호하는 프레임워크 중 하나로 Google에서 만든 딥러닝용 프레임워크입니다. Theano를 대체하고자 만들었으며, [Tensorflow](https://www.tensorflow.org/) 개발진에 Theano 개발자들도 일부 포함되어 있었습니다. 

Tensorflow는 기본적으로 C++과 Python을 지원하고 있으며, 내부적으로는 전부 C/C++ 엔진으로 되어 있어 Python으로 개발하더라도 속도에 큰 차이가 없습니다. 다만 Java나 Scala 언어 등은 지원하고 있지 않습니다.

Tensorflow는 딥러닝 외에도 강화학습(Reinforcement Learning) 등을 위한 알고리즘도 제공하고 있습니다.  

Theno보다 컴파일 속도가 빠르며, TensorBoard라는 시각화 툴을 제공하고 있습니다. CPU나 GPU를 활용한 계산이 수월하며 데이터나 모델들의 병렬 처리에 적합합니다. 또한 GPU에 최적화가 되어 있고, 분산 병렬 컴퓨팅 환경을 지원한다는 점도 다른 프레임워크보다 우수한 점입니다. 

Tensorflow는 구글에서 상업적 지원은 하지 않고 있으며, 앞으로도 계획은 없다고 합니다. 그리고 사전 학습 모델이 많지 않다는 단점이 있으며, 다른 딥러닝 프레임워크보다 늦게 개발되었기 때문에 아직 다양한 플랫폼에서의 최적화가 부족한 경우가 있습니다.

<br>

## Caffe

[Caffe](http://caffe.berkeleyvision.org/)는 비전(Vision)과 머신러닝을 위한 라이브러리로 Matlab에서 C/C++ 기반으로 고속 CNN(Convolution Neural Network)을 구현해놓은 것입니다. 문자나 음성 등 범용적인 딥러닝을 위한 라이브러리는 아닙니다. C++과 Python을 모두 지원합니다.

범용성이 없다보니 확장성이 떨어지고 상업적 지원은 별도로 없습니다. 또한 회귀망에 적합하지 않고 대규모 네트워크에도 적합하지 않습니다. 대학교에서 순수 연구 목적으로 개발된 프레임워크이다보니 업데이트나 발전이 느려서 최신 GPU와의 호환성 검증이나 최적화 지원이 신속하지 못하며, 확장형 분산 컴퓨팅 API 지원이 부족한 점이 단점입니다.

<br>

## Keras

[Keras](https://deeplearning4j.org/kr/keras.io)는 내부적으로 Theano와 Tensorflow를 바탕으로 하고 있으며 직관적이며 Python API 형태에 가장 가까운 인터페이스를 제공하고 있습니다.

Google 개발자에 의해 만들어졌으며, 가장 빠르게 발전하고 있는 프레임워크 중 하나라고 볼 수 있습니다. 

직관성이 떨어지고 진입 장벽이 높은 Tensorflow와는 달리 일반 Python API 형태로 제공하다보니 사용하기가 훨씬 쉽고 사용자 친화적인 환경을 제공하고 있습니다. 향후 Python 표준 딥러닝 API 형태로 자리 잡을 가능성이 있습니다.

<br>

## CNTK

[CNTK](https://github.com/Microsoft/CNTK)는 Microsoft의 딥러닝 오픈 소스 프레임워크입니다. DNN, CNN, 회귀 등의 알고리즘이 포함되어 있으며 C++ 기반에 Python API를 제공하고 있습니다. 