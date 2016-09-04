---
layout: post
title: 머신 러닝(Machine Learning) 알고리즘들
category: Software 상식
tag: [machine learning]
---

Machine Learning 알고리즘들에 대해서 알아보도록 하겠습니다.
여기서는 몇 가지 알고리즘들만 간단히 포스팅해보도록 하고, 좀 더 자세한 정보를 원하시는 분들은

* [A Tour of Machine Learning Algorithms](http://machinelearningmastery.com/a-tour-of-machine-learning-algorithms/)
* [How to choose algorithms for Microsoft Azure Machine Learning](https://azure.microsoft.com/en-gb/documentation/articles/machine-learning-algorithm-choice/)

같은 사이트를 참조하시길 바랍니다.

<br>

## Linear Regression

![Image]({{ site.baseurl }}/assets/2016-09-03-machine-learning-algorithms/image3[1].png) 

위의 그림과 같이 모든 데이터들이 선형적인 분포 형태를 갖고 있다는 
가정으로부터 시작하는 알고리즘입니다. 자주 사용되고, 비교적 간단한 구조로 되어 있어 속도도 빠르지만 정확도면에서는 떨어질 수 밖에 없는 
형태입니다.

<br>

## Logistic Regression

![Image]({{ site.baseurl }}/assets/2016-09-03-machine-learning-algorithms/image4[1].png) 

Linear Regression이 가진 문제 때문에 다중 클래스를 갖는 Logistic Regression 알고리즘이 나타나게 되었습니다. (위 이미지는 2개의 클래스를 가진 Logistic Regression 알고리즘 예시입니다.)

덕분에 S 자 형태의 그래프를 가질 수 있게 되었고, 데이터를 그룹으로 분류해서 관리하는데 유용하게 사용될 수 있습니다.

<br>

## Trees, forests, and jungles

![Image]({{ site.baseurl }}/assets/2016-09-03-machine-learning-algorithms/image5[1].png) 

'결정 트리'라고 해서 많이 알려진 Decision Tree나, Decision Forest, Decision Jungle입니다. 전체 영역을 같은 레이블(label)을 갖는 데이터들의 영역들로 나누어 관리합니다.

~~~
Decision Forests는 많은 양의 메모리를 필요로 합니다.
Decision Jungle은 메모리 소모는 줄어들지만, 학습 시간이 오래 걸립니다.
~~~

<br>

## Neural Networks 

![Image]({{ site.baseurl }}/assets/2016-09-03-machine-learning-algorithms/image6[1].png) 

사람의 두뇌 구조에서 영감을 얻어서 만들어진 학습 기술입니다.
무제한적인 다양성을 갖게 해주지만, 상당히 복잡하고 필요로 하는 Cost 또한 높습니다. 학습 시간도 오래 걸리며, 다른 알고리즘들에 비해 
필요로 하는 파라메터(parameter)도 많습니다.

하지만 잠재력은 무궁무진하다고 볼 수 있습니다.

<br>

## Support Vector Machine

![Image]({{ site.baseurl }}/assets/2016-09-03-machine-learning-algorithms/image7[1].png) 

각 데이터를 구분하는 경계를 찾는 방식입니다. 경계는 위의 그림과 같이 선형으로 구할 수도 있고, 비선형으로도 구할 수 있습니다.

<br>

## K-means Algorithm 

![Image]({{ site.baseurl }}/assets/2016-09-03-machine-learning-algorithms/image9[1].png) 

주어진 데이터들을 K 개의 클러스터(Cluster)로 나누는 알고리즘입니다.
데이터들의 거리를 기반으로 가까운 데이터들끼리 Group화 하면서 클러스터를 나눕니다.
간단히 사용하기에 괜찮은 알고리즘이긴 하지만 다음과 같은 한계점을 갖고 있습니다.

~~~
- 파라메터 K 값에 따라 그 결과가 완전히 달라집니다.
- 이상값(outlier)에 따라 중심값이 크게 왜곡될 수 있습니다.
- 구형(spherical) 데이터가 아닌 경우 적절하지 않은 상황이 발생할 수 있습니다.
~~~