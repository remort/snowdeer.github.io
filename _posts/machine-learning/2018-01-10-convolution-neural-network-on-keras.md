---
layout: post
title: CNN(Convolution Neural Network)
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝]
---
[여기](https://adeshpande3.github.io/A-Beginner%27s-Guide-To-Understanding-Convolutional-Neural-Networks/)를 보면 좀 더 자세한 내용을 볼 수 있습니다.

# CNN

CNN(Convoluion Neural Network)은 딥러닝에서 이미지 인식의 꽃이라고 불릴 정도로 강력한 성능을 가진 기법입니다.

CNN은 입력된 이미지에서 특징을 좀 더 잘 뽑아내기 위해서 그 위에 필터(마스크, 윈도우, 커널 등으로 불려짐)를 입히는 기법입니다. 그 이후 풀링(Pooling)이라고 불리우는 다운 샘플링(Down-sampling) 과정을 거쳐 신경망에 데이터를 입력합니다.

![Image](/assets/machine-learning/026.png)

<br>

# 이미지 인식

사람은 이미지를 눈으로 보고 바로 인식을 할 수 있지만, 컴퓨터는 이미지를 각 픽셀에 입력되어 있는 숫자값들의 배열로 인식합니다.

![Image](/assets/machine-learning/028.png)

<br>

# 필터 사용

아래 그림처럼 포토샵 등에서 다양한 필터를 적용해서 이미지를 변형하는 것을 보신 경험이 있을겁니다. 어떤 필터는 이미지를 더 부드럽게 만들기도 하고, 어떤 필터는 더 날카롭게, 경계면을 뚜렷하게 만드는 필터들도 있습니다.

![Image](/assets/machine-learning/029.jpg)

CNN에서도 이미지 인식을 보다 잘할 수 있도록 필터를 사용합니다. 

![Image](/assets/machine-learning/027.gif)

원본 이미지와 필터간 합성곱(Convolution) 연산을 통해 새로운 Layer를 생성하고, 이를 Convolution Layer라고 합니다. 이러한 필터를 여러 개 사용할 경우 여러 개의 Convolution이 만들어집니다.

<br>

# 필터 사용 예제

필터를 이용해서 원본 이미지와 Convolution 연산을 하는 예제를 살펴보겠습니다.

![Image](/assets/machine-learning/030.png)

위 이미지와 같은 필터가 있다고 가정합니다.

![Image](/assets/machine-learning/031.png)

그리고 위 이미지에서 노란색 박스 부분을 필터로 검출하게 되면 아래와 같은 결과가 나오게 됩니다.

![Image](/assets/machine-learning/032.png)

위 이미지의 아랫 부분에 `Multiplication and Summation` 결과를 보면 상당히 큰 크기의 값이 도출되는 것을 알 수 있습니다.

![Image](/assets/machine-learning/033.png)

만약 위 그림처럼 필터와 비슷한 부분이 거의 없는 영역에 Convolution 계산을 하면 아랫쪽의 계산 결과처럼 `0`이 나왔음을 확인할 수 있습니다.

이렇게 필터를 이용하면 이미지에서 특징점들을 쉽게 찾아낼 수 있습니다.

<br>

# 딥러닝에 Convolution을 적용

![Image](/assets/machine-learning/034.png)

딥러닝에 Convolution을 적용하면 위 그림과 같은 구조가 됩니다. 각 Layer에서 Convolution 연산을 한 후 Activation Function을 거쳐 다음 Layer에서 또 Convolution 연산을 하는 계층 형태로 되어 있습니다.