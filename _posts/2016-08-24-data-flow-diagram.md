---
layout: post
title: DFD (Data Flow Diagram)
category: UML
tag: [uml]
---

일단, UML 카테고리에 넣긴 했지만 DFD(Data Flow Diagram)은 
UML은 아닙니다. DFD는 모델간의 관계를 표현하는게 아니라 데이터의 흐름을 표현합니다. 

다만 UML이 DFD를 개량하는데서부터 시작했다는 점도 있고, 데이터 흐름을
보여주는 것도 중요한 기능이며 거의 UML과 같이 사용하는 경우가 많아서
UML 카테고리에 집어넣었습니다.

DFD의 예를 들면 다음과 같습니다.

![Image]({{ site.baseurl }}/assets/2016-08-24-data-flow-diagram/dfd-pays-workers.gif) 

<br>

## DFD의 특성

DFD의 특성은 다음과 같습니다.

* 데이터(자료)의 흐름에 중심을 두고 있습니다.
* 제어(Control)의 흐름은 중요하지 않습니다.

<br>

## DFD의 구성 요소

### 프로세스(Process)

![Image]({{ site.baseurl }}/assets/2016-08-24-data-flow-diagram/dfd-process.jpg) 

Function이라고도 합니다. 원 안에는 프로세스가 수행하는 일을 표기합니다.
프로세스는 자체적으로 데이터를 생성할 수 없고 입력되는 데이터가 있어야 
합니다.

<br>

### 외부 엔터티(External Entity)

![Image]({{ site.baseurl }}/assets/2016-08-24-data-flow-diagram/external-entity.jpg) 

외부 엔티티는 프로세스 처리과정에서의 데이터발생의 시작 및 종료를 나타냅니다.
외부 엔티티는 DFD에서 프로세스와의 상호 관련성을 표현하며, 일반적으로 DFD 범위 밖의 사각형 형태로 표시합니다. 

<br>

### 데이터 저장소(Data Store)

![Image]({{ site.baseurl }}/assets/2016-08-24-data-flow-diagram/dfd-data-store.jpg) 

![Image]({{ site.baseurl }}/assets/2016-08-24-data-flow-diagram/dfd-data-store2.jpg) 

데이터베이스(Database)라고도 하며, 데이터가 저장되어 있는 곳입니다.
데이터 저장소는 단순한 데이터의 저장을 나타내며, 데이터의 변화는 표시하지 않습니다.

<br>

### 데이터 흐름(Flow)

![Image]({{ site.baseurl }}/assets/2016-08-24-data-flow-diagram/dfd-data-flow.jpg) 

데이터 흐름은 DFD의 구성요소들간의 인터페이스를 나타냅니다.

<br>

보다 자세한 설명은 [여기](http://kinzz.com/resources/articles/116-data-flow-diagram?showall=1)를 참조하세요.

<br>

## DFD 작성 순서

DFD 작성 순서에 대한 참조는 [여기](http://www.slideshare.net/mohit4192/dfd-examples)에서 했습니다.

1. Create a list of activies
2. Construct Context Level DFD
(identifies sources and sink)
3. Construct Level 0 DFD
(identifies manageable sub processes)
4. Construct Level N DFD
(identifies actual data flows and data stores)

차례대로 예제를 살펴보도록 합시다.

<br>

### 1. Create a list of activies

액티비티들을 나열해봅니다.

* Customer Order
* Serve Product
* Collect Payment
* Produce Product
* Store Product
* Order Raw Material
* Pay for Raw Materials
* Pay for Labor

그리고 각각의 액티비티들을 그룹으로 분류를 해봅니다.

* Customer Order
* Serve Product
* Collect Payment

* Produce Product
* Store Product

* Order Raw Material
* Pay for Raw Materials

* Pay for Labor

<br>

### 2. Construct Context Level DFD

Create a context level diagram identifying the sources and sinks(users).

![Image]({{ site.baseurl }}/assets/2016-08-24-data-flow-diagram/20160824_222357.png) 



<br>

### 3. Construct Level 0 DFD

Create a level 0 diagram identifying the logical subsytems that may exist.


![Image]({{ site.baseurl }}/assets/2016-08-24-data-flow-diagram/20160824_222409.png) 


<br>

### 4. Construct Level N DFD

Create a level 1 decomposing the processes in level 0 and identifying data stores.

![Image]({{ site.baseurl }}/assets/2016-08-24-data-flow-diagram/20160824_222422.png) 

![Image]({{ site.baseurl }}/assets/2016-08-24-data-flow-diagram/20160824_222507.png) 

![Image]({{ site.baseurl }}/assets/2016-08-24-data-flow-diagram/20160824_222532.png) 

<br>

## 또 다른 예제

아래는 또 다른 예제입니다. 

<br>

### Level 1

![Image]({{ site.baseurl }}/assets/2016-08-24-data-flow-diagram/first-level-dfd.gif) 

<br>

### Level 2

![Image]({{ site.baseurl }}/assets/2016-08-24-data-flow-diagram/second-level-dfd.gif) 

<br>

### Level 3

![Image]({{ site.baseurl }}/assets/2016-08-24-data-flow-diagram/third-level-dfd.gif) 

<br>

### Level 4

![Image]({{ site.baseurl }}/assets/2016-08-24-data-flow-diagram/fourth-level-dfd.gif) 