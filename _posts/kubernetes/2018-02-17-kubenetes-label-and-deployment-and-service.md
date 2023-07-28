---
layout: post
title: Label, Deployment, Service
category: Kubernetes
permalink: /kubernetes/:year/:month/:day/:title/

tag: [Kubernetes, k8s]
---

# Label

Kubenetes를 이용해 큰 규모의 서비스를 구동하게 되면 다수의 Pod들을 운영하게 될 수 있습니다. 이 경우 다수의 Pod들을 좀 더 편리하게 관리하기 위해서 Kubenetes에서는 Label이라는 기능을 제공하고 있습니다. Label은 key-value 데이터 쌍 형태로 되어 있습니다.

Pod definition에 Label을 추가하는 방법은 다음과 같습니다.

<pre class="prettyprint">
  labels:
    app: nginx
</pre>

<br>

## pod-nginx-with-label.yaml

<pre class="prettyprint">
apiVersion: v1
kind: Pod
metadata:
  name: nginx
  labels:
    app: nginx
spec:
  containers:
  - name: nginx
    image: nginx
    ports:
    - containerPort: 80
</pre>

<br>

다음 명령어를 이용해 Pod를 생성하고 `nginx`라는 Label을 가진 Pod 리스트를 조회할 수 있습니다.

<pre class="prettyprint">
$ kubectl create -f ./pod-nginx-with-label.yaml

$ kubectl get pods -l app=nginx
</pre>

<br>

# Deployment

이제 다수의 컨테이너, Label이 붙은 Pod 등을 이용해서 App을 만들 수 있게 되었지만, 여전히 많은 의문점이 생길 수 있습니다. 예를 들어 Pod를 어떻게 Scaling 할 것인지, 새로운 릴리즈를 어떻게 Roll Out 할 것인지 등의 이슈가 발생할 수 있습니다.

이 경우 Deployment를 이용해서 Pod 들을 관리하거나 업데이트할 수 있습니다.

Deployment Object에는 Pod를 생성하는 템플릿이나 희망 Replica 개수를 정의할 수 있습니다. Label Selector를 이용해서 Pods를 생성하거나 제거하기도 하며, 새로운 업데이트나 릴리즈를 현재 구동중인 Pod들에 안전하게 반영할 수 있게 도와줍니다.

<br>

## deployment.yaml

다음은 2개의 nginx Pod가 정의된 `deployment.yaml` 파일입니다.

<pre class="prettyprint">
apiVersion: apps/v1
kind: Deployment
metadata:
  name: nginx-deployment
spec:
  selector:
    matchLabels:
      app: nginx
  replicas: 2 # tells deployment to run 2 pods matching the template
  template: # create pods using pod definition in this template
    metadata:
      # unlike pod-nginx.yaml, the name is not included in the meta data as a unique name is
      # generated from the deployment name
      labels:
        app: nginx
    spec:
      containers:
      - name: nginx
        image: nginx:1.7.9
        ports:
        - containerPort: 80
</pre>

<br>

## Deployment 생성

다음 명령어를 이용해서 Deployment를 생성할 수 있습니다.

<pre class="prettyprint">
$ kubectl create -f ./deployment.yaml

$ kubectl get deployment

$ kubectl get pods -l app=nginx
</pre>

<br>

## Deployment 업그레이드하기

만약 위의 Deployment에서 nginx 컨테이너의 버전을 `1.7.9`에서 `1.8`로 업그레이드하는 경우 다음과 같은 작업을 수행하면 됩니다.

### deployment-update.yaml

<pre class="prettyprint">
apiVersion: apps/v1
kind: Deployment
metadata:
  name: nginx-deployment
spec:
  selector:
    matchLabels:
      app: nginx
  replicas: 2
  template:
    metadata:
      labels:
        app: nginx
    spec:
      containers:
      - name: nginx
        image: nginx:1.8 # Update the version of nginx from 1.7.9 to 1.8
        ports:
        - containerPort: 80
</pre>

<br>

<pre class="prettyprint">
$ kubectl apply -f ./deployment-update.yaml

$ kubectl get pods -l app=nginx
</pre>

<br>

## Deployment 삭제

Deployment 삭제는 다음 명령어로 수행합니다.

<pre class="prettyprint">
$ kubectl delete deployment nginx-deployment
</pre>

<br>

# Services

### service.yaml

Once you have a replicated set of Pods, you need an abstraction that enables connectivity between the layers of your application. For example, if you have a Deployment managing your backend jobs, you don’t want to have to reconfigure your front-ends whenever you re-scale your backends. Likewise, if the Pods in your backends are scheduled (or rescheduled) onto different machines, you can’t be required to re-configure your front-ends. In Kubernetes, the service abstraction achieves these goals. A service provides a way to refer to a set of Pods (selected by labels) with a single static IP address. It may also provide load balancing, if supported by the provider.

<pre class="prettyprint">
apiVersion: v1
kind: Service
metadata:
  name: nginx-service
spec:
  ports:
  - port: 8000 # the port that this service should serve on
    # the container on each pod to connect to, can be a name
    # (e.g. 'www') or a number (e.g. 80)
    targetPort: 80
    protocol: TCP
  # just like the selector in the deployment,
  # but this time it identifies the set of pods to load balance
  # traffic to.
  selector:
    app: nginx
</pre>

<br>

## Service 관리

<pre class="prettyprint">
$ kubectl create -f ./service.yaml

$ kubectl get services

$ kubectl delete service nginx-service
</pre>

When created, each service is assigned a unique IP address. This address is tied to the lifespan of the Service, and will not change while the Service is alive. Pods can be configured to talk to the service, and know that communication to the service will be automatically load-balanced out to some Pod that is a member of the set identified by the label selector in the Service.
