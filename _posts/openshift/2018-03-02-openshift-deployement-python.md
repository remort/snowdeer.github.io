---
layout: post
title: Python Project Deployment 템플릿
category: OpenShift
permalink: /openshift/:year/:month/:day/:title/

tag: [OpenShift]
---
# Python 프로젝트 배포

Docker Image를 이용해서 배포하는 경우와 Git 소스 코드를 이용해서 배포하는 `yaml` 코드 예제입니다. 

사용방법은 예시는 다음과 같습니다.

<pre class="prettyprint">
$ oc create -f deployment-using-docker-image.yaml

$ oc create -f deployment-using-git.yaml
</pre>

<br>

전체 코드는 [GitHub에서 확인](https://github.com/snowdeer/openshift-python-sample)할 수 있습니다.

## Dockerfile

<pre class="prettyprint">
FROM ubuntu

# basic setting for python
RUN apt-get update
RUN apt-get install -y python python-pip


# install my app
WORKDIR /app

COPY . .

RUN pip install --upgrade pip
RUN pip install -r requirements.txt

# expose ports
EXPOSE 8080


## launch
CMD [ "python", "./app/server.py" ]
</pre>

<br>

## deployment-using-docker-image.yaml

<pre class="prettyprint">
apiVersion: extensions/v1beta1
kind: Deployment
metadata:
  name: python-sample
  labels:
    app: python-sample
spec:
  template:
    metadata:
      labels:
        app: python-sample
    spec:
      containers:
      - image:  snowdeer/python-sample:latest
        name: python-sample
        ports:
        - containerPort: 8080
          protocol: TCP
---
apiVersion: v1
kind: Service
metadata:
  name: python-sample
  labels:
    app: python-sample
spec:
  ports:
  - name: python-sample
    port: 8080
    protocol: TCP
    targetPort: 8080
  selector:
    app: python-sample
  type: LoadBalancer
---
apiVersion: v1
kind: Route
metadata:
  name: python-sample
  labels:
    app: python-sample
spec:
  port: 
    targetPort: python-sample
  to:
    kind: Service
    name: python-sample
    weight: 100
</pre>

<br>

## deployment-using-git.yaml

<pre class="prettyprint">
apiVersion: v1
kind: BuildConfig
metadata:
  labels:
    app: python-sample
  name: python-sample
spec:
  failedBuildsHistoryLimit: 3
  nodeSelector: null
  postCommit: {}
  resources: {}
  runPolicy: Serial
  source:
    git:
      uri: 'https://github.com/snowdeer/openshift-python-sample.git'
    type: Git
  strategy:
    dockerStrategy:
      from:
        kind: DockerImage
        name: 'ubuntu:latest'
        
    type: Docker    
  successfulBuildsHistoryLimit: 5 
  output:
    to:
      kind: ImageStreamTag
      name: 'python-sample:latest'
---
apiVersion: extensions/v1beta1
kind: Deployment
metadata:
  name: python-sample
  labels:
    app: python-sample
spec:
  template:
    metadata:
      labels:
        app: python-sample
    spec:
      containers:
      - image:  snowdeer/python-sample:latest
        name: python-sample
        ports:
        - containerPort: 8080
          protocol: TCP
---
apiVersion: v1
kind: Service
metadata:
  name: python-sample
  labels:
    app: python-sample
spec:
  ports:
  - name: python-sample
    port: 8080
    protocol: TCP
    targetPort: 8080
  selector:
    app: python-sample
  type: LoadBalancer
---
apiVersion: v1
kind: Route
metadata:
  name: python-sample
  labels:
    app: python-sample
spec:
  port: 
    targetPort: python-sample
  to:
    kind: Service
    name: python-sample
    weight: 100
</pre>