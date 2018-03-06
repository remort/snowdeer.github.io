---
layout: post
title: Java Project Deployment 템플릿
category: OpenShift
permalink: /openshift/:year/:month/:day/:title/

tag: [OpenShift]
---
# Java 프로젝트 배포

Docker Image를 이용해서 배포하는 경우와 Git 소스 코드를 이용해서 배포하는 `yaml` 코드 예제입니다. 

사용방법은 예시는 다음과 같습니다.

<pre class="prettyprint">
$ oc create -f deployment-using-docker-image.yaml

$ oc create -f deployment-using-git.yaml
</pre>

<br>

전체 코드는 [GitHub에서 확인](https://github.com/snowdeer/openshift-java-sample)할 수 있습니다.

## Dockerfile

<pre class="prettyprint">
FROM centos:7

RUN yum update -y && \
yum install -y wget && \
yum install -y java-1.8.0-openjdk java-1.8.0-openjdk-devel && \
yum clean all

ENV HOME /app
WORKDIR /app

COPY ./app .

EXPOSE 8080

RUN javac /app/SimpleWebServer.java
CMD ["java", "SimpleWebServer"]
</pre>

<br>

## deployment-using-docker-image.yaml

<pre class="prettyprint">
apiVersion: extensions/v1beta1
kind: Deployment
metadata:
  name: java-sample
  labels:
    app: java-sample
spec:
  template:
    metadata:
      labels:
        app: java-sample
    spec:
      containers:
      - image:  snowdeer/java-sample:latest
        name: java-sample
        ports:
        - containerPort: 8080
          protocol: TCP
---
apiVersion: v1
kind: Service
metadata:
  name: java-sample
  labels:
    app: java-sample
spec:
  ports:
  - name: java-sample
    port: 8080
    protocol: TCP
    targetPort: 8080
  selector:
    app: java-sample
  type: LoadBalancer
---
apiVersion: v1
kind: Route
metadata:
  name: java-sample
  labels:
    app: java-sample
spec:
  port: 
    targetPort: java-sample
  to:
    kind: Service
    name: java-sample
    weight: 100
</pre>

<br>

## deployment-using-git.yaml

<pre class="prettyprint">
apiVersion: v1
kind: BuildConfig
metadata:
  labels:
    app: java-sample
  name: java-sample
spec:
  failedBuildsHistoryLimit: 3
  nodeSelector: null
  postCommit: {}
  resources: {}
  runPolicy: Serial
  source:
    git:
      uri: 'https://github.com/snowdeer/openshift-java-sample.git'
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
      name: 'java-sample:latest'
---
apiVersion: extensions/v1beta1
kind: Deployment
metadata:
  name: java-sample
  labels:
    app: java-sample
spec:
  template:
    metadata:
      labels:
        app: java-sample
    spec:
      containers:
      - image:  snowdeer/java-sample:latest
        name: java-sample
        ports:
        - containerPort: 8080
          protocol: TCP
---
apiVersion: v1
kind: Service
metadata:
  name: java-sample
  labels:
    app: java-sample
spec:
  ports:
  - name: java-sample
    port: 8080
    protocol: TCP
    targetPort: 8080
  selector:
    app: java-sample
  type: LoadBalancer
---
apiVersion: v1
kind: Route
metadata:
  name: java-sample
  labels:
    app: java-sample
spec:
  port: 
    targetPort: java-sample
  to:
    kind: Service
    name: java-sample
    weight: 100
</pre>