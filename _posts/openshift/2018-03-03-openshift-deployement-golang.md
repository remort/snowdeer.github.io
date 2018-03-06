---
layout: post
title: Go Project Deployment 템플릿
category: OpenShift
permalink: /openshift/:year/:month/:day/:title/

tag: [OpenShift]
---
# Go 프로젝트 배포

Docker Image를 이용해서 배포하는 경우와 Git 소스 코드를 이용해서 배포하는 `yaml` 코드 예제입니다. 

사용방법은 예시는 다음과 같습니다.

<pre class="prettyprint">
$ oc create -f deployment-using-docker-image.yaml

$ oc create -f deployment-using-git.yaml
</pre>

<br>

전체 코드는 [GitHub에서 확인](https://github.com/snowdeer/openshift-go-sample)할 수 있습니다.

## Dockerfile

<pre class="prettyprint">
FROM golang:1.9.1-alpine3.6

WORKDIR /go/src/app
COPY ./app .

RUN go get -d -v ./...
RUN go install -v ./...

EXPOSE 8080
EXPOSE 8888

CMD ["/go/bin/app"]
</pre>

<br>

## deployment-using-docker-image.yaml

<pre class="prettyprint">
apiVersion: extensions/v1beta1
kind: Deployment
metadata:
  name: go-sample
  labels:
    app: go-sample
spec:
  template:
    metadata:
      labels:
        app: go-sample
    spec:
      containers:
      - image:  snowdeer/go-sample:latest
        name: go-sample
        ports:
        - containerPort: 8080
          protocol: TCP
        - containerPort: 8888
          protocol: TCP
---
apiVersion: v1
kind: Service
metadata:
  name: go-sample
  labels:
    app: go-sample
spec:
  ports:
  - name: go-sample-port1
    port: 8080
    protocol: TCP
    targetPort: 8080
  - name: go-sample-port2
    port: 8888
    protocol: TCP
    targetPort: 8888
  selector:
    app: go-sample
  type: LoadBalancer
---
apiVersion: v1
kind: Route
metadata:
  name: go-sample
  labels:
    app: go-sample
spec:
  port: 
    targetPort: go-sample-port1
  to:
    kind: Service
    name: go-sample
    weight: 100
</pre>

<br>

## deployment-using-git.yaml

<pre class="prettyprint">
apiVersion: v1
kind: BuildConfig
metadata:
  labels:
    app: go-sample
  name: go-sample
spec:
  failedBuildsHistoryLimit: 3
  nodeSelector: null
  postCommit: {}
  resources: {}
  runPolicy: Serial
  source:
    git:
      uri: 'https://github.com/snowdeer/openshift-go-sample.git'
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
      name: 'go-sample:latest'
---
apiVersion: extensions/v1beta1
kind: Deployment
metadata:
  name: go-sample
  labels:
    app: go-sample
spec:
  template:
    metadata:
      labels:
        app: go-sample
    spec:
      containers:
      - image:  snowdeer/go-sample:latest
        name: go-sample
        ports:
        - containerPort: 8080
          protocol: TCP
---
apiVersion: v1
kind: Service
metadata:
  name: go-sample
  labels:
    app: go-sample
spec:
  ports:
  - name: go-sample-port1
    port: 8080
    protocol: TCP
    targetPort: 8080
  - name: go-sample-port2
    port: 8888
    protocol: TCP
    targetPort: 8888
  selector:
    app: go-sample
  type: LoadBalancer
---
apiVersion: v1
kind: Route
metadata:
  name: go-sample
  labels:
    app: go-sample
spec:
  port: 
    targetPort: go-sample-port1
  to:
    kind: Service
    name: go-sample
    weight: 100
</pre>