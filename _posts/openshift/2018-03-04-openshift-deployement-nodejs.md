---
layout: post
title: Node.js Project Deployment 템플릿
category: OpenShift
permalink: /openshift/:year/:month/:day/:title/

tag: [OpenShift]
---
# Node.js 프로젝트 배포

Docker Image를 이용해서 배포하는 경우와 Git 소스 코드를 이용해서 배포하는 `yaml` 코드 예제입니다. 

사용방법은 예시는 다음과 같습니다.

<pre class="prettyprint">
$ oc create -f deployment-using-docker-image.yaml

$ oc create -f deployment-using-git.yaml
</pre>

<br>

전체 코드는 [GitHub에서 확인](https://github.com/snowdeer/openshift-nodejs-sample)할 수 있습니다.

## Dockerfile

<pre class="prettyprint">
FROM node:carbon

WORKDIR /usr/src/app

COPY package*.json ./

RUN npm install

COPY ./app .

EXPOSE 8080

CMD [ "npm", "start" ]
</pre>

<br>

## deployment-using-docker-image.yaml

<pre class="prettyprint">
apiVersion: extensions/v1beta1
kind: Deployment
metadata:
  name: nodejs-sample
  labels:
    app: nodejs-sample
spec:
  template:
    metadata:
      labels:
        app: nodejs-sample
    spec:
      containers:
      - image:  snowdeer/nodejs-sample:latest
        name: nodejs-sample
        ports:
        - containerPort: 8080
          protocol: TCP
---
apiVersion: v1
kind: Service
metadata:
  name: nodejs-sample
  labels:
    app: nodejs-sample
spec:
  ports:
  - name: nodejs-sample
    port: 8080
    protocol: TCP
    targetPort: 8080
  selector:
    app: nodejs-sample
  type: LoadBalancer
---
apiVersion: v1
kind: Route
metadata:
  name: nodejs-sample
  labels:
    app: nodejs-sample
spec:
  port: 
    targetPort: nodejs-sample
  to:
    kind: Service
    name: nodejs-sample
    weight: 100
</pre>

<br>

## deployment-using-git.yaml

<pre class="prettyprint">
apiVersion: v1
kind: BuildConfig
metadata:
  labels:
    app: nodejs-sample
  name: nodejs-sample
spec:
  failedBuildsHistoryLimit: 3
  nodeSelector: null
  postCommit: {}
  resources: {}
  runPolicy: Serial
  source:
    git:
      uri: 'https://github.com/snowdeer/openshift-nodejs-sample.git'
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
      name: 'nodejs-sample:latest'
---
apiVersion: extensions/v1beta1
kind: Deployment
metadata:
  name: nodejs-sample
  labels:
    app: nodejs-sample
spec:
  template:
    metadata:
      labels:
        app: nodejs-sample
    spec:
      containers:
      - image:  snowdeer/nodejs-sample:latest
        name: nodejs-sample
        ports:
        - containerPort: 8080
          protocol: TCP
---
apiVersion: v1
kind: Service
metadata:
  name: nodejs-sample
  labels:
    app: nodejs-sample
spec:
  ports:
  - name: nodejs-sample
    port: 8080
    protocol: TCP
    targetPort: 8080
  selector:
    app: nodejs-sample
  type: LoadBalancer
---
apiVersion: v1
kind: Route
metadata:
  name: nodejs-sample
  labels:
    app: nodejs-sample
spec:
  port: 
    targetPort: nodejs-sample
  to:
    kind: Service
    name: nodejs-sample
    weight: 100
</pre>