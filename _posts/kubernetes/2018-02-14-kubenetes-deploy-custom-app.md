---
layout: post
title: Custom Node.js App 배포해보기
category: Kubernetes
permalink: /kubernetes/:year/:month/:day/:title/

tag: [Kubernetes]
---
# Custom Node.js App 배포

## 샘플 node.js 파일 생성

먼저 다음과 같은 `server.js` 파일을 생성합니다.

<pre class="prettyprint">
var http = require('http');

var handleRequest = function(request, response) {
  console.log('Received request for URL: ' + request.url);
  response.writeHead(200);
  response.end('Hello SnowDeer!');
};
var www = http.createServer(handleRequest);
www.listen(8080);
</pre>

<br>

정상적으로 동작하는지 확인은 

~~~
node server.js
~~~

명령 수행 후 웹브라우저에 접속해서 확인할 수 있습니다.

<br>

## Dockerfile 생성

그리고 다음과 같은 Dockerfile 파일을 생성합니다.

<pre class="prettyprint">
FROM node:6.9.2
EXPOSE 8080
COPY server.js .
CMD node server.js
</pre>

다음 명령어를 이용해서 Docker 이미지를 빌드합니다.

~~~
docker build -t snowdeer/hello-nodejs:v1 .
~~~

그리고 

~~~
docker run --rm --name snowdeer -d -p 8080:8080 snowdeer/hello-nodejs:v1
~~~

와 같이 Docker 컨테이너를 생성한다음 웹브라우저에서 접속해서 해당 컨테이너가 잘 생성되는지 확인합니다.

<br>

## Docker Hub에 이미지 업로드

Kubenetes에서 Docker 이미지를 이용해서 Deployment를 생성하기 위해서는 해당 이미지가 Docker Hub에 등록이 되어 있어야 합니다. 다음 명령어를 이용해서 위에서 생성한 이미지를 Docker Hub에 `push` 합니다.

~~~
docker push snowdeer/hello-nodejs:v1
~~~

<br>

## Deployment 생성

Kubenetes에서 `Pod`는 관리 또는 네트워크 목적으로 묶여있는 하나 이상의 컨테이너들의 그룹입니다. `kubectl run` 명령어를 이용해서 Deployment를 생성할 수 있습니다. Deployment는 Kubenetes에서 Pod들의 생성과 스케일링(Scaling)을 위해 추천하는 방식입니다.

다음 명령어를 이용하여 Docker Hub에 등록되어 있는 `hello-nodejs:v1` 이미지로부터 Deployment를 생성합니다.

~~~
kubectl run hello-nodejs --image=snowdeer/hello-nodejs:v1 --port=8080
~~~

다음 명령어로 Deployment 정보를 확인할 수 있습니다.

~~~
$ kubectl get deployment

NAME           DESIRED   CURRENT   UP-TO-DATE   AVAILABLE   AGE
hello-nodejs   1         1         1            1           1m
~~~

다음 명령어로 Pods 정보를 확인할 수 있습니다.

~~~
$ kubectl get pods

NAME                            READY     STATUS    RESTARTS   AGE
hello-nodejs-868459c9d5-hghv8   1/1       Running   0          3m
~~~

만약 이 과정에서 `AVAILABLE` 항목이나 `READY` 값이 `1`이 되지 않는다면, `kubectl describe pod [pod name]` 명령어를 이용해서 에러 원인을 찾아야 합니다. 보통 위에서 생성한 이미지가 Docker Hub에 등록되지 않아서 이미지를 pull 하는 과정에서 오류가 발생하는 경우가 많이 있습니다.

<br>

다음 명령어를 이용해서 클러스터의 이벤트 로그를 조회할 수 있습니다.

~~~
kubectl get events
~~~

다음 명령어로 `kubectl` 설정 정보를 조회할 수 있습니다.

~~~
$ kubectl config view

apiVersion: v1
clusters:
- cluster:
    certificate-authority-data: REDACTED
    server: https://172.31.5.20:6443
  name: kubernetes
contexts:
- context:
    cluster: kubernetes
    user: kubernetes-admin
  name: kubernetes-admin@kubernetes
current-context: kubernetes-admin@kubernetes
kind: Config
preferences: {}
users:
- name: kubernetes-admin
  user:
    client-certificate-data: REDACTED
    client-key-data: REDACTED
~~~

<br>

## Service 생성

기본적으로 Pod를 생성할 경우 클러스터 내부에서만 접근가능한 내부 IP Address로 설정이 됩니다. 그래서 위에서 생성한 `hello-nodejs` 컨테이너를 외부 네트워크에서도 접속할 수 있게 하려면 해당 Pod를 Kubenetes Service로 노출(Expose)시켜줘야 합니다.

~~~
$ kubectl expose deployment hello-nodejs --type=LoadBalancer

service "hello-nodejs" exposed
~~~

위 명령어를 수행한 다음 `kubectl get services` 명령어를 실행하면 다음과 같이 출력됩니다.

~~~
$ kubectl get services

NAME           TYPE           CLUSTER-IP      EXTERNAL-IP   PORT(S)          AGE
hello-nodejs   LoadBalancer   10.103.63.176   <pending>     8080:31853/TCP   19s
kubernetes     ClusterIP      10.96.0.1       <none>        443/TCP          1h
~~~

`--type=LoadBalancer` 옵션을 이용해서 해당 Service를 외부 네트워크에 노출할 수 있습니다. (Load Balance 기능을 제공하는 클라우드 서비스 제공자들이 해당 서비스에 외부 IP Address를 부여할 것입니다.)

이제 웹브라우저를 통해 접속을 해봅니다. 위의 `kubectl get services` 명령어 출력 결과를 보면 `8088` 포트가 `31583` 포트로 포워딩된 것을 알 수 있습니다. 접속 주소를 `[IP Address]:31583`으로 시도해봅니다.

<br>

<br>

## 소스 업데이트

이제 위에서 작성한 `server.js` 파일의 내용을 수정하고 그 내용을 컨테이너에 반영해보도록 합시다.

<pre class="prettyprint">
var http = require('http');

var handleRequest = function(request, response) {
  console.log('Received request for URL: ' + request.url);
  response.writeHead(200);
  response.end('Hello SnowDeer! This is the send version !!!');
};
var www = http.createServer(handleRequest);
www.listen(8080);
</pre>

<br>

Docker 이미지를 새로 빌드하고 Docker Hub에 `push` 합니다.

~~~
docker build -t snowdeer/hello-nodejs:v2 .

docker push snowdeer/hello-nodejs:v2
~~~

<br>

Deployment의 이미지 업데이트를 해줍니다.

~~~
$ kubectl set image deployment/hello-nodejs hello-nodejs=snowdeer/hello-nodejs:v2

deployment "hello-nodejs" image updated
~~~

잠시 후 웹브라우저로 기존 주소에 접속해보면 변경된 내용을 확인할 수 있습니다.