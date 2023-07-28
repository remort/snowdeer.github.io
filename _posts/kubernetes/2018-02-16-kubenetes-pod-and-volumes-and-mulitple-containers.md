---
layout: post
title: Pod, Volumes, Multiple Containers
category: Kubernetes
permalink: /kubernetes/:year/:month/:day/:title/

tag: [Kubernetes, k8s]
---

# Pod

Kubenetes에서 Pod는 하나 이상의 컨테이너 그룹을 의미합니다. 하나의 Pod 안에 있는 컨테이너들은 동시에 배포되며 시작, 정지, 교체(Replicated) 등의 작업도 동시에 이루어집니다.

<br>

## Pod Definition

간단한 Pod 예제는 다음과 같습니다. `yaml` 형태의 파일로 표현합니다.

### pod-nginx.yaml

<pre class="prettyprint">
apiVersion: v1
kind: Pod
metadata:
  name: nginx
spec:
  containers:
  - name: nginx
    image: nginx:1.7.9
    ports:
    - containerPort: 80
</pre>

<br>

## Pod Management

아래의 명령어로 Pod를 생성할 수 있습니다.

<pre class="prettyprint">
$ kubectl create -f ./pod-nginx.yaml

$ kubectl get pods
</pre>

<br>

사용이 끝난 pod는 삭제를 해줍니다.

<pre class="prettyprint">
$ kubectl delete pod nginx
</pre>

<br>

# Volumes

일반적으로 컨테이너 안의 데이터는 컨테이너의 생명 주기가 끝나면 소멸됩니다. 영구적인 데이터 저장을 위해서 Pod에 호스팅 컴퓨터의 저장소를 Volume이라는 이름으로 마운트해서 사용할 수 있습니다.

Volume의 정의는 다음과 같이 할 수 있습니다.

<pre class="prettyprint">
volumes:
    - name: redis-persistent-storage
        emptyDir: {}
</pre>

그리고 컨테이너 안에는 위에서 정의한 Volume과 동일한 이름으로 마운트할 수 있습니다.

<pre class="prettyprint">
volumeMounts:
    # name must match the volume name defined in volumes
    - name: redis-persistent-storage
      # mount path within the container
      mountPath: /data/redis
</pre>

<br>

### pod-redis.yaml

Volume 및 마운트를 정의한 Pod definition은 다음과 같습니다.

<pre class="prettyprint">
apiVersion: v1
kind: Pod
metadata:
  name: redis
spec:
  containers:
  - name: redis
    image: redis
    volumeMounts:
    - name: redis-persistent-storage
      mountPath: /data/redis
  volumes:
  - name: redis-persistent-storage
    emptyDir: {}
</pre>

<br>

Volume 타입으로는 다음과 같은 종류가 있습니다.

- EmptyDir : Pod가 동작하는 동안 존재하는 새로운 디렉토리를 생성함. Pod가 죽거나 재시작되어도 데이터는 유지됨
- HostPath : 호스트에 이미 존재하는 파일 시스템 (ex. /var/logs)

<br>

# Multiple Containers

다음과 같이 Pod definition 안에는 복수의 컨테이너를 정의를 할 수 있습니다.

<pre class="prettyprint">
apiVersion: v1
kind: Pod
metadata:
  name: www
spec:
  containers:
  - name: nginx
    image: nginx
    volumeMounts:
    - mountPath: /srv/www
      name: www-data
      readOnly: true
  - name: git-monitor
    image: kubernetes/git-monitor
    env:
    - name: GIT_REPO
      value: http://github.com/some/repo.git
    volumeMounts:
    - mountPath: /data
      name: www-data
  volumes:
  - name: www-data
    emptyDir: {}
</pre>

Volume은 한 군데서 정의를 했지만 `nginx`와 `git-monitor`라는 이름의 컨테이너에서 해당 Volume을 동시에 참고하고 있는 것을 확인할 수 있습니다.
