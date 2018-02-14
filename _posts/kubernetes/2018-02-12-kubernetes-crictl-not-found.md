---
layout: post
title: crictl not found in system path 오류 발생시
category: Kubernetes
permalink: /kubernetes/:year/:month/:day/:title/

tag: [Kubernetes]
---
# crictl not found in system path 오류 해결법

Kubenetes의 `kubeadm init` 명령어를 실행했을 때 나오는 `crictl not found in system path` 오류는 경고(Warning) 메시지입니다. 무시하고 진행을 해도 되지만 경고 메시지를 아예 없애고 싶을 때는 crictl을 설치하면 됩니다.

crictl 소스 코드는 [여기](https://github.com/kubernetes-incubator/cri-tools/tree/master/cmd/crictl)에 있습니다.

Go 언어로 되어 있기 때문에 먼저 Go 언어를 설치해야 합니다. 그런 다음 아래의 명령어를 이용해서 crictl을 설치할 수 있습니다.

~~~
go get github.com/kubernetes-incubator/cri-tools/cmd/crictl
~~~