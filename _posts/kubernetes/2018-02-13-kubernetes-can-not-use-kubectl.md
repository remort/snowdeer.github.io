---
layout: post
title: The connection to the server localhost:8080 was refused 오류 발생시
category: Kubernetes
permalink: /kubernetes/:year/:month/:day/:title/

tag: [Kubernetes, k8s]
---

# The connection to the server localhost:8080 was refused 오류 해결법

`kubectl` 명령어를 입력하면 다음 오류 메시지가 발생하는 경우가 있습니다.

```
The connection to the server localhost:8080 was refused - did you specify the right host or port?
```

이 문제는 다음 명령어를 입력하면 해결이 됩니다.

```
mkdir -p $HOME/.kube
sudo cp -i /etc/kubernetes/admin.conf $HOME/.kube/config
sudo chown $(id -u):$(id -g) $HOME/.kube/config
```

위에서 `admin.conf` 파일은 `kubeadm init` 명령어를 수행했을 때 생성됩니다. 즉, `master` 노드에서만 `kubectl` 명령어를 사용할 수 있으며, 다른 노드에서 `kubectl` 명령어를 사용하고 싶을 때는 `master` 노드에서 생성한 `admin.conf` 파일을 복사해오면 일반 노드에서도 `kubectl` 명령어를 사용할 수 있습니다.
