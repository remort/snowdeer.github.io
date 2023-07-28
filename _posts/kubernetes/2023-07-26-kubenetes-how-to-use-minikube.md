---
layout: post
title: Minikube 사용방법
category: Kubernetes
permalink: /kubernetes/:year/:month/:day/:title/

tag: [Kubernetes, k8s]
---

# Minikube 사용방법

| 명령어               | 설명                              |
| -------------------- | --------------------------------- |
| minikube start       | minikube 가상머신 실행            |
| minikube stop        | minikube 가상머신 정지            |
| minikube delete      | 가상머신 제거                     |
| minikube status      | 상태 확인                         |
| minikube ip          | IP 주소 확인                      |
| minikube ssh         | ssh 접속                          |
| minikube addons list | 애드온 목록 조회                  |
| minikube dashboard   | 브라우저로 minikube 대시보드 접속 |

## 대시보드 url 확인

<pre class="prettyprint">
$ minikube dashboard  --url

🤔  Verifying dashboard health ...
🚀  Launching proxy ...
🤔  Verifying proxy health ...
http://127.0.0.1:53502/api/v1/namespaces/kubernetes-dashboard/services/http:kubernetes-dashboard:/proxy/
</pre>
