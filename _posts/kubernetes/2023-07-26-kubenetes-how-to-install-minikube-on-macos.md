---
layout: post
title: MAC OS에 minikube 설치하는 방법
category: Kubernetes
permalink: /kubernetes/:year/:month/:day/:title/

tag: [Kubernetes, k8s]
---

# MAC OS에 minikube 설치

다음과 같이 `brew`를 이용해서 설치하면 됩니다.

<pre class="prettyprint">
$ brew install minikube

Running `brew update --auto-update`...
==> Downloading https://ghcr.io/v2/homebrew/portable-ruby/portable-ruby/blobs/sha256:905b0c3896164ae8067a22fff2fd0b80b16d3c8bb72441403eedf69da71ec717
################################################################################################################# 100.0%
==> Pouring portable-ruby-2.6.10_1.arm64_big_sur.bottle.tar.gz
==> Homebrew collects anonymous analytics.
Read the analytics documentation (and how to opt-out) here:
  https://docs.brew.sh/Analytics
No analytics have been recorded yet (nor will be during this `brew` run).

Installing from the API is now the default behaviour!
You can save space and time by running:
  brew untap homebrew/core
  brew untap homebrew/cask
==> Downloading https://formulae.brew.sh/api/formula.jws.json
################################################################################################################# 100.0%
==> Downloading https://formulae.brew.sh/api/cask.jws.json
################################################################################################################# 100.0%
==> Fetching dependencies for minikube: kubernetes-cli
==> Fetching kubernetes-cli

...


==> `brew cleanup` has not been run in the last 30 days, running now...
Disable this behaviour by setting HOMEBREW_NO_INSTALL_CLEANUP.
Hide these hints with HOMEBREW_NO_ENV_HINTS (see `man brew`).
Removing: /Users/snowdeer/Library/Caches/Homebrew/nvm--0.39.3... (47.2KB)
Removing: /Users/snowdeer/Library/Caches/Homebrew/nvm_bottle_manifest--0.39.3... (1.7KB)
Removing: /Users/snowdeer/Library/Logs/Homebrew/nvm... (64B)
==> Caveats
==> minikube
zsh completions have been installed to:
  /opt/homebrew/share/zsh/site-functions
</pre>

만약 `brew`를 사용하지 않으면 다음 명령어를 이용해서 minikube를 설치할 수 있습니다.

<pre class="prettyprint">
$ curl -LO https://storage.googleapis.com/minikube/releases/latest/minikube-darwin-arm64
$ sudo install minikube-darwin-arm64 /usr/local/bin/minikube
</pre>

## 설치 확인

<pre class="prettyprint">
$ minikube version

minikube version: v1.31.1
commit: fd3f3801765d093a485d255043149f92ec0a695f
</pre>

## minikube 실행

<pre class="prettyprint">
$ minikube start

😄  minikube v1.31.1 on Darwin 12.4 (arm64)
✨  Automatically selected the docker driver
📌  Using Docker Desktop driver with root privileges
👍  Starting control plane node minikube in cluster minikube
🚜  Pulling base image ...
💾  Downloading Kubernetes v1.27.3 preload ...
    > preloaded-images-k8s-v18-v1...:  327.72 MiB / 327.72 MiB  100.00% 1.65 Mi
    > gcr.io/k8s-minikube/kicbase...:  404.50 MiB / 404.50 MiB  100.00% 1.76 Mi
🔥  Creating docker container (CPUs=2, Memory=1988MB) ...
🐳  Preparing Kubernetes v1.27.3 on Docker 24.0.4 ...
    ▪ Generating certificates and keys ...
    ▪ Booting up control plane ...
    ▪ Configuring RBAC rules ...
🔗  Configuring bridge CNI (Container Networking Interface) ...
    ▪ Using image gcr.io/k8s-minikube/storage-provisioner:v5
🔎  Verifying Kubernetes components...
🌟  Enabled addons: storage-provisioner, default-storageclass
🏄  Done! kubectl is now configured to use "minikube" cluster and "default" namespace by default
</pre>

## kubectl 명령어로 pods 정보 확인

<pre class="prettyprint">
$ kubectl get po -A

NAMESPACE              NAME                                         READY   STATUS    RESTARTS        AGE
kube-system            coredns-5d78c9869d-bmsck                     1/1     Running   0               3m28s
kube-system            etcd-minikube                                1/1     Running   0               3m41s
kube-system            kube-apiserver-minikube                      1/1     Running   0               3m41s
kube-system            kube-controller-manager-minikube             1/1     Running   0               3m41s
kube-system            kube-proxy-qcp9m                             1/1     Running   0               3m28s
kube-system            kube-scheduler-minikube                      1/1     Running   0               3m41s
kube-system            storage-provisioner                          1/1     Running   1 (2m57s ago)   3m39s
kubernetes-dashboard   dashboard-metrics-scraper-5dd9cbfd69-nvbmj   1/1     Running   0               113s
kubernetes-dashboard   kubernetes-dashboard-5c5cfc8747-8g7mn        1/1     Running   0               113s
</pre>

## Reference

- https://minikube.sigs.k8s.io/docs/start/
