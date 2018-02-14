---
layout: post
title: Cluster 생성하기 (kubeadm 이용)
category: Kubernetes
permalink: /kubernetes/:year/:month/:day/:title/

tag: [Kubernetes]
---
# Create your Kubenetes Cluster

Kubenetes에서 클러스터(Cluster)를 생성하는 방법입니다. 보다 자세한 내용은 [여기를 참고](https://kubernetes.io/docs/setup/independent/create-cluster-kubeadm/)하세요.

저는 편의상 AWS에 인스턴스를 3개 만들어서 테스트해보았습니다. 각각의 인스턴스 이름을 `master`, `node1`, `node2`라고 정의했습니다.

<br>

## 각 인스턴스에 Docker 및 Kubenetes 설치

각 인스턴스에 Docker와 Kubenetes를 설치합니다. 설치 방법은 [여기를 참고](/kubernetes/2018/02/10/how-to-install-kubenetes/)하시면 됩니다.

<br>

## Master 설정

`master` 노드에서 다음 명령어를 입력합니다.

~~~
sudo kubeadm init
~~~ 

설치는 몇 분간 걸리기 때문에 기다려줍니다.

~~~
$ sudo kubeadm init

[init] Using Kubernetes version: v1.9.3
[init] Using Authorization modes: [Node RBAC]
[preflight] Running pre-flight checks.
        [WARNING FileExisting-crictl]: crictl not found in system path
[certificates] Generated ca certificate and key.
[certificates] Generated apiserver certificate and key.
[certificates] apiserver serving cert is signed for DNS names [ip-172-31-1-36 kubernetes kubernetes.default kubernetes.default.svc kubernetes.default.svc.cluster.local] and IPs [10.96.0.1 172.31.1.36]
[certificates] Generated apiserver-kubelet-client certificate and key.
[certificates] Generated sa key and public key.
[certificates] Generated front-proxy-ca certificate and key.
[certificates] Generated front-proxy-client certificate and key.
[certificates] Valid certificates and keys now exist in "/etc/kubernetes/pki"
[kubeconfig] Wrote KubeConfig file to disk: "admin.conf"
[kubeconfig] Wrote KubeConfig file to disk: "kubelet.conf"
[kubeconfig] Wrote KubeConfig file to disk: "controller-manager.conf"
[kubeconfig] Wrote KubeConfig file to disk: "scheduler.conf"
[controlplane] Wrote Static Pod manifest for component kube-apiserver to "/etc/kubernetes/manifests/kube-apiserver.yaml"
[controlplane] Wrote Static Pod manifest for component kube-controller-manager to "/etc/kubernetes/manifests/kube-controller-manager.yaml"
[controlplane] Wrote Static Pod manifest for component kube-scheduler to "/etc/kubernetes/manifests/kube-scheduler.yaml"
[etcd] Wrote Static Pod manifest for a local etcd instance to "/etc/kubernetes/manifests/etcd.yaml"
[init] Waiting for the kubelet to boot up the control plane as Static Pods from directory "/etc/kubernetes/manifests".
[init] This might take a minute or longer if the control plane images have to be pulled.
[apiclient] All control plane components are healthy after 90.001359 seconds
[uploadconfig] Storing the configuration used in ConfigMap "kubeadm-config" in the "kube-system" Namespace
[markmaster] Will mark node ip-172-31-1-36 as master by adding a label and a taint
[markmaster] Master ip-172-31-1-36 tainted and labelled with key/value: node-role.kubernetes.io/master=""
[bootstraptoken] Using token: f4938e.4b23af938d801cf6
[bootstraptoken] Configured RBAC rules to allow Node Bootstrap tokens to post CSRs in order for nodes to get long term certificate credentials
[bootstraptoken] Configured RBAC rules to allow the csrapprover controller automatically approve CSRs from a Node Bootstrap Token
[bootstraptoken] Configured RBAC rules to allow certificate rotation for all node client certificates in the cluster
[bootstraptoken] Creating the "cluster-info" ConfigMap in the "kube-public" namespace
[addons] Applied essential addon: kube-dns
[addons] Applied essential addon: kube-proxy

Your Kubernetes master has initialized successfully!

To start using your cluster, you need to run the following as a regular user:

  mkdir -p $HOME/.kube
  sudo cp -i /etc/kubernetes/admin.conf $HOME/.kube/config
  sudo chown $(id -u):$(id -g) $HOME/.kube/config

You should now deploy a pod network to the cluster.
Run "kubectl apply -f [podnetwork].yaml" with one of the options listed at:
  https://kubernetes.io/docs/concepts/cluster-administration/addons/

You can now join any number of machines by running the following on each node
as root:

  kubeadm join --token f4938e.4b23af938d801cf6 172.31.1.36:6443 --discovery-token-ca-cert-hash sha256:aecc8acc0450992c780cb4b809e50b5583e37d7403480a43d0e2c3b67037c25f

ubuntu@ip-172-31-1-36:~$
~~~

위와 같은 메시지가 나오면 설치가 완료된 것입니다. 마지막 부분에 있는

~~~
kubeadm join --token f4938e.4b23af938d801cf6 172.31.1.36:6443 --discovery-token-ca-cert-hash sha256:aecc8acc0450992c780cb4b809e50b5583e37d7403480a43d0e2c3b67037c25f
~~~

부분을 메모해둡니다.

<br>

## Pod Network Add-On 설치

본격적인 통신을 하기 위해서는 Pod Network Add-On을 설치해야 합니다. Kube-DNS를 비록하여 각종 어플리케이션을 배포(Deploy)하기 전에 네트워크 설정부터 해야 합니다. 네트워크 설정이 되기 전까지는 각 어플들은 실행되지 않을 것입니다.

Network Add-On은 다음과 같은 종류들이 있으며, 각 클러스터마다 하나의 Network Add-On만 설치 가능합니다.

* Calico
* Canal
* Flannel
* Kube-router
* Romana
* Weave Net

예를 들어 `Weave Net`은 다음 스크립트를 이용해서 설치할 수 있습니다.

~~~
export kubever=$(kubectl version | base64 | tr -d '\n')
kubectl apply -f "https://cloud.weave.works/k8s/net?k8s-version=$kubever"
~~~

Pod Network가 설치되고 나면 `kube-dns` Pod가 Running 상태로 바뀌게 됩니다. `kubectl get pods --all-namespaces` 명령어를 이용해서 확인할 수 있습니다.

~~~
$ kubectl get pods --all-namespaces

NAMESPACE     NAME                                     READY     STATUS    RESTARTS   AGE
kube-system   etcd-ip-172-31-1-36                      1/1       Running   0          18m
kube-system   kube-apiserver-ip-172-31-1-36            1/1       Running   0          18m
kube-system   kube-controller-manager-ip-172-31-1-36   1/1       Running   0          19m
kube-system   kube-dns-6f4fd4bdf-z9mvq                 3/3       Running   0          19m
kube-system   kube-proxy-8rqmn                         1/1       Running   0          16m
kube-system   kube-proxy-bhjmm                         1/1       Running   0          19m
kube-system   kube-proxy-fqppb                         1/1       Running   0          17m
kube-system   kube-scheduler-ip-172-31-1-36            1/1       Running   0          19m
kube-system   weave-net-2tgkz                          2/2       Running   0          3m
kube-system   weave-net-jw88d                          2/2       Running   0          3m
kube-system   weave-net-wlxvd                          2/2       Running   0          3m
~~~

<br>

## 노드 연결

`node1`과 `node2`에서 위에서 메모해둔 `kubeadm join` 스크립트를 실행합니다.

~~~
$ sudo kubeadm join --token f4938e.4b23af938d801cf6 172.31.1.36:6443 --discovery-token-ca-cert-hash sha256:aecc8acc0450992c780cb4b809e50b5583e37d7403480a43d0e2c3b67037c25f

[preflight] Running pre-flight checks.
        [WARNING FileExisting-crictl]: crictl not found in system path
[discovery] Trying to connect to API Server "172.31.1.36:6443"
[discovery] Created cluster-info discovery client, requesting info from "https://172.31.1.36:6443"
[discovery] Requesting info from "https://172.31.1.36:6443" again to validate TLS against the pinned public key
[discovery] Cluster info signature and contents are valid and TLS certificate validates against pinned roots, will use API Server "172.31.1.36:6443"
[discovery] Successfully established connection with API Server "172.31.1.36:6443"

This node has joined the cluster:
* Certificate signing request was sent to master and a response
  was received.
* The Kubelet was informed of the new secure connection details.

Run 'kubectl get nodes' on the master to see this node join the cluster.
~~~

<br>

## 연결된 Node 확인

이제 `master` 노드가 생성되었고, `node1`과 `node2`가 연결(Join)되었습니다. `master` 노드에서 다음 명령어를 이용해서 노드 리스트를 조회할 수 있습니다.

~~~
$ kubectl get nodes

NAME               STATUS    ROLES     AGE       VERSION
ip-172-31-1-36     Ready     master    21m       v1.9.3
ip-172-31-15-223   Ready     <none>    18m       v1.9.3
ip-172-31-2-21     Ready     <none>    18m       v1.9.3
~~~

<br>

## 종료하기

지금까지 `kubeadm`을 이용해서 작업한 내용을 종료시키기 위해서는 각 노드들을 먼저 정리를 해야 합니다. 다음 명령어를 이용해서 각 노드들의 데이터를 지울 수 있습니다.

~~~
kubectl drain <node name> --delete-local-data --force --ignore-daemonsets

kubectl delete node <node name>
~~~

노드가 제거 완료되면 `kubeadm reset` 명령어를 이용해서 모든 상태를 원래대로 돌립니다.

~~~
kubeadm reset
~~~
