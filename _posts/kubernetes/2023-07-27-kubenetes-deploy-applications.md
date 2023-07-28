---
layout: post
title: Minikube에 Application Deploy 방법
category: Kubernetes
permalink: /kubernetes/:year/:month/:day/:title/

tag: [Kubernetes, k8s]
---

# Minikube에 Application Deploy 방법

## Service

아래와 같이 샘플 Service를 생성할 수 있습니다.

<pre class="prettyprint">
$ kubectl create deployment hello-minikube --image=kicbase/echo-server:1.0

deployment.apps/hello-minikube created
</pre>

<pre class="prettyprint">
$ kubectl expose deployment hello-minikube --type=NodePort --port=8080

service/hello-minikube exposed
</pre>

잠시후 다음 명령어를 통해 위에서 생성한 `hello-minikube` 서비스 상태를 확인할 수 있습니다.

<pre class="prettyprint">
$ kubectl get services hello-minikube

NAME             TYPE       CLUSTER-IP      EXTERNAL-IP   PORT(S)          AGE
hello-minikube   NodePort   10.105.140.39   <none>        8080:30634/TCP   59s
</pre>

여기서 실제 브라우저로 동작을 확인하고 싶으면 아래 명령어를 실행하면 됩니다.

<pre class="prettyprint">
$ minikube service hello-minikube

|-----------|----------------|-------------|---------------------------|
| NAMESPACE |      NAME      | TARGET PORT |            URL            |
|-----------|----------------|-------------|---------------------------|
| default   | hello-minikube |        8080 | http://192.168.49.2:30634 |
|-----------|----------------|-------------|---------------------------|
🏃  Starting tunnel for service hello-minikube.
|-----------|----------------|-------------|------------------------|
| NAMESPACE |      NAME      | TARGET PORT |          URL           |
|-----------|----------------|-------------|------------------------|
| default   | hello-minikube |             | http://127.0.0.1:53680 |
|-----------|----------------|-------------|------------------------|
🎉  Opening service default/hello-minikube in default browser...
❗  Because you are using a Docker driver on darwin, the terminal needs to be open to run it.
</pre>

그러면 자동으로 브라우저가 열리며, `hello-minikube` 페이지를 볼 수 있습니다.

### Port forwarding

<pre class="prettyprint">
$ kubectl port-forward service/hello-minikube 7080:8080

Forwarding from 127.0.0.1:7080 -> 8080
Forwarding from [::1]:7080 -> 8080
</pre>

이 경우에는 http://localhost:7080/ 주소에서 위에서 만든 웹페이지를 확인할 수 있습니다.

## LoadBalancer

<pre class="prettyprint">
$ kubectl create deployment balanced --image=kicbase/echo-server:1.0

deployment.apps/balanced created
</pre>

<pre class="prettyprint">
$ kubectl expose deployment balanced --type=LoadBalancer --port=8080

service/balanced exposed
</pre>

다른 터미널 창을 열고 아래 명령어를 이용해서 `tunnel`을 실행해서 `balanced` 배포 대상의 routable IP를 생성해줍니다.

<pre class="prettyprint">
$ minikube tunnel

✅  Tunnel successfully started
📌  NOTE: Please do not close this terminal as this process must stay alive for the tunnel to be accessible ...
🏃  Starting tunnel for service balanced.
</pre>

<pre class="prettyprint">
$ kubectl get services balanced

NAME       TYPE           CLUSTER-IP      EXTERNAL-IP   PORT(S)          AGE
balanced   LoadBalancer   10.99.117.109   127.0.0.1     8080:31198/TCP   83s

# 만약 위에서 `tunnel`을 실행하지 않았다면 아래와 같이 출력됩니다.
$ kubectl get services balanced

NAME       TYPE           CLUSTER-IP      EXTERNAL-IP   PORT(S)          AGE
balanced   LoadBalancer   10.99.117.109   <pending>     8080:31198/TCP   2m41s
</pre>

그리고 브라우저에서 `http://127.0.0.1:8080`에 접속해서 위의 `EXTERNAL-IP`가 잘 작동하는지 확인할 수 있습니다.

## Ingress

Ingress를 사용하기 위해서는 아래 명령어로 minikube에 addon을 설치해줍니다.

<pre class="prettyprint">
$ minikube addons enable ingress

💡  ingress is an addon maintained by Kubernetes. For any concerns contact minikube on GitHub.
You can view the list of minikube maintainers at: https://github.com/kubernetes/minikube/blob/master/OWNERS
💡  After the addon is enabled, please run "minikube tunnel" and your ingress resources would be available at "127.0.0.1"
    ▪ Using image registry.k8s.io/ingress-nginx/kube-webhook-certgen:v20230407
    ▪ Using image registry.k8s.io/ingress-nginx/controller:v1.8.1
    ▪ Using image registry.k8s.io/ingress-nginx/kube-webhook-certgen:v20230407
🔎  Verifying ingress addon...
🌟  The 'ingress' addon is enabled
</pre>

<pre class="prettyprint">
$ kubectl apply -f https://storage.googleapis.com/minikube-site-examples/ingress-example.yaml

pod/foo-app created
service/foo-service created
pod/bar-app created
service/bar-service created
ingress.networking.k8s.io/example-ingress created
</pre>

#### ingress-example.yaml

위 파일을 브라우저 등으로 다운로드해보면 아래와 같은 내용이 입력되어 있는 것을 확인할 수 있습니다.

<pre class="prettyprint">
kind: Pod
apiVersion: v1
metadata:
  name: foo-app
  labels:
    app: foo
spec:
  containers:
    - name: foo-app
      image: 'kicbase/echo-server:1.0'
---
kind: Service
apiVersion: v1
metadata:
  name: foo-service
spec:
  selector:
    app: foo
  ports:
    - port: 8080
---
kind: Pod
apiVersion: v1
metadata:
  name: bar-app
  labels:
    app: bar
spec:
  containers:
    - name: bar-app
      image: 'kicbase/echo-server:1.0'
---
kind: Service
apiVersion: v1
metadata:
  name: bar-service
spec:
  selector:
    app: bar
  ports:
    - port: 8080
---
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: example-ingress
spec:
  rules:
    - http:
        paths:
          - pathType: Prefix
            path: /foo
            backend:
              service:
                name: foo-service
                port:
                  number: 8080
          - pathType: Prefix
            path: /bar
            backend:
              service:
                name: bar-service
                port:
                  number: 8080
---
</pre>

### Ingress 확인

<pre class="prettyprint">
$ kubectl get ingress

NAME              CLASS   HOSTS   ADDRESS        PORTS   AGE
example-ingress   nginx   *       192.168.49.2   80      48s
</pre>

다시 새로운 터미널을 열고 `minikube tunnel` 명령어를 이용해서 tunnel을 실행한다음,
브라우저에서는 `http://127.0.0.1/foo` 또는 `http://127.0.0.1/bar`로 ingress 및 LoadBalancer가 실행되고 있는 것을 확인할 수 있습니다.

## Reference

- https://minikube.sigs.k8s.io/docs/start/
