---
layout: post
title: OpenShift origin 설치 방법
category: openshift
permalink: /openshift/:year/:month/:day/:title/

tag: [OpenShift]
---
# OpenShift origin

OpenShift origin 설치 방법에 대한 자세한 내용은 [여기를 참고](https://docs.openshift.org/latest/getting_started/administrators.html#getting-started-administrators)하시면 됩니다.

아래 포스팅은 제가 직접 OpenShift origin을 구동시킨 방법입니다.

Docker를 이용한 컨테이너로 구동하는 방법과 Binary 파일을 직접받아 설치하는 방법이 있는데, 저는 후자를 선택했습니다. 또한 설치 OS는 'CentOS'를 선택했습니다.

<br>

## Docker 설치 및 실행

<pre class="prettyprint">
sudo yum install -y yum-utils

sudo yum-config-manager --add-repo https://download.docker.com/linux/centos/docker-ce.repo

sudo yum makecache fast

sudo yum install -y docker-ce

sudo systemctl start docker

sudo systemctl enable docker

sudo gpasswd -a centos docker
</pre>

<br>

## Docker insecure registry 설정

기본적으로 Docker는 Docker Registry 접속을 위해 `https`를 사용하도록 되어 있습니다. 하지만 `http`를 이용한 insecure registry 세팅을 해주어야 하는 경우가 있습니다.

CentOS 기준으로 다음과 같이 세팅해줍니다.

더 자세한 내용은 [여기를 참조](https://github.com/openshift/origin/blob/master/docs/cluster_up_down.md)하세요.

<pre class="prettyprint">
sudo sysctl -w net.ipv4.ip_forward=1

sudo su

cat&lt;&lt;EOF&gt;&gt; /etc/docker/daemon.json
{
    "insecure-registries" : ["172.30.0.0/16"]
}
EOF

exit

sudo systemctl daemon-reload

sudo systemctl restart docker
</pre>

<br>

## OpenShift 설치

OpenShift 최신 버전은 [여기에서 확인](https://github.com/openshift/origin/releases)할 수 있습니다. 현재 최신 버전인 3.9.0은 알파 버전이라 저는 3.7.1 버전으로 설치를 진행했습니다.

<pre class="prettyprint">
wget https://github.com/openshift/origin/releases/download/v3.7.1/openshift-origin-server-v3.7.1-ab0f056-linux-64bit.tar.gz

tar -xvzf openshift-origin-server-v3.7.1-ab0f056-linux-64bit.tar.gz

mv openshift-origin-server-v3.7.1-ab0f056-linux-64bit openshift-v3.7.1

cd openshift-v3.7.1

sudo cp openshift oc kubectl /usr/local/bin
</pre>

<br>

## Cluster Up

그런 다음 다음 명령어를 이용해서 Cluster Up을 해줄 수 있습니다. 

<pre class="prettyprint">
oc cluster up
</pre>

하지만, 만약 AWS EC2의 가상 머신에서 작업하는 경우에는 다음과 같은 작업으로 해주어야 합니다.

<pre class="prettyprint">
metadata_endpoint="http://169.254.169.254/latest/meta-data"

public_hostname="$( curl "${metadata_endpoint}/public-hostname" )"

public_ip="$( curl "${metadata_endpoint}/public-ipv4" )"

oc cluster up --public-hostname="${public_hostname}" --routing-suffix="${public_ip}.nip.io"
</pre>

다음과 같은 결과가 나오면 성공입니다.

<pre class="prettyprint">
$ oc cluster up --public-hostname="${public_hostname}" --routing-suffix="${public_ip}.nip.io"

Starting OpenShift using openshift/origin:v3.7.1 ...
OpenShift server started.

The server is accessible via web console at:
    https://ec2-13-125-14-154.ap-northeast-2.compute.amazonaws.com:8443

You are logged in as:
    User:     developer
    Password: &lt;any value&gt;

To login as administrator:
    oc login -u system:admin
</pre>

이제 브라우저를 이용해서 위에 나온 Web Console 주소로 접속해서 사용해보면 됩니다.