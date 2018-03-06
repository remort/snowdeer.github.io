---
layout: post
title: OpenShift Cluster UP/DOWN 스크립트(AWS 기준)
category: OpenShift
permalink: /openshift/:year/:month/:day/:title/

tag: [OpenShift]
---
# OpenShift Cluster UP/DOWN

AWS가 아닌 다른 서버에서 OpenShift의 클러스터(Cluster)를 UP/DOWN 시킬 때는 단순히

<pre class="prettyprint">
$ oc cluster up

$ oc cluster down
</pre>

명령어만 사용하면 됩니다.

AWS에서 클러스터 UP을 하기 위해서는 다음 스크립트를 실행하면 됩니다.

<pre class="prettyprint">
metadata_endpoint="http://169.254.169.254/latest/meta-data"

public_hostname="$( curl "${metadata_endpoint}/public-hostname" )"

public_ip="$( curl "${metadata_endpoint}/public-ipv4" )"

oc cluster up --public-hostname="${public_hostname}" --routing-suffix="${public_ip}.nip.io"
</pre>

클러스터 Down 명령어는 동일합니다.