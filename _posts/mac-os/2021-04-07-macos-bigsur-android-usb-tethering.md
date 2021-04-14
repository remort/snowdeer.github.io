---
layout: post
title: BigSur에서 안드로이드로 USB 테더링하기

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# USB 테더링

기존에 [포스팅](http://snowdeer.github.io/mac-os/2017/02/15/use-android-phone-tethering-on-macbook/)한 내용이 있는데,
BigSur 버전에서는 추가적인 별도의 작업을 해줘야 해서 포스팅을 다시 합니다.

<br>

## HoRNDIS 9.2

기존과 마찬가지로 [HoRNDIS](https://joshuawise.com/horndis#available_versions)를 설치해줘야 합니다.
최신 버전인 `9.2` 버전을 다운도르하면 됩니다. 다만, BigSur에서는 설치 마지막 단계에서 실패가 됩니다.

<br>

## HoRNDIS 개발자 ID

다음 내용을 기억해줍시다. 생략해도 됩니다. 다만, 개발자 ID가 다를까봐 다시 한 번 확인하는 과정입니다.

<pre class="prettyprint">
$ sudo su

$ spctl -a -vv -t install /Library/Extensions/HoRNDIS.kext
/Library/Extensions/HoRNDIS.kext: accepted 
source=Notarized Developer ID 
origin=Developer ID Application: Joshua Wise (54GTJ2AU36)
</pre>

<br>

## 복구 모드 진입

그런 다음 맥북 종료 후

<kbd>command</kbd> + <kbd>R</kbd> 버튼을 눌러서 재부팅하면 복구모드로 진입합니다.
그리고 터미널을 실행한 다음, 아래 명령어를 입력합니다.

<pre class="prettyprint">
$ csrutil disable
System Integrity Protection is off.

$ /usr/sbin/spctl kext-consent list
spctl: no kext consent configuration found.

$ /usr/sbin/spctl kext-consent add 54GTJ2AU36
$ /usr/sbin/spctl kext-consent list
Allowed Team Identifiers:
54GTJ2AU36
</pre>

<br>

## 노멀 부팅

다시 맥북을 재부팅 합니다. 노멀 모드로 진입한 다음 다음 명령어를 실행합니다.

<pre class="prettyprint">
$ sudo su 
$ sqlite3 /var/db/SystemPolicyConfiguration/KextPolicy 
SQLite version 3.32.3 2020-06-18 14:16:19 
Enter ".help" for usage hints. 

sqlite> delete from kext_load_history_v3 where team_id='54GTJ2AU36'; 
sqlite> delete from kext_policy where team_id='54GTJ2AU36'; 
sqlite> .quit 
</pre>

그 이후 다음 파일을 삭제해줍니다.

<pre class="prettyprint">
$ cd /Library/Extensions

$ sudo rm -rf HoRNDIS.kext
</pre>

<br>

## HoRNDIS 재설치

그 이후 다시, `HoRNDIS` 프로그램을 재설치해줍니다. 그리고 다음 권한을 허용해줍니다.
`System Preferences > Security & Privacy`에서 `General` 탭 하단에서 `Joshua Wise` 개발자 항목을 허용해줍니다.

이제 재부팅 후 안드로이드 폰과 USB 케이블 연결 후 테더링을 실행해보면 잘 동작하는 것을 알 수 있습니다.