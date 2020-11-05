---
layout: post
title: MS Surface Pro 7에 Ubuntu 20.04 설치하기

category: Linux
permalink: /mac-os/:year/:month/:day/:title/
tag: [리눅스]
---
MS Surface Pro 7에 Ubuntu 20.04를 설치하는 방법입니다. 
이 글을 포스팅하는 시점에 Ubuntu 최신 버전은 20.10인데, 여기서는 그냥 20.04 LTS 버전으로 설치합니다.
(어차피 20.10 버전도 같은 방법입니다.)

## Surface Pro 윈도우에서 할 일

먼저 암호화 시스템인 `BitLocker`를 해제합니다. 기본 세팅으로 SSD 전체에 BitLocker가 걸려 있더군요. 
탐색기 등에서 C 드라이브를 선택한다음 마우스 오른 버튼을 이용한 팝업에서 'BitLocker 관리'를 선택합니다.
그 다음 'BitLocker 끄기'를 선택하면 됩니다. 경우에 따라서는 `BitLocker` 프로그램을 설치하는 작업을 
거칠 수도 있습니다.

<br>

## 파티션 나누기

저는 윈도우즈 상에서 `하드 디스크 파티션 만들기 및 포맷` 기능을 이용해서 파티션을 나누었습니다.
256GB SSD라 적당히 100GB 및 나머지 용량으로 나누었습니다.
이 화면에서도 BitLocker가 정상적으로 적용/해제되었는지 여부를 확인할 수도 있습니다.

<br>

## Ubuntu 부팅 디스크 만들기

이 부분은 생략합니다. 우분투 이미지 다운로드는 [여기](https://ubuntu.com/download/desktop)에서 
할 수 있습니다. USB 디스크에 부팅 디스크 옵션으로 만들면 됩니다.

<br>

## UEFI 해제

`UEFI` 옵션은 Surface Pro에 Windows가 아닌 다른 OS가 설치되었을 경우 해당 부팅되지 않도록 해주는 보안 옵션입니다.
우리는 Ubuntu를 사용할 것이기 때문에 `UEFI` 옵션을 해제시켜줍니다.

`BIOS`에서 해제할 수 있는데, 들어가는 방법은 Surface Pro 전원이 꺼진 상태에서 <kbd>전원</kbd> + <kbd>Volume Up</kbd> 키를
오래동안 누르고 있으면 됩니다.

<br>

## Ubuntu 설치

이제 USB로 부팅을 하기 위해서 <kbd>전원</kbd> + <kbd>Volume Down</kbd> 카를 이용해서 전원을 넣습니다.
USB로 부팅한 다음 Ubuntu 설치 방법은 특별하지 않습니다. 기존에 Ubuntu 설치하듯이 하시면 됩니다.

<br>

## Surface에 Ubuntu를 설치했을 때 발생하는 문제점 해결하기

Surface Pro에 Ubuntu를 설치했을 때 남은 배터리 잔량이 표시안되거나 터치 스크린이 안되는 문제, 재부팅시 Freeze 되는 현상 등이
있습니다. (Surface Pro 7 가준)

다음 방법을 통해 해결할 수 있습니다.

<br>

### 배터리 잔량 표시

커널을 Surface 용으로 설치하면 해결됩니다. 설치하는 방법은 다음과 같습니다.

<pre class="prettyprint">
wget -qO - https://raw.githubusercontent.com/linux-surface/linux-surface/master/pkg/keys/surface.asc \
    | sudo apt-key add -

echo "deb [arch=amd64] https://pkg.surfacelinux.com/debian release main" | sudo tee /etc/apt/sources.list.d/linux-surface.list

sudo apt-get update
sudo apt-get install linux-headers-surface linux-image-surface surface-ipts-firmware libwacom-surface iptsd
</pre>

이와 같이 설치한다음 재부팅을 하면 새로운 커널을 이용해서 부팅을 하면 됩니다. 새로운 커널 확인은 `uname -a` 명령어를 이용해서 확인할 수 있습니다.
현재 Surface 커널 최신 버전은 5.9입니다.

<br>

## Touch 스크린 활성화

위에서 커널을 새로 설치한 다음 `systemctl start iptsd.service` 명령을 수행하면 터치 스크린이 활성화됩니다. 만약 재부팅 후에도
서비스를 계속 활성화하고 싶으면 `sudo systemctl enable iptsd.service` 명령어를 실행하면 됩니다.

현재는 멀티 터치는 지원하지 않고 싱글 터치만 지원합니다.

<br>

## Reboot 시 Freeze 되는 현상 수정

`Boot Parameter`에 `reboot=pci`를 추가하면 됩니다. `Boot Parameter`는 다음 방법을 이용해서 수정할 수 있습니다.

<pre class="prettyprint">
sudo code /etc/default/grub
</pre>

그 이후 `GRUB_CMDLINE_LINUX_DEFAULT` 변수에 `reboot=pci`룰 추가하고 저장을 합니다.
아래 명령어를 이용해서 GRUB에 반영을 해줘야 합니다.

<pre class="prettyprint">
sudo update-grub
</pre>

<br>

## GRUB 해상도 조절

윈도우와 멀티 부팅으로 했을 경우 부팅 OS를 선택하는 화면인 `GRUB` 화면의 해상도가 너무 높아서
글씨가 아주 작게 나옵니다. 이를 해결하기 위해서 다음과 같은 작업을 해줍니다.

<pre class="prettyprint">
sudo nano /etc/default/grub
</pre>

여기에 `GRUB_GFXMODE=640x480` 옵션을 추가해줍니다. 
(기본적으로 주석으로 존재하고 있습니다. 주석을 제거해줘도 됩니다.)

수정 후 다음 명령어를 이용해서 GRUB에 반영을 해줍니다. 

<pre class="prettyprint">
sudo update-grub

sudo reboot
</pre>

