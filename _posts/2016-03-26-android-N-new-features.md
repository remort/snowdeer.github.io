---
layout: post
title: Android N for Developers
category: 안드로이드
---

얼마 전에 발표한 Android N의 새로운 기능들을 정리해보았습니다.
구글 Developer 사이트의 내용을 간단히 번역해보았고,
원문은 [여기](https://developer.android.com/preview/api-overview.html)있습니다. 

### 멀티 윈도우 지원

드디어 안드로이드 N 부터 멀티윈도우 지원을 정식으로 지원하기 시작했습니다.
기존에는 삼성이나 LG 등 스마트 제조사에서 자체적으로 개발해서 제공을 해줬으나 
안드로이드 N 부터는 정식 지원합니다. 덕분에 더 다양한(아마도 대부분의) 어플들이
멀티 윈도우를 지원하게 될 것 같습니다.

![MultiWindow](https://developer.android.com/preview/images/mw-portrait.png)

* 스마트 폰 이나 타블렛에서 화면 분할 모드를 통해 2개의 어플을 실행시킬 수 있습니다. 분할 바를 드래그해서 각 어플들의 화면 크기를 조절할 수 있습니다.
* 안드로이드 TV에서는 어플들이 [PIP(Picture-in-Picture)](https://developer.android.com/preview/features/picture-in-picture.html) 모드를 지원합니다.

![MultiWindow](https://developer.android.com/preview/images/mw-splitscreen_2x.png)

멀티 윈도우 기능에 대해서 좀 더 알고 싶으시면 [여기](https://developer.android.com/preview/features/multi-window.html)에서 보실 수 있습니다.

<br><br>

### 노티피케이션 강화

안드로이드 N에서는 기존의 노티피케이션을 더 쉽고 빠르게 쓸 수 있도록 기능을 강화했습니다.

* 템플릿 업데이트 : 최소한의 코드 수정만으로 다양한 템플릿을 사용할 수 있도록 했습니다.
* 그룹 알림 : 예를 들어 같은 주제나 그룹의 메세지를 묶어서 관리할 수 있는 기능이며, 사용자들 묶여있는 알림에 대해 실행이나 해제 등을 수행할 수 있습니다. (Android Wear에는 이미 적용되어 있는 기능입니다.)
* 즉시 응답 기능 : 실시간 채팅 어플 등에서 노티바를 통해 바로 응답을 할 수 있는 기능입니다. 
* Custom View를 위한 2개의 API가 추가되었습니다.

![Notification](https://developer.android.com/preview/images/notifications-1.png)

자세한 내용은 [여기](https://developer.android.com/preview/features/notification-updates.html)에서 보실 수 있습니다.

<br><br>

### 프로파일링 기반 JIT/AOT 컴파일

안드로이드 N 부터 어플의 수행 속도 향상을 위해 프로파일링 기반 JIT(Just in Time)을
추가했습니다. JIT 컴파일러는 현재의 ART의 AOT(Ahead of Time) 컴파일러를 보완하고 실행 속도 향상, 저장 공간 절약, 어플 및 시스템의 업데이트 속도를 향상시켜줍니다.

프로파일링은 각 어플들의 실제 사용 빈도나 그 당시 디바이스 상황 등을 관리하며, 
예를 들어 어플이 최고의 성능을 낼 수 있도록 자주 쓰는 함수들에 대해 미리 컴파일(precompile)을 하거나 캐쉬 등을 할 수 있게 해줍니다.

각 어플들의 성능을 향상 시키기도 하지만, 각 어플들의 메모리 소비량, 
실행 파일의 크기 등도 절약 시켜 주며, 더 나가 배터리 소모까지 줄여줍니다. 
프리-컴파일(precompile)의 경우 디바이스가 쉬고 있을 때(충전 중일 때) 이루어지며
미리 컴파일을 해 놓기 때문에 실제 작업을 할 때는 속도 향상 및 배터리를 절약할 수
있게 해줍니다. 

<br><br>

### Quick path to app install

기존에는 OS를 업데이트 할 경우 맨 처음에 각 어플들을 최적화하는 작업들 때문에
많은 시간이 소모가 되었지만, 안드로이드 N 부터는 각 어플의 최적화 단계가 없고
시스템 업데이트 속도도 향상이 되어 초기 OS 설정 속도가 훨씬 더 빨라집니다.

<br><br>

### Doze

Doze는 디바이스가 쉬고 있는(idle) 상태를 감지하여 CPU나 네트워크 활동을 조정하여
배터리를 절약할 수 있게 해주는 시스템입니다. 이로써 사용자가 디바이스를 호주머니에
넣고 돌아다니거나 책상 위, 사랍 안에 놓아 둘 때 보다 효율적으로 배터리를 절약할 수
있습니다.

![Doze](https://developer.android.com/preview/images/doze-diagram-1.png)

어플에 Doze를 적용시키는 방법은 [여기](https://developer.android.com/training/monitoring-device-state/doze-standby.html#assessing_your_app)를 참고하세요.

<br><br>

### Project Svelte

Project Svelte는 백그라운드 작업을 최적화하여 디바이스의 RAM 사용량을 최소화 시키려는 프로젝트입니다. 안드로이드 5.0 이후의 JobScheduler와 GCMNetworkManager를 접목시켜 


<br><br>

### Data Saver

<br><br>

### Quick Settings Tile API


<br><br>

### Number-blocking


<br><br>

### Call screening



<br><br>

### Multi-locale support, more languages




<br><br>

### CICU4J APIs in Android




<br><br>

### OpenGL™ ES 3.2 API




<br><br>

### Android TV recording




<br><br>

### Android for Work



### Direct boot




<br><br>

### Key Attestation



### Network Security Config




<br><br>

### Default Trusted Certificate Authority


