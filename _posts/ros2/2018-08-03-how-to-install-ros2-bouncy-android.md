---
layout: post
title: ROS 2.0 Bouncy Android 설치 방법
category: ROS2
tag: [ROS]
---

ROS 2.0 Android 설치 방법입니다. 기존의 `Ardent` 버전에 비해서 크게 변경된 점은 없습니다.
공식 홈페이지는 [여기](https://github.com/esteve/ros2_java)입니다.

여기서는 Ubuntu 18.04 기준으로 설명을 적지만, Ubuntu 16.04에서도 동일하게 동작합니다.

ROS 2.0 Java는 설치하지 않아도 됩니다.

<br>

## JDK 설치

<pre class="prettyprint">
sudo apt-add-repository ppa:webupd8team/java
sudo apt-get update
sudo apt-get install oracle-java8-installer
</pre>

<br>

## gradle 3.2 이상 버전으로 업그레이드

<pre class="prettyprint">
sudo add-apt-repository ppa:cwchien/gradle
sudo apt install -y gradle
</pre>

<br>

## Ament Workspace 다운로드

<pre class="prettyprint">
ROOT_DIR=${HOME}
AMENT_WORKSPACE=${ROOT_DIR}/ament_ws
ROS2_ANDROID_WORKSPACE=${ROOT_DIR}/ros2_android_ws

mkdir -p ${AMENT_WORKSPACE}/src
cd ${AMENT_WORKSPACE}
wget https://raw.githubusercontent.com/esteve/ament_java/master/ament_java.repos
vcs import ${AMENT_WORKSPACE}/src < ament_java.repos
src/ament/ament_tools/scripts/ament.py build --symlink-install --isolated
</pre>

<br>

## ROS2 for Java 설치 

<pre class="prettyprint">
mkdir -p ~/ros2_java_ws/src
cd ~/ros2_java_ws
curl -skL https://raw.githubusercontent.com/esteve/ros2_java/master/ros2_java_desktop.repos -o ros2_java_desktop.repos
vcs import src < ros2_java_desktop.repos
. ../ament_ws/install_isolated/local_setup.sh
ament build --symlink-install --isolated
</pre>

<br>

## Android NDK 설치

[공식 홈페이지](https://developer.android.com/ndk/downloads/)에서 NDK를 다운로드 합니다.
`ROS2 for Android` 설치 가이드에서는 `~/android_ndk`에 압축 해제를 권장하지만, 여기서는 폴더 구조를 조금 더 
깔끔하게 가져가기 위해서 `~/Android/Ndk`에 압축 해제를 합니다.(`Android Studio`를 설치했을 때 SDK를 `~/Android/SDK`에 설치하기 때문에 통일성을 맞추기 위해서입니다.)

즉, NDK 설치 경로는 `~/Android/Ndk/android-ndk-r17b`입니다.

그리고 환경 변수를 다음과 같이 등록해줍니다.

<pre class="prettyprint">
export ANDROID_NDK=~/Android/Ndk/android-ndk-r17b
</pre>

<br>

## ROS2 for Android 다운로드

<pre class="prettyprint">
export PYTHON3_EXEC="$( which python3 )"
export ANDROID_ABI=armeabi-v7a
export ANDROID_NATIVE_API_LEVEL=android-21
export ANDROID_TOOLCHAIN_NAME=arm-linux-androideabi-clang


mkdir -p ${ROS2_ANDROID_WORKSPACE}/src
cd ${ROS2_ANDROID_WORKSPACE}
wget https://raw.githubusercontent.com/esteve/ros2_java/master/ros2_java_android.repos
vcs import ${ROS2_ANDROID_WORKSPACE}/src < ros2_java_android.repos
source ${AMENT_WORKSPACE}/install_isolated/local_setup.sh
</pre>

## ROS2 for Android 빌드

<pre class="prettyprint">
ament build --isolated --skip-packages test_msgs \
  --cmake-args \
  -DPYTHON_EXECUTABLE=${PYTHON3_EXEC} \
  -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake \
  -DANDROID_FUNCTION_LEVEL_LINKING=OFF \
  -DANDROID_NATIVE_API_LEVEL=${ANDROID_NATIVE_API_LEVEL} \
  -DANDROID_TOOLCHAIN_NAME=${ANDROID_TOOLCHAIN_NAME} \
  -DANDROID_STL=gnustl_shared \
  -DANDROID_ABI=${ANDROID_ABI} \
  -DANDROID_NDK=${ANDROID_NDK} \
  -DTHIRDPARTY=ON \
  -DCOMPILE_EXAMPLES=OFF \
  -DCMAKE_FIND_ROOT_PATH="$AMENT_WORKSPACE/install_isolated;$ROS2_ANDROID_WORKSPACE/install_isolated" \
  -- \
  --parallel \
  --ament-gradle-args \
  -Pament.android_stl=gnustl_shared -Pament.android_abi=$ANDROID_ABI -Pament.android_ndk=$ANDROID_NDK --
</pre>

## .bashrc에 환경 변수 추가

`.bashrc`에 아래 항목을 추가해줍니다.

<pre class="prettyprint">
# for Java
export JAVA_HOME=/usr/lib/jvm/java-8-oracle

# for Android
export ANDROID_SDK=~/Android/Sdk
export ANDROID_NDK=~/Android/Ndk/android-ndk-r17b
export ANDROID_HOME=$ANDROID_SDK

export PATH=$PATH:$ANDROID_SDK:$ANDROID_TOOLS:$ANDROID_NDK:$ANDROID_HOME/tools:$ANDROID_SDK/platform-tools
</pre>

<br>

## android_bouncy.sh 생성

이 파일은 꼭 만들 필요는 없지만, 만들어두면 편리하게 사용할 수 있습니다.

<pre class="prettyprint">
ROOT_DIR=${HOME}
AMENT_WORKSPACE=${ROOT_DIR}/ament_ws
ROS2_ANDROID_WORKSPACE=${ROOT_DIR}/ros2_android_ws

export PYTHON3_EXEC="$( which python3 )"
export ANDROID_ABI=armeabi-v7a
export ANDROID_NATIVE_API_LEVEL=android-21
export ANDROID_TOOLCHAIN_NAME=arm-linux-androideabi-clang

source ~/ament_ws/install_isolated/local_setup.sh
source ~/ros2_android_ws/install_isolated/local_setup.bash
</pre>

그 이후 안드로이드 ROS2 환경이 필요하면 터미널 실행했을 때마다 `source android_bouncy.sh`를 실행해주면 됩니다.