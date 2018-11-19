---
layout: post
title: MAC OS에 ROS2 for Android 설치하는 방법
category: ROS2
tag: [ROS]
---
# MAC OS에 ROS2 for Android 설치하는 방법

ROS 2.0 설치 방법은 자주 업데이트되거나 변경되기 때문에 아래 방법이 잘 되지 않으면 [Esteve github](https://github.com/esteve/ros2_java)를 참고하는 편이 좋습니다.
2018년 11월 6일 기준 제가 직접 설치하면서 검증한 방법입니다.

<br>

## JAVA 8 설치

ROS 2 for Android는 Java 8 버전에서만 제대로 동작합니다. 저같은 경우는 그보다 높은 버전이 설치되어 있어서 하위 버전의 JDK를 추가로 설치해줬습니다.
JDK 1.8은 [여기](https://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html)에서 다운로드 할 수 있습니다.

다음 명령어를 이용해서 `JAVA_HOME` 설정을 해주고 `gradle`도 설치합니다.

<pre class="prettyprint">
$ brew tap caskroom/versions
$ brew cask install java8
$ export JAVA_HOME=/Library/Java/JavaVirtualMachines/jdk1.8.0_192.jdk/Contents/Home

$ brew cask install java

$ brew install gradle
</pre>

맥 OS에는 `vcstool`도 기본적으로 설치되어 있지 않기 때문에 설치해줍니다. 

<pre class="prettyprint">
pip3 install vcstool
</pre>

<br>

## ament_ws 설치

<pre class="prettyprint">
mkdir ament_ws/src
cd ament_ws
curl -skL https://raw.githubusercontent.com/esteve/ament_java/master/ament_java.repos -o ament_java.repos
vcs import src < ament_java.repos
src/ament/ament_tools/scripts/ament.py build --symlink-install --isolated
</pre>

<br>

## Anroid Studio 및 NDK 설치

Android NDK를 다운로드해서 설치합니다. [여기](https://developer.android.com/ndk/downloads/?hl=ko)에서 다운로드 가능하며 버전은 `16b` 이상이면 가능합니다. 
저는 `17b` 버전으로 설치했습니다.
물론 Android Studio나 Android SDK도 비슷한 방법으로 다 설치가 되어 있어야 합니다.

그 이후 환경 변수를 아래와 같이 설정합니다.

<pre class="prettyprint">
export ANDROID_PATH=/Users/snowdeer/Library/Android
export ANDROID_SDK=${ANDROID_PATH}/sdk
export ANDROID_NDK=${ANDROID_PATH}/ndk/android-ndk-r17b
export ANDROID_NDK_HOME=${ANDROID_NDK}
export ANDROID_HOME=${ANDROID_SDK}
export ANDROID_TOOLS=${ANDROID_SDK}/platform-tools
export PATH=$PATH:$ANDROID_SDK:$ANDROID_TOOLS:$ANDROID_NDK
</pre>

<pre class="prettyprint">

</pre>
ROOT_DIR=${HOME}
AMENT_WORKSPACE=${ROOT_DIR}/ament_ws
ROS2_ANDROID_WORKSPACE=${ROOT_DIR}/ros2_android_ws
export JAVA_HOME=$(/usr/libexec/java_home -v 1.8)

export PYTHON3_EXEC="$( which python3 )"
export ANDROID_ABI=armeabi-v7a
export ANDROID_NATIVE_API_LEVEL=android-21
export ANDROID_TOOLCHAIN_NAME=arm-linux-androideabi-clang
<br>

## ROS 2 for Android 소스 다운로드

<pre class="prettyprint">
mkdir -p ${ROS2_ANDROID_WORKSPACE}/src
cd ${ROS2_ANDROID_WORKSPACE}
wget https://raw.githubusercontent.com/esteve/ros2_java/master/ros2_java_android.repos
vcs import ${ROS2_ANDROID_WORKSPACE}/src < ros2_java_android.repos
source ${AMENT_WORKSPACE}/install_isolated/local_setup.sh
</pre>

<br>

## 빌드

`rcl_lifecycle` 패키지 같은 경우 최근에 ROS 2에 추가되었는데 아직 ROS 2 for Android에서는 지원하지 않는 것 같아서 제외했습니다.

<pre class="prettyprint">
ament build --isolated --skip-packages test_msgs rcl_lifecycle \
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