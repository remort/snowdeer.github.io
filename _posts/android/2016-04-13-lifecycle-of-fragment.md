---
layout: post
title: Fragment의 라이프사이클
category: Android
tag: [Android]
---
# Fragment의 라이프사이클

Fragment의 라이프사이클은 Activity의 라이프사이클과 비슷하지만 약간 더 복잡합니다.
다음 이미지와 같은 모습의 라이프사이클을 갖고 있습니다.

![image -fullwidth](/assets/android-fragment/fragment_lifecycle.png)

`onAttach()`와 `onDetach()` 이벤트가 Fragment가 실제로 Activity와 연결되거나 끊어지는 부분입니다.

단계 | 설명
--- | ---
onAttach | Activity와 연결될 때 호출. 이 시점에서 `getActivity()`는 `null` 리턴함
onCreate | Fragment 생성시 호출
onCreateView | View 생성
onActivityCreated | 뷰 생성(setContentView 호출 등)
onStart | Fragment 시작될 때 호출(화면에 보이기 직전)
onResume | 화면에 표시될 때 호출
onPause | 화면에서 가려질 때 호출
onStop | Fragment가 종료될 때 호출
onDestoryView | View가 해제될 때 호출
onDestory | Fragment가 해제될 때 호출
onDetach | Activity와 연결이 해제될 때 호출. 가장 마지막 단계