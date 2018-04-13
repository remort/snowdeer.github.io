---
layout: post
title: Constraint Layout 소개
category: Android
tag: [Android]
---
# Constraint Layout

`ConstraintLayout`은 비교적 최근에 나온 레이아웃 컴포넌트로 현재도 계속 발전되고 있는 레이아웃입니다. 다른 컴포넌트들과 상대적인 위치를 지정할 수 있다는 점에서 `RelativeLayout`과 비슷한 성격을 갖고 있습니다. 또한 성능도 대부분의 경우에서 `RelativeLayout`보다 더 좋기 때문에(RelativLayout를 사용할 때는 중첩된 Layout을 사용해야 하는 경우가 많은데, ConstraintLayout은 평면적인(Flat한) 레이아웃을 더 쉽게 만들 수 있습니다.) 새로운 레이아웃을 만들 때는 `ConstraintLayout`을 한 번쯤 고려해보는 것도 좋을 것 같습니다.

<br>

![image](/assets/android/003.png)

특히 `ConstraintLayout`은 위와 같이 중첩된 레이아웃에서 빛을 발합니다. 상위단의 레이아웃이 수정될 경우 그 안의 레이아웃에 큰 영향을 미치는 경우가 많은데(`RelativeLayout`은 그런 영향이 적습니다만) `ConstraintLayout`을 사용하면 그 영향을 상당부분 줄일 수 있습니다.

또한 [Layout Editor](https://developer.android.com/studio/write/layout-editor.html?hl=ko)와 같이 사용하기에도 너무나 좋은 레이아웃입니다. 그동안 `RelativeLayout`를 비롯한 다른 레이아웃들은 Layout Editor에서 사용하기가 쉽지 않았는데 `ConstraintLayout`은 아주 자연스럽게 사용할 수 있어서 상당히 좋습니다.

안드로이드 기본 SDK에 포함되어 있지 않아서, 향후 `ConstraintLayout`이 버전업이 되더라도 기존 프로젝트에 영향을  미치지 않는 점도 장점입니다. `gradle` 빌드 세팅에서 버전을 명시해놓을 수 있기 때문입니다.

[여기](https://academy.realm.io/kr/posts/constraintlayout-it-can-do-what-now/)에서 더 자세히 확인할 수 있습니다.

<br>

## 속성들

`ConstraintLayout`의 속성들은 다음과 같이 사용할 수 있습니다.

<pre class="prettyprint">
...
app:layout_constraintLeft_toLeftOf="parent"
app:layout_constraintRight_toRightOf="parent"
...
</pre>

대충 `[source] to [target] of "view"`라는 형태의 공식이라고 생각하면 됩니다.

실제로 아래와 같은 레이아웃으로 작성하면 화면 중앙에 버튼이 위치되는 것을 확인할 수 있습니다.

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;android.support.constraint.ConstraintLayout
  xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:app="http://schemas.android.com/apk/res-auto"
  xmlns:tools="http://schemas.android.com/tools"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  tools:context=".MainActivity"
  tools:layout_editor_absoluteY="81dp"&gt;

  &lt;Button
    android:id="@+id/button1"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:text="Button"
    app:layout_constraintBottom_toBottomOf="parent"
    app:layout_constraintEnd_toEndOf="parent"
    app:layout_constraintStart_toStartOf="parent"
    app:layout_constraintTop_toTopOf="parent"/&gt;

&lt;/android.support.constraint.ConstraintLayout&gt;
</pre>

이 상태에서 

<pre class="prettyprint">
  app:layout_constraintVertical_bias="0.7"
  app:layout_constraintHorizontal_bias="0.2"
</pre>

`bias` 속성을 이용해서 해당 위치의 퍼센트를 조절할 수 있습니다. 위 예제는 세로 70%, 가로 20% 위치에 버튼이 존재하게 됩니다.

또한 이 상태에서

<pre class="prettyprint">
  android:layout_width="0px"
  android:layout_height="0px"
</pre>

와 같이 컴포넌트의 너비와 높이를 `0px`로 세팅하게 되면, `Match Constraint`라고 해서 꽉 채운 크기의 컴포넌트로 설정됩니다.

<pre class="prettyprint">
app:layout_constraintDimensionRatio="16:9"

또는

app:layout_constraintDimensionRatio="h, 16:9"
</pre>

와 같이 `layout_constraintDimensionRatio` 비율을 이용해서 가로/세로 비율을 설정할 수 있으며, 이 때 컴포넌트의 너비나 높이 둘 중 하나는 `Match Constraint`로 설정되어있어야 사용가능합니다.

<br>

## Animation 효과

`Constraint Layout`은 컴포넌트의 재배치시 애니메이션 효과를 줄 수 있습니다.

</pre class="prettyprint">
ConstraintSet mConstraintSet1 = new ConstraintSet(); // create a Constraint Set
ConstraintSet mConstraintSet2 = new ConstraintSet(); // create a Constraint Set

mConstraintSet2.clone(context, R.layout.state2); // get constraints from layout
setContentView(R.layout.state1);
mConstraintLayout = (ConstraintLayout) findViewById(R.id.activity_main);
mConstraintSet1.clone(mConstraintLayout); // get constraints from ConstraintSet

TransitionManager.beginDelayedTransition(mConstraintLayout);
mConstraintSet1.applyTo(mConstraintLayout
</pre>

* 참고 : http://androidkt.com/constraintlayout-circular-positioning/
* 참고 : https://www.captechconsulting.com/blogs/starting-out-with-constraint-layout-and-android-studios-visual-editor