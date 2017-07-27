---
layout: post
title: 7.0(Nougat) 에서 Dialog 들의 버튼들이 보이지 않는 현상
category: Android
tag: [Android, UX]
---

안드로이드 7.0 Nougat에서 다음 이미지와 같이 Dialog들의 버튼이 사라져 버리는 현상이 있습니다.

![image -fullwidth](/assets/2017-04-12-android-nougat-dialog-no-button/01.png)

AlertDialog 뿐만 아니라 PickerDialog 들도 마찬가지 현상이 발생했습니다.

![image -fullwidth](/assets/2017-04-12-android-nougat-dialog-no-button/02.png)

기존에 잘 되던 코드였는데, 갑자기 안드로이드 7.0을 올린 사람들에게 이런 반응이 나와서
찾아보니 7.0 부터는 Dialog에 테마(theme)를 적용해야 하는 정책이 생겼다고 합니다.
테마를 적용하지 않은 기존의 Dialog 들의 버튼은 투명 처리되어 보이지 않습는다.
(투명 처리만 되어있어서 해당 버튼이 있는 위치를 누르면 동작은 한다고 합니다;;) 
갑자기 왜 이런 하위 호환성을 무시해버린 정책을 만들었는지 모르겠지만, 일단은 해결법을
알아보도록 하겠습니다.

<br>

먼저 Dialog용 테마를 하나 만듭니다.

# styles.xml

<pre class="prettyprint">&lt;style name="AlertDialogTheme" parent="Theme.AppCompat.Light.Dialog.Alert"&gt;
  &lt;item name="colorPrimary"&gt;@color/colorPrimary&lt;/item&gt;
  &lt;item name="colorPrimaryDark"&gt;@color/colorPrimaryDark&lt;/item&gt;
  &lt;item name="colorAccent"&gt;@color/colorAccent&lt;/item&gt;
  &lt;item name="borderlessButtonStyle"&gt;@style/Widget.AppCompat.Button.Borderless.Colored&lt;/item&gt;
&lt;/style&gt;</pre>
<br>

그리고 기존에 AlertDialog를 작성하던 코드가 다음과 같았다면

# 기존 코드

<pre class="prettyprint">mAlertDialog = new AlertDialog.Builder(getActivity())
    .setTitle("쿠폰 구입")
    .setMessage("'" + item.name + "'를 정말로 구매하시겠습니까? 포인트가 " + item.price + " 원 차감됩니다.")</pre>
다음과 같이 수정합니다.

<br>

# 수정된 코드

<pre class="prettyprint">mAlertDialog = new AlertDialog.Builder(getActivity(), R.style.AlertDialogTheme)
    .setTitle("쿠폰 구입")
    .setMessage("'" + item.name + "'를 정말로 구매하시겠습니까? 포인트가 " + item.price + " 원 차감됩니다.")</pre>
