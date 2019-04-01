---
layout: post
title: Fragment 추가 및 삭제 예제
category: Android
tag: [Android]
---

<pre class="prettyprint">
void addLogViewerFragment() {
  logViewerFragment = new LogViewerFragment();
  FragmentManager fragmentManager = getSupportFragmentManager();
  FragmentTransaction fragmentTransaction = fragmentManager.beginTransaction();
  fragmentTransaction.add(R.id.logviewer_layout, logViewerFragment).commit();
}
  
void removeLogviewerFragment() {
  FragmentManager fragmentManager = getSupportFragmentManager();
  FragmentTransaction fragmentTransaction = fragmentManager.beginTransaction();
  fragmentTransaction.remove(logViewerFragment).commit();
  logViewerFragment = null;
}
</pre>
