---
layout: post
title: Eclipse 실행 할 때 Failed to create the Java Virtual Machine 오류 뜰 때
category: Android
tag: [Android, Eclipse]
---

이제는 Android Studio 만을 사용하고 있어서 거의 필요가 없는 글일 수도 있지만,
가끔씩 Eclipse를 사용해야 할 경우도 있어서(과거에 작성한 프로젝트를 수행한다거나)
포스팅을 해봅니다.
Eclipse를 사용하다가 가끔씩 `Failed to create the Java Virtual Machine` 오류가 뜨는 경우가 있습니다.

이런 경우는 Eclipse가 있는 폴더에 가서 `eclipse.ini` 파일을 수정해주면 됩니다.
(하지만, 역시 Eclipse에서 Android Studio로 갈아타는게 제일 좋은거 같습니다.)

<br>
## 수정 전 eclipse.ini
<pre class="prettyprint">-startup
plugins/org.eclipse.equinox.launcher_1.3.0.v20120522-1813.jar
--launcher.library
plugins/org.eclipse.equinox.launcher.win32.win32.x86_1.1.200.v20120522-1813
-product
com.android.ide.eclipse.adt.package.product
--launcher.XXMaxPermSize
256M
-showsplash
com.android.ide.eclipse.adt.package.product
--launcher.XXMaxPermSize
256m
--launcher.defaultAction
openFile
-vmargs
-Dosgi.requiredJavaVersion=1.6
-Xms40m
-Xmx768m
-Declipse.buildId=v21.0.1-543035
</pre>
<br>
## 수정 후 eclipse.ini
<pre class="prettyprint">-startup

plugins/org.eclipse.equinox.launcher_1.3.0.v20120522-1813.jar
--launcher.library
plugins/org.eclipse.equinox.launcher.win32.win32.x86_1.1.200.v20120522-1813
-product
com.android.ide.eclipse.adt.package.product
--launcher.XXMaxPermSize
128M
-showsplash
com.android.ide.eclipse.adt.package.product
--launcher.XXMaxPermSize
128m
--launcher.defaultAction
openFile
-vmargs
-Dosgi.requiredJavaVersion=1.6
-Xms40m
-Xmx768m
-Declipse.buildId=v21.0.1-543035
</pre>
