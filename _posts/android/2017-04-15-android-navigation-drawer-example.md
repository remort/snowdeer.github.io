---
layout: post
title: 안드로이드 Navigation Drawer 사용하기
category: Android
tag: [Android, UX]
---

요즘 나오는 App들은 Navigation Drawer를 사용한 UX가 많습니다.

![image -fullwidth]({{ site.baseurl }}/assets/2017-04-15-android-navigation-drawer-example/01.png)

<br>

[Google Developer 사이트에서 훌륭한 예제 코드](https://developer.android.com/training/implementing-navigation/nav-drawer.html?hl=ko)를
다루고 있는데, 여기서는 Android Studio에서 자동으로 생성해준 템플릿 코드 기반으로
간단하게 구현을 해보았습니다.

먼저 메뉴 항목들을 나열할 xml 파일을 만들어줍니다.
(res 폴더 아래 menu 폴더 아래 만들어 주시면 됩니다.)

<br>
## activity_main_drawer.xml
<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;menu xmlns:android="http://schemas.android.com/apk/res/android"&gt;

  &lt;group android:checkableBehavior="single"&gt;
    &lt;item
      android:id="@+id/menu_bt_server"
      android:icon="@drawable/ic_menu_camera"
      android:title="Bluetooth Server" /&gt;
    &lt;item
      android:id="@+id/menu_bt_client"
      android:icon="@drawable/ic_menu_gallery"
      android:title="Bluetooth Client" /&gt;

  &lt;/group&gt;

&lt;/menu&gt;</pre>
<br>

그리고, 액티비티에서 사용할 레이아웃은 다음과 같이 작성합니다.
## activity_main.xml
<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;android.support.v4.widget.DrawerLayout xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:app="http://schemas.android.com/apk/res-auto"
  xmlns:tools="http://schemas.android.com/tools"
  android:id="@+id/drawer_layout"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:fitsSystemWindows="true"
  tools:openDrawer="start"&gt;

  &lt;include
    layout="@layout/app_bar_main"
    android:layout_width="match_parent"
    android:layout_height="match_parent" /&gt;

  &lt;android.support.design.widget.NavigationView
    android:id="@+id/nav_view"
    android:layout_width="wrap_content"
    android:layout_height="match_parent"
    android:layout_gravity="start"
    android:fitsSystemWindows="true"
    app:headerLayout="@layout/nav_header_main"
    app:menu="@menu/activity_main_drawer" /&gt;

&lt;/android.support.v4.widget.DrawerLayout&gt;</pre>
<br>
## app_bar_main.xml
<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;android.support.design.widget.CoordinatorLayout xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:app="http://schemas.android.com/apk/res-auto"
  xmlns:tools="http://schemas.android.com/tools"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  tools:context="snowdeer.bluetooth.sample.MainActivity"&gt;

  &lt;android.support.design.widget.AppBarLayout
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:theme="@style/AppTheme.AppBarOverlay"&gt;

    &lt;android.support.v7.widget.Toolbar
      android:id="@+id/toolbar"
      android:layout_width="match_parent"
      android:layout_height="?attr/actionBarSize"
      android:background="?attr/colorPrimary"
      app:popupTheme="@style/AppTheme.PopupOverlay" /&gt;

  &lt;/android.support.design.widget.AppBarLayout&gt;

  &lt;RelativeLayout
    android:id="@+id/content_main"
    android:layout_width="match_parent"
    android:layout_height="match_parent"
    app:layout_behavior="@string/appbar_scrolling_view_behavior" /&gt;

&lt;/android.support.design.widget.CoordinatorLayout&gt;</pre>
<br>
## nav_header_main.xml
<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:app="http://schemas.android.com/apk/res-auto"
  android:layout_width="match_parent"
  android:layout_height="@dimen/nav_header_height"
  android:paddingTop="@dimen/activity_vertical_margin"
  android:paddingBottom="@dimen/activity_vertical_margin"
  android:paddingLeft="@dimen/activity_horizontal_margin"
  android:paddingRight="@dimen/activity_horizontal_margin"
  android:background="@drawable/side_nav_bar"
  android:gravity="bottom"
  android:orientation="vertical"
  android:theme="@style/ThemeOverlay.AppCompat.Dark"&gt;

  &lt;ImageView
    android:id="@+id/imageView"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:paddingTop="@dimen/nav_header_vertical_spacing"
    app:srcCompat="@android:drawable/sym_def_app_icon" /&gt;

  &lt;TextView
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:paddingTop="@dimen/nav_header_vertical_spacing"
    android:text="Android Studio"
    android:textAppearance="@style/TextAppearance.AppCompat.Body1" /&gt;

  &lt;TextView
    android:id="@+id/textView"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:text="android.studio@android.com" /&gt;

&lt;/LinearLayout&gt;</pre>
<br>

그리고 실제 Java 코드는 다음과 같이 작성하면 됩니다.
## MainActivity.java
<pre class="prettyprint">public class MainActivity extends AppCompatActivity
    implements NavigationView.OnNavigationItemSelectedListener {

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);
    Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
    setSupportActionBar(toolbar);

    DrawerLayout drawer = (DrawerLayout) findViewById(R.id.drawer_layout);
    ActionBarDrawerToggle toggle = new ActionBarDrawerToggle(
        this, drawer, toolbar, R.string.navigation_drawer_open, R.string.navigation_drawer_close);
    drawer.setDrawerListener(toggle);
    toggle.syncState();

    NavigationView navigationView = (NavigationView) findViewById(R.id.nav_view);
    navigationView.setNavigationItemSelectedListener(this);

    replaceServerFragment();
  }

  @Override
  public void onBackPressed() {
    DrawerLayout drawer = (DrawerLayout) findViewById(R.id.drawer_layout);
    if (drawer.isDrawerOpen(GravityCompat.START)) {
      drawer.closeDrawer(GravityCompat.START);
    } else {
      super.onBackPressed();
    }
  }

  @SuppressWarnings("StatementWithEmptyBody")
  @Override
  public boolean onNavigationItemSelected(MenuItem item) {
    int id = item.getItemId();

    if (id == R.id.menu_bt_server) {
      replaceServerFragment();
    } else if (id == R.id.menu_bt_client) {
      replaceClientFragment();
    }

    DrawerLayout drawer = (DrawerLayout) findViewById(R.id.drawer_layout);
    drawer.closeDrawer(GravityCompat.START);
    return true;
  }

  void replaceServerFragment() {
    setTitle("Bluetooth Server");
    FragmentTransaction ft = getSupportFragmentManager().beginTransaction();
    Fragment fragment = new BTServerFragment();
    ft.replace(R.id.content_main, fragment);
    ft.commit();
  }

  void replaceClientFragment() {
    setTitle("Bluetooth Client");
    FragmentTransaction ft = getSupportFragmentManager().beginTransaction();
    Fragment fragment = new BTClientFragment();
    ft.replace(R.id.content_main, fragment);
    ft.commit();
  }
}</pre>
