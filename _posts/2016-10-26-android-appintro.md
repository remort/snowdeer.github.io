---
layout: post
title: App Intro 화면 꾸미기
category: Android
tag: [android, appintro]
---

일반적인 AppIntro 화면은 대략 다음과 같은 형태를 하고 있습니다.

![image](https://github.com/PaoloRotolo/AppIntro/raw/master/art/intro.png) 

많은 사람들이 사용하고 있는 오픈소스가 있습니다.
[여기](https://github.com/PaoloRotolo/AppIntro)에서 확인할 수 있으며, 
Android Studio에서는 간단히 gradle에 다음 라인만 추가하면 AppIntro 컴포넌트를 사용할 수 있습니다.

<pre class="prettyprint" style="font-size:0.7em;">
dependencies {
    compile 'com.github.paolorotolo:appintro:4.1.0'
}
</pre>

실제로 사용하는 코드는 다음과 같습니다. Activity를 AppIntro 클래스를 상속받도록 하면
됩니다. 

그리고 중요한 점은 onCreate() 함수 내에서 setContentView() 함수는 지워야 합니다.

각각의 Intro 화면들은 Fragment를 상속받아서 작성할 수 있습니다.


<pre class="prettyprint" style="font-size:0.7em;">
import android.content.Intent;
import android.os.Build;
import android.support.annotation.Nullable;
import android.support.v4.app.Fragment;
import android.os.Bundle;
import android.view.Window;
import android.view.WindowManager;

import com.github.paolorotolo.appintro.AppIntro;

public class SplashActivity extends AppIntro {

    Fragment mSplash1 = new SplashFragment1();
    Fragment mSplash2 = new SplashFragment2();
    Fragment mSplash3 = new SplashFragment3();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        //setContentView(R.layout.activity_splash);

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.KITKAT) {
            Window w = getWindow(); // in Activity's onCreate() for instance
            w.setFlags(WindowManager.LayoutParams.FLAG_LAYOUT_NO_LIMITS, 
                    WindowManager.LayoutParams.FLAG_LAYOUT_NO_LIMITS);
        }

        addSlide(mSplash1);
        addSlide(mSplash2);
        addSlide(mSplash3);
    }

    @Override
    public void onSkipPressed(Fragment currentFragment) {
        super.onSkipPressed(currentFragment);
        startMainActivity();
    }

    @Override
    public void onDonePressed(Fragment currentFragment) {
        super.onDonePressed(currentFragment);
        startMainActivity();
    }

    @Override
    public void onSlideChanged(@Nullable Fragment oldFragment, 
                               @Nullable Fragment newFragment) {
        super.onSlideChanged(oldFragment, newFragment);
    }

    @Override
    protected void onResume() {
        super.onResume();
    }

    private void startMainActivity() {
        Intent intent = new Intent(SplashActivity.this, MainActivity.class);
        startActivity(intent);
        finish();
    }
}
</pre>


