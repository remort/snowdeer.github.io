---
layout: post
title: Kotlin Firebase Google SignIn 예제
category: Android
tag: [Android]
---

## build.gradle (프로젝트)

<pre class="prettyprint">
dependencies {
    classpath 'com.android.tools.build:gradle:3.4.0'
    classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlin_version"
    classpath 'com.google.gms:google-services:4.2.0'
}
</pre>

<br>

## build.gradle (모듈)

<pre class="prettyprint">
dependencies {
    implementation fileTree(dir: 'libs', include: ['*.jar'])
    implementation "org.jetbrains.kotlin:kotlin-stdlib-jdk7:$kotlin_version"
    implementation 'com.android.support:appcompat-v7:28.0.0'
    implementation 'com.android.support.constraint:constraint-layout:1.1.3'
    implementation 'com.google.firebase:firebase-auth:16.0.5'
    implementation 'com.google.firebase:firebase-database:16.0.4'

    testImplementation 'junit:junit:4.12'
    androidTestImplementation 'com.android.support.test:runner:1.0.2'
    androidTestImplementation 'com.android.support.test.espresso:espresso-core:3.0.2'
    implementation 'com.google.android.gms:play-services:12.0.1'
    implementation 'com.google.android.gms:play-services-auth:16.0.1'
}
</pre>

<br>

## activity_main.xml

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;RelativeLayout
  xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:tools="http://schemas.android.com/tools"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:orientation="vertical"
  tools:context=".MainActivity"&gt;

  &lt;Button
    android:id="@+id/logout_button"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:layout_centerInParent="true"
    android:text="logout"/&gt;

&lt;/RelativeLayout&gt;
</pre>

<br>

## activity_sign_in.xml

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;RelativeLayout
  xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:tools="http://schemas.android.com/tools"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:orientation="vertical"
  tools:context=".MainActivity"&gt;

  &lt;com.google.android.gms.common.SignInButton
    android:id="@+id/sign_in_button"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:layout_centerInParent="true"/&gt;

&lt;/RelativeLayout&gt;
</pre>

<br>

## MainActivity.kt

<pre class="prettyprint">
import android.content.Intent
import android.support.v7.app.AppCompatActivity
import android.os.Bundle
import com.google.android.gms.auth.api.Auth
import com.google.android.gms.common.ConnectionResult
import com.google.android.gms.common.api.GoogleApiClient
import com.google.firebase.auth.FirebaseAuth
import kotlinx.android.synthetic.main.activity_main.*

class MainActivity : AppCompatActivity(), GoogleApiClient.OnConnectionFailedListener {
    override fun onConnectionFailed(p0: ConnectionResult) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    lateinit var googleApiClient: GoogleApiClient

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        googleApiClient = GoogleApiClient.Builder(applicationContext)
                .enableAutoManage(this, this)
                .addApi(Auth.GOOGLE_SIGN_IN_API)
                .build()

        val currentUser = FirebaseAuth.getInstance().currentUser

        if (currentUser == null) {
            val intent = Intent(applicationContext, LoginActivity::class.java)
            startActivity(intent)
        } else {
            // TODO ...
        }

        logout_button.setOnClickListener {
            FirebaseAuth.getInstance().signOut()
            Auth.GoogleSignInApi.signOut(googleApiClient)

            val intent = Intent(applicationContext, LoginActivity::class.java)
            startActivity(intent)
        }
    }

    override fun onResume() {
        super.onResume()

        val currentUser = FirebaseAuth.getInstance().currentUser
        supportActionBar?.title = currentUser.toString()

    }
}
</pre>

<br>

## LoginActivity.kt

<pre class="prettyprint">
import android.content.Intent
import android.support.v7.app.AppCompatActivity
import android.os.Bundle
import android.widget.Toast
import com.google.android.gms.auth.api.Auth
import com.google.android.gms.auth.api.signin.GoogleSignIn
import com.google.android.gms.auth.api.signin.GoogleSignInOptions
import com.google.android.gms.common.ConnectionResult
import com.google.android.gms.common.api.GoogleApiClient
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.auth.GoogleAuthProvider
import kotlinx.android.synthetic.main.activity_login.*

class LoginActivity : AppCompatActivity(), GoogleApiClient.OnConnectionFailedListener {

    private val REQUEST_CODE_SIGN_IN = 1001

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_login)

        val gso = GoogleSignInOptions.Builder(GoogleSignInOptions.DEFAULT_SIGN_IN)
                .requestIdToken(getString(R.string.default_web_client_id))
                .requestEmail()
                .build()

        val googleSignInClient = GoogleSignIn.getClient(this, gso)

        login_button.setOnClickListener {
            val intent = googleSignInClient.signInIntent
            startActivityForResult(intent, REQUEST_CODE_SIGN_IN)
        }
    }

    override fun onConnectionFailed(result: ConnectionResult) {
        Toast.makeText(applicationContext, "연결 실패", Toast.LENGTH_SHORT).show()
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        if (requestCode == REQUEST_CODE_SIGN_IN) {
            val result = Auth.GoogleSignInApi.getSignInResultFromIntent(data)
            if (result.isSuccess) {
                val account = result.signInAccount
                val credential = GoogleAuthProvider.getCredential(account?.idToken, null)
                FirebaseAuth.getInstance().signInWithCredential(credential)
                        .addOnSuccessListener {
                            Toast.makeText(applicationContext, "인증 성공", Toast.LENGTH_SHORT).show()
                            startActivity(Intent(applicationContext, MainActivity::class.java))
                            finish()
                        }
                        .addOnFailureListener {
                            Toast.makeText(applicationContext, "인증 실패(${it.message})", Toast.LENGTH_SHORT).show()
                        }
            } else {
                Toast.makeText(applicationContext, "로그인 실패", Toast.LENGTH_SHORT).show()
            }
        }
    }
}

</pre>