---
layout: post
title: Firebase 회원 가입, 로그인 예제
tag: [Android]
---

## 회원 가입

<pre class="prettyprint">
import android.support.v7.app.AppCompatActivity
import android.os.Bundle
import android.widget.Toast
import com.google.firebase.auth.FirebaseAuth
import kotlinx.android.synthetic.main.activity_main.*

class MainActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        create_userid_button.setOnClickListener {
            val email = email_input.text.toString()
            val password = password_input.text.toString()

            registerUser(email, password)
        }
    }

    private fun registerUser(email: String, password: String) {
        FirebaseAuth.getInstance().createUserWithEmailAndPassword(email, password)
            .addOnSuccessListener {
                val userId = FirebaseAuth.getInstance().currentUser
                Toast.makeText(applicationContext, "UserId(${userId?.email}) 생성 성공", Toast.LENGTH_SHORT).show()
            }
            .addOnFailureListener {
                it.printStackTrace()
                Toast.makeText(applicationContext, "UserId 생성 실패(${it.message})", Toast.LENGTH_SHORT).show()
            }
    }
}
</pre>

<br>

## 로그인

<pre class="prettyprint">
import android.support.v7.app.AppCompatActivity
import android.os.Bundle
import android.widget.Toast
import com.google.firebase.auth.FirebaseAuth
import kotlinx.android.synthetic.main.activity_main.*

class MainActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        login_button.setOnClickListener {
            val email = email_input.text.toString()
            val password = password_input.text.toString()

            login(email, password)
        }
    }

    private fun login(email: String, password: String) {
        FirebaseAuth.getInstance().signInWithEmailAndPassword(email, password)
            .addOnSuccessListener {
                val userId = FirebaseAuth.getInstance().currentUser
                Toast.makeText(applicationContext, "로그인 성공", Toast.LENGTH_SHORT).show()
            }
            .addOnFailureListener {
                it.printStackTrace()
                Toast.makeText(applicationContext, "로그인 실패(${it.message})", Toast.LENGTH_SHORT).show()
            }
    }
}
</pre>

<br>

## 인증 메일 전송

인증 메일 양식은 [Firebase Console](https://console.firebase.google.com/)에서 수정할 수 있습니다.
하지만 악용될 가능성이 있기 때문에 제목만 수정이 가능하며, 내요을 수정하지는 못합니다.

<pre class="prettyprint">
private fun verifyEmail() {
    val currentUser = FirebaseAuth.getInstance().currentUser
    currentUser?.let {
        it.sendEmailVerification()
            .addOnSuccessListener {
                Toast.makeText(applicationContext, "인증 메일을 전송했습니다.", Toast.LENGTH_SHORT).show()
            }
            .addOnFailureListener {
                it.printStackTrace()
                Toast.makeText(applicationContext, "인증 메일 전송 실패(${it.message})", Toast.LENGTH_SHORT).show()
            }
    }
}
</pre>

<br>

## Email 변경 및 패스워드 변경

Email 및 패스워드 변경은 다음 명령어를 이용해서 할 수 있습니다.

* currentUser.updateEmail(newEmail: String)
* currentUser.updatePassword(newPassword: String)
