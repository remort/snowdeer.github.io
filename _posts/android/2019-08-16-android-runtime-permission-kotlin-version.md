---
layout: post
title: Runtime Permission (Kotlin 버전)
category: Android
tag: [Android]
---

기존에 [Java 버전의 Runtime Permission](http://snowdeer.github.io/android/2017/07/02/android-runtime-permission/)을 포스팅 했었지만
이번에는 Kotlin 버전으로 포스팅합니다.

<pre class="prettyprint">
private const val PERMISSION_REQUEST_CODE = 1231

class MainActivity : AppCompatActivity() {

    private val permissions = arrayOf(
            Manifest.permission.WRITE_EXTERNAL_STORAGE
    )

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        setContentView(R.layout.activity_main)

        if (!checkPermissions(permissions)) {
            requestPermissions(permissions, PERMISSION_REQUEST_CODE)
        }
    }

    private fun checkPermissions(permissions: Array&lt;String&gt;): Boolean {
        for (permission in permissions) {
            if (checkSelfPermission(permission) != PackageManager.PERMISSION_GRANTED) {
                return false
            }
        }
        return true
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array&lt;out String&gt;, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)

        when (requestCode) {
            PERMISSION_REQUEST_CODE -> {
                if ((grantResults.isNotEmpty()) && (grantResults[0] == PackageManager.PERMISSION_GRANTED)) {

                } else {
                    Toast.makeText(applicationContext, "Permission is not granted.", Toast.LENGTH_SHORT).show()
                }
            }

        }
    }
}
</pre>