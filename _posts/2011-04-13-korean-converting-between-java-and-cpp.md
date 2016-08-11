---
layout: post
title: 안드로이드 JNI 환경에서 C++과 Java간 한글 전송 문제 
category: Android
tag: [android, korean, jni]
---

예전에 안드로이드에서 JNI를 통해 한글 데이터를 전송하다가
글자들이 전부 깨지는 현상이 있어서 구현했던 코드입니다.

Java 단에서는 UTF-8 방식을 이용하고, 
C++에서는 KSC5601 방식의 문자열이었기 때문에 발생하는 문제였습니다.

<pre class="prettyprint">
#ifndef _STRING_CONVERTER_FOR_JNI_
#define _STRING_CONVERTER_FOR_JNI_

#include <jni.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

	char *jbyteArray2cstr( JNIEnv *env, jbyteArray javaBytes );
	jbyteArray cstr2jbyteArray( JNIEnv *env, const char *nativeStr);
	jbyteArray javaGetBytes( JNIEnv *env, jstring str );
	jbyteArray javaGetBytesEncoding( JNIEnv *env, jstring str, const char *encoding );
	jstring javaNewString( JNIEnv *env, jbyteArray javaBytes );
	jstring javaNewStringEncoding(JNIEnv *env, jbyteArray javaBytes, const char *encoding );

	jstring javaNewStringChar(JNIEnv* env, const char* nativeStr);

#ifdef __cplusplus
}
#endif

#endif //_STRING_CONVERTER_FOR_JNI_
</pre>


<pre class="prettyprint">
#include "StringConverter_For_JNI.h"

static jclass class_String;
static jmethodID mid_getBytes, mid_getBytesEncoding;
static jmethodID mid_newString, mid_newStringEncoding;

char *jbyteArray2cstr( JNIEnv *env, jbyteArray javaBytes ) {
	size_t len = env->GetArrayLength(javaBytes);
	jbyte *nativeBytes = env->GetByteArrayElements(javaBytes, 0);
	char *nativeStr = (char *)malloc(len+1);
	strncpy( nativeStr, (const char*)nativeBytes, len );
	nativeStr[len] = '\0';
	env->ReleaseByteArrayElements(javaBytes, nativeBytes, JNI_ABORT);

	return nativeStr;
}

jbyteArray cstr2jbyteArray( JNIEnv *env, const char *nativeStr) {
	jbyteArray javaBytes;
	int len = strlen( nativeStr );
	javaBytes = env->NewByteArray(len);
	env->SetByteArrayRegion(javaBytes, 0, len, (jbyte *) nativeStr );

	return javaBytes;
}

jbyteArray javaGetBytes( JNIEnv *env, jstring str ) {
	if ( mid_getBytes == 0 ) {
		if ( class_String == 0 ) {
			jclass cls = env->FindClass("java/lang/String");
			if ( cls == 0 ) {
				return 0;
			}

			class_String = (jclass)env->NewGlobalRef(cls);
			env->DeleteLocalRef(cls);
			if ( class_String == 0 ) {
				return 0;
			}
		}

		mid_getBytes = env->GetMethodID(class_String, "getBytes", "()[B");
		if (mid_getBytes == 0) {
			return 0;
		}
	}

	return (jbyteArray)env->CallObjectMethod(str, mid_getBytes );
}

jbyteArray javaGetBytesEncoding( JNIEnv *env, jstring str, const char *encoding ) {
	if ( mid_getBytesEncoding == 0 ) {
		if ( class_String == 0 ) {
			jclass cls = env->FindClass("java/lang/String");
			if ( cls == 0 ) {
				return 0;
			}

			class_String = (jclass)env->NewGlobalRef(cls);
			env->DeleteLocalRef(cls);
			if ( class_String == 0 ) {
				return 0;
			}
		}

		mid_getBytesEncoding = env->GetMethodID(class_String, "getBytes", "(Ljava/lang/String;)[B");
		if (mid_getBytesEncoding == 0) {
			return 0;
		}
	}

	jstring jstr = env->NewStringUTF(encoding);
	jbyteArray retArray = (jbyteArray)env->CallObjectMethod(str, mid_getBytesEncoding, jstr);
	env->DeleteLocalRef(jstr);
	
	return retArray;
}

jstring javaNewString( JNIEnv *env, jbyteArray javaBytes ) {
	if ( mid_newString == 0 ) {
		if ( class_String == 0 ) {
			jclass cls = env->FindClass("java/lang/String");
			if ( cls == 0 ) {
				return 0;
			}

			class_String = (jclass)env->NewGlobalRef(cls);
			env->DeleteLocalRef(cls);
			if ( class_String == 0 ) {\
				return 0;
			}
		}

		mid_newString = env->GetMethodID(class_String, "<init>", "([B)V");
		if ( mid_newString == 0 ) {
			return 0;
		}
	}

	return (jstring)env->NewObject(class_String, mid_newString, javaBytes );
}

jstring javaNewStringEncoding(JNIEnv *env, jbyteArray javaBytes, const char *encoding )
{
	int len;
	jstring str;
	if ( mid_newString == 0 ) {
		if ( class_String == 0 ) {
			jclass cls = env->FindClass("java/lang/String");
			if ( cls == 0 ) {
				return 0;
			}

			class_String = (jclass)env->NewGlobalRef(cls);
			env->DeleteLocalRef(cls);
			if ( class_String == 0 ) {
				return 0;
			}

		}

		mid_newString = env->GetMethodID(class_String, "<init>", "([BLjava/lang/String;)V");
		if ( mid_newString == 0 ) {
			return 0;
		}
	}

	jstring jstr = env->NewStringUTF(encoding);
	str = (jstring)env->NewObject(class_String, mid_newString, javaBytes, jstr);
	env->DeleteLocalRef(jstr);

	return str;
}

jstring javaNewStringChar(JNIEnv* env, const char* nativeStr) {
	jbyteArray byteArray = cstr2jbyteArray(env, nativeStr);
	jstring jstr = javaNewString(env, byteArray);
	env->DeleteLocalRef(byteArray);

	return jstr;
}
</pre>