---
layout: post
title: React CORS 문제 해결하기(Proxy 이용)
category: React
permalink: /openshift/:year/:month/:day/:title/

tag: [React]
---

## CORS

`CORS`(Cross-Origin Resource Sharing)는 교차 출처 리소스 공유라는 기능으로 실행 중인 웹 어플리케이션이 다른 출처의 리소스에 
접근할 수 있는 권한을 부여할 수 있도록 웹브라우저에 알려주는 기능입니다. 

React로 개발을 하다 다른 서버에 있는 데이터를 가져올 때 다음과 같은 오류가 발생하는 경우가 있습니다.

~~~
Access to XMLHttpRequest at 'http://xxx' from origin 'http://localhost:3000' has been blocked by CORS policy: Response to preflight request doesn't pass access control check: No 'Access-Control-Allow-Origin' header is present on the requested resource.
~~~

<br>

이 경우 해당 컨텐츠를 제공하는 서버쪽에서 `CORS` 설정을 헤더(header)에 실어서 보내주는 방법이 정석적인 방식이지만 
제3자가 다른 웹사이트의 데이터를 가져오는 경우에는 쉽지 않습니다.

Proxy 역할을 해주는 중간 서버를 만들어서 문제를 해결할 수도 있지만 여전히 번거롭습니다.
그런데 `Webpack`에서 간단한 방법으로 Proxy 기능을 지원해주기 때문에 `package.json` 파일에 다음 항목만 추가해주면
간단하게 `CORS` 문제를 해결할 수 있습니다.

<br>

## package.json

<pre class="prettyprint">
{
  "proxy": "http://xxx"
}
</pre>

위에서 `http://xxx`는 실제 접속하고자하는 서버의 루트 URL 입니다.

<br>

그 이후 실제 http 리퀘스트를 전송하는 코드에서는 위에서 선언한 루트 URL을 뺀 나머지 부분을 요청하면 됩니다.

<br>

# 예제

`package.json`에 다음과 같이 입력합니다.

<pre class="prettyprint">
{
  "proxy": "https://snowdeer.com"
}
</pre>

그리고 원래 http 리퀘스트 코드가 다음과 같다면

<pre class="prettyprint">
import React, { useState, useEffect } from 'react';
import './App.css';
import axios from 'axios'

import WellstoryMenuApp from './component/WellstoryMenuApp'

const App = () => {
  const URL = 'https://snowdeer.com/menu/getMenuList.do?type=2'

  const [data, setData] = useState(null)
  const [loading, setLoading] = useState(false)

  useEffect(() => {
    const fetchData = async () => {
      setLoading(true)

      try {
        const response = await axios.get(URL, )
        console.log(response)
      }
      catch (e) {
        console.log(e)
      }
      setLoading(false)
    }

    fetchData()
  }, [])

  return (
    &lt;div&gt;
      Hello
    &lt;/div&gt;
  )

}

export default App;
</pre>

<br>

여기서 URL을 다음과 같이 수정하면 됩니다.

<pre class="prettyprint">
const App = () => {
  const URL = '/menu/getMenuList.do?type=2'
}
</pre>
