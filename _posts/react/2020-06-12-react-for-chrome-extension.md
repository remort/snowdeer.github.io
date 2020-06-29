---
layout: post
title: React 기반 Chrome Extension Sample
category: React
permalink: /openshift/:year/:month/:day/:title/

tag: [React]
---

## React 프로젝트 생성

<pre class="prettyprint">
yarn create react-app snowdeer-react-chrome-extension-sample

cd snowdeer-react-chrome-extension-sample
</pre>

<br>

## public/manifest.json 파일 수정

<pre class="prettyprint">
{
  "manifest_version": 2,

  "name": "Extension Sample",
  "description": "SnowDeers' Sample extension for Chrome extension",
  "version": "0.0.1",

  "browser_action": {
    "default_popup": "index.html",
    "default_title": "SnowDeer's React Chrome Extension Sample"
  },
  "icons": {
    "16": "logo192.png",
    "48": "logo192.png",
    "128": "logo192.png"
  },
  "content_security_policy": "script-src 'self' 'sha256-[여기는 별도 해시 생성을 해야 합니다]'; object-src 'self'",  
  "permissions": [
  ]
}
</pre>

`manifest_version` 버전 `2` 부터는 CSP(Content Security Policy)가 추가되었습니다. 
더 자세한 내용은 [여기](https://developer.chrome.com/extensions/contentSecurityPolicy)를 참고하세요.

`SHA-256` 값은 별도 해시 함수 등을 이용해서 생성할 수 있습니다. 만약 생성이 어렵더라도 나중에 크롬 브라우저에서 실행을 해보면
해당 스크립트에 알맞은 해시값이 포함된 오류를 볼 수 있기 때문에 그 때 코드를 획득해도 됩니다.

<br>

## src/index.css 파일 수정

굳이 안해도 되는 부분이지만, 작성한 프로그램의 실행 창의 최소 크기를 지정해줬습니다.

<pre class="prettyprint">
body {
  ...
  min-width:800px;
  min-height:800px;
  ...
}
</pre>

<br>

## 빌드

다음 명령어를 이용해서 프로젝트를 빌드합니다. 

<pre class="prettyprint">
yarn build
</pre>

빌드 결과물은 `build` 디렉토리에 생성됩니다.

<br>

## 크롬 브라우저로 실행

크롬 브라우저에서 다음 주소로 접속합니다.

<pre class="prettyprint">
chrome://extensions/ 
</pre>

그리고 오른쪽 상단의 `Developer Mode`를 활성화합니다.

왼쪽 상단 부분에 `Load Unpacked` 버튼을 누른 다음 위에서 `yarn build`로 빌드한 결과물 폴더 `build`를 선택합니다.
정상적으로 설치가 되면 아래 화면과 같이 방금 작성한 App이 리스트에 표시되며, 크롬 브라우저 툴바에도 아이콘이 하나 생성되었음을 
확인할 수 있습니다.

![Image](/assets/react/001.png)

실행 화면은 다음과 같습니다.

![Image](/assets/react/002.png)