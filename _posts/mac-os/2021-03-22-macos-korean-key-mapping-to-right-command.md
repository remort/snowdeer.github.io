---
layout: post
title: 오른 Command 버튼으로 한/영 전환 기능 사용하기

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# Big Sur 버전에서 Karabiner-Elements 오류

기존에는 `Karabiner-Elements`를 이용해서 오른 <kbd>command</kbd> 키를 이용해서 한/영 전환을 잘 했었는데,
MacOS가 `Big Sur` 버전으로 업그레이드되고 나서 문제가 발생했습니다. 

그래서 다음 방법은 아예 별도의 프로그램없이 사용하는 방법입니다. 

<pre class="prettyprint">
mkdir -p /Users/Shared/bin
printf '%s\n' '#!/bin/sh' \
    'hidutil property --set '"'"'{"UserKeyMapping":[{"HIDKeyboardModifierMappingSrc":0x7000000e7,"HIDKeyboardModifierMappingDst":0x70000006d}]}'"'" \
    >/Users/Shared/bin/userkeymapping
chmod 755 /Users/Shared/bin/userkeymapping
sudo cat<<: >/Users/Shared/bin/userkeymapping.plist
&lt;?xml version="1.0" encoding="UTF-8"?&gt;
&lt;!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "
http://www.apple.com/DTDs/PropertyList-1.0.dtd
"&gt;
&lt;plist version="1.0"&gt;
&lt;dict&gt;
    &lt;key&gt;Label&lt;/key&gt;
    &lt;string&gt;userkeymapping&lt;/string&gt;
    &lt;key&gt;ProgramArguments&lt;/key&gt;
    &lt;array&gt;
        &lt;string&gt;/Users/Shared/bin/userkeymapping&lt;/string&gt;
    &lt;/array&gt;
    &lt;key&gt;RunAtLoad&lt;/key&gt;
    &lt;true/&gt;
&lt;/dict&gt;
&lt;/plist&gt;
:
sudo mv /Users/Shared/bin/userkeymapping.plist /Library/LaunchAgents/userkeymapping.plist
sudo chown root /Library/LaunchAgents/userkeymapping.plist
sudo launchctl load /Library/LaunchAgents/userkeymapping.plist
</pre>

위 스크립트를 실행하고 나면, 오른 <kbd>command</kbd> 키가 <kbd>F18</kbd>로 대체됩니다.

그 이후 `Setting -> Keyboard -> Shortcuts -> Input Source -> Select next source in input menu`로 둘어가서
해당 키를 오른 <kbd>command</kbd> 키로 설정하면 됩니다.

<br>

## 기능 해제

만약 위 기능을 삭제하고 싶은 경우는,

<pre class="prettyprint">
sudo launchctl remove userkeymapping
</pre>

명령어를 수행하면 됩니다.