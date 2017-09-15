---
layout: post
title: Windows 10 - 복구 파티션 삭제하기

category: Windows
permalink: /windows/:year/:month/:day/:title/
tag: [Windows]
---
# 복구 파티션 삭제하기

집에 있는 데스크탑을 업그레이드하고, 예전에 사용하던 하드 디스크를 연결했습니다. 덕분에 현재 컴퓨터에 하드 디스크 3개가 주렁주렁 매달려 있습니다. 세세하게 나눴던 파티션을 좀 정리하고 커다란 공간을 쓰고 싶어서 각 하드 디스크의 파티션 정리를 하려고 했습니다.

![image](/assets/tips-windows/017.png)

<br>

하지만, 삭제가 안되는 파티션이 있네요 !!! ‘복구 파티션’이라고 표시된 영역은 컴퓨터 관리 윈도우에서 건드릴 수가 없게 되어 있었습니다. 그냥 무시하고 나머지 영역만 쓸까 생각도 했지만, 17.08 GB는 너무 큰 용량이라서 완전히 삭제하는 방법을 찾아보게 되었습니다.

## 관리자 모드로 커맨드 창 실행

먼저 관리자 모드로 커맨드 창을 실행합니다.

![image](/assets/tips-windows/018.png)

그리고 커맨드 창에서 다음 명령어를 실행합니다.

![image](/assets/tips-windows/019.png)

그 이후, 아까 삭제를 할 수 없었던 물리 디스크를 선택하고(저는 디스크 3번이 삭제가 안되었기 때문에 3번 디스크를 선택했습니다.)

~~~
select disk [번호]
~~~

파티션 리스트를 확인합니다.

~~~
list partition
~~~

![image](/assets/tips-windows/020.png)

그 이후 지우고자 하는 파티션을 선택하고

~~~
select partition [번호]
~~~

삭제를 합니다.

~~~
delete partition override
~~~

![image](/assets/tips-windows/021.png)

그런 다음 컴퓨터 관리 윈도우로 간 다음 파티션이 정상적으로 삭제된 것을 확인할 수 있습니다.

![image](/assets/tips-windows/022.png)