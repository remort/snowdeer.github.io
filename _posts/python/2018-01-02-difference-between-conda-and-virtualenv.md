---
layout: post
title: 파이썬 가상환경 conda vs virtualenv
category: Python
tag: [Python]
---
# conda vs virtualenv

`conda`와 `virtualenv`의 차이는 다음과 같습니다.


Task | Conda package and environment manager command | Pip package manager command | Virtualenv environment manager command
---|---|---|---
패키지 설치 | conda install <PACKAGE_NAME> | pip install <PACKAGE_NAME> | 미지원
패키지 업데이트 | conda update --name <ENVIRONMENT_NAME> <PACKAGE_NAME>	| pip install --upgrade <PACKAGE_NAME> | 미지원
삭제 | conda remove --name <ENVIRONMENT_NAME> <PACKAGE_NAME> | pip uninstall <PACKAGE_NAME> | 미지원
가상환경 생성 | Create an environment | conda create --name <ENVIRONMENT_NAME> python | 미지원 | cd <ENV_BASE_DIR>; virtualenv <ENVIRONMENT_NAME>
가상환경 활성화 | source activate <ENVIRONMENT_NAME> | 미지원 | source <ENV_BASE_DIR>/<ENVIRONMENT_NAME>/bin/activate
가상환경 비활성화 | source deactivate | 미지원 | deactivate
이용가능한 패키지 검색 | conda search <SEARCH_TERM> | pip search <SEARCH_TERM> | 미지원
특정 소스로부터 패키지 설치 | conda install --channel <URL> <PACKAGE_NAME> | pip install --index-url <URL> <PACKAGE_NAME> | 미지원
설치된 패키지 리스트 조회 | conda list --name <ENVIRONMENT_NAME> | pip list | 미지원
가상환경 리스트 조회 | conda info --envs | 미지원 | `virtualenv wrapper` 설치 후, `lsvirtualenv` 실행
파이썬 설치 | conda install python=x.x | 미지원 | 미지원
파이썬 업데이트 | conda update python * | 미지원 | 미지원