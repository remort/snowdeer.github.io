---
layout: post
title: dp와 px간 관계
category: Android
tag: [Android, ADB]
---
# 각 화면 밀도에서의 dp와 px간 비율

각 화면 밀도에서 `dp`와 `px`간의 환산표입니다. 하지만, 모든 단말에 대해 100% 일치하지는 않습니다. 단말에 따라 같은 밀도라도 비율이 조금 다른 경우도 있으니 주의해야 합니다.

밀도 | dp | px
--- | --- | ---
ldpi | 1dp | 0.75 px
mdpi | 1dp | 1 px
hdpi | 1dp | 1.5 px
xhdpi | 1dp | 2 px
xxhdpi | 1dp | 3 px
xxxhdpi | 1dp | 4 px