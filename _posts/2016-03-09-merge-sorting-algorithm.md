---
layout: post
title: Merge Sort
category: 알고리즘
tag: algorithm, merge, sort
---

~~~
O(N log N) 성능을 가지는 정렬 알고리즘입니다.
~~~

알고리즘 공부를 하다보면 간단한 정렬 알고리즘 하나 정도는 알아 놓는 편이 좋은데,
정렬 알고리즘들은 정말 다양합니다. 성능도 각양각색이구요.

보통 Quick Sort 또는 Merge Sort가 가장 대표적인데, 
둘 다 분할 정복 기반 알고리즘이며 성능도 O(N log N)으로 비슷합니다.

어떤 알고리즘을 쓰나 상관은 없지만
Quick Sort는 Worst Case에서 O(N^2)으로 성능이 저하되지만, 
Merge Sort는 항상 O(N log N)을 유지해주는 고마운(?) 알고리즘입니다. 
평균적으로는 Quick Sort가 더 빠르다는 말도 있는데,
사실 평소에는 어떤 것을 쓰더라도 거기서 거기인 것 같습니다.

~~~
참고로 C++ 표준 라이브러리인 STL에서는 2가지의 sort 함수를 제공하고 있는데,
일반적인 sort는 Quick Sort, stable_sort는 Merge Sort를 사용하고 있다고 합니다.

Merge Sort의 경우 추가적인 메모리가 필요하고, 
메모리 할당, 복사, 삭제 등이 빈번하게 일어나서
실제로는 Quick Sort가 더 빠른 경우가 많다고 합니다.

stable_sort란 특정 알고리즘을 지칭하는 이름은 아니고, 알고리즘의 성격을 말합니다.
정렬 이후에도 값은 값들에 대해서는 처음 순서를 유지할 경우 
해당 정렬 알고리즘이 stable하다고 합니다.
~~~


저는 Merge Sort 알고리즘이 개념적으로 이해하기도 쉽고 구현하기도 쉬워서
애용하고 있습니다.

<br>

자, 그럼 일단 개념부터 한 번 보면,

<p class="message">
분할 정복 알고리즘 기반입니다. 분할을 해볼까요?
</p>

### 분할

![Divide]({{ site.baseurl }}/assets/2016-03-09-merge-sorting-algorithm/2016-03-09-merge-sorting-algorithm-01.png)

그냥 무조건 1/2씩 나누기만 하면 됩니다.
<br><br>

### 정합

이렇게 나누어진 값들은 합치면서 정렬을 해줍니다.

![Conquer]({{ site.baseurl }}/assets/2016-03-09-merge-sorting-algorithm/2016-03-09-merge-sorting-algorithm-02.png)
이런 식으로 1/2로 나누었던 구간들을 다시 합해주면 됩니다.
물론 합칠 때는 정렬을 해주면서 합쳐야겠죠.

대충 이런 느낌?

![Merge]({{ site.baseurl }}/assets/2016-03-09-merge-sorting-algorithm/2016-03-09-merge-sorting-algorithm-03.png)

여기서는 왼쪽의 1이 작으니깐 1을 빈 배열에 넣고, 빈 배열의 offset과 왼쪽 편의
offset1을 각각 증가시켜 줍니다.

![Merge]({{ site.baseurl }}/assets/2016-03-09-merge-sorting-algorithm/2016-03-09-merge-sorting-algorithm-04.png)

이번에는 왼쪽의 4보다는 오른 쪽의 2가 더 작으니 2를 배열에 넣어 주고
각각의 offset을 하나씩 증가시켜 줍니다.

![Merge]({{ site.baseurl }}/assets/2016-03-09-merge-sorting-algorithm/2016-03-09-merge-sorting-algorithm-05.png)

이런 식이 되겠죠. 이렇게 합해가면 Merge Sort 완성입니다. 

<br>

### 코드

c++로 작성한 Merge Sort 코드입니다.

<pre class="prettyprint">
int temp[MAX];
void mergeSort(int* arr, int left, int right) {
    // 종료 조건: 나눌 수 없을 때까지 나눈다.
    if (left >= right) return;

    // 분할
    int mid = (left + right) / 2;
    mergeSort(arr, left, mid);
    mergeSort(arr, mid + 1, right);

    // 정합
    int length = right - left + 1;
    int offset = 0, offset1 = left, offset2 = mid + 1;

    while ((offset1 <= mid) || (offset2 <= right)) {
        if (offset1 > mid) {
            temp[offset++] = arr[offset2++];
            continue;
        }
        if (offset2 > right) {
            temp[offset++] = arr[offset1++];
            continue;
        }
        if (arr[offset1] <= arr[offset2]) {
            temp[offset++] = arr[offset1++];
            continue;
        }
        else {
            temp[offset++] = arr[offset2++];
            continue;
        }
    }

    for (int i = 0; i < length; i++) {
        arr[left + i] = temp[i];
    }
}

</pre>

정합 부분이 코드가 다소 길지만 그래도 사용하기가 쉬워서 저는 Merge Sort를 애용합니다.