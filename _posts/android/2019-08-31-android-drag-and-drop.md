---
layout: post
title: Drag and Drop 구현
category: Android
tag: [Android]
---

안드로이드에서 드래그&드랍(Drag & Drop) 기능은 API로 제공을 해주기 때문에 아래와 같은 요소만 구현해주면 간편하게 구현할 수 있습니다. 

* Drag를 위해 클립보트(Clipboard)에 저장할 내용
* Drag 동안 화면에 반투명하게 보여줄 이미지
* Drag 이벤트를 처리하는 부분

<br>

## Drag 시작

<pre class="prettyprint">

</pre>
private fun onLongClick(View view) {
        val data = ClipData.newPlainText("message", "hello")
        val builder = View.DragShadowBuilder(view); 

        view.startDragAndDrop(data, builder, view, 0)
}
<br>

## Drag 이벤트 처리

`setOnDragListener` 인터페이스를 구현해주면 됩니다. 여기서 주의할 점은 `return true` 부분입니다. 
`true`로 리턴해야만 `DragEvent.ACTION_DROP` 등의 이벤트를 수신할 수 있습니다. 만약 `false`를 리턴하게 되면,
`DragEvent.ACTION_DRAG_STARTED` 이벤트만 수신하며 나머지 이벤트는 받지 못합니다.

<pre class="prettyprint">
override fun onDrag(v: View?, event: DragEvent?): Boolean {
        when(event?.action) {
            DragEvent.ACTION_DRAG_STARTED -> {
                Log.i("[snowdeer] ACTION_DRAG_STARTED()")

            }

            DragEvent.ACTION_DROP -> {
                Log.i("[snowdeer] ACTION_DROP(${event.x}, ${event.y})")

                // TODO : Item을 Drop 했을 때 처리
            }
        }

        return true
    }
</pre>

