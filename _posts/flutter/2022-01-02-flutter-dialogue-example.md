---
layout: post
title: Flutter Dialog 예제
category: Flutter

tag: [Flutter]
---

## Flutter Dialog

<pre class="prettyprint">
showDialog(
  context: context,
  builder: (BuildContext context) {
    // return object of type Dialog
    return AlertDialog(
      title: Text(node.name),
      content: const Text("Edit node."),
      actions: <Widget>[
        TextButton(
          child: const Text("Ok"),
          onPressed: () {
            // TODO
            setState(() {
              node.name = node.name + '+';
            });
            Navigator.pop(context);
          },
        ),
      ],
    );
  },
);
</pre>