---
layout: post
title: Introduction to widgets - (3)
category: Flutter

tag: [Flutter]
---

## 조금 복잡한 예제

<pre class="prettyprint">
class Product {
  const Product({this.name});
  final String name;
}

typedef void CartChangedCallback(Product product, bool inCart);

class ShoppingListItem extends StatelessWidget {
  ShoppingListItem({this.product, this.inCart, this.onCartChanged})
      : super(key: ObjectKey(product));

  final Product product;
  final bool inCart;
  final CartChangedCallback onCartChanged;

  Color _getColor(BuildContext context) {
    // The theme depends on the BuildContext because different parts
    // of the tree can have different themes.
    // The BuildContext indicates where the build is
    // taking place and therefore which theme to use.

    return inCart ? Colors.black54 : Theme.of(context).primaryColor;
  }

  TextStyle _getTextStyle(BuildContext context) {
    if (!inCart) return null;

    return TextStyle(
      color: Colors.black54,
      decoration: TextDecoration.lineThrough,
    );
  }

  @override
  Widget build(BuildContext context) {
    return ListTile(
      onTap: () {
        onCartChanged(product, inCart);
      },
      leading: CircleAvatar(
        backgroundColor: _getColor(context),
        child: Text(product.name[0]),
      ),
      title: Text(product.name, style: _getTextStyle(context)),
    );
  }
}
</pre>

`ShoppingListItem` 클래스는 일반적인 `StatelessWidget` 위젯의 패턴을 따르고 있습니다.
`final`로 정의된 변수들(`product`, `inCart`, `onCartChanged`)은 생성자로부터 전달받아 할당됩니다.
그리고 `build()` 메소드내에서 렌더링에 활용됩니다.

사용자가 위젯을 터치(`onTab()`)하면 `ShoppingListItem` 클래스 내에서 이벤트를 처리하는 것이 아니라 
부모로부터 생성자를 통해 전달받은 `onCartChanged()` 콜백 함수를 호출합니다. 이러한 패턴은 상태(state)값을 해당 위젯이 아닌
상위의 부모에게 전달함으로써 `state`를 더 오래동안 지속되도록 할 수 있으며 `ShoppingListItem` 위젯은 상태 관리할 필요 없이
주어진 값만 렌더링하면 되기 때문에 훨씬 가벼운 위젯이 될 수 있습니다.

`onCartChanged()` 이벤트를 전달받은 상위 부모는 그 안에서 `state`를 바꾸며, 그 결과에 따라 `ShoppingListItem` 인스턴스를
새로 생성하며 렌더링도 다시 이루어지도록 합니다. 이러한 동작은 프레임워크가 변경된 부분만을 갱신하기 때문에 가볍고 빠릅니다. 

<br>

### 부모 위젯 예제

<pre class="prettyprint">
class ShoppingList extends StatefulWidget {
  ShoppingList({Key key, this.products}) : super(key: key);

  final List&lt;Product&gt; products;

  // The framework calls createState the first time a widget
  // appears at a given location in the tree.
  // If the parent rebuilds and uses the same type of
  // widget (with the same key), the framework re-uses the State object
  // instead of creating a new State object.

  @override
  _ShoppingListState createState() => _ShoppingListState();
}

class _ShoppingListState extends State&lt;ShoppingList&gt; {
  Set&lt;Product&gt; _shoppingCart = Set&lt;Product&gt;();

  void _handleCartChanged(Product product, bool inCart) {
    setState(() {
      // When a user changes what's in the cart, you need to change
      // _shoppingCart inside a setState call to trigger a rebuild.
      // The framework then calls build, below,
      // which updates the visual appearance of the app.

      if (!inCart)
        _shoppingCart.add(product);
      else
        _shoppingCart.remove(product);
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Shopping List'),
      ),
      body: ListView(
        padding: EdgeInsets.symmetric(vertical: 8.0),
        children: widget.products.map((Product product) {
          return ShoppingListItem(
            product: product,
            inCart: _shoppingCart.contains(product),
            onCartChanged: _handleCartChanged,
          );
        }).toList(),
      ),
    );
  }
}

void main() {
  runApp(MaterialApp(
    title: 'Shopping App',
    home: ShoppingList(
      products: &lt;Product&gt;[
        Product(name: 'Eggs'),
        Product(name: 'Flour'),
        Product(name: 'Chocolate chips'),
      ],
    ),
  ));
}
</pre>

`ShoppingList` 위젯은 `StatefulWidget` 위젯입니다. `ShoppingList` 위젯이 UI Tree에 추가되면, 프레임워크에서는
`createState()` 메소드를 호출하여 `_ShoppingListState` 안스턴스를 생성합니다.

만약 `ShoppingList` 위젯의 부모 위젯이 새로 갱신되면, 하위 자식 위젯들도 갱신이 됩니다. 이 때, `ShoppingList` 인스턴스는
새로 만들어지지만 `_ShoppingListState` 인스턴스는 기존에 만들어진 인스턴스를 재활용하게 됩니다. `_ShoppingListState`는 
`build()` 메소드를 통해 새로운 위젯을 그리게 되는데, 이 때 `didUpdateWidget()` 메소드를 오버라이딩(overriding)하게 되면 
old Widget 과 current Widget을 비교 후 다시 그릴 필요가 있는지를 정할 수 있습니다.

`setState()`는 `state`가 변경되었음을 프레임워크에 알려주며 프레임워크에서는 자식 위젯들을 새로 `build()`합니다.