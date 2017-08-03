---
layout: post
title: Inheritance vs Composition
category: C++
tag: [C++, CleanCode]
---

개발을 하다보면 남발하는 것 중 하나가 '상속(Inheritance)'입니다. 여러 클래스가 있을 때 공통되는 부분을 부모 클래스로 만들어서 상속으로 구현하면 보기에도 좋아보이고 코드양도 줄어들고 뿌듯할 때가 있습니다.

하지만, 상속보다는 이양을 사용하는 편이 설계 측면에서는 훨씬 더 유리합니다. '이양'이란 부모/자식간의 관계처럼 서로 상속하는 관계가 아니라 인스턴스안에 다른 인스턴스를 품어서 그 역할을 전달하는 방법입니다.

상속과 이양은 'Inheritance vs Composition'으로 표현하기도 하며, 'is-A vs has-A'로 표현하기도 합니다. 

상속 관계는 부모와 자식간의 관계가 아주 밀접하게 결합합니다. 부모 클래스가 수정이 되면 자식 클래스에도 영향을 미칩니다. 상속의 예는 다음과 같습니다.

<br>

# Inheritance 예제 코드

<pre class="prettyprint">
class Robot {
 public:
  void update() {
    move();
  }
  // ...
  
 private:
  virtual void move() = 0;
  // ...
};

class CleanerRobot : public Robot {
 private:
  virtual void move() override {
    clean();
    moveForward();
    // ...
  }
};

class CombatRobot : public Robot {
 private:
  virtual void move() override {
    attack();
    rotateLeft();
    // ...
  }
};
</pre>

<br>

# Composition 예제 코드

위 코드를 이양 관계로 바꾸면 아래와 같습니다.

<pre class="prettyprint">
class RobotBehavior {
 public:
  virtual ~RobotBehavior() {}

  virtual void move() = 0;
};

class Robot {
 public:
  Robot(RobotBehavior* behavior) {
    mBehavior(behavior);
  }

  ~Robot() {
    delete mBehavior;
  }

  void update() {
    mBehavior->move();
  }

 private:
  RobotBehavior* mBehavior;
};

class CleanerRobot : public RobotBehavior {
 public:
  virtual void move() override {
    clean();
    moveForward();
    // ...
  }
};

class CombatRobot : public RobotBehavior {
 private:
  virtual void move() override {
    attack();
    rotateLeft();
    // ...
  }
};
</pre>

코드가 길어지고 조금 더 복잡해졌습니다. 하지만 `RobotBehavior`이라는 행동을 하는 추상 클래스가 생기면서 `Robot` 클래스와 나머지 다른 클래스들인 `CleanerRobot`, `CompatRobot` 클래스와의 결합이 끊어졌습니다. 따라서 Robot 클래스의 내용이 변경되더라도 다른 로봇 클래스들에게는 영향을 주지 않습니다.

물론, `CleanerRobot`, `CompatRobot` 클래스들은 `RobotBehavior` 클래스를 상속받기 때문에 `RobotBehavior` 클래스가 수정이 되면 상속받은 클래스들은 모두 변경이 됩니다. 하지만, `RobotBehavior` 클래스는 아주 추상적인 레벨의 행동 인터페이스만 정의되어 있기 때문에 향후 변경될 가능성은 많이 낮습니다. 만약 변수나 구현부가 포함이 된 경우에는 향후 변경될 가능성이 아주 높습니다. 따라서 `RobotBehavior` 클래스에는 최소한의 인터페이스만 정의되어 있어야 합니다.

또한, 상속은 컴파일시 생성이 됩니다. 하지만 이양의 경우는 실행중에도 동적으로 변경이 가능한 장점도 있습니다.

따라서 상속과 이양이 둘 다 가능한 경우에는 가급적 상속보다는 이양을 우선하는게 좋습니다.