---
layout: post
title: 커맨드(Command) 패턴 for Game (C++)
category: 디자인패턴
permalink: /designpattern/:year/:month/:day/:title/

tag: [디자인패턴, C++]
---
# Command 패턴

커맨드(Command) 패턴은 메소드 호출을 객체로 감싼 패턴입니다.

커맨드 패턴을 적용하지 않고 일반적으로 구현한 경우를 살펴보겠습니다.

게임에서 조이패드의 버튼이 X, Y, A, B의 4개가 있다고 하고, 각 버튼을 눌렀을 때의 행동을 처리하는 `handleInput()` 메소드가 있다면 다음과 같이 구현될 수 있습니다.

<br>

## InputHandler

<pre class="prettyprint">
void InputHandler::handleInput() {
  if (isPressed(BUTTON_X)) jump();
  else if (isPressed(BUTTON_Y)) attack();
  else if (isPressed(BUTTON_A)) changeWeapon();
  else if (isPressed(BUTTON_B)) dash();
}
</pre>

단순하게 위와 같은 코드로 구현을 해도 되지만, 많은 게임들이 입력 키의 맵핑을 변경할 수 있는 기능을 제공하고 있습니다.

<br>

## Command

키 변경 지원을 위해서는 각 명령을 버튼에 직접 할당하는 것이 아닌, 특별한 객체를 이용할 필요가 있습니다. 따라서 다음과 같은 `Command` 추상 클래스를 선언할 수 있습니다.

<pre class="prettyprint">
class Command {
 public:
  virtual ~Command() {}
  virtual void execute() = 0;
};
</pre>

이제 각 행동별로 하위 클래스를 만들면 됩니다.

<pre class="prettyprint">
class JumpCommand: public Command {
 public:
  virtual void execute() { jump(); }
};
</pre>

<br>

## InputHandler 수정

InputHandler 클래스는 다음과 같이 수정될 것입니다.

<pre class="prettyprint">
class InputHandler {
 public:
  void handleInput();

 private:
  Command *pButtonX;
  Command *pButtonY;
  Command *pButtonA;
  Command *pButtonB;
};
</pre>

또한 구현부의 `handleInput()` 메소드도 다음과 같이 변경될 수 있습니다.

<pre class="prettyprint">
void InputHandler::handleInput() {
  if (isPressed(BUTTON_X)) pButtonX->execute();
  else if (isPressed(BUTTON_Y)) pButtonY->execute();
  else if (isPressed(BUTTON_A)) pButtonA->execute();
  else if (isPressed(BUTTON_B)) pButtonB->execute();
}
</pre>

여기까지가 커맨드 패턴의 핵심 부분입니다.

<br>

# Actor와 Command 분리하기

위의 코드와 같이 커맨드 패턴을 이용하여 각 키들의 맵핑과 행동간의 분리를 할 수 있었습니다. 하지만, Actor와 Command 간에는 여전히 커플링(Coupling)이 존재합니다. `jump()`, `attack()` 등의 행동은 플레이어 객체만 할 수 있습니다.

Actor와 Command를 분리하기 위해서 Command 클래스를 다음과 같이 변경합니다.

<pre class="prettyprint">
class Command {
 public:
  virtual ~Command() {}
  virtual void execute(GameActor &actor) = 0;
};
</pre>

JumpCommand 클래스는 다음과 같이 변경됩니다.

<pre class="prettyprint">
class JumpCommand: public Command {
 public:
  virtual void execute(GameActor &actor) { actor.jump(); }
};
</pre>

이제 JumpCommand 클래스는 어떤 캐릭터에게도 적용할 수 있게 되었습니다. `handleInput()` 메소드는 다음과 같이 변경하여 Command 객체를 리턴하도록 합니다.

<pre class="prettyprint">
Command *InputHandler::handleInput() {
  if (isPressed(BUTTON_X)) pButtonX;
  else if (isPressed(BUTTON_Y)) pButtonY;
  else if (isPressed(BUTTON_A)) pButtonA;
  else if (isPressed(BUTTON_B)) pButtonB;

  return nullptr;
}
</pre>

이제 GameActor 클래스에는 다음 코드가 필요합니다.

<pre class="prettyprint">
  Command* command = inputHandler.handleInput();
  if(command) {
    command->execute(actor);
  }
</pre>

이제 명령을 실행할 때 Actor만 변경하면 어떤 Actor라도 제어가 가능하게 되었습니다.

<br>

# Undo & Redo

Undo 기능과 Redo 기능은 Command 패턴으로 쉽게 구현할 수 있는 대표적인 기능들입니다.

Command들을 리스트로 관리하는 CommandStack을 이용해서 구현할 수도 있고, Command 클래스 자체가 이전 상태를 관리하도록 하고 그 안에 `undo()` 함수를 넣어서 구현할 수도 있습니다.

[CommandStack을 이용하는 방법은 여기](/designpattern/2016/05/06/design-pattern-command/)에 있습니다.

여기서는 Command 클래스 자체에 이전 상태를 저장하고, `undo()` 함수도 가지도록 하겠습니다.

<pre class="prettyprint">
class Command {
 public:
  virtual ~Command() {}
  virtual void execute() = 0;
  virtual void undo() = 0;
};
</pre>

<pre class="prettyprint">
class MoveUnitCommand: public Command {
 public:
  MoveUnitCommand(Unit *unit, int x, int y) : pUnit(unit) {
    prevX(0);
    prevY(0);
    curX(x);
    curY(y);
  }

  virtual void execute() {
    prevX = pUnit->x();
    prevY = pUnit->y();
    pUnit->moveTo(curX, curY);
  }

  virtual void undo() {
    pUnit->moveTo(prevX, prevY);
  }

 private:
  Unit *pUnit;
  int curX, curY;
  int prevX, prevY;
};
</pre>

하지만, 결국 Undo 기능과 Redo 기능을 제대로 구현하려면 Stack이 필요하긴 합니다. 그래야 현재의 Command 포인터를 효율적으로 관리할 수 있습니다.