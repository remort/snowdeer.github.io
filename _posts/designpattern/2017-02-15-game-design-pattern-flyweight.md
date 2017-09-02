---
layout: post
title: 플라이웨이트(Flyweight) 패턴 for Game (C++)
category: 디자인패턴
permalink: /designpattern/:year/:month/:day/:title/

tag: [디자인패턴, C++]
---
# Flyweight 패턴

플라이웨이트(Flyweight) 패턴은 객체간 동일한 정보나 유사한 정보들을 같이 공유(Sharing)해서 메모리 사용량을 최소화하는 패턴입니다. 게임에서 나무나 관객 등의 배경을 표현할 때 많이 사용되는 패턴입니다.

<br>

## 나무를 표현하는 클래스

숲의 나무를 표현한다고 할 때, 다음과 같은 데이터들이 필요합니다.

* 줄기, 잎 등을 표현하는 폴리곤 메시(Mesh)
* 나무 껍질과 입사귀 텍스처(Texture)
* 나무의 높이와 굵기
* 각각의 나무들의 색상을 조금씩 다르게 하기 위한 색의 농도(Tint)

클래스로 표현하면 다음과 같습니다.

<pre class="prettyprint">
class Tree {
 private:
  Mesh mMesh;
  Texture mBark;
  Texture mLeaves;
  Vector mPosition;
  double mHeight;
  double mThickness;
  Color mBarkColor;
  Color mLeavesColor;
};
</pre>

데이터 종류도 많지만, 메시나 텍스처의 경우 데이터 크기도 큽니다. 이런 나무를 수십, 수백 그루를 표현해야 할 때, 메모리 소모나 성능적인 면에서 한계가 있습니다.

<br>

## 메모리 사용량을 줄이기 위한 방법

특히 메모리를 많이 사용하면서, 공통적인 부분들을 추출하여 하나의 클래스로 만들 수 있습니다. 예를 들어, 다음과 같은 'TreeModel' 클래스를 만들 수 있습니다.

<pre class="prettyprint">
class TreeModel {
 private:
  Mesh mMesh;
  Texture mBark;
  Texture mLeaves;
};
</pre>

기존 'Tree' 클래스는

<pre class="prettyprint">
class Tree {
 private:
  TreeModel *mTreeModel;

  Vector mPosition;
  double mHeight;
  double mThickness;
  Color mBarkTint;
  Color mLeavesTint;
};
</pre>
가 됩니다. 나무가 아무리 많더라도 'TreeModel' 클래스는 하나이기 때문에 메모리 사용량은 확실히 줄어듭니다.

<br>

## 지형을 표현하는 방법

조금 예제를 바꾸어서 이번에는 지형을 표현하는 방법입니다. 지형 정보에는 다음과 같은 정보들이 필요합니다.

* 화면에 렌더링할 때 필요한 텍스처
* 이동할 때 드는 비용(Cost)
* 물인지 육지인지

코드로 표현하면 다음과 같습니다.

<pre class="prettyprint">
enum Terrain {
  TERRAIN_GRASS,
  TERRAIN_HILL,
  TERRAIN_RIVER,
};
</pre>

<pre class="prettyprint">
class World {
 private:
  Terrain mTiles[WIDTH][HEIGHT];
};

int World::getMovementCost(int x, int y) {
  switch (mTiles[x][y]) {
    case TERRAIN_GRASS:
      return 1;
    case TERRAIN_HILL:
      return 2;
    case TERRAIN_RIVER:
      return 3;
  }
}

bool World::isWater(int x, int y) {
  switch (mTiles[x][y]) {
    case TERRAIN_GRASS:
      return false;
    case TERRAIN_HILL:
      return false;
    case TERRAIN_RIVER:
      return true;
  }
}
</pre>

문제없이 돌아가는 코드이지만 지저분하게 구현되어 있습니다. 데이터와 함수들이 분리되어 있어서 응집도도 떨어지기 때문에 이런 경우는 '지형 클래스'를 하나 별도로 구현하는 것이 좋습니다.

<pre class="prettyprint">
class Terrain {
 public:
  Terrain(int movementCost, isWater, Texture texture)
      : mMovementCost(movementCost), mIsWater(isWater), mTexture(texture) {
  }

  int getMovementCost() const {
    return mMovementCost;
  }

  bool isWater() const {
    return mIsWater;
  }

  const Texture &getTexture() const {
    return mTexture;
  }

 private:
  int mMovementCost;
  bool mIsWater;
  Texture mTexture;
};
</pre>

그리고 'World' 클래스도 각 타일마다 'Terrain' 인스턴스를 하나씩 가지도록 하는 것이 아니라 객체 포인터를 가지게 하여 중복되는 데이터를 공유해서 사용할 수 있도록 하는 편이 바람직합니다.

<pre class="prettyprint">
class World {
 private:
  Terrain *mTiles[WIDTH][HEIGHT];
};
</pre>

Terrain 인스턴스의 생명 주기를 좀 더 쉽게 관리하기 위해서 'World' 클래스를 다음과 같은 코드로 수정합니다.

<pre class="prettyprint">
class World {
 public:
  World() : mGrassTerrain(1, false, TEXTURE_GRASS), mHillTerrain(2, false, TEXTURE_HILL),
            mRiverTerrain(3, true, TEXURE_RIVER) {}

  const Terrain &getTile() const;

 private:
  void generateTerrain();
  
  Terrain *mTiles[WIDTH][HEIGHT];

  Terrain mGrassTerrain;
  Terrain mHillTerrain;
  Terrain mRiverTerrain;
};

void World::generateTerrain() {
  for (int x = 0; x < WIDTH; x++) {
    for (int y = 0; y < HEIGHT; y++) {
      if (random(10) == 0) {
        mTiles[x][y] = &mHillTerrain;
      } else {
        mTiles[x][y] = &mGrassTerrain;
      }
    }
  }

  int x = random(WIDTH);
  for (int y = 0; y < HEIGHT; y++) {
    mTiles[x][y] = &mRiverTerrain;
  }
}

const Terrain &World::getTile(int x, int y) const {
  return *mTile[x][y];
}
</pre>