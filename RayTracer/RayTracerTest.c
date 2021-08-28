#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define EPSILON 0.000001

struct tuple { float x, y, z, w; };
struct point { float x, y, z; };

bool equal(float a, float b) {
  if (abs(a - b) < EPSILON) return true;
  return false;
}

bool tupleIsPoint(struct tuple t) { return t.w == 1.0 ? true : false; }
bool tupleIsPoint2(struct tuple t) { 
  if (t.w == 1.0) { return true; }
  return false;
}

bool tupleIsVector(struct tuple t) { return t.w == 0.0 ? true : false; }

bool tupleIsVector2(struct tuple t) { 
  if (t.w == 0.0) { return true;  }
  return false;
}

struct tuple tupleAdd(struct tuple t1, struct tuple t2) {
  struct tuple t3 = { t1.x + t2.x, t1.y + t2.y, t1.z + t2.z, t1.w + t2.w };
  return t3;
}

struct point tupleSub(struct point t1, struct point t2) {
  struct point t3 = { t1.x - t2.x, t1.y - t2.y, t1.z - t2.z};
  return t3;
}

// Pg4
void tupleWithW0IsAPointTest()
{
  struct tuple a = { 4.3f, -4.2f, 3.1f, 1.0f };
  assert(equal(a.x,  4.3f));
  assert(equal(a.y, -4.2f));
  assert(equal(a.z,  3.1f));
  assert(equal(a.w,  1.0f));
  assert(tupleIsPoint(a)  == true);
  assert(tupleIsVector(a) == false);

  struct tuple b = { 4.3f, -4.2f, 3.1f, 0.0f };
  assert(equal(b.x,  4.3f));
  assert(equal(b.y, -4.2f));
  assert(equal(b.z,  3.1f));
  assert(equal(b.w,  0.0f));
  assert(tupleIsPoint(b)  == false);
  assert(tupleIsVector(b) == true);
}

void tupleAddTest() {
  struct tuple a = { 3.0f, -2.0f, 5.0f, 1.0f };
  struct tuple b = { -2.0f, 3.0f, 1.0f, 0.0f };
  struct tuple c = tupleAdd(a, b);
  assert(equal(c.x, 1.0f));
  assert(equal(c.y, 1.0f));
  assert(equal(c.z, 6.0f));
  assert(equal(c.w, 1.0f));
}

void tupleSubTest() {
  struct point a = { 3.0f, 2.0f, 1.0f };
  struct point b = { 5.0f, 6.0f, 7.0f };
  struct point c = tupleSub(a, b);
  assert(equal(c.x, -2.0f));
  assert(equal(c.y, -4.0f));
  assert(equal(c.z, -6.0f));
}

/*
void pointSubVecPointTest() {
  struct point p1 = { 3.0f, 2.0f, 1.0f };
  struct point p2 = { 3.0f, -2.0f, 5.0f };
  struct point p3 = pointSubVecPoint(p1, p2);
  assert(equal(p3.x, -2.0f));
  assert(equal(p3.y, -4.0f));
  assert(equal(p3.z, -6.0f));
}
*/

int main() {
  tupleWithW0IsAPointTest();
  tupleAddTest();
  tupleSubTest();
  return 0;
}