#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define EPSILON 0.000001

struct tuple { float x, y, z, w; };

bool equal(float a, float b) {
  if (abs(a - b) < EPSILON) return true;
  return false;
}

struct tuple createPoint(float x, float y, float z) {
  struct tuple t = { x, y, z, 1.0f };
  return t;
}

struct tuple createVector(float x, float y, float z) {
  struct tuple t = { x, y, z, 0.0f };
  return t;
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

struct tuple tupleSub(struct tuple t1, struct tuple t2) {
  struct tuple t3 = { t1.x - t2.x, t1.y - t2.y, t1.z - t2.z};
  return t3;
}

struct tuple tupleNegate(struct tuple t) {
  struct tuple neg = { 0.0f, 0.0f, 0.0f, 0.0f };
  struct tuple ret = tupleSub(neg, t);
  return ret;
}

struct tuple tupleMultScalar(struct tuple t, float s) {
  struct tuple ret = { t.x * s, t.y * s, t.z * s, t.w *s };
  return ret;
}

struct tuple tupleDivScalar(struct tuple t, float s) {
  struct tuple ret = { t.x / s, t.y / s, t.z / s, t.w / s };
  return ret;
}

float tupleMagVec(struct tuple t) {
  float magx = pow(t.x, 2);
  float magy = pow(t.y, 2);
  float magz = pow(t.z, 2);
  float mag = sqrtf( magx + magy + magz);
  return mag;
}

struct tuple normVec(struct tuple t) {
  float mag = tupleMagVec(t);
  struct tuple ret = { t.x / mag, t.y / mag, t.z / mag};
  return ret;
}

float dot(struct tuple t1, struct tuple t2) {
  float prod1 = t1.x * t2.x;
  float prod2 = t1.y * t2.y;
  float prod3 = t1.z * t2.z;
  float prod4 = t1.w * t2.w;
  float dot = prod1 + prod2 + prod3 + prod4;
  return dot;
}

struct tuple cross(struct tuple a, struct tuple b) {
  float x = a.y * b.z - a.z * b.y;
  float y = a.z * b.x - a.x * b.z;
  float z = a.x * b.y - a.y * b.x;
  struct tuple cross = createVector(x, y, z);
  return cross;
}

/*-------------------------------------------------------------*/

// 4 creates tuples with w=1
void createPointTest() {
  struct tuple t = createPoint(4.0f, -4.0f, 3.0f);
  assert(equal(t.x, 4.0f));
  assert(equal(t.y, -4.0f));
  assert(equal(t.z, 3.0f));
  assert(equal(t.w, 1.0f));
}

// 4 creates tuples with w=0
void createVectorTest() {
  struct tuple t = createVector(4.0f, -4.0f, 3.0f);
  assert(equal(t.x, 4.0f));
  assert(equal(t.y, -4.0f));
  assert(equal(t.z, 3.0f));
  assert(equal(t.w, 0.0f));
}

// 4 A tuple with w=1.0 is a point
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

// 6 Adding two tuples
void tupleAddTest() {
  struct tuple a = { 3.0f, -2.0f, 5.0f, 1.0f };
  struct tuple b = { -2.0f, 3.0f, 1.0f, 0.0f };
  struct tuple c = tupleAdd(a, b);
  assert(equal(c.x, 1.0f));
  assert(equal(c.y, 1.0f));
  assert(equal(c.z, 6.0f));
  assert(equal(c.w, 1.0f));
}

// 6 Subtracting two points
void tupleSubTest() {
  struct tuple a = { 3.0f, 2.0f, 1.0f };
  struct tuple b = { 5.0f, 6.0f, 7.0f };
  struct tuple c = tupleSub(a, b);
  assert(equal(c.x, -2.0f));
  assert(equal(c.y, -4.0f));
  assert(equal(c.z, -6.0f));
}

// 6 Subtracting vector from a point
void subtractVetorFromAPointTest() {
  struct tuple pt = createPoint(3.0f, 2.0f, 1.0f);
  struct tuple vec = createVector(5.0f, 6.0f, 7.0f);
  struct tuple ans = tupleSub(pt, vec);
  assert(equal(ans.x, -2.0f));
  assert(equal(ans.y, -4.0f));
  assert(equal(ans.z, -6.0f));
}

// 7 Subtracting two vectors
void subtractTwoVectorsTest() {
  struct tuple vec1 = createVector(3.0f, 2.0f, 1.0f);
  struct tuple vec2 = createVector(5.0f, 6.0f, 7.0f);
  struct tuple vec3 = tupleSub(vec1, vec2);
  assert(equal(vec3.x, -2.0f));
  assert(equal(vec3.y, -4.0f));
  assert(equal(vec3.z, -6.0f));
}

// 7 Subtracting a vector from zero vector
void subtractVectorFromZeroVectorTest() {
  struct tuple zero = createVector(0.0f, 0.0f, 0.0f);
  struct tuple vec1 = createVector(1.0f, -2.0f, 3.0f);
  struct tuple vec2 = tupleSub(zero, vec1);
  assert(equal(vec2.x, -1.0f));
  assert(equal(vec2.y,  2.0f));
  assert(equal(vec2.z, -3.0f));
}

// 7 Negating a tuple
void negatingTupleTest() {
  struct tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  vec1 = tupleNegate(vec1);
  assert(equal(vec1.x, -1.0f));
  assert(equal(vec1.y,  2.0f));
  assert(equal(vec1.z, -3.0f));
}

// 8 Multiply tuple by a scalar
void tupleMultScalarTest() {
  struct tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  float scalar = 3.5f;
  vec1 = tupleMultScalar(vec1, scalar);
  assert(equal(vec1.x,   3.5f));
  assert(equal(vec1.y,  -7.0f));
  assert(equal(vec1.z,  10.5f));
  assert(equal(vec1.w, -14.0f));
}

// 8 Multiply tuple by a fraction
void tupleMultScalarFractionTest() {
  struct tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  float scalar = 0.5f;
  vec1 = tupleMultScalar(vec1, scalar);
  assert(equal(vec1.x, 0.5f));
  assert(equal(vec1.y, -1.0f));
  assert(equal(vec1.z, 1.5f));
  assert(equal(vec1.w, -2.0f));
}

// 8 Divide a tuple by a scalar
void tupleDivScalarTest() {
  struct tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  float scalar = 2.0f;
  vec1 = tupleDivScalar(vec1, scalar);
  assert(equal(vec1.x, 0.5f));
  assert(equal(vec1.y, -1.0f));
  assert(equal(vec1.z, 1.5f));
  assert(equal(vec1.w, -2.0f));
}

// 8 Computing the magnitude of vector(1, 0, 0)
void tupleMagVecTest() {
  struct tuple vec1 = createVector(1.0f, 0.0f, 0.0f);
  float mag = tupleMagVec(vec1);
  assert(equal(mag, 1.0f));

  struct tuple vec2 = createVector(0.0f, 1.0f, 0.0f);
  mag = tupleMagVec(vec2);
  assert(equal(mag, 1.0f));

  struct tuple vec3 = createVector(0.0f, 0.0f, 1.0f);
  mag = tupleMagVec(vec3);
  assert(equal(mag, 1.0f));

  struct tuple vec4 = createVector(1.0f, 2.0f, 3.0f);
  mag = tupleMagVec(vec4);
  assert(equal(mag, sqrt(14.0f)));

  struct tuple vec5 = createVector(-1.0f, -2.0f, -3.0f);
  mag = tupleMagVec(vec5);
  assert(equal(mag, sqrt(14.0f)));
}

// 10 Normalizing vector(4,0,0) gives (1,0,0)
void normVecTest() {
  struct tuple vec1 = createVector(4.0f, 0.0f, 0.0f);
  struct tuple norm = normVec(vec1);
  assert(equal(norm.x, 1.0f));
  assert(equal(norm.y, 0.0f));
  assert(equal(norm.z, 0.0f));

  struct tuple vec2 = createVector(1.0f, 2.0f, 3.0f);
  norm = normVec(vec2);
  float ans1 = 1 / sqrt(14);
  float ans2 = 2 / sqrt(14);
  float ans3 = 3 / sqrt(14);
  assert(equal(norm.x, ans1));
  assert(equal(norm.y, ans2));
  assert(equal(norm.z, ans3));

  struct tuple vec3 = createVector(1.0f, 2.0f, 3.0f);
  norm = normVec(vec3);
  float mag = tupleMagVec(norm);
  assert(equal(mag, 1.0f));
}

// 10 dot rpoduct of two tuples
void dotTest() {
  struct tuple vec1 = createVector(1.0f, 2.0f, 3.0f);
  struct tuple vec2 = createVector(2.0f, 3.0f, 4.0f);
  float dotProd = dot(vec1, vec2);
  assert(equal(dotProd, 20.0f));
}

// 11 cross product of two vectors
void crossTest() {
  struct tuple vec1 = createVector(1.0f, 2.0f, 3.0f);
  struct tuple vec2 = createVector(2.0f, 3.0f, 4.0f);
  struct tuple cross1 = cross(vec1, vec2);
  assert(equal(cross1.x, -1.0f));
  assert(equal(cross1.y,  2.0f));
  assert(equal(cross1.z, -1.0f));
  struct tuple cross2 = cross(vec2, vec1);
  assert(equal(cross2.x,  1.0f));
  assert(equal(cross2.y, -2.0f));
  assert(equal(cross2.z,  1.0f));
}

int main() {
  createPointTest();
  createPointTest();
  tupleWithW0IsAPointTest();
  tupleAddTest();
  tupleSubTest();
  subtractVetorFromAPointTest();
  subtractTwoVectorsTest();
  subtractVectorFromZeroVectorTest();
  negatingTupleTest();
  tupleMultScalarTest();
  tupleMultScalarFractionTest();
  tupleDivScalarTest();
  tupleMagVecTest();
  normVecTest();
  dotTest();
  crossTest();
  return 0;
}