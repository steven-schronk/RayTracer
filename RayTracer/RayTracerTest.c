#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define EPSILON 0.000001

#define HEIGHT 50
#define WIDTH  50

struct tuple { float x, y, z, w; };

struct tuple canvas[WIDTH][HEIGHT];

void writePixel(int x, int y, struct tuple color) {
  canvas[x][y] = color;
}

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

struct tuple hadamardProduct(struct tuple c1, struct tuple c2) {
  struct tuple color = { c1.x * c2.x, c1.y * c2.y, c1.z * c2.z };
  return color;
}

// TODO: Merge these three matrix methods together into one.
bool matEqual2x2(float m1[][2], float m2[][2]) {
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j)
      if (m1[i][j] != m2[i][j]) return false;
  return true;
}

bool matEqual3x3(float m1[][3], float m2[][3]) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (m1[i][j] != m2[i][j]) return false;
  return true;
}

bool matEqual4x4(float m1[][4], float m2[][4]) {
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      if (m1[i][j] != m2[i][j]) return false;
  return true;
}

/*-------------------------------------------------------------*/

void unitTest(char* msg, int assert) {
  int msg_length = strlen(msg);
  printf("%s", msg);

  /* 74 is 80 - length of "PASSED" */
  while (msg_length < 74) {
    putchar('.');
    msg_length++;
  }

  if (assert == 1) {
    printf("PASSED\n");
  } else {
    printf("FAILED\n");
  }
}

// TODO: Clamp the values between 0.0 and 1.0
int colorConvert(float x) { return x * 255; }

int writeCanvasToFile() {
  FILE* fp;
  fp = fopen("canvas.ppm", "w");
  fprintf(fp, "P3\n");
  fprintf(fp, "%d %d\n255\n", WIDTH, HEIGHT);
  for (int i = 0; i < WIDTH; ++i) {
    for (int j = 0; j < HEIGHT; ++j) {
      int color = colorConvert(canvas[i][j].x);
      fprintf(fp, "%d ", color);
      color = colorConvert(canvas[i][j].y);
      fprintf(fp, "%d ", color);
      color = colorConvert(canvas[i][j].z);
      fprintf(fp, "%d \n", color);
    }
  }
  return 1;
}

// 4 creates tuples with w=1
int createPointTest() {
  struct tuple t = createPoint(4.0f, -4.0f, 3.0f);
  assert(equal(t.x, 4.0f));
  assert(equal(t.y, -4.0f));
  assert(equal(t.z, 3.0f));
  assert(equal(t.w, 1.0f));
}

// 4 creates tuples with w=0
int createVectorTest() {
  struct tuple t = createVector(4.0f, -4.0f, 3.0f);
  assert(equal(t.x, 4.0f));
  assert(equal(t.y, -4.0f));
  assert(equal(t.z, 3.0f));
  assert(equal(t.w, 0.0f));
}

// 4 A tuple with w=1.0 is a point
int tupleWithW0IsAPointTest()
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
int tupleAddTest() {
  struct tuple a = { 3.0f, -2.0f, 5.0f, 1.0f };
  struct tuple b = { -2.0f, 3.0f, 1.0f, 0.0f };
  struct tuple c = tupleAdd(a, b);
  assert(equal(c.x, 1.0f));
  assert(equal(c.y, 1.0f));
  assert(equal(c.z, 6.0f));
  assert(equal(c.w, 1.0f));
}

// 6 Subtracting two points
int tupleSubTest() {
  struct tuple a = { 3.0f, 2.0f, 1.0f };
  struct tuple b = { 5.0f, 6.0f, 7.0f };
  struct tuple c = tupleSub(a, b);
  assert(equal(c.x, -2.0f));
  assert(equal(c.y, -4.0f));
  assert(equal(c.z, -6.0f));
}

// 6 Subtracting vector from a point
int subtractVetorFromAPointTest() {
  struct tuple pt = createPoint(3.0f, 2.0f, 1.0f);
  struct tuple vec = createVector(5.0f, 6.0f, 7.0f);
  struct tuple ans = tupleSub(pt, vec);
  assert(equal(ans.x, -2.0f));
  assert(equal(ans.y, -4.0f));
  assert(equal(ans.z, -6.0f));
}

// 7 Subtracting two vectors
int subtractTwoVectorsTest() {
  struct tuple vec1 = createVector(3.0f, 2.0f, 1.0f);
  struct tuple vec2 = createVector(5.0f, 6.0f, 7.0f);
  struct tuple vec3 = tupleSub(vec1, vec2);
  assert(equal(vec3.x, -2.0f));
  assert(equal(vec3.y, -4.0f));
  assert(equal(vec3.z, -6.0f));
}

// 7 Subtracting a vector from zero vector
int subtractVectorFromZeroVectorTest() {
  struct tuple zero = createVector(0.0f, 0.0f, 0.0f);
  struct tuple vec1 = createVector(1.0f, -2.0f, 3.0f);
  struct tuple vec2 = tupleSub(zero, vec1);
  assert(equal(vec2.x, -1.0f));
  assert(equal(vec2.y,  2.0f));
  assert(equal(vec2.z, -3.0f));
}

// 7 Negating a tuple
int negatingTupleTest() {
  struct tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  vec1 = tupleNegate(vec1);
  assert(equal(vec1.x, -1.0f));
  assert(equal(vec1.y,  2.0f));
  assert(equal(vec1.z, -3.0f));
}

// 8 Multiply tuple by a scalar
int tupleMultScalarTest() {
  struct tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  float scalar = 3.5f;
  vec1 = tupleMultScalar(vec1, scalar);
  assert(equal(vec1.x,   3.5f));
  assert(equal(vec1.y,  -7.0f));
  assert(equal(vec1.z,  10.5f));
  assert(equal(vec1.w, -14.0f));
}

// 8 Multiply tuple by a fraction
int tupleMultScalarFractionTest() {
  struct tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  float scalar = 0.5f;
  vec1 = tupleMultScalar(vec1, scalar);
  assert(equal(vec1.x, 0.5f));
  assert(equal(vec1.y, -1.0f));
  assert(equal(vec1.z, 1.5f));
  assert(equal(vec1.w, -2.0f));
}

// 8 Divide a tuple by a scalar
int tupleDivScalarTest() {
  struct tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  float scalar = 2.0f;
  vec1 = tupleDivScalar(vec1, scalar);
  assert(equal(vec1.x, 0.5f));
  assert(equal(vec1.y, -1.0f));
  assert(equal(vec1.z, 1.5f));
  assert(equal(vec1.w, -2.0f));
}

// 8 Computing the magnitude of vector(1, 0, 0)
int tupleMagVecTest() {
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
int normVecTest() {
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
int dotTest() {
  struct tuple vec1 = createVector(1.0f, 2.0f, 3.0f);
  struct tuple vec2 = createVector(2.0f, 3.0f, 4.0f);
  float dotProd = dot(vec1, vec2);
  assert(equal(dotProd, 20.0f));
}

// 11 cross product of two vectors
int crossTest() {
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

int hadamardProductTest() {
  struct tuple col1 = createVector(1.0f, 0.2f, 0.4f);
  struct tuple col2 = createVector(0.9f, 1.0f, 0.1f);
  struct tuple col3 = hadamardProduct(col1, col2);
  assert(equal(col3.x, 0.9f));
  assert(equal(col3.x, 0.2f));
  assert(equal(col3.x, 0.04f));
}

int writePixelTest() {
  struct tuple red = createVector(1.0f, 0.0f, 0.0f);
  writePixel(0, 0, red);

  // horizonatal axis
  struct tuple green = createVector(0.0f, 1.0f, 0.0f);
  writePixel(0, 1, green);

  struct tuple blue = createVector(0.0f, 0.0f, 1.0f);
  writePixel(0, 2, blue);

  // vertical axis
  struct tuple sky = createVector(0.3f, 0.6f, 0.9f);
  writePixel(1, 1, sky);

  struct tuple orange = createVector(1.0f, 0.5f, 0.25f);
  writePixel(1, 2, orange);

  assert(equal(canvas[0][0].x, 1.0f));
  assert(equal(canvas[0][0].y, 0.0f));
  assert(equal(canvas[0][0].z, 0.0f));

  assert(equal(canvas[0][1].x, 0.0f));
  assert(equal(canvas[0][1].y, 1.0f));
  assert(equal(canvas[0][1].z, 0.0f));

  assert(equal(canvas[0][2].x, 0.0f));
  assert(equal(canvas[0][2].y, 0.0f));
  assert(equal(canvas[0][2].z, 1.0f));

  assert(equal(canvas[1][1].x, 0.3f));
  assert(equal(canvas[1][1].y, 0.6f));
  assert(equal(canvas[1][1].z, 0.9f));

  assert(equal(canvas[1][2].x, 1.0f));
  assert(equal(canvas[1][2].y, 0.5f));
  assert(equal(canvas[1][2].z, 0.25f));
}

int colorConvertTest() {
  int color = colorConvert(0.0f);
  assert(color == 0);
  color = colorConvert(0.5f);
  assert(color == 127);
  color = colorConvert(1.0f);
  assert(color == 255);
  return 1;
}

int matEqualTest() {
  float oldValue;
  float mat2x2a[2][2] = { { 0.0f, 1.0f }, { 2.0f, 3.0f } };
  float mat2x2b[2][2] = { { 0.0f, 1.0f }, { 2.0f, 3.0f } };
  bool test1 = matEqual2x2(mat2x2a, mat2x2b);
  assert(true == test1);

  // set each element of the first array different one at a time.
  // verify that method catches it
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      oldValue = mat2x2a[i][j];
      mat2x2a[i][j] = 9.0f;
      test1 = matEqual2x2(mat2x2a, mat2x2b);
      assert(false == test1);
      mat2x2a[i][j] = oldValue;
    }
  }

  float mat3x3a[3][3] = { { 0.0f, 1.0f, 2.0f }, { 3.0f, 4.0f, 5.0f }, { 6.0f, 7.0f, 8.0f } };
  float mat3x3b[3][3] = { { 0.0f, 1.0f, 2.0f }, { 3.0f, 4.0f, 5.0f }, { 6.0f, 7.0f, 8.0f } };
  bool test2 = matEqual3x3(mat3x3a, mat3x3b);
  assert(true == test2);

  // set each element of the first array different one at a time.
  // verify that method catches it
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      oldValue = mat3x3a[i][j];
      mat3x3a[i][j] = 9.0f;
      test2 = matEqual3x3(mat3x3a, mat3x3b);
      assert(false == test2);
      mat3x3a[i][j] = oldValue;
    }
  }

  float mat4x4a[4][4] = { { 0.0f, 1.0f, 2.0f }, { 3.0f, 4.0f, 5.0f }, { 6.0f, 7.0f, 8.0f }, { 9.0f, 10.0f, 11.0f } };
  float mat4x4b[4][4] = { { 0.0f, 1.0f, 2.0f }, { 3.0f, 4.0f, 5.0f }, { 6.0f, 7.0f, 8.0f }, { 9.0f, 10.0f, 11.0f } };
  bool test3 = matEqual4x4(mat4x4a, mat4x4b);
  assert(true == test3);

  // set each element of the first array different one at a time.
  // verify that method catches it
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      oldValue = mat4x4a[i][j];
      mat4x4a[i][j] = 12.0f;
      test3 = matEqual4x4(mat4x4a, mat4x4b);
      assert(false == test3);
      mat4x4a[i][j] = oldValue;
    }
  }
  return 1;
}

int main() {
  unitTest("Create Point Test", createPointTest());
  unitTest("Create Vector Test", createVectorTest());
  unitTest("Tuple With 0 Is A Point Test", tupleWithW0IsAPointTest());
  unitTest("Tuple Add Test", tupleAddTest());
  unitTest("Tuple Subtract Test", tupleSubTest());
  unitTest("Subtract Vector From A Point Test", subtractVetorFromAPointTest());
  unitTest("Subtract Two Vectors Test", subtractTwoVectorsTest());
  unitTest("Subtract Vector From Zero Vector Test", subtractVectorFromZeroVectorTest());
  unitTest("Negative Tuple Test", negatingTupleTest());
  unitTest("Tuple Multiplication Scalar Test", tupleMultScalarTest());
  unitTest("Tuple Multiplication Scalar Fraction Test", tupleMultScalarFractionTest());
  unitTest("Tuple Division Scalar Test", tupleDivScalarTest());
  unitTest("Tuple Magnigude Vector Test", tupleMagVecTest());
  unitTest("Normal Vector Test", normVecTest());
  unitTest("Dot Product Test", dotTest());
  unitTest("Cross Product Test", crossTest());
  unitTest("Hadamard Product Test", hadamardProductTest());
  unitTest("Write Pixel Test", writePixelTest());
  unitTest("Color Conversion Test", colorConvertTest());
  unitTest("Matrix Equality Test", matEqualTest());
  unitTest("Write Canvas To File Test", writeCanvasToFile());
  return 0;
}