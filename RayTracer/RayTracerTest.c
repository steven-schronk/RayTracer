#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define EPSILON 0.00000000000000001

#define HEIGHT 50
#define WIDTH  50

struct tuple { float x, y, z, w; };

typedef float Mat2x2[2][2];
typedef float Mat3x3[3][3];
typedef float Mat4x4[4][4];

struct tuple canvas[WIDTH][HEIGHT];

void writePixel(int x, int y, struct tuple color) {
  canvas[x][y] = color;
}

bool equal(float a, float b) {
  assert(!isnan(a)); // Indicates a problem before getting here.
  assert(!isnan(b));
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
bool mat2x2Equal(Mat2x2 m1, Mat2x2 m2) {
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j)
      if (m1[i][j] != m2[i][j]) return false;
  return true;
}

bool mat3x3Equal(float m1[][3], float m2[][3]) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (m1[i][j] != m2[i][j]) return false;
  return true;
}

bool mat4x4Equal(float m1[][4], float m2[][4]) {
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      if (m1[i][j] != m2[i][j]) return false;
  return true;
}

void mat4x4Mul(const Mat4x4 a, const Mat4x4 b, Mat4x4 m) {
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      m[row][col] =
        a[row][0] * b[0][col] +
        a[row][1] * b[1][col] +
        a[row][2] * b[2][col] +
        a[row][3] * b[3][col];
    }
  }
}

void mat4x4MulTuple(const Mat4x4 a, const struct tuple b, struct tuple *c) {
    c->x = b.x * a[0][0] + b.y * a[0][1] + b.z * a[0][2] + b.w * a[0][3];
    c->y = b.x * a[1][0] + b.y * a[1][1] + b.z * a[1][2] + b.w * a[1][3];
    c->z = b.x * a[2][0] + b.y * a[2][1] + b.z * a[2][2] + b.w * a[2][3];
    c->w = b.x * a[3][0] + b.y * a[3][1] + b.z * a[3][2] + b.w * a[3][3];
}

void mat4x4Transpose(Mat4x4 a) {
  float temp;
  for (int i = 0; i < 4; ++i) {
    for (int j = i; j < 4; ++j) {
      temp = a[i][j];
      a[i][j] = a[j][i];
      a[j][i] = temp;
    }
  }
}

void printMat(int rows, int cols, float* mat) {
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      printf("%.2f\t", mat[i * cols + j]);
    }
    printf("\n");
  }
}

float mat2x2Det(Mat2x2 a) {
  return a[0][0] * a[1][1] - a[0][1] * a[1][0];
}

// TODO: Make these more generic
void mat3x3Submat2x2(Mat3x3 a, Mat2x2 b, int row, int col) {
  int current_row = -1;
  int current_col = -1;
  for (int i = 0; i < 3; ++i) {
    if (i == row) { continue; }
    else { ++current_row; }
    current_col = -1;
    for (int j = 0; j < 3; ++j) {
      if (j == col) { continue; }
      else { ++current_col; }
      b[current_row][current_col] = a[i][j];
    }
  }
}

void mat4x4Submat3x3(Mat4x4 a, Mat3x3 b, int row, int col) {
  int current_row = -1;
  int current_col = -1;
  for (int i = 0; i < 4; ++i) {
    if (i == row) { continue; }
    else { ++current_row; }
    current_col = -1;
    for (int j = 0; j < 4; ++j) {
      if (j == col) { continue; }
      else { ++current_col; }
      b[current_row][current_col] = a[i][j];
    }
  }
}

float mat3x3Minor(Mat3x3 a) {
  Mat2x2 b = { { 0.0f, 0.0f }, { 0.0f, 0.0f } };
  mat3x3Submat2x2(a, b, 1, 0);
  return mat2x2Det(b);
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
  Mat2x2 mat2x2a = { { 0.0f, 1.0f }, { 2.0f, 3.0f } };
  Mat2x2 mat2x2b = { { 0.0f, 1.0f }, { 2.0f, 3.0f } };

  bool test1 = mat2x2Equal(mat2x2a, mat2x2b);
  assert(true == test1);

  // set each element of the first array different one at a time.
  // verify that method catches it
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      oldValue = mat2x2a[i][j];
      mat2x2a[i][j] = 9.0f;
      test1 = mat2x2Equal(mat2x2a, mat2x2b);
      assert(false == test1);
      mat2x2a[i][j] = oldValue;
    }
  }

  Mat3x3 mat3x3a = { { 0.0f, 1.0f, 2.0f }, { 3.0f, 4.0f, 5.0f }, { 6.0f, 7.0f, 8.0f } };
  Mat3x3 mat3x3b = { { 0.0f, 1.0f, 2.0f }, { 3.0f, 4.0f, 5.0f }, { 6.0f, 7.0f, 8.0f } };
  bool test2 = mat3x3Equal(mat3x3a, mat3x3b);
  assert(true == test2);

  // set each element of the first array different one at a time.
  // verify that method catches it
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      oldValue = mat3x3a[i][j];
      mat3x3a[i][j] = 9.0f;
      test2 = mat3x3Equal(mat3x3a, mat3x3b);
      assert(false == test2);
      mat3x3a[i][j] = oldValue;
    }
  }

  Mat4x4 mat4x4a = { { 0.0f, 1.0f, 2.0f }, { 3.0f, 4.0f, 5.0f },\
    { 6.0f, 7.0f, 8.0f }, { 9.0f, 10.0f, 11.0f } };
  Mat4x4 mat4x4b = { { 0.0f, 1.0f, 2.0f }, { 3.0f, 4.0f, 5.0f },\
    { 6.0f, 7.0f, 8.0f }, { 9.0f, 10.0f, 11.0f } };
  bool test3 = mat4x4Equal(mat4x4a, mat4x4b);
  assert(true == test3);

  // set each element of the first array different one at a time.
  // verify that method catches it
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      oldValue = mat4x4a[i][j];
      mat4x4a[i][j] = 12.0f;
      test3 = mat4x4Equal(mat4x4a, mat4x4b);
      assert(false == test3);
      mat4x4a[i][j] = oldValue;
    }
  }
  return 1;
}

int mat4x4MulTest() {
  Mat4x4 a = { { 1.0f, 2.0f, 3.0f, 4.0f }, { 5.0f, 6.0f, 7.0f, 8.0f },\
    { 9.0f, 8.0f, 7.0f, 6.0f }, { 5.0f, 4.0f, 3.0f, 2.0f } };
  Mat4x4 b = { { -2.0f, 1.0f, 2.0f, 3.0f }, { 3.0f, 2.0f, 1.0f, -1.0f },\
    { 4.0f, 3.0f, 6.0f, 5.0f }, { 1.0f, 2.0f, 7.0f, 8.0f } };
  Mat4x4 m;
  mat4x4Mul(a, b, m);
  assert(equal(m[0][0],  20.0f));
  assert(equal(m[0][1],  22.0f));
  assert(equal(m[0][2],  50.0f));
  assert(equal(m[0][3],  48.0f));
  assert(equal(m[1][0],  44.0f));
  assert(equal(m[1][1],  54.0f));
  assert(equal(m[1][2], 114.0f));
  assert(equal(m[1][3], 108.0f));
  assert(equal(m[2][0],  40.0f));
  assert(equal(m[2][1],  58.0f));
  assert(equal(m[2][2], 110.0f));
  assert(equal(m[2][3], 102.0f));
  assert(equal(m[3][0],  16.0f));
  assert(equal(m[3][1],  26.0f));
  assert(equal(m[3][2],  46.0f));
  assert(equal(m[3][3],  42.0f));
  return 1;
}

// 30 Matrix multipled by a tuple
int mat4x4MulTupleTest() {
  Mat4x4 a = { { 1.0f, 2.0f, 3.0f, 4.0f }, { 2.0f, 4.0f, 4.0f, 2.0f },\
    { 8.0f, 6.0f, 4.0f, 1.0f }, { 0.0f, 0.0f, 0.0f, 1.0f } };
  struct tuple b = createPoint(1.0f, 2.0f, 3.0f, 1.0f);
  struct tuple c = createPoint(0.0f, 0.0f, 0.0f, 0.0f);
  mat4x4MulTuple(a, b, &c);
  assert(equal(c.x, 18.0f));
  assert(equal(c.y, 24.0f));
  assert(equal(c.z, 33.0f));
  assert(equal(c.w,  1.0f));
  return 1;
}

// 32 Multiply matrix by identity matrix
int mat4x4MultIdentTest() {
  Mat4x4 ident = { { 1.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f },\
    { 0.0f, 0.0f, 1.0f, 0.0f}, { 0.0f, 0.0f, 0.0f, 1.0f } };
  Mat4x4 a = { { 0.0f, 1.0f, 2.0f, 4.0f }, { 1.0f, 2.0f, 4.0f, 8.0f },\
    { 2.0f, 4.0f, 8.0f, 16.0f }, { 4.0f, 8.0f, 16.0f, 32.0f } };  
  Mat4x4 b = { { 0.0f, 1.0f, 2.0f, 4.0f }, { 1.0f, 2.0f, 4.0f, 8.0f },\
    { 2.0f, 4.0f, 8.0f, 16.0f }, { 4.0f, 8.0f, 16.0f, 32.0f } };
  mat4x4Mul(a, ident, a);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      assert(equal(a[i][j], b[i][j]));
    }
  }
  return 1;
}

// TODO: Make these more generic
void mat2x2ResetToZero(Mat2x2 mat) {
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      mat[i][j] = 0.0f;
    }
  }
}

void mat3x3ResetToZero(Mat3x3 mat) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      mat[i][j] = 0.0f;
    }
  }
}

void mat4x4ResetToZero(Mat4x4 mat) {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mat[i][j] = 0.0f;
    }
  }
}

// 33 Transpose a matrix
int mat4x4TransposeTest() {
  Mat4x4 a = { { 0.0f, 9.0f, 3.0f, 0.0f },{ 9.0f, 8.0f, 0.0f, 8.0f },\
    { 1.0f, 8.0f, 5.0f, 3.0f}, { 0.0f, 0.0f, 5.0f, 8.0f } };
  Mat4x4 b = { { 0.0f, 9.0f, 1.0f, 0.0f },{ 9.0f, 8.0f, 8.0f, 0.0f },\
    { 3.0f, 0.0f, 5.0f, 5.0f}, { 0.0f, 8.0f, 3.0f, 8.0f } };
  mat4x4Transpose(&a);
  assert(mat4x4Equal(a, b));
  return 1;
}

// 34 Calculating the determinant of a 2x2 matrix
int mat2x2DetTest() {
  Mat2x2 a = { { 1.0f, 5.0f },{ -3.0f, 2.0f } };
  float det = mat2x2Det(a);
  assert(equal(det, 17.0f));

  Mat2x2 b = { { 5.0f, 0.0f },{ -1.0f, 5.0f } };
  det = mat2x2Det(b);
  assert(equal(det, 25.0f));

  mat2x2ResetToZero(b);
  det = mat2x2Det(b);
  assert(equal(det, 0.0f));

  Mat2x2 c = { { 1.0f, 0.0f },{ 0.0f, -1.0f } };
  det = mat2x2Det(c);
  assert(equal(det, -1.0f));

  Mat2x2 d = { { -1.0f, -1.0f },{ -1.0f, -1.0f } };
  det = mat2x2Det(d);
  assert(equal(det, 0.0f));

  Mat2x2 e = { { 1.0f, 2.0f },{ 3.0f, 4.0f } };
  det = mat2x2Det(e);
  assert(equal(det, -2.0f));
  return 1;
}

// 35 Submatrix of 3x3 matrix is a 2x2 matrix
int mat3x3Submat2x2Test() {
  Mat3x3 a = { { 1.0f, 2.0f, 3.0f },{ 4.0f, 5.0f, 6.0f },{ 7.0f, 8.0f, 9.0f } };
  Mat2x2 b = { { 0.0f, 0.0f },{ 0.0f, 0.0f } };
  mat3x3Submat2x2(a, b, 0, 0);
  assert(equal(b[0][0], 5.0f));
  assert(equal(b[0][1], 6.0f));
  assert(equal(b[1][0], 8.0f));
  assert(equal(b[1][1], 9.0f));

  mat2x2ResetToZero(b);
  mat3x3Submat2x2(a, b, 0, 2);
  assert(equal(b[0][0], 4.0f));
  assert(equal(b[0][1], 5.0f));
  assert(equal(b[1][0], 7.0f));
  assert(equal(b[1][1], 8.0f));

  mat2x2ResetToZero(b);
  mat3x3Submat2x2(a, b, 1, 1);
  assert(equal(b[0][0], 1.0f));
  assert(equal(b[0][1], 3.0f));
  assert(equal(b[1][0], 7.0f));
  assert(equal(b[1][1], 9.0f));
}

// 35 Submatrix of 4x4 matrix is a 3x3 matrix
int mat4x4Submat3x3Test() {
  Mat4x4 a = { { 1.0f, 2.0f, 3.0f, 4.0f },{ 5.0f, 6.0f, 7.0f, 8.0f },\
    { 9.0f, 10.0f, 11.0f, 12.0f},{ 13.0f, 14.0f, 15.0f, 16.0f } };
  Mat3x3 b = { { 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f } };

  mat4x4Submat3x3(a, b, 0, 0);
  assert(equal(b[0][0], 6.0f));
  assert(equal(b[0][1], 7.0f));
  assert(equal(b[0][2], 8.0f));
  assert(equal(b[1][0], 10.0f));
  assert(equal(b[1][1], 11.0f));
  assert(equal(b[1][2], 12.0f));
  assert(equal(b[2][0], 14.0f));
  assert(equal(b[2][1], 15.0f));
  assert(equal(b[2][2], 16.0f));

  mat3x3ResetToZero(b);
  mat4x4Submat3x3(a, b, 2, 1);
  assert(equal(b[0][0],  1.0f));
  assert(equal(b[0][1],  3.0f));
  assert(equal(b[0][2],  4.0f));
  assert(equal(b[1][0],  5.0f));
  assert(equal(b[1][1],  7.0f));
  assert(equal(b[1][2],  8.0f));
  assert(equal(b[2][0], 13.0f));
  assert(equal(b[2][1], 15.0f));
  assert(equal(b[2][2], 16.0f));

  mat3x3ResetToZero(b);
  mat4x4Submat3x3(a, b, 3, 3);
  assert(equal(b[0][0], 1.0f));
  assert(equal(b[0][1], 2.0f));
  assert(equal(b[0][2], 3.0f));
  assert(equal(b[1][0], 5.0f));
  assert(equal(b[1][1], 6.0f));
  assert(equal(b[1][2], 7.0f));
  assert(equal(b[2][0], 9.0f));
  assert(equal(b[2][1], 10.0f));
  assert(equal(b[2][2], 11.0f));
}

int mat3x3MinorTest() {
  Mat3x3 a = { { 3.0f, 5.0f, 0.0f },{ 2.0f, -1.0f, -7.0f },{ 6.0f, -1.0f, 5.0f } };
  float minor = mat3x3Minor(a);
  assert(equal(minor, 25.0f));
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
  unitTest("4x4 Matrix Multiply Test", mat4x4MulTest());
  unitTest("4x4 Matrix Multiply By Tuple Test", mat4x4MulTupleTest());
  unitTest("4x4 Matrix Multiply By Identity Test", mat4x4MultIdentTest());
  unitTest("4x4 Matrix Transposition Test", mat4x4TransposeTest());
  unitTest("2x2 Matrix Determinant Test", mat2x2DetTest());
  unitTest("2x2 Submatrix From 3x3 Matrix Test",mat3x3Submat2x2Test());
  unitTest("3x3 Submatrix From 4x4 Matrix Test", mat4x4Submat3x3Test());
  unitTest("3x3 Matrix Minor Test", mat3x3MinorTest());
  unitTest("Write Canvas To File Test", writeCanvasToFile());
  return 0;
}