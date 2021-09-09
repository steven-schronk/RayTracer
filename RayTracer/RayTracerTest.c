#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EPSILON 0.000001

#define HEIGHT 100
#define WIDTH  100

struct tuple { double x, y, z, w; };

typedef double Mat2x2[2][2];
typedef double Mat3x3[3][3];
typedef double Mat4x4[4][4];

struct tuple canvas[WIDTH][HEIGHT];

void writePixel(int x, int y, struct tuple color) {
  canvas[x][y] = color;
}

// 5 Comparing floating point numbers
bool equal(double a, double b) {
  assert(!isnan(a)); // Indicates a problem before getting here.
  assert(!isnan(b));
  if (fabs(a - b) < EPSILON) return true;
  return false;
}

struct tuple createPoint(double x, double y, double z) {
  struct tuple t = { x, y, z, 1.0f };
  return t;
}

struct tuple createVector(double x, double y, double z) {
  struct tuple t = { x, y, z, 0.0f };
  return t;
}

/*
Mat4x4 *Mat4x4Create() {
  Mat4x4 mat = { { 3.0f, -9.0f, 7.0f, 3.0f },{ 3.0f, -8.0f, 2.0f, -9.0f },\
      { -4.0f, 4.0f, 4.0f, 1.0f},{ -6.0f, 5.0f, -1.0f, 1.0f } };
  return mat;
}
*/

bool tupleIsPoint(struct tuple t) { return t.w == 1.0 ? true : false; }

bool tupleIsVector(struct tuple t) { return t.w == 0.0 ? true : false; }

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

struct tuple tupleMultScalar(struct tuple t, double s) {
  struct tuple ret = { t.x * s, t.y * s, t.z * s, t.w *s };
  return ret;
}

struct tuple tupleDivScalar(struct tuple t, double s) {
  struct tuple ret = { t.x / s, t.y / s, t.z / s, t.w / s };
  return ret;
}

double tupleMagVec(struct tuple t) {
  double magx = pow(t.x, 2);
  double magy = pow(t.y, 2);
  double magz = pow(t.z, 2);
  double mag = sqrt( magx + magy + magz);
  return mag;
}

struct tuple normVec(struct tuple t) {
  double mag = tupleMagVec(t);
  struct tuple ret = { t.x / mag, t.y / mag, t.z / mag};
  return ret;
}

double dot(struct tuple t1, struct tuple t2) {
  double prod1 = t1.x * t2.x;
  double prod2 = t1.y * t2.y;
  double prod3 = t1.z * t2.z;
  double prod4 = t1.w * t2.w;
  double dot = prod1 + prod2 + prod3 + prod4;
  return dot;
}

struct tuple cross(struct tuple a, struct tuple b) {
  double x = a.y * b.z - a.z * b.y;
  double y = a.z * b.x - a.x * b.z;
  double z = a.x * b.y - a.y * b.x;
  struct tuple cross = createVector(x, y, z);
  return cross;
}

struct tuple hadamardProduct(struct tuple c1, struct tuple c2) {
  struct tuple color = { c1.x * c2.x, c1.y * c2.y, c1.z * c2.z };
  return color;
}

// TODO: Merge these three matrix methods together into one.
// TODO: Might should use the equal method.
bool mat2x2Equal(Mat2x2 m1, Mat2x2 m2) {
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j)
      if (m1[i][j] != m2[i][j]) return false;
  return true;
}

bool mat3x3Equal(double m1[][3], double m2[][3]) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (m1[i][j] != m2[i][j]) return false;
  return true;
}

bool mat4x4Equal(double m1[][4], double m2[][4]) {
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      if (!equal(m1[i][j],m2[i][j])) return false;
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
  double temp;
  for (int i = 0; i < 4; ++i) {
    for (int j = i; j < 4; ++j) {
      temp = a[i][j];
      a[i][j] = a[j][i];
      a[j][i] = temp;
    }
  }
}


void printTuple(struct tuple t) {
  printf("{ %.8f, %.8f, %.8f, %.8f }\n", t.x, t.y, t.z, t.w);
}

void printMat(const int rows, const int cols, const double* mat) {
  printf("{ ");
  for (int i = 0; i < rows; ++i) {
    printf(" { ");
    for (int j = 0; j < cols; ++j) {
      printf("%.3f", mat[i * cols + j]);
      if (j < cols-1) { printf(", "); }
    }
    printf(" }");
    if (i < rows - 1) { printf(", "); }
  }
  printf("}\n");
}

double mat2x2Det(Mat2x2 a) {
  return a[0][0] * a[1][1] - a[0][1] * a[1][0];
}

double mat3x3Det(Mat3x3 m) {
  double a = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]);
  double b = m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]);
  double c = m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  double ans = a - b + c;
  return ans;
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

double mat3x3Minor(Mat3x3 a, int row, int col) {
  Mat2x2 b = { { 0.0f, 0.0f }, { 0.0f, 0.0f } };
  mat3x3Submat2x2(a, b, row, col);
  return mat2x2Det(b);
}

double mat4x4Minor(Mat4x4 a, int row, int col) {
  Mat3x3 b = { { 0.0f, 0.0f }, { 0.0f, 0.0f }, { 0.0f, 0.0f } };
  mat4x4Submat3x3(a, b, row, col);
  return mat3x3Det(b);
}

double mat3x3Cofactor(Mat3x3 a, int row, int col) {
  double minor = mat3x3Minor(a, row, col);
  if ((row + col) % 2 != 0) { minor *= -1; }
  return minor;
}

double mat4x4Cofactor(Mat4x4 a, int row, int col) {
  double minor = mat4x4Minor(a, row, col);
  if ((row + col) % 2 != 0) { minor *= -1; }
  return minor;
}

double mat4x4Det(Mat4x4 m, int size) {
  double detVal = 0.0f;
  double mat3Cof = 0.0f;
  for (int column = 0; column < size; ++column) {
    mat3Cof = mat4x4Cofactor(m, 0, column);
    detVal = detVal + m[0][column] * mat3Cof;
  }
  return detVal;
}

bool invertableMatrix(Mat4x4 m) {
  if(equal(mat4x4Det(m, 4),0)) { return false; }
  return true;
}

bool mat4x4Inverse(Mat4x4 a, Mat4x4 b) {
  bool invert = invertableMatrix(a);
  if (!invert) { return false; }
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double c = mat4x4Cofactor(a, i, j);
      b[j][i] = c / mat4x4Det(a, 4);
    }
  }
  return true;
}

void genTranslateMatrix(const double x, const double y, const double z, Mat4x4 m) {
  m[0][0] = 1.0f; m[0][1] = 0.0f; m[0][2] = 0.0f; m[0][3] = x;
  m[1][0] = 0.0f; m[1][1] = 1.0f; m[1][2] = 0.0f; m[1][3] = y;
  m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = 1.0f; m[2][3] = z;
  m[3][0] = 0.0f; m[3][1] = 0.0f; m[3][2] = 0.0f; m[3][3] = 1.0f;
}

void genScaleMatrix(const double x, const double y, const double z, Mat4x4 m) {
  m[0][0] = x;    m[0][1] = 0.0f; m[0][2] = 0.0f; m[0][3] = 0.0f;
  m[1][0] = 0.0f; m[1][1] = y;    m[1][2] = 0.0f; m[1][3] = 0.0f;
  m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = z;    m[2][3] = 0.0f;
  m[3][0] = 0.0f; m[3][1] = 0.0f; m[3][2] = 0.0f; m[3][3] = 1.0f;
}

void genRotationMatrixX(const double rad, Mat4x4 m) {
  m[0][0] = 1.0f; m[0][1] = 0.0f;     m[0][2] = 0.0f;      m[0][3] = 0.0f;
  m[1][0] = 0.0f; m[1][1] = cos(rad); m[1][2] = -sin(rad); m[1][3] = 0.0f;
  m[2][0] = 0.0f; m[2][1] = sin(rad); m[2][2] = cos(rad);  m[2][3] = 0.0f;
  m[3][0] = 0.0f; m[3][1] = 0.0f;     m[3][2] = 0.0f;      m[3][3] = 1.0f;
}

void genRotationMatrixY(const double rad, Mat4x4 m) {
  m[0][0] = cos(rad);  m[0][1] = 0.0f; m[0][2] = sin(rad); m[0][3] = 0.0f;
  m[1][0] = 0.0f;      m[1][1] = 1.0f; m[1][2] = 0.0f;     m[1][3] = 0.0f;
  m[2][0] = -sin(rad); m[2][1] = 0.0f; m[2][2] = cos(rad); m[2][3] = 0.0f;
  m[3][0] = 0.0f;      m[3][1] = 0.0f; m[3][2] = 0.0f;     m[3][3] = 1.0f;
}

void genRotationMatrixZ(const double rad, Mat4x4 m) {
  m[0][0] = cos(rad); m[0][1] = -sin(rad); m[0][2] = 0.0f; m[0][3] = 0.0f;
  m[1][0] = sin(rad); m[1][1] = cos(rad);  m[1][2] = 0.0f; m[1][3] = 0.0f;
  m[2][0] = 0.0f;     m[2][1] = 0.0f;      m[2][2] = 1.0f; m[2][3] = 0.0f;
  m[3][0] = 0.0f;     m[3][1] = 0.0f;      m[3][2] = 0.0f; m[3][3] = 1.0f;
}

void genShearMatrix(const double xy, const double xz, const double yx, const double yz, const double zx, const double zy, Mat4x4 m) {
  m[0][0] = 1.0f; m[0][1] = xy;   m[0][2] = xz;   m[0][3] = 0.0f;
  m[1][0] = yx;   m[1][1] = 1.0f; m[1][2] = yz;   m[1][3] = 0.0f;
  m[2][0] = zx;   m[2][1] = zy;   m[2][2] = 1.0f; m[2][3] = 0.0f;
  m[3][0] = 0.0f; m[3][1] = 0.0f; m[3][2] = 0.0f; m[3][3] = 1.0f;
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
int colorConvert(double x) { return x * 255; }

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
  return 1;
}

// 4 creates tuples with w=0
int createVectorTest() {
  struct tuple t = createVector(4.0f, -4.0f, 3.0f);
  assert(equal(t.x, 4.0f));
  assert(equal(t.y, -4.0f));
  assert(equal(t.z, 3.0f));
  assert(equal(t.w, 0.0f));
  return 1;
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
  return 1;
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
  return 1;
}

// 6 Subtracting two points
int tupleSubTest() {
  struct tuple a = { 3.0f, 2.0f, 1.0f };
  struct tuple b = { 5.0f, 6.0f, 7.0f };
  struct tuple c = tupleSub(a, b);
  assert(equal(c.x, -2.0f));
  assert(equal(c.y, -4.0f));
  assert(equal(c.z, -6.0f));
  return 1;
}

// 6 Subtracting vector from a point
int subtractVetorFromAPointTest() {
  struct tuple pt = createPoint(3.0f, 2.0f, 1.0f);
  struct tuple vec = createVector(5.0f, 6.0f, 7.0f);
  struct tuple ans = tupleSub(pt, vec);
  assert(equal(ans.x, -2.0f));
  assert(equal(ans.y, -4.0f));
  assert(equal(ans.z, -6.0f));
  return 1;
}

// 7 Subtracting two vectors
int subtractTwoVectorsTest() {
  struct tuple vec1 = createVector(3.0f, 2.0f, 1.0f);
  struct tuple vec2 = createVector(5.0f, 6.0f, 7.0f);
  struct tuple vec3 = tupleSub(vec1, vec2);
  assert(equal(vec3.x, -2.0f));
  assert(equal(vec3.y, -4.0f));
  assert(equal(vec3.z, -6.0f));
  return 1;
}

// 7 Subtracting a vector from zero vector
int subtractVectorFromZeroVectorTest() {
  struct tuple zero = createVector(0.0f, 0.0f, 0.0f);
  struct tuple vec1 = createVector(1.0f, -2.0f, 3.0f);
  struct tuple vec2 = tupleSub(zero, vec1);
  assert(equal(vec2.x, -1.0f));
  assert(equal(vec2.y,  2.0f));
  assert(equal(vec2.z, -3.0f));
  return 1;
}

// 7 Negating a tuple
int negatingTupleTest() {
  struct tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  vec1 = tupleNegate(vec1);
  assert(equal(vec1.x, -1.0f));
  assert(equal(vec1.y,  2.0f));
  assert(equal(vec1.z, -3.0f));
  return 1;
}

// 8 Multiply tuple by a scalar
int tupleMultScalarTest() {
  struct tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  double scalar = 3.5f;
  vec1 = tupleMultScalar(vec1, scalar);
  assert(equal(vec1.x,   3.5f));
  assert(equal(vec1.y,  -7.0f));
  assert(equal(vec1.z,  10.5f));
  assert(equal(vec1.w, -14.0f));
  return 1;
}

// 8 Multiply tuple by a fraction
int tupleMultScalarFractionTest() {
  struct tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  double scalar = 0.5f;
  vec1 = tupleMultScalar(vec1, scalar);
  assert(equal(vec1.x, 0.5f));
  assert(equal(vec1.y, -1.0f));
  assert(equal(vec1.z, 1.5f));
  assert(equal(vec1.w, -2.0f));
  return 1;
}

// 8 Divide a tuple by a scalar
int tupleDivScalarTest() {
  struct tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  double scalar = 2.0f;
  vec1 = tupleDivScalar(vec1, scalar);
  assert(equal(vec1.x, 0.5f));
  assert(equal(vec1.y, -1.0f));
  assert(equal(vec1.z, 1.5f));
  assert(equal(vec1.w, -2.0f));
  return 1;
}

// 8 Computing the magnitude of vector(1, 0, 0)
int tupleMagVecTest() {
  struct tuple vec1 = createVector(1.0f, 0.0f, 0.0f);
  double mag = tupleMagVec(vec1);
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
  return 1;
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
  double ans1 = 1 / sqrt(14);
  double ans2 = 2 / sqrt(14);
  double ans3 = 3 / sqrt(14);
  assert(equal(norm.x, ans1));
  assert(equal(norm.y, ans2));
  assert(equal(norm.z, ans3));

  struct tuple vec3 = createVector(1.0f, 2.0f, 3.0f);
  norm = normVec(vec3);
  double mag = tupleMagVec(norm);
  assert(equal(mag, 1.0f));
  return 1;
}

// 10 dot rpoduct of two tuples
int dotTest() {
  struct tuple vec1 = createVector(1.0f, 2.0f, 3.0f);
  struct tuple vec2 = createVector(2.0f, 3.0f, 4.0f);
  double dotProd = dot(vec1, vec2);
  assert(equal(dotProd, 20.0f));
  return 1;
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
  return 1;
}

// 18 Hadamard product
int hadamardProductTest() {
  struct tuple col1 = createVector(1.0f, 0.2f, 0.4f);
  struct tuple col2 = createVector(0.9f, 1.0f, 0.1f);
  struct tuple col3 = hadamardProduct(col1, col2);
  assert(equal(col3.x, 0.899999976f));
  assert(equal(col3.y, 0.2f));
  assert(equal(col3.z, 0.04f));
  return 1;
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
  return 1;
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
  double oldValue;
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
  struct tuple b = createPoint(1.0f, 2.0f, 3.0f);
  struct tuple c = createPoint(0.0f, 0.0f, 0.0f);
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
  mat4x4Transpose(a);
  assert(mat4x4Equal(a, b));
  return 1;
}

// 34 Calculating the determinant of a 2x2 matrix
int mat2x2DetTest() {
  Mat2x2 a = { { 1.0f, 5.0f },{ -3.0f, 2.0f } };
  double det = mat2x2Det(a);
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
  return 1;
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
  return 1;
}

// 35 Calculating a minor of a 3x3 matrix
int mat3x3MinorTest() {
  Mat3x3 a = { { 3.0f, 5.0f, 0.0f },{ 2.0f, -1.0f, -7.0f },{ 6.0f, -1.0f, 5.0f } };
  double minor = mat3x3Minor(a, 1, 0);
  assert(equal(minor, 25.0f));
  return 1;
}

// 36 Calculating a cofactor of a 3x3 matrix
int mat3x3CofactorTest() {
  Mat3x3 a = { { 3.0f, 5.0f, 0.0f },{ 2.0f, -1.0f, -7.0f },{ 6.0f, -1.0f, 5.0f } };
  double minor = mat3x3Minor(a, 0, 0);
  assert(equal(minor, -12.0f));

  double cofactor = mat3x3Cofactor(a, 0, 0);
  assert(equal(cofactor, -12.0f));

  minor = mat3x3Minor(a, 1, 0);
  assert(equal(minor, 25.0f));

  cofactor = mat3x3Cofactor(a, 1, 0);
  assert(equal(cofactor, -25.0f));
  return 1;
}

// 37 Calculating the determinant of a 3x3 matrix
int mat3x3DetTest() {
  Mat3x3 a = { { 1.0f, 2.0f, 6.0f },{ -5.0f, 8.0f, -4.0f },{ 2.0f, 6.0f, 4.0f } };
  double cofactor = mat3x3Cofactor(a, 0, 0);
  assert(equal(cofactor, 56.0f));

  cofactor = mat3x3Cofactor(a, 0, 1);
  assert(equal(cofactor, 12.0f));

  cofactor = mat3x3Cofactor(a, 0, 2);
  assert(equal(cofactor, -46.0f));

  double det = mat3x3Det(a);
  assert(equal(det, -196.0f));
  return 1;
}

// 37 Calculating the determinant of a 4x4 matrix
int mat4x4DetTest() {
  Mat4x4 a = { { -2.0f, -8.0f, 3.0f, 5.0f },{ -3.0f, 1.0f, 7.0f, 3.0f },\
    { 1.0f, 2.0f, -9.0f, 6.0f},{ -6.0f, 7.0f, 7.0f, -9.0f } };

  double cofactor = mat4x4Cofactor(a, 0, 0);
  assert(equal(cofactor, 690.0f));
  cofactor = mat4x4Cofactor(a, 0, 1);
  assert(equal(cofactor, 447.0f));
  cofactor = mat4x4Cofactor(a, 0, 2);
  assert(equal(cofactor, 210.0f));
  cofactor = mat4x4Cofactor(a, 0, 3);
  assert(equal(cofactor, 51.0f));
  double det = mat4x4Det(a, 4);
  assert(equal(det, -4071.0f));
  return 1;
}

// 39 Testing an invertable matrix for invertability
int invertableMatrixTest() {
  Mat4x4 a = { { 6.0f, 4.0f, 4.0f, 4.0f },{ 5.0f, 5.0f, 7.0f, 6.0f },\
    { 4.0f, -9.0f, 3.0f, -7.0f},{ 9.0f, 1.0f, 7.0f, -6.0f } };
  bool inv = invertableMatrix(a);
  assert(inv == true);

  Mat4x4 b = { { -4.0f, 2.0f, -2.0f, -3.0f },{ 9.0f, 6.0f, 2.0f, 6.0f },\
    { 0.0f, -5.0f, 1.0f, -5.0f},{ 0.0f, 0.0f, 0.0f, 0.0f } };
  inv = invertableMatrix(b);
  assert(inv == false);
  return 1;
}

// 39 Calculating the inverse of a matrix
int inverseMatrixTest() {
  Mat4x4 a = { { -5.0f, 2.0f, 6.0f, -8.0f },{ 1.0f, -5.0f, 1.0f, 8.0f },\
      { 7.0f, 7.0f, -6.0f, -7.0f},{ 1.0f, -3.0f, 7.0f, 4.0f } };
  Mat4x4 b = { { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f },\
    { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f } };
  Mat4x4 c = { { 0.21804512f, 0.45112783f, 0.24060151f, -0.04511278f },{ -0.80827069f, -1.45676696f, -0.44360903f, 0.52067667f },\
    { -0.07894737f, -0.22368421f, -0.05263158f, 0.19736843f},{ -0.52255636f, -0.81390977f, -0.30075186f, 0.30639097f } };

  bool inversable = mat4x4Inverse(a, b);
  assert(inversable == true);
  double det = mat4x4Det(a, 4);
  assert(equal(det, 532.0f));
  double cof = mat4x4Cofactor(a, 2, 3);
  assert(equal(cof, -160.0f));
  assert(equal(b[3][2], -160.0f/532.0f));
  cof = mat4x4Cofactor(a, 3, 2);
  assert(equal(b[2][3], 105.0f/532.0f));
  assert(mat4x4Equal(b, c) == true);

  Mat4x4 d = { { 8.0f, -5.0f, 9.0f, 2.0f },{ 7.0f, 5.0f, 6.0f, 1.0f },\
        { -6.0f, 0.0f, 9.0f, 6.0f},{ -3.0f, 0.0f, -9.0f, -4.0f } };
  Mat4x4 e = { { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f },\
    { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f } };
  Mat4x4 f = { { -0.15384616f, -0.15384616f, -0.28205130f, -0.53846157f },{ -0.07692308f, 0.12307692f, 0.02564103f, 0.03076923f },\
      { 0.35897437f, 0.35897437f, 0.43589744f, 0.92307693f},{ -0.69230771f, -0.69230771f, -0.76923078f, -1.92307687f } };

  inversable = mat4x4Inverse(d, e);
  assert(inversable == true);
  assert(mat4x4Equal(e, f) == true);

  Mat4x4 g = { { 9.0f, 3.0f, 0.0f, 9.0f },{ -5.0f, -2.0f, -6.0f, -3.0f },\
        { -4.0f, 9.0f, 6.0f, 4.0f},{ -7.0f, 6.0f, 6.0f, 2.0f } };
  Mat4x4 h = { { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f },\
    { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f } };
  Mat4x4 i = { { -0.04074074f, -0.07777778f, 0.14444445f, -0.22222222f },{ -0.07777778f, 0.03333334f, 0.36666667f, -0.33333334f },\
      { -0.02901234f, -0.14629629f, -0.10925926f, 0.12962963f },{ 0.17777778f, 0.06666667f, -0.26666668f, 0.33333334f } };

  inversable = mat4x4Inverse(g, h);
  assert(inversable == true);
  assert(mat4x4Equal(e, f) == true);
  return 1;
}

// 41 Multiply product by its inverse
int MultProdByInverseTest() {
  Mat4x4 a = { { 3.0f, -9.0f, 7.0f, 3.0f },{ 3.0f, -8.0f, 2.0f, -9.0f },\
      { -4.0f, 4.0f, 4.0f, 1.0f},{ -6.0f, 5.0f, -1.0f, 1.0f } };
  Mat4x4 b = { { 8.0f, 2.0f, 2.0f, 2.0f },{ 3.0f, -1.0f, 7.0f, 0.0f },\
    { 7.0f, 0.0f, 5.0f, 4.0f },{ 6.0f, -2.0f, 0.0f, 5.0f } };
  Mat4x4 c = { { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f },\
    { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f } };

  mat4x4Mul(a, b, c);
  Mat4x4 t;
  bool inversable = mat4x4Inverse(b, t);
  Mat4x4 u;
  mat4x4Mul(c, t, u);
  assert(inversable == true);
  assert(equal(u[0][0], a[0][0]));
  assert(equal(u[0][1], a[0][1]));
  assert(equal(u[0][2], a[0][2]));
  assert(equal(u[0][3], a[0][3]));
  return 1;
}

// 45 Multiply by a translation matrix
int PointTransTest() {
  struct tuple point1 = createPoint(-3.0f, 4.0f, 5.0f);
  struct tuple point2 = createPoint( 0.0f, 0.0f, 0.0f);
  Mat4x4 trans;
  genTranslateMatrix(5.0f, -3.0f, 2.0f, trans);
  mat4x4MulTuple(trans, point1, &point2);
  assert(equal(point2.x, 2.0f));
  assert(equal(point2.y, 1.0f));
  assert(equal(point2.z, 7.0f));
  assert(equal(point2.w, 1.0f));
  return 1;
}

// 45 Multiply by the inverse of a traslation matrix
int pointMultInverseTranslationTest() {
  Mat4x4 trans;
  Mat4x4 transInverse;
  genTranslateMatrix(5.0f, -3.0f, 2.0f, trans);
  mat4x4Inverse(trans, transInverse);
  struct tuple p1 = createPoint(-3.0f, 4.0f, 5.0f);
  struct tuple p2 = createPoint(0.0f, 0.0f, 0.0f);
  mat4x4MulTuple(transInverse, p1, &p2);
  assert(equal(p2.x, -8.0f));
  assert(equal(p2.y,  7.0f));
  assert(equal(p2.z,  3.0f));
  assert(equal(p2.w,  1.0f));
  return 1;
}

// 45 Translation does not affect vectors
int vectorTranslationHasNoEffectTest() {
  Mat4x4 trans;
  genTranslateMatrix(5.0f, -3.0f, 2.0f, trans);
  struct tuple v1 = createVector(-3.0f, 4.0f, 5.0f);
  struct tuple v2 = createPoint(0.0f, 0.0f, 0.0f);
  mat4x4MulTuple(trans, v1, &v2);
  assert(equal(v2.x, -3.0f));
  assert(equal(v2.y,  4.0f));
  assert(equal(v2.z,  5.0f));
  assert(equal(v2.w,  0.0f));
  return 1;
}

// 46 Scaling matrix applied to a point
int pointScaleMat4x4Test() {
  struct tuple p1 = createPoint(-4.0f, 6.0f, 8.0f);
  struct tuple p2 = createPoint(0.0f, 0.0f, 0.0f);
  Mat4x4 scaleMat;
  genScaleMatrix(2.0f, 3.0f, 4.0f, scaleMat);
  mat4x4MulTuple(scaleMat, p1, &p2);
  assert(equal(p2.x, -8.0f));
  assert(equal(p2.y, 18.0f));
  assert(equal(p2.z, 32.0f));
  assert(equal(p2.w,  1.0f));
  return 1;
}

// 46 Scaling matrix applied to a vector
int vecScaleMat4x4Test() {
  struct tuple p1 = createVector(-4.0f, 6.0f, 8.0f);
  struct tuple p2 = createVector(0.0f, 0.0f, 0.0f);
  Mat4x4 scaleMat;
  genScaleMatrix(2.0f, 3.0f, 4.0f, scaleMat);
  mat4x4MulTuple(scaleMat, p1, &p2);
  assert(equal(p2.x, -8.0f));
  assert(equal(p2.y, 18.0f));
  assert(equal(p2.z, 32.0f));
  assert(equal(p2.w,  0.0f));
  return 1;
}

// 46 Multiply inverse of scaling matrix
int multInverseScaleMatrixTest() {
  Mat4x4 scaleMat;
  Mat4x4 scaleMatInv;
  struct tuple p1 = createVector(-4.0f, 6.0f, 8.0f);
  struct tuple p2 = createVector(0.0f, 0.0f, 0.0f);
  genScaleMatrix(2.0f, 3.0f, 4.0f, scaleMat);
  mat4x4Inverse(scaleMat, scaleMatInv);
  mat4x4MulTuple(scaleMatInv, p1, &p2);
  assert(equal(p2.x, -2.0f));
  assert(equal(p2.y,  2.0f));
  assert(equal(p2.z,  2.0f));
  assert(equal(p2.w,  0.0f));
  return 1;
}

// 48 Rotating a point around the x axis
int genRotationMatrixXTest() {
  Mat4x4 rotMat;
  struct tuple p1 = createPoint(0.0f, 1.0f, 0.0f);
  struct tuple p2 = createPoint(7.0f, 8.0f, 9.0f);
  genRotationMatrixX(M_PI / 4, rotMat);
  mat4x4MulTuple(rotMat, p1, &p2);
  assert(equal(p2.x, 0.0f));
  assert(equal(p2.y, sqrt(2.0f)/2.0f));
  assert(equal(p2.z, sqrt(2.0f) / 2.0f));
  assert(equal(p2.w, 1.0f));

  genRotationMatrixX(M_PI / 2, rotMat);
  mat4x4MulTuple(rotMat, p1, &p2);
  assert(equal(p2.x, 0.0f));
  assert(equal(p2.y, 0.0f));
  assert(equal(p2.z, 1.0f));
  assert(equal(p2.w, 1.0f));
  return 1;
}

// 49 Inverse of an x rotation rotates the opposite direction
int genRotationMatrixReverseTest() {
  Mat4x4 rotMat;
  Mat4x4 rotMatInv;
  struct tuple p1 = createPoint(0.0f, 1.0f, 0.0f);
  struct tuple p2 = createPoint(7.0f, 8.0f, 9.0f);
  genRotationMatrixX(M_PI / 4, rotMat);
  mat4x4Inverse(rotMat, rotMatInv);
  mat4x4MulTuple(rotMatInv, p1, &p2);
  assert(equal(p2.x, 0.0f));
  assert(equal(p2.y, sqrt(2.0f) / 2.0f));
  assert(equal(p2.z, -sqrt(2.0f) / 2.0f));
  assert(equal(p2.w, 1.0f));
  return 1;
}

// 50 Rotating a point around the y axis
int genRotationMatrixYTest() {
  Mat4x4 rotMat;
  struct tuple p1 = createPoint(0.0f, 0.0f, 1.0f);
  struct tuple p2 = createPoint(7.0f, 8.0f, 9.0f);
  genRotationMatrixY(M_PI / 4, rotMat);
  mat4x4MulTuple(rotMat, p1, &p2);
  assert(equal(p2.x, sqrt(2.0f) / 2.0f));
  assert(equal(p2.y,  0.0f));
  assert(equal(p2.z, sqrt(2.0f) / 2.0f));
  assert(equal(p2.w,  1.0f));

  genRotationMatrixY(M_PI / 2, rotMat);
  mat4x4MulTuple(rotMat, p1, &p2);
  assert(equal(p2.x, 1.0f));
  assert(equal(p2.y, 0.0f));
  assert(equal(p2.z, 0.0f));
  assert(equal(p2.w, 1.0f));
  return 1;
}

// 50 Rotating a point around the y axis
int genRotationMatrixZTest() {
  Mat4x4 rotMat;
  struct tuple p1 = createPoint(0.0f, 1.0f, 0.0f);
  struct tuple p2 = createPoint(7.0f, 8.0f, 9.0f);
  genRotationMatrixZ(M_PI / 4, rotMat);
  mat4x4MulTuple(rotMat, p1, &p2);
  assert(equal(p2.x, -sqrt(2.0f) / 2.0f));
  assert(equal(p2.y, sqrt(2.0f) / 2.0f));
  assert(equal(p2.z, 0.0f));
  assert(equal(p2.w, 1.0f));

  genRotationMatrixZ(M_PI / 2, rotMat);
  mat4x4MulTuple(rotMat, p1, &p2);
  assert(equal(p2.x, -1.0f));
  assert(equal(p2.y, 0.0f));
  assert(equal(p2.z, 0.0f));
  assert(equal(p2.w, 1.0f));
  return 1;
}

// 52 Shearing transformation moves x in proportion to y
int genShearMatrixTest() {
  Mat4x4 shearMat;
  genShearMatrix(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, shearMat);
  struct tuple p1 = createPoint(2.0f, 3.0f, 4.0f);
  struct tuple p2 = createPoint(7.0f, 8.0f, 9.0f);
  mat4x4MulTuple(shearMat, p1, &p2);
  assert(equal(p2.x, 5.0f));
  assert(equal(p2.y, 3.0f));
  assert(equal(p2.z, 4.0f));
  assert(equal(p2.w, 1.0f));

  // moves x in proportion to y
  genShearMatrix(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, shearMat);
  p2.x = 2.0f; p2.y = 3.0f; p2.z = 4.0f;
  mat4x4MulTuple(shearMat, p1, &p2);
  assert(equal(p2.x, 6.0f));
  assert(equal(p2.y, 3.0f));
  assert(equal(p2.z, 4.0f));
  assert(equal(p2.w, 1.0f));

  // moves y in proportion to x
  genShearMatrix(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, shearMat);
  p2.x = 2.0f; p2.y = 3.0f; p2.z = 4.0f;
  mat4x4MulTuple(shearMat, p1, &p2);
  assert(equal(p2.x, 2.0f));
  assert(equal(p2.y, 5.0f));
  assert(equal(p2.z, 4.0f));
  assert(equal(p2.w, 1.0f));

  // moves y in proportion to z
  genShearMatrix(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, shearMat);
  p2.x = 2.0f; p2.y = 3.0f; p2.z = 4.0f;
  mat4x4MulTuple(shearMat, p1, &p2);
  assert(equal(p2.x, 2.0f));
  assert(equal(p2.y, 7.0f));
  assert(equal(p2.z, 4.0f));
  assert(equal(p2.w, 1.0f));

  // moves x in proportion to x
  genShearMatrix(0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, shearMat);
  p2.x = 2.0f; p2.y = 3.0f; p2.z = 4.0f;
  mat4x4MulTuple(shearMat, p1, &p2);
  assert(equal(p2.x, 2.0f));
  assert(equal(p2.y, 3.0f));
  assert(equal(p2.z, 6.0f));
  assert(equal(p2.w, 1.0f));

  // moves x in proportion to x
  genShearMatrix(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, shearMat);
  p2.x = 2.0f; p2.y = 3.0f; p2.z = 4.0f;
  mat4x4MulTuple(shearMat, p1, &p2);
  assert(equal(p2.x, 2.0f));
  assert(equal(p2.y, 3.0f));
  assert(equal(p2.z, 7.0f));
  assert(equal(p2.w, 1.0f));
  return 1;
}

// 54 Individual transormations are applied in sequence
int transformationsAppliedInSequenceTest() {
  Mat4x4 rotMat;
  Mat4x4 scaleMat;
  Mat4x4 shearMat;
  genRotationMatrixX(M_PI / 2, rotMat);
  genScaleMatrix(5.0f, 5.0f, 5.0f, scaleMat);
  genTranslateMatrix(10.0f, 5.0f, 7.0f, shearMat);
  struct tuple p1 = createPoint(1.0f, 0.0f, 1.0f);
  struct tuple p2 = createPoint(1.0f, -1.0f, 0.0f);
  struct tuple p3 = createPoint(5.0f, -5.0f, 0.0f);
  struct tuple p4 = createPoint(15.0f, 0.0f, 7.0f);
  mat4x4MulTuple(rotMat, p1, &p2);
  assert(equal(p2.x, 1.0f));
  assert(equal(p2.y, -1.0f));
  assert(equal(p2.z, 0.0f));
  assert(equal(p2.w, 1.0f));
  mat4x4MulTuple(scaleMat, p2, &p3);
  assert(equal(p3.x, 5.0f));
  assert(equal(p3.y, -5.0f));
  assert(equal(p3.z, 0.0f));
  assert(equal(p3.w, 1.0f));
  mat4x4MulTuple(shearMat, p3, &p4);
  assert(equal(p4.x, 15.0f));
  assert(equal(p4.y, 0.0f));
  assert(equal(p4.z, 7.0f));
  assert(equal(p4.w, 1.0f));
  p1.x = 1.0f; p1.y = 0.0f; p1.z = 1.0f;
  mat4x4Mul(shearMat, scaleMat, scaleMat);
  mat4x4Mul(scaleMat, rotMat, rotMat);
  mat4x4MulTuple(rotMat, p1, &p2);
  assert(equal(p2.x, 15.0f));
  assert(equal(p2.y, 0.0f));
  assert(equal(p2.z, 7.0f));
  assert(equal(p2.w, 1.0f));
  return 1;
}

int drawClockTest() {
  double rotation = 2 * 3.14159 / 12;
  struct tuple twelve = createPoint(0, 0, 1);
  struct tuple three = createPoint(0, 0, 0);
  Mat4x4 rotMat;
  for (int i = 0; i < 12; ++i) {
    genRotationMatrixY(rotation * i, rotMat);
    mat4x4MulTuple(rotMat, twelve, &three);
    three.x = three.x * 40 + 50;  // 40 is radius of circle
                                  // 50 is center in x and z
    three.z = three.z * 40 + 50;
    canvas[(int)three.x][(int)three.z].x = 1.0f;
  }
  canvas[50][50].y = 1.0f;
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
  //unitTest("Write Pixel Test", writePixelTest());
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
  unitTest("3x3 Matrix Cofactor Test", mat3x3CofactorTest());
  unitTest("3x3 Matrix Determinant Test", mat3x3DetTest());
  unitTest("4x4 Matrix Determinant Test", mat4x4DetTest());
  unitTest("Invertable Matrix Test", invertableMatrixTest());
  unitTest("4x4 Matrix Invert Test", inverseMatrixTest());
  unitTest("Multiply Product By Its Inverse Test", MultProdByInverseTest());
  unitTest("Multiply By Translation Matrix Test", PointTransTest());
  unitTest("Multiply By Inverse Of Translation Matrix Test", pointMultInverseTranslationTest());
  unitTest("Vector Translation Has No Effect Test", vectorTranslationHasNoEffectTest());
  unitTest("Scaling Matrix Applied To A Point Test", pointScaleMat4x4Test());
  unitTest("Scaling Matrix Applied To A Vector Test", vecScaleMat4x4Test());
  unitTest("Multiply Inverse Of Scaling Matrix Test", multInverseScaleMatrixTest());
  unitTest("Generate Rotation Matrix X Test", genRotationMatrixXTest());
  unitTest("Generate  Rotation Matrix X Reverse Test()", genRotationMatrixReverseTest());
  unitTest("Generate Rotation Matrix Y Test", genRotationMatrixYTest());
  unitTest("Generate Rotation Matrix Z Test", genRotationMatrixZTest());
  unitTest("Generate Sheer Matrix Test", genShearMatrixTest());
  unitTest("Transformations Applied In Sequence Test", transformationsAppliedInSequenceTest());
  unitTest("Draw Clock Test", drawClockTest());
  unitTest("Write Canvas To File Test", writeCanvasToFile());
  return 0;
}