#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EPSILON 0.000001

#define HEIGHT 600
#define WIDTH  600

typedef double Mat2x2[2][2];
typedef double Mat3x3[3][3];
typedef double Mat4x4[4][4];

typedef struct { double x, y, z, w; } tuple;

typedef struct { tuple color; double ambient; double diffuse; double specular; double shininess; } material;

typedef struct { tuple position; tuple intensity; struct point_light* next; } point_light;

typedef struct _sphere { tuple location; double t; Mat4x4 transform; material material; struct _sphere* next; } sphere;

typedef struct { tuple originPoint; tuple directionVector; } ray;

typedef struct { double t; sphere *object_id; } intersection;

typedef struct { struct _sphere* objects; point_light* lights; } world;

#define INTERSECTIONS_SIZE 10

typedef struct { intersection itersection[10]; int count; } intersections;

material create_material(tuple color, double ambient, double diffuse, double specular, double shininess) {
    material m;
    m.color = color;
    m.ambient = ambient;
    m.diffuse = diffuse;
    m.specular = specular;
    m.shininess = shininess;
    return m;
}

material create_material_default() {
    material m;
    m.color.x = 1.0f; m.color.y = 1.0f; m.color.z = 1.0f; m.color.w = 0.0f;
    m.ambient = 0.1f;
    m.diffuse = 0.9f;
    m.specular = 0.9f;
    m.shininess = 200.0f;
    return m;
}

point_light create_point_light(tuple position, tuple intensity) {
    point_light pl;
    pl.position = position;
    pl.intensity = intensity;
    pl.next = NULL;
    return pl;
}

intersections create_intersections() {
    intersections intersects;
    intersects.count = 0;
    for (int i = 0; i < INTERSECTIONS_SIZE; ++i) {
        intersects.itersection[i].t = DBL_MIN;
        intersects.itersection[i].object_id = NULL;
    }
    return intersects;
}

void clear_intersections(intersections *intersection_list) {
    assert(intersection_list != NULL);
    intersection_list->count = 0;
    for (int i = 0; i < INTERSECTIONS_SIZE; ++i) {
        intersection_list->itersection[i].t = DBL_MIN;
        intersection_list->itersection[i].object_id = NULL;
    }
}

int intersect_compare(const intersection* a, const intersection* b) {
    if (a->t == b->t) {
        return 0;
    } else if (a->t < b->t) {
        return -1;
    }
    return 1;
}

void sort_intersects(intersections* intersects) {
    qsort(intersects, intersects->count, sizeof(intersection), intersect_compare);
}

intersection* hit(intersections *intersection_list) {
  assert(intersection_list != NULL);
  if (0 == intersection_list->count) return NULL;
  intersection* intersect1 = NULL;
  double t = DBL_MAX;
  for (int i = 0; i < INTERSECTIONS_SIZE; ++i) {
      if (intersection_list->itersection[i].t == DBL_MIN) { break; } // sentinal value
      if (intersection_list->itersection[i].t < 0) { continue; }
      if(intersection_list->itersection[i].t < t) {
          t = intersection_list->itersection[i].t;
          intersect1 = &intersection_list->itersection[i];
      }
  }
  return intersect1;
}

void add_intersection_to_list(intersections* intersection_list, double t, sphere *sp ) {
  assert(intersection_list != NULL && "Call to add insertion to list cannot contain null intersection list");
  int i = 0;
  intersection* temp_int = &intersection_list->itersection[0];
  while (i < INTERSECTIONS_SIZE-1 && temp_int->t && temp_int->t != DBL_MIN) {
      ++i;
      if (i >= INTERSECTIONS_SIZE) { assert("Not enough room in intersections list."); }
      temp_int = &intersection_list->itersection[i];
      
  }
  intersection_list->count++;
  intersection_list->itersection[i].object_id = sp;
  intersection_list->itersection[i].t = t;
}

tuple canvas[WIDTH][HEIGHT];

void write_pixel(int x, int y, tuple color) {
  canvas[x][y] = color;
}

// 5 Comparing floating point numbers
bool equal(double a, double b) {
  assert(!isnan(a)); // Indicates a problem before getting here.
  assert(!isnan(b));
  if (fabs(a - b) < EPSILON) return true;
  return false;
}

tuple create_point(double x, double y, double z) {
  tuple t = { x, y, z, 1.0f };
  return t;
}

tuple create_vector(double x, double y, double z) {
  tuple t = { x, y, z, 0.0f };
  return t;
}

ray create_ray(double originPoint_x, double originPoint_y, double originPoint_z, double dirVector_x, double dirVector_y, double dirVector_z) {
    ray r;
    r.originPoint.x = originPoint_x;
    r.originPoint.y = originPoint_y;
    r.originPoint.z = originPoint_z;
    r.originPoint.w = 1.0f;

    r.directionVector.x = dirVector_x;
    r.directionVector.y = dirVector_y;
    r.directionVector.z = dirVector_z;
    r.directionVector.w = 0.0f;
    return r;
}

bool tuple_is_point(tuple t) { return t.w == 1.0 ? true : false; }

bool tuple_is_vector(tuple t) { return t.w == 0.0 ? true : false; }

void tuple_copy(tuple *t1, tuple *t2) {
  t2->x = t1->x;
  t2->y = t1->y;
  t2->z = t1->z;
  t2->w = t1->w;
}

tuple tuple_add(tuple t1, tuple t2) {
  tuple t3 = { t1.x + t2.x, t1.y + t2.y, t1.z + t2.z, t1.w + t2.w };
  return t3;
}

tuple tuple_sub(tuple t1, tuple t2) {
  tuple t3 = { t1.x - t2.x, t1.y - t2.y, t1.z - t2.z};
  return t3;
}

tuple tuple_negate(tuple t) {
  tuple neg = { 0.0f, 0.0f, 0.0f, 0.0f };
  tuple ret = tuple_sub(neg, t);
  return ret;
}

tuple tuple_mult_scalar(tuple t, double s) {
  tuple ret = { t.x * s, t.y * s, t.z * s, t.w *s };
  return ret;
}

tuple tuple_div_scalar(tuple t, double s) {
  tuple ret = { t.x / s, t.y / s, t.z / s, t.w / s };
  return ret;
}

tuple tuple_mult_tuple(tuple t, tuple s) {
    tuple ret = { t.x * s.x, t.y * s.y, t.z * s.z, t.w * s.w };
    return ret;
}

double tuple_mag_vec(tuple t) {
  double magx = pow(t.x, 2);
  double magy = pow(t.y, 2);
  double magz = pow(t.z, 2);
  double mag = sqrt( magx + magy + magz);
  return mag;
}

tuple norm_vec(tuple t) {
  double mag = tuple_mag_vec(t);
  tuple ret = { t.x / mag, t.y / mag, t.z / mag};
  return ret;
}

double dot(tuple t1, tuple t2) {
  double prod1 = t1.x * t2.x;
  double prod2 = t1.y * t2.y;
  double prod3 = t1.z * t2.z;
  double prod4 = t1.w * t2.w;
  double dot = prod1 + prod2 + prod3 + prod4;
  return dot;
}

tuple cross(tuple a, tuple b) {
  double x = a.y * b.z - a.z * b.y;
  double y = a.z * b.x - a.x * b.z;
  double z = a.x * b.y - a.y * b.x;
  tuple cross = create_vector(x, y, z);
  return cross;
}

tuple hadamard_product(tuple c1, tuple c2) {
  tuple color = { c1.x * c2.x, c1.y * c2.y, c1.z * c2.z };
  return color;
}

void Mat4x4_copy(Mat4x4 m1, Mat4x4 m2) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            m2[i][j] = m1[i][j];
        }
    }
}

// TODO: Merge these three matrix methods together into one.
// TODO: Might should use the equal method.
bool mat2x2_equal(Mat2x2 m1, Mat2x2 m2) {
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j)
      if (m1[i][j] != m2[i][j]) return false;
  return true;
}

bool mat3x3_equal(double m1[][3], double m2[][3]) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (m1[i][j] != m2[i][j]) return false;
  return true;
}

bool mat4x4_equal(double m1[][4], double m2[][4]) {
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      if (!equal(m1[i][j], m2[i][j])) {
          return false;
      }
  return true;
}

void mat4x4_mul(const Mat4x4 a, const Mat4x4 b, Mat4x4 m) {
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

void mat4x4_mul_in_place(const Mat4x4 a, const Mat4x4 b, Mat4x4 m) {
    Mat4x4 orig;
    Mat4x4_copy(m, orig);
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            orig[row][col] =
                a[row][0] * b[0][col] +
                a[row][1] * b[1][col] +
                a[row][2] * b[2][col] +
                a[row][3] * b[3][col];
        }
    }
    Mat4x4_copy(orig, m);
}

void mat4x4_mul_tuple(const Mat4x4 a, const tuple b, tuple *c) {
    c->x = b.x * a[0][0] + b.y * a[0][1] + b.z * a[0][2] + b.w * a[0][3];
    c->y = b.x * a[1][0] + b.y * a[1][1] + b.z * a[1][2] + b.w * a[1][3];
    c->z = b.x * a[2][0] + b.y * a[2][1] + b.z * a[2][2] + b.w * a[2][3];
    c->w = b.x * a[3][0] + b.y * a[3][1] + b.z * a[3][2] + b.w * a[3][3];
}

void mat4x4_transpose(Mat4x4 a) {
  double temp;
  for (int i = 0; i < 4; ++i) {
    for (int j = i; j < 4; ++j) {
      temp = a[i][j];
      a[i][j] = a[j][i];
      a[j][i] = temp;
    }
  }
}

void print_tuple(tuple t) {
  printf("{ %.8f, %.8f, %.8f, %.8f }\n", t.x, t.y, t.z, t.w);
}

void print_mat(const int rows, const int cols, const double* mat) {
  printf("{ ");
  for (int i = 0; i < rows; ++i) {
    printf(" { ");
    for (int j = 0; j < cols; ++j) {
      printf("%.3f", mat[i * cols + j]);
      if (j < cols-1) { printf(", "); }
    }
    printf(" }");
    if (i < rows - 1) { printf(", "); }
    printf("\n");
  }
  printf("}\n\n");
}

double mat2x2_det(Mat2x2 a) {
  return a[0][0] * a[1][1] - a[0][1] * a[1][0];
}

double mat3x3_det(Mat3x3 m) {
  double a = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]);
  double b = m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]);
  double c = m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  double ans = a - b + c;
  return ans;
}

// TODO: Make these more generic
void mat3x3_submat2x2(Mat3x3 a, Mat2x2 b, int row, int col) {
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

void mat4x4_submat3x3(Mat4x4 a, Mat3x3 b, int row, int col) {
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

double mat3x3_minor(Mat3x3 a, int row, int col) {
  Mat2x2 b = { { 0.0f, 0.0f }, { 0.0f, 0.0f } };
  mat3x3_submat2x2(a, b, row, col);
  return mat2x2_det(b);
}

double mat4x4_minor(Mat4x4 a, int row, int col) {
  Mat3x3 b = { { 0.0f, 0.0f }, { 0.0f, 0.0f }, { 0.0f, 0.0f } };
  mat4x4_submat3x3(a, b, row, col);
  return mat3x3_det(b);
}

double mat3x3_cofactor(Mat3x3 a, int row, int col) {
  double minor = mat3x3_minor(a, row, col);
  if ((row + col) % 2 != 0) { minor *= -1; }
  return minor;
}

double mat4x4_cofactor(Mat4x4 a, int row, int col) {
  double minor = mat4x4_minor(a, row, col);
  if ((row + col) % 2 != 0) { minor *= -1; }
  return minor;
}

double mat4x4_det(Mat4x4 m, int size) {
  double detVal = 0.0f;
  for (int column = 0; column < size; ++column) {
    double mat3Cof = mat4x4_cofactor(m, 0, column);
    detVal = detVal + m[0][column] * mat3Cof;
  }
  return detVal;
}

bool invertable_matrix(Mat4x4 m) {
  if(equal(mat4x4_det(m, 4),0)) { return false; }
  return true;
}

bool mat4x4_inverse(Mat4x4 a, Mat4x4 b) {
  bool invert = invertable_matrix(a);
  if (!invert) { return false; }
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double c = mat4x4_cofactor(a, i, j);
      b[j][i] = c / mat4x4_det(a, 4);
    }
  }
  return true;
}

void Mat4x4_set_ident(Mat4x4 m) {
  m[0][0] = 1.0f; m[0][1] = 0.0f; m[0][2] = 0.0f; m[0][3] = 0.0f;
  m[1][0] = 0.0f; m[1][1] = 1.0f; m[1][2] = 0.0f; m[1][3] = 0.0f;
  m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = 1.0f; m[2][3] = 0.0f;
  m[3][0] = 0.0f; m[3][1] = 0.0f; m[3][2] = 0.0f; m[3][3] = 1.0f;
}

void gen_translate_matrix(const double x, const double y, const double z, Mat4x4 m) {
  m[0][0] = 1.0f; m[0][1] = 0.0f; m[0][2] = 0.0f; m[0][3] = x;
  m[1][0] = 0.0f; m[1][1] = 1.0f; m[1][2] = 0.0f; m[1][3] = y;
  m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = 1.0f; m[2][3] = z;
  m[3][0] = 0.0f; m[3][1] = 0.0f; m[3][2] = 0.0f; m[3][3] = 1.0f;
}

void gen_scale_matrix(const double x, const double y, const double z, Mat4x4 m) {
  m[0][0] = x;    m[0][1] = 0.0f; m[0][2] = 0.0f; m[0][3] = 0.0f;
  m[1][0] = 0.0f; m[1][1] = y;    m[1][2] = 0.0f; m[1][3] = 0.0f;
  m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = z;    m[2][3] = 0.0f;
  m[3][0] = 0.0f; m[3][1] = 0.0f; m[3][2] = 0.0f; m[3][3] = 1.0f;
}

void gen_rotate_matrix_X(const double rad, Mat4x4 m) {
  m[0][0] = 1.0f; m[0][1] = 0.0f;     m[0][2] = 0.0f;      m[0][3] = 0.0f;
  m[1][0] = 0.0f; m[1][1] = cos(rad); m[1][2] = -sin(rad); m[1][3] = 0.0f;
  m[2][0] = 0.0f; m[2][1] = sin(rad); m[2][2] = cos(rad);  m[2][3] = 0.0f;
  m[3][0] = 0.0f; m[3][1] = 0.0f;     m[3][2] = 0.0f;      m[3][3] = 1.0f;
}

void gen_rotate_matrix_Y(const double rad, Mat4x4 m) {
  m[0][0] = cos(rad);  m[0][1] = 0.0f; m[0][2] = sin(rad); m[0][3] = 0.0f;
  m[1][0] = 0.0f;      m[1][1] = 1.0f; m[1][2] = 0.0f;     m[1][3] = 0.0f;
  m[2][0] = -sin(rad); m[2][1] = 0.0f; m[2][2] = cos(rad); m[2][3] = 0.0f;
  m[3][0] = 0.0f;      m[3][1] = 0.0f; m[3][2] = 0.0f;     m[3][3] = 1.0f;
}

void gen_rotate_matrix_Z(const double rad, Mat4x4 m) {
  m[0][0] = cos(rad); m[0][1] = -sin(rad); m[0][2] = 0.0f; m[0][3] = 0.0f;
  m[1][0] = sin(rad); m[1][1] = cos(rad);  m[1][2] = 0.0f; m[1][3] = 0.0f;
  m[2][0] = 0.0f;     m[2][1] = 0.0f;      m[2][2] = 1.0f; m[2][3] = 0.0f;
  m[3][0] = 0.0f;     m[3][1] = 0.0f;      m[3][2] = 0.0f; m[3][3] = 1.0f;
}

void gen_shear_matrix(const double xy, const double xz, const double yx,\
  const double yz, const double zx, const double zy, Mat4x4 m) {
  m[0][0] = 1.0f; m[0][1] = xy;   m[0][2] = xz;   m[0][3] = 0.0f;
  m[1][0] = yx;   m[1][1] = 1.0f; m[1][2] = yz;   m[1][3] = 0.0f;
  m[2][0] = zx;   m[2][1] = zy;   m[2][2] = 1.0f; m[2][3] = 0.0f;
  m[3][0] = 0.0f; m[3][1] = 0.0f; m[3][2] = 0.0f; m[3][3] = 1.0f;
}

// TODO: Make these more generic
void mat2x2_reset_to_zero(Mat2x2 mat) {
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            mat[i][j] = 0.0f;
        }
    }
}

void mat3x3_reset_to_zero(Mat3x3 mat) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat[i][j] = 0.0f;
        }
    }
}

void mat4x4_reset_to_zero(Mat4x4 mat) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            mat[i][j] = 0.0f;
        }
    }
}

sphere* create_sphere() {
    sphere* s = (sphere*)malloc(sizeof(sphere));
    if (s) {
        s->t = 1.0f;
        s->location.x = 0.0f;
        s->location.y = 0.0f;
        s->location.z = 0.0f;
        s->location.w = 0.0f;
        Mat4x4_set_ident(s->transform);
        s->material = create_material_default();
        s->next = NULL;
    }
    return s;
}

tuple position(ray r, double t) {
    tuple pos = tuple_mult_scalar(r.directionVector, t); 
    pos = tuple_add(r.originPoint, pos);
    return pos;
}

ray transform(ray* r, Mat4x4 m) {
    ray ray_out = *r;
    mat4x4_mul_tuple(m, r->originPoint, &ray_out.originPoint);
    mat4x4_mul_tuple(m, r->directionVector, &ray_out.directionVector);
    return ray_out;
}

void intersect(sphere* sp, ray* r, intersections* intersects) {
    assert(sp != NULL);
    assert(r != NULL);
    Mat4x4 invScaleMat;
    Mat4x4_set_ident(invScaleMat);
    mat4x4_inverse(sp->transform, invScaleMat);
    ray r2 = transform(r, invScaleMat);

    tuple origin = create_point(0.0f, 0.0f, 0.0f);
    tuple sphere_to_ray = tuple_sub(r2.originPoint, origin);
    double a = dot(r2.directionVector, r2.directionVector);
    double b = 2 * dot(r2.directionVector, sphere_to_ray);
    double c = dot(sphere_to_ray, sphere_to_ray) - 1.0f;
    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) { return; }
    double t1 = (-b - sqrt(discriminant)) / (2 * a);
    double t2 = (-b + sqrt(discriminant)) / (2 * a);
    if (t1 < t2) {
        add_intersection_to_list(intersects, t1, sp);
        add_intersection_to_list(intersects, t2, sp);
    }
    else {
        add_intersection_to_list(intersects, t2, sp);
        add_intersection_to_list(intersects, t1, sp);
    }
    return;
}

void intersect_world(world* w, ray* r, intersections* intersects) {
    assert(w != NULL);
    assert(r != NULL);
    sphere* sp = w->objects;
    if (sp == NULL) { return; } // empty world
    do {
        intersect(sp, r, intersects);
        sp = sp->next;
    } while (sp != NULL);
    sort_intersects(intersects);
    return;
}

void set_transform(sphere *sp, Mat4x4 m) {
    Mat4x4_copy(m, sp->transform);
}

tuple normal_at(sphere* sphere, tuple world_point) {
    // Line 1
    tuple object_point = create_point(0.0f, 0.0f, 0.0f);
    Mat4x4 inverse_sphere;
    mat4x4_inverse(sphere->transform, inverse_sphere);
    mat4x4_mul_tuple(inverse_sphere, world_point, &object_point);

    // Line 2
    tuple temp_point = create_point(0.0f, 0.0f, 0.0f);
    tuple objectNormal = tuple_sub(object_point, temp_point);

    // Line 3
    tuple world_normal;
    // mat4x4Inverse(sphere->transform, inverse_sphere);
    mat4x4_transpose(inverse_sphere);
    mat4x4_mul_tuple(inverse_sphere, objectNormal, &world_normal);
    world_normal.w = 0.0f;

    // Line 4
    return norm_vec(world_normal);
}

tuple reflect(tuple in, tuple normal) {
    tuple normTwo = tuple_mult_scalar(normal, 2.0f);
    double dotNNormal = dot(in, normal);
    return tuple_sub(in, tuple_mult_scalar(normTwo, dotNNormal));
}

world create_world() {
    world w;
    w.lights = NULL;
    w.objects = NULL;
    return w;
}

world default_world() {
    world w;
    w.lights = NULL;
    w.objects = NULL;
    tuple light_pos = create_point(-10.0f, 10.0f, -10.0f);
    tuple light_intensity = create_point(1.0f, 1.0f, 1.0f);
    point_light light = create_point_light(light_pos, light_intensity);
    w.lights = &light;
    w.lights->next = NULL;
    sphere* s1 = create_sphere();
    tuple material_color = create_point(0.8f, 1.0f, 0.6f);
    material matl = create_material_default();
    matl.color = material_color;
    matl.diffuse = 0.7;
    matl.specular = 0.2;
    s1->material = matl;
    w.objects = s1;
    w.objects->next = NULL;

    sphere* s2 = create_sphere();
    gen_scale_matrix(0.5f, 0.5f, 0.5f, s2->transform);
    w.objects->next = s2;
 
    return w;
}

tuple lighting(material material, point_light light, tuple point, tuple eyev, tuple normalv) {
    tuple effective_color = tuple_mult_tuple(material.color, light.intensity);
    tuple diffuse;
    tuple specular;
    tuple ambient;

    tuple color_black = create_vector(0.0f, 0.0f, 0.0f);
    tuple light_sub_point = tuple_sub(light.position, point);
    tuple lightv = norm_vec(light_sub_point);

    ambient = tuple_mult_scalar(effective_color, material.ambient);

    double light_dot_normal = dot(lightv, normalv);

    if (light_dot_normal < 0) {
        diffuse = color_black;
        specular = color_black;
    }
    else {
        diffuse = tuple_mult_scalar( tuple_mult_scalar(effective_color, material.diffuse), light_dot_normal);

        tuple reflectv = reflect( tuple_negate(lightv), normalv);
        double reflect_dot_eye = dot(reflectv, eyev);

        if (reflect_dot_eye <= 0) {
            specular = color_black;
        }
        else {
            double factor = pow(reflect_dot_eye, material.shininess);
            specular = tuple_mult_scalar(tuple_mult_scalar(light.intensity, material.specular), factor);
        }
    }
    tuple light_out = tuple_add(tuple_add(ambient, specular),diffuse);
    return light_out;
}

int color_convert(double x) {
    int color = (int)(x * 255);
    if (color < 0) { color = 0; }
    if (color > 255) { color = 255; }
    return color;
}

#pragma warning(disable:4996)

int write_canvas_to_file() {
    FILE* fp;
    fp = fopen("canvas.ppm", "w");
    fprintf(fp, "P3\n");
    fprintf(fp, "%d %d\n255\n", WIDTH, HEIGHT);
    for (int i = 0; i < WIDTH; ++i) {
        for (int j = 0; j < HEIGHT; ++j) {
            int color = color_convert(canvas[i][j].x);
            fprintf(fp, "%d ", color);
            color = color_convert(canvas[i][j].y);
            fprintf(fp, "%d ", color);
            color = color_convert(canvas[i][j].z);
            fprintf(fp, "%d \n", color);
        }
    }
    return 1;
}

/*------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------*/

#if defined _DEBUG

void unit_test(char* msg, int assert) {
  size_t msg_length = strlen(msg);
  printf("%s", msg);

  /* 74 is 80 - length of "PASSED" */
  while (msg_length < 74) {
    putchar('.');
    msg_length++;
  }

  if (assert == 0) {
    printf("PASSED\n");
  } else {
    printf("FAILED\n");
    exit(1);
  }
}

// 4 creates tuples with w=1
int create_point_test() {
  tuple t = create_point(4.0f, -4.0f, 3.0f);
  assert(equal(t.x, 4.0f));
  assert(equal(t.y, -4.0f));
  assert(equal(t.z, 3.0f));
  assert(equal(t.w, 1.0f));
  return 0;
}

// 4 creates tuples with w=0
int create_vector_test() {
  tuple t = create_vector(4.0f, -4.0f, 3.0f);
  assert(equal(t.x, 4.0f));
  assert(equal(t.y, -4.0f));
  assert(equal(t.z, 3.0f));
  assert(equal(t.w, 0.0f));
  return 0;
}

// 4 A tuple with w=1.0 is a point
int tuple_with_W_0_is_point_test()
{
  tuple a = { 4.3f, -4.2f, 3.1f, 1.0f };
  assert(equal(a.x,  4.3f));
  assert(equal(a.y, -4.2f));
  assert(equal(a.z,  3.1f));
  assert(equal(a.w,  1.0f));
  assert(tuple_is_point(a)  == true);
  assert(tuple_is_vector(a) == false);

  tuple b = { 4.3f, -4.2f, 3.1f, 0.0f };
  assert(equal(b.x,  4.3f));
  assert(equal(b.y, -4.2f));
  assert(equal(b.z,  3.1f));
  assert(equal(b.w,  0.0f));
  assert(tuple_is_point(b)  == false);
  assert(tuple_is_vector(b) == true);
  return 0;
}

// 6 Adding two tuples
int tuple_add_test() {
  tuple a = { 3.0f, -2.0f, 5.0f, 1.0f };
  tuple b = { -2.0f, 3.0f, 1.0f, 0.0f };
  tuple c = tuple_add(a, b);
  assert(equal(c.x, 1.0f));
  assert(equal(c.y, 1.0f));
  assert(equal(c.z, 6.0f));
  assert(equal(c.w, 1.0f));
  return 0;
}

// 6 Subtracting two points
int tuple_sub_test() {
  tuple a = { 3.0f, 2.0f, 1.0f };
  tuple b = { 5.0f, 6.0f, 7.0f };
  tuple c = tuple_sub(a, b);
  assert(equal(c.x, -2.0f));
  assert(equal(c.y, -4.0f));
  assert(equal(c.z, -6.0f));
  return 0;
}

// 6 Subtracting vector from a point
int subtract_vector_from_point_test() {
  tuple pt = create_point(3.0f, 2.0f, 1.0f);
  tuple vec = create_vector(5.0f, 6.0f, 7.0f);
  tuple ans = tuple_sub(pt, vec);
  assert(equal(ans.x, -2.0f));
  assert(equal(ans.y, -4.0f));
  assert(equal(ans.z, -6.0f));
  return 0;
}

// 7 Subtracting two vectors
int subtract_two_vectors_test() {
  tuple vec1 = create_vector(3.0f, 2.0f, 1.0f);
  tuple vec2 = create_vector(5.0f, 6.0f, 7.0f);
  tuple vec3 = tuple_sub(vec1, vec2);
  assert(equal(vec3.x, -2.0f));
  assert(equal(vec3.y, -4.0f));
  assert(equal(vec3.z, -6.0f));
  return 0;
}

// 7 Subtracting a vector from zero vector
int subtract_vector_from_zero_vector_test() {
  tuple zero = create_vector(0.0f, 0.0f, 0.0f);
  tuple vec1 = create_vector(1.0f, -2.0f, 3.0f);
  tuple vec2 = tuple_sub(zero, vec1);
  assert(equal(vec2.x, -1.0f));
  assert(equal(vec2.y,  2.0f));
  assert(equal(vec2.z, -3.0f));
  return 0;
}

// 7 Negating a tuple
int negating_tuple_test() {
  tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  vec1 = tuple_negate(vec1);
  assert(equal(vec1.x, -1.0f));
  assert(equal(vec1.y,  2.0f));
  assert(equal(vec1.z, -3.0f));
  return 0;
}

// 8 Multiply tuple by a scalar
int tuple_mult_scalar_test() {
  tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  double scalar = 3.5f;
  vec1 = tuple_mult_scalar(vec1, scalar);
  assert(equal(vec1.x,   3.5f));
  assert(equal(vec1.y,  -7.0f));
  assert(equal(vec1.z,  10.5f));
  assert(equal(vec1.w, -14.0f));
  return 0;
}

// 8 Multiply tuple by a fraction
int tuple_mult_scalar_fraction_test() {
  tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  double scalar = 0.5f;
  vec1 = tuple_mult_scalar(vec1, scalar);
  assert(equal(vec1.x, 0.5f));
  assert(equal(vec1.y, -1.0f));
  assert(equal(vec1.z, 1.5f));
  assert(equal(vec1.w, -2.0f));
  return 0;
}

// 8 Divide a tuple by a scalar
int tuple_div_scalar_test() {
  tuple vec1 = { 1.0f, -2.0f, 3.0f, -4.0f };
  double scalar = 2.0f;
  vec1 = tuple_div_scalar(vec1, scalar);
  assert(equal(vec1.x, 0.5f));
  assert(equal(vec1.y, -1.0f));
  assert(equal(vec1.z, 1.5f));
  assert(equal(vec1.w, -2.0f));
  return 0;
}

// 8 Computing the magnitude of vector(1, 0, 0)
int tuple_mag_vec_test() {
  tuple vec1 = create_vector(1.0f, 0.0f, 0.0f);
  double mag = tuple_mag_vec(vec1);
  assert(equal(mag, 1.0f));

  tuple vec2 = create_vector(0.0f, 1.0f, 0.0f);
  mag = tuple_mag_vec(vec2);
  assert(equal(mag, 1.0f));

  tuple vec3 = create_vector(0.0f, 0.0f, 1.0f);
  mag = tuple_mag_vec(vec3);
  assert(equal(mag, 1.0f));

  tuple vec4 = create_vector(1.0f, 2.0f, 3.0f);
  mag = tuple_mag_vec(vec4);
  assert(equal(mag, sqrt(14.0f)));

  tuple vec5 = create_vector(-1.0f, -2.0f, -3.0f);
  mag = tuple_mag_vec(vec5);
  assert(equal(mag, sqrt(14.0f)));
  return 0;
}

// 10 Normalizing vector(4,0,0) gives (1,0,0)
int norm_vec_test() {
  tuple vec1 = create_vector(4.0f, 0.0f, 0.0f);
  tuple norm = norm_vec(vec1);
  assert(equal(norm.x, 1.0f));
  assert(equal(norm.y, 0.0f));
  assert(equal(norm.z, 0.0f));

  tuple vec2 = create_vector(1.0f, 2.0f, 3.0f);
  norm = norm_vec(vec2);
  double ans1 = 1 / sqrt(14);
  double ans2 = 2 / sqrt(14);
  double ans3 = 3 / sqrt(14);
  assert(equal(norm.x, ans1));
  assert(equal(norm.y, ans2));
  assert(equal(norm.z, ans3));

  tuple vec3 = create_vector(1.0f, 2.0f, 3.0f);
  norm = norm_vec(vec3);
  double mag = tuple_mag_vec(norm);
  assert(equal(mag, 1.0f));
  return 0;
}

// 10 dot rpoduct of two tuples
int dot_prod_test() {
  tuple vec1 = create_vector(1.0f, 2.0f, 3.0f);
  tuple vec2 = create_vector(2.0f, 3.0f, 4.0f);
  double dotProd = dot(vec1, vec2);
  assert(equal(dotProd, 20.0f));
  return 0;
}

// 11 cross product of two vectors
int cross_prod_test() {
  tuple vec1 = create_vector(1.0f, 2.0f, 3.0f);
  tuple vec2 = create_vector(2.0f, 3.0f, 4.0f);
  tuple cross1 = cross(vec1, vec2);
  assert(equal(cross1.x, -1.0f));
  assert(equal(cross1.y,  2.0f));
  assert(equal(cross1.z, -1.0f));
  tuple cross2 = cross(vec2, vec1);
  assert(equal(cross2.x,  1.0f));
  assert(equal(cross2.y, -2.0f));
  assert(equal(cross2.z,  1.0f));
  return 0;
}

// 18 Hadamard product
int hadamard_product_test() {
  tuple col1 = create_vector(1.0f, 0.2f, 0.4f);
  tuple col2 = create_vector(0.9f, 1.0f, 0.1f);
  tuple col3 = hadamard_product(col1, col2);
  assert(equal(col3.x, 0.899999976f));
  assert(equal(col3.y, 0.2f));
  assert(equal(col3.z, 0.04f));
  return 0;
}

int write_pixel_test() {
  tuple red = create_vector(1.0f, 0.0f, 0.0f);
  write_pixel(0, 0, red);

  // horizonatal axis
  tuple green = create_vector(0.0f, 1.0f, 0.0f);
  write_pixel(0, 1, green);

  tuple blue = create_vector(0.0f, 0.0f, 1.0f);
  write_pixel(0, 2, blue);

  // vertical axis
  tuple sky = create_vector(0.3f, 0.6f, 0.9f);
  write_pixel(1, 1, sky);

  tuple orange = create_vector(1.0f, 0.5f, 0.25f);
  write_pixel(1, 2, orange);

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
  return 0;
}

int color_convert_test() {
  int color = color_convert(0.0f);
  assert(color == 0);
  color = color_convert(0.5f);
  assert(color == 127);
  color = color_convert(1.0f);
  assert(color == 255);
  color = color_convert(2.0f);
  assert(color == 255);
  color = color_convert(-2.0f);
  assert(color == 0);
  return 0;
}

int mat_equal_test() {
  double oldValue;
  Mat2x2 mat2x2a = { { 0.0f, 1.0f }, { 2.0f, 3.0f } };
  Mat2x2 mat2x2b = { { 0.0f, 1.0f }, { 2.0f, 3.0f } };

  bool test1 = mat2x2_equal(mat2x2a, mat2x2b);
  assert(true == test1);

  // set each element of the first array different one at a time.
  // verify that method catches it
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      oldValue = mat2x2a[i][j];
      mat2x2a[i][j] = 9.0f;
      test1 = mat2x2_equal(mat2x2a, mat2x2b);
      assert(false == test1);
      mat2x2a[i][j] = oldValue;
    }
  }

  Mat3x3 mat3x3a = { { 0.0f, 1.0f, 2.0f }, { 3.0f, 4.0f, 5.0f }, { 6.0f, 7.0f, 8.0f } };
  Mat3x3 mat3x3b = { { 0.0f, 1.0f, 2.0f }, { 3.0f, 4.0f, 5.0f }, { 6.0f, 7.0f, 8.0f } };
  bool test2 = mat3x3_equal(mat3x3a, mat3x3b);
  assert(true == test2);

  // set each element of the first array different one at a time.
  // verify that method catches it
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      oldValue = mat3x3a[i][j];
      mat3x3a[i][j] = 9.0f;
      test2 = mat3x3_equal(mat3x3a, mat3x3b);
      assert(false == test2);
      mat3x3a[i][j] = oldValue;
    }
  }

  Mat4x4 mat4x4a = { { 0.0f, 1.0f, 2.0f }, { 3.0f, 4.0f, 5.0f },\
    { 6.0f, 7.0f, 8.0f }, { 9.0f, 10.0f, 11.0f } };
  Mat4x4 mat4x4b = { { 0.0f, 1.0f, 2.0f }, { 3.0f, 4.0f, 5.0f },\
    { 6.0f, 7.0f, 8.0f }, { 9.0f, 10.0f, 11.0f } };
  bool test3 = mat4x4_equal(mat4x4a, mat4x4b);
  assert(true == test3);

  // set each element of the first array different one at a time.
  // verify that method catches it
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      oldValue = mat4x4a[i][j];
      mat4x4a[i][j] = 12.0f;
      test3 = mat4x4_equal(mat4x4a, mat4x4b);
      assert(false == test3);
      mat4x4a[i][j] = oldValue;
    }
  }
  return 0;
}

int mat4x4_mul_test() {
  Mat4x4 a = { { 1.0f, 2.0f, 3.0f, 4.0f }, { 5.0f, 6.0f, 7.0f, 8.0f },\
    { 9.0f, 8.0f, 7.0f, 6.0f }, { 5.0f, 4.0f, 3.0f, 2.0f } };
  Mat4x4 b = { { -2.0f, 1.0f, 2.0f, 3.0f }, { 3.0f, 2.0f, 1.0f, -1.0f },\
    { 4.0f, 3.0f, 6.0f, 5.0f }, { 1.0f, 2.0f, 7.0f, 8.0f } };
  Mat4x4 m;
  mat4x4_mul(a, b, m);
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
  return 0;
}

int mat4x4_mul_in_place_test() {
    Mat4x4 a = { { 1.0f, 2.0f, 3.0f, 4.0f }, { 5.0f, 6.0f, 7.0f, 8.0f },\
      { 9.0f, 8.0f, 7.0f, 6.0f }, { 5.0f, 4.0f, 3.0f, 2.0f } };
    Mat4x4 b = { { -2.0f, 1.0f, 2.0f, 3.0f }, { 3.0f, 2.0f, 1.0f, -1.0f },\
      { 4.0f, 3.0f, 6.0f, 5.0f }, { 1.0f, 2.0f, 7.0f, 8.0f } };

    mat4x4_mul_in_place(a, b, b);

    assert(equal(a[0][0], 1.0f));
    assert(equal(a[0][1], 2.0f));
    assert(equal(a[0][2], 3.0f));
    assert(equal(a[0][3], 4.0f));
    assert(equal(a[1][0], 5.0f));
    assert(equal(a[1][1], 6.0f));
    assert(equal(a[1][2], 7.0f));
    assert(equal(a[1][3], 8.0f));
    assert(equal(a[2][0], 9.0f));
    assert(equal(a[2][1], 8.0f));
    assert(equal(a[2][2], 7.0f));
    assert(equal(a[2][3], 6.0f));
    assert(equal(a[3][0], 5.0f));
    assert(equal(a[3][1], 4.0f));
    assert(equal(a[3][2], 3.0f));
    assert(equal(a[3][3], 2.0f));

    assert(equal(b[0][0], 20.0f));
    assert(equal(b[0][1], 22.0f));
    assert(equal(b[0][2], 50.0f));
    assert(equal(b[0][3], 48.0f));
    assert(equal(b[1][0], 44.0f));
    assert(equal(b[1][1], 54.0f));
    assert(equal(b[1][2], 114.0f));
    assert(equal(b[1][3], 108.0f));
    assert(equal(b[2][0], 40.0f));
    assert(equal(b[2][1], 58.0f));
    assert(equal(b[2][2], 110.0f));
    assert(equal(b[2][3], 102.0f));
    assert(equal(b[3][0], 16.0f));
    assert(equal(b[3][1], 26.0f));
    assert(equal(b[3][2], 46.0f));
    assert(equal(b[3][3], 42.0f));
    return 0;
}

// 30 Matrix multipled by a tuple
int mat4x4_mul_tuple_test() {
  Mat4x4 a = { { 1.0f, 2.0f, 3.0f, 4.0f }, { 2.0f, 4.0f, 4.0f, 2.0f },\
    { 8.0f, 6.0f, 4.0f, 1.0f }, { 0.0f, 0.0f, 0.0f, 1.0f } };
  tuple b = create_point(1.0f, 2.0f, 3.0f);
  tuple c = create_point(0.0f, 0.0f, 0.0f);
  mat4x4_mul_tuple(a, b, &c);
  assert(equal(c.x, 18.0f));
  assert(equal(c.y, 24.0f));
  assert(equal(c.z, 33.0f));
  assert(equal(c.w,  1.0f));
  return 0;
}

// 32 Multiply matrix by identity matrix
int mat4x4_mult_ident_test() {
  Mat4x4 ident = { { 1.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 1.0f, 0.0f, 0.0f },\
    { 0.0f, 0.0f, 1.0f, 0.0f}, { 0.0f, 0.0f, 0.0f, 1.0f } };
  Mat4x4 a = { { 0.0f, 1.0f, 2.0f, 4.0f }, { 1.0f, 2.0f, 4.0f, 8.0f },\
    { 2.0f, 4.0f, 8.0f, 16.0f }, { 4.0f, 8.0f, 16.0f, 32.0f } };  
  Mat4x4 b = { { 0.0f, 1.0f, 2.0f, 4.0f }, { 1.0f, 2.0f, 4.0f, 8.0f },\
    { 2.0f, 4.0f, 8.0f, 16.0f }, { 4.0f, 8.0f, 16.0f, 32.0f } };
  mat4x4_mul(a, ident, a);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      assert(equal(a[i][j], b[i][j]));
    }
  }
  return 0;
}

// 33 Transpose a matrix
int mat4x4_transpose_test() {
  Mat4x4 a = { { 0.0f, 9.0f, 3.0f, 0.0f },{ 9.0f, 8.0f, 0.0f, 8.0f },\
    { 1.0f, 8.0f, 5.0f, 3.0f}, { 0.0f, 0.0f, 5.0f, 8.0f } };
  Mat4x4 b = { { 0.0f, 9.0f, 1.0f, 0.0f },{ 9.0f, 8.0f, 8.0f, 0.0f },\
    { 3.0f, 0.0f, 5.0f, 5.0f}, { 0.0f, 8.0f, 3.0f, 8.0f } };
  mat4x4_transpose(a);
  assert(mat4x4_equal(a, b));
  return 0;
}

// 34 Calculating the determinant of a 2x2 matrix
int mat2x2_det_test() {
  Mat2x2 a = { { 1.0f, 5.0f },{ -3.0f, 2.0f } };
  double det = mat2x2_det(a);
  assert(equal(det, 17.0f));

  Mat2x2 b = { { 5.0f, 0.0f },{ -1.0f, 5.0f } };
  det = mat2x2_det(b);
  assert(equal(det, 25.0f));

  mat2x2_reset_to_zero(b);
  det = mat2x2_det(b);
  assert(equal(det, 0.0f));

  Mat2x2 c = { { 1.0f, 0.0f },{ 0.0f, -1.0f } };
  det = mat2x2_det(c);
  assert(equal(det, -1.0f));

  Mat2x2 d = { { -1.0f, -1.0f },{ -1.0f, -1.0f } };
  det = mat2x2_det(d);
  assert(equal(det, 0.0f));

  Mat2x2 e = { { 1.0f, 2.0f },{ 3.0f, 4.0f } };
  det = mat2x2_det(e);
  assert(equal(det, -2.0f));
  return 0;
}

// 35 Submatrix of 3x3 matrix is a 2x2 matrix
int mat3x3_submat_2x2_test() {
  Mat3x3 a = { { 1.0f, 2.0f, 3.0f },{ 4.0f, 5.0f, 6.0f },{ 7.0f, 8.0f, 9.0f } };
  Mat2x2 b = { { 0.0f, 0.0f },{ 0.0f, 0.0f } };
  mat3x3_submat2x2(a, b, 0, 0);
  assert(equal(b[0][0], 5.0f));
  assert(equal(b[0][1], 6.0f));
  assert(equal(b[1][0], 8.0f));
  assert(equal(b[1][1], 9.0f));

  mat2x2_reset_to_zero(b);
  mat3x3_submat2x2(a, b, 0, 2);
  assert(equal(b[0][0], 4.0f));
  assert(equal(b[0][1], 5.0f));
  assert(equal(b[1][0], 7.0f));
  assert(equal(b[1][1], 8.0f));

  mat2x2_reset_to_zero(b);
  mat3x3_submat2x2(a, b, 1, 1);
  assert(equal(b[0][0], 1.0f));
  assert(equal(b[0][1], 3.0f));
  assert(equal(b[1][0], 7.0f));
  assert(equal(b[1][1], 9.0f));
  return 0;
}

// 35 Submatrix of 4x4 matrix is a 3x3 matrix
int mat4x4_submat_3x3_test() {
  Mat4x4 a = { { 1.0f, 2.0f, 3.0f, 4.0f },{ 5.0f, 6.0f, 7.0f, 8.0f },\
    { 9.0f, 10.0f, 11.0f, 12.0f},{ 13.0f, 14.0f, 15.0f, 16.0f } };
  Mat3x3 b = { { 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f } };

  mat4x4_submat3x3(a, b, 0, 0);
  assert(equal(b[0][0], 6.0f));
  assert(equal(b[0][1], 7.0f));
  assert(equal(b[0][2], 8.0f));
  assert(equal(b[1][0], 10.0f));
  assert(equal(b[1][1], 11.0f));
  assert(equal(b[1][2], 12.0f));
  assert(equal(b[2][0], 14.0f));
  assert(equal(b[2][1], 15.0f));
  assert(equal(b[2][2], 16.0f));

  mat3x3_reset_to_zero(b);
  mat4x4_submat3x3(a, b, 2, 1);
  assert(equal(b[0][0],  1.0f));
  assert(equal(b[0][1],  3.0f));
  assert(equal(b[0][2],  4.0f));
  assert(equal(b[1][0],  5.0f));
  assert(equal(b[1][1],  7.0f));
  assert(equal(b[1][2],  8.0f));
  assert(equal(b[2][0], 13.0f));
  assert(equal(b[2][1], 15.0f));
  assert(equal(b[2][2], 16.0f));

  mat3x3_reset_to_zero(b);
  mat4x4_submat3x3(a, b, 3, 3);
  assert(equal(b[0][0], 1.0f));
  assert(equal(b[0][1], 2.0f));
  assert(equal(b[0][2], 3.0f));
  assert(equal(b[1][0], 5.0f));
  assert(equal(b[1][1], 6.0f));
  assert(equal(b[1][2], 7.0f));
  assert(equal(b[2][0], 9.0f));
  assert(equal(b[2][1], 10.0f));
  assert(equal(b[2][2], 11.0f));
  return 0;
}

// 35 Calculating a minor of a 3x3 matrix
int mat3x3_minor_test() {
  Mat3x3 a = { { 3.0f, 5.0f, 0.0f },{ 2.0f, -1.0f, -7.0f },{ 6.0f, -1.0f, 5.0f } };
  double minor = mat3x3_minor(a, 1, 0);
  assert(equal(minor, 25.0f));
  return 0;
}

// 36 Calculating a cofactor of a 3x3 matrix
int mat3x3_cofactor_test() {
  Mat3x3 a = { { 3.0f, 5.0f, 0.0f },{ 2.0f, -1.0f, -7.0f },{ 6.0f, -1.0f, 5.0f } };
  double minor = mat3x3_minor(a, 0, 0);
  assert(equal(minor, -12.0f));

  double cofactor = mat3x3_cofactor(a, 0, 0);
  assert(equal(cofactor, -12.0f));

  minor = mat3x3_minor(a, 1, 0);
  assert(equal(minor, 25.0f));

  cofactor = mat3x3_cofactor(a, 1, 0);
  assert(equal(cofactor, -25.0f));
  return 0;
}

// 37 Calculating the determinant of a 3x3 matrix
int mat3x3_det_test() {
  Mat3x3 a = { { 1.0f, 2.0f, 6.0f },{ -5.0f, 8.0f, -4.0f },{ 2.0f, 6.0f, 4.0f } };
  double cofactor = mat3x3_cofactor(a, 0, 0);
  assert(equal(cofactor, 56.0f));

  cofactor = mat3x3_cofactor(a, 0, 1);
  assert(equal(cofactor, 12.0f));

  cofactor = mat3x3_cofactor(a, 0, 2);
  assert(equal(cofactor, -46.0f));

  double det = mat3x3_det(a);
  assert(equal(det, -196.0f));
  return 0;
}

// 37 Calculating the determinant of a 4x4 matrix
int mat4x4_det_test() {
  Mat4x4 a = { { -2.0f, -8.0f, 3.0f, 5.0f },{ -3.0f, 1.0f, 7.0f, 3.0f },\
    { 1.0f, 2.0f, -9.0f, 6.0f},{ -6.0f, 7.0f, 7.0f, -9.0f } };

  double cofactor = mat4x4_cofactor(a, 0, 0);
  assert(equal(cofactor, 690.0f));
  cofactor = mat4x4_cofactor(a, 0, 1);
  assert(equal(cofactor, 447.0f));
  cofactor = mat4x4_cofactor(a, 0, 2);
  assert(equal(cofactor, 210.0f));
  cofactor = mat4x4_cofactor(a, 0, 3);
  assert(equal(cofactor, 51.0f));
  double det = mat4x4_det(a, 4);
  assert(equal(det, -4071.0f));
  return 0;
}

// 39 Testing an invertable matrix for invertability
int invertable_matrix_test() {
  Mat4x4 a = { { 6.0f, 4.0f, 4.0f, 4.0f },{ 5.0f, 5.0f, 7.0f, 6.0f },\
    { 4.0f, -9.0f, 3.0f, -7.0f},{ 9.0f, 1.0f, 7.0f, -6.0f } };
  bool inv = invertable_matrix(a);
  assert(inv == true);

  Mat4x4 b = { { -4.0f, 2.0f, -2.0f, -3.0f },{ 9.0f, 6.0f, 2.0f, 6.0f },\
    { 0.0f, -5.0f, 1.0f, -5.0f},{ 0.0f, 0.0f, 0.0f, 0.0f } };
  inv = invertable_matrix(b);
  assert(inv == false);
  return 0;
}

// 39 Calculating the inverse of a matrix
int inverse_matrix_test() {
  Mat4x4 a = { { -5.0f, 2.0f, 6.0f, -8.0f },{ 1.0f, -5.0f, 1.0f, 8.0f },\
      { 7.0f, 7.0f, -6.0f, -7.0f},{ 1.0f, -3.0f, 7.0f, 4.0f } };
  Mat4x4 b = { { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f },\
    { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f } };
  Mat4x4 c = { { 0.21804512f, 0.45112783f, 0.24060151f, -0.04511278f },{ -0.80827069f, -1.45676696f, -0.44360903f, 0.52067667f },\
    { -0.07894737f, -0.22368421f, -0.05263158f, 0.19736843f},{ -0.52255636f, -0.81390977f, -0.30075186f, 0.30639097f } };

  bool inversable = mat4x4_inverse(a, b);
  assert(inversable == true);
  double det = mat4x4_det(a, 4);
  assert(equal(det, 532.0f));
  double cof = mat4x4_cofactor(a, 2, 3);
  assert(equal(cof, -160.0f));
  assert(equal(b[3][2], -160.0f/532.0f));
  cof = mat4x4_cofactor(a, 3, 2);
  assert(equal(cof, 105.0f));
  assert(equal(b[2][3], 105.0f/532.0f));
  assert(mat4x4_equal(b, c) == true);

  Mat4x4 d = { { 8.0f, -5.0f, 9.0f, 2.0f },{ 7.0f, 5.0f, 6.0f, 1.0f },\
        { -6.0f, 0.0f, 9.0f, 6.0f},{ -3.0f, 0.0f, -9.0f, -4.0f } };
  Mat4x4 e = { { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f },\
    { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f } };
  Mat4x4 f = { { -0.15384616f, -0.15384616f, -0.28205130f, -0.53846157f },{ -0.07692308f, 0.12307692f, 0.02564103f, 0.03076923f },\
      { 0.35897437f, 0.35897437f, 0.43589744f, 0.92307693f},{ -0.69230771f, -0.69230771f, -0.76923078f, -1.92307687f } };

  inversable = mat4x4_inverse(d, e);
  assert(inversable == true);
  assert(mat4x4_equal(e, f) == true);

  Mat4x4 g = { { 9.0f, 3.0f, 0.0f, 9.0f },{ -5.0f, -2.0f, -6.0f, -3.0f },\
        { -4.0f, 9.0f, 6.0f, 4.0f},{ -7.0f, 6.0f, 6.0f, 2.0f } };
  Mat4x4 h = { { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f },\
    { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f } };

  inversable = mat4x4_inverse(g, h);
  assert(inversable == true);
  assert(mat4x4_equal(e, f) == true);
  return 0;
}

// 41 Multiply product by its inverse
int mult_prod_by_inverse_test() {
  Mat4x4 a = { { 3.0f, -9.0f, 7.0f, 3.0f },{ 3.0f, -8.0f, 2.0f, -9.0f },\
      { -4.0f, 4.0f, 4.0f, 1.0f},{ -6.0f, 5.0f, -1.0f, 1.0f } };
  Mat4x4 b = { { 8.0f, 2.0f, 2.0f, 2.0f },{ 3.0f, -1.0f, 7.0f, 0.0f },\
    { 7.0f, 0.0f, 5.0f, 4.0f },{ 6.0f, -2.0f, 0.0f, 5.0f } };
  Mat4x4 c = { { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f },\
    { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f } };

  mat4x4_mul(a, b, c);
  Mat4x4 t;
  bool inversable = mat4x4_inverse(b, t);
  Mat4x4 u;
  mat4x4_mul(c, t, u);
  assert(inversable == true);
  assert(equal(u[0][0], a[0][0]));
  assert(equal(u[0][1], a[0][1]));
  assert(equal(u[0][2], a[0][2]));
  assert(equal(u[0][3], a[0][3]));
  return 0;
}

// 45 Multiply by a translation matrix
int point_trans_test() {
  tuple point1 = create_point(-3.0f, 4.0f, 5.0f);
  tuple point2 = create_point( 0.0f, 0.0f, 0.0f);
  Mat4x4 trans;
  gen_translate_matrix(5.0f, -3.0f, 2.0f, trans);
  mat4x4_mul_tuple(trans, point1, &point2);
  assert(equal(point2.x, 2.0f));
  assert(equal(point2.y, 1.0f));
  assert(equal(point2.z, 7.0f));
  assert(equal(point2.w, 1.0f));
  return 0;
}

// 45 Multiply by the inverse of a traslation matrix
int point_mult_inverse_translation_test() {
  Mat4x4 trans;
  Mat4x4 transInverse;
  gen_translate_matrix(5.0f, -3.0f, 2.0f, trans);
  mat4x4_inverse(trans, transInverse);
  tuple p1 = create_point(-3.0f, 4.0f, 5.0f);
  tuple p2 = create_point(0.0f, 0.0f, 0.0f);
  mat4x4_mul_tuple(transInverse, p1, &p2);
  assert(equal(p2.x, -8.0f));
  assert(equal(p2.y,  7.0f));
  assert(equal(p2.z,  3.0f));
  assert(equal(p2.w,  1.0f));
  return 0;
}

// 45 Translation does not affect vectors
int vector_translation_has_no_effect_test() {
  Mat4x4 trans;
  gen_translate_matrix(5.0f, -3.0f, 2.0f, trans);
  tuple v1 = create_vector(-3.0f, 4.0f, 5.0f);
  tuple v2 = create_point(0.0f, 0.0f, 0.0f);
  mat4x4_mul_tuple(trans, v1, &v2);
  assert(equal(v2.x, -3.0f));
  assert(equal(v2.y,  4.0f));
  assert(equal(v2.z,  5.0f));
  assert(equal(v2.w,  0.0f));
  return 0;
}

// 46 Scaling matrix applied to a point
int point_scale_Mat4x4_test() {
  tuple p1 = create_point(-4.0f, 6.0f, 8.0f);
  tuple p2 = create_point(0.0f, 0.0f, 0.0f);
  Mat4x4 scaleMat;
  gen_scale_matrix(2.0f, 3.0f, 4.0f, scaleMat);
  mat4x4_mul_tuple(scaleMat, p1, &p2);
  assert(equal(p2.x, -8.0f));
  assert(equal(p2.y, 18.0f));
  assert(equal(p2.z, 32.0f));
  assert(equal(p2.w,  1.0f));
  return 0;
}

// 46 Scaling matrix applied to a vector
int vec_scale_Mat4x4_test() {
  tuple p1 = create_vector(-4.0f, 6.0f, 8.0f);
  tuple p2 = create_vector(0.0f, 0.0f, 0.0f);
  Mat4x4 scaleMat;
  gen_scale_matrix(2.0f, 3.0f, 4.0f, scaleMat);
  mat4x4_mul_tuple(scaleMat, p1, &p2);
  assert(equal(p2.x, -8.0f));
  assert(equal(p2.y, 18.0f));
  assert(equal(p2.z, 32.0f));
  assert(equal(p2.w,  0.0f));
  return 0;
}

// 46 Multiply inverse of scaling matrix
int mult_inverse_scale_matrix_test() {
  Mat4x4 scaleMat;
  Mat4x4 scaleMatInv;
  tuple p1 = create_vector(-4.0f, 6.0f, 8.0f);
  tuple p2 = create_vector(0.0f, 0.0f, 0.0f);
  gen_scale_matrix(2.0f, 3.0f, 4.0f, scaleMat);
  mat4x4_inverse(scaleMat, scaleMatInv);
  mat4x4_mul_tuple(scaleMatInv, p1, &p2);
  assert(equal(p2.x, -2.0f));
  assert(equal(p2.y,  2.0f));
  assert(equal(p2.z,  2.0f));
  assert(equal(p2.w,  0.0f));
  return 0;
}

// 47 Reflection is scaling by a negative value
int reflection_scaling_neg_value_test() {
    Mat4x4 scaleMat;
    gen_scale_matrix(-1.0f, 1.0f, 1.0f, scaleMat);
    tuple p1 = create_point(2.0f, 3.0f, 4.0f);
    tuple p2 = create_point(0.0f, 0.0f, 0.0f);
    mat4x4_mul_tuple(scaleMat, p1, &p2);
    assert(equal(p2.x, -2.0f));
    assert(equal(p2.y, 3.0f));
    assert(equal(p2.z, 4.0f));
    return 0;
}

// 48 Rotating a point around the x axis
int gen_rotation_matrix_X_test() {
  Mat4x4 rotMat;
  tuple p1 = create_point(0.0f, 1.0f, 0.0f);
  tuple p2 = create_point(7.0f, 8.0f, 9.0f);
  gen_rotate_matrix_X(M_PI / 4, rotMat);
  mat4x4_mul_tuple(rotMat, p1, &p2);
  assert(equal(p2.x, 0.0f));
  assert(equal(p2.y, sqrt(2.0f)/2.0f));
  assert(equal(p2.z, sqrt(2.0f) / 2.0f));
  assert(equal(p2.w, 1.0f));

  gen_rotate_matrix_X(M_PI / 2, rotMat);
  mat4x4_mul_tuple(rotMat, p1, &p2);
  assert(equal(p2.x, 0.0f));
  assert(equal(p2.y, 0.0f));
  assert(equal(p2.z, 1.0f));
  assert(equal(p2.w, 1.0f));
  return 0;
}

// 49 Inverse of an x rotation rotates the opposite direction
int gen_rotation_matrix_reverse_test() {
  Mat4x4 rotMat;
  Mat4x4 rotMatInv;
  tuple p1 = create_point(0.0f, 1.0f, 0.0f);
  tuple p2 = create_point(7.0f, 8.0f, 9.0f);
  gen_rotate_matrix_X(M_PI / 4, rotMat);
  mat4x4_inverse(rotMat, rotMatInv);
  mat4x4_mul_tuple(rotMatInv, p1, &p2);
  assert(equal(p2.x, 0.0f));
  assert(equal(p2.y, sqrt(2.0f) / 2.0f));
  assert(equal(p2.z, -sqrt(2.0f) / 2.0f));
  assert(equal(p2.w, 1.0f));
  return 0;
}

// 50 Rotating a point around the y axis
int gen_rotation_matrix_Y_test() {
  Mat4x4 rotMat;
  tuple p1 = create_point(0.0f, 0.0f, 1.0f);
  tuple p2 = create_point(7.0f, 8.0f, 9.0f);
  gen_rotate_matrix_Y(M_PI / 4, rotMat);
  mat4x4_mul_tuple(rotMat, p1, &p2);
  assert(equal(p2.x, sqrt(2.0f) / 2.0f));
  assert(equal(p2.y,  0.0f));
  assert(equal(p2.z, sqrt(2.0f) / 2.0f));
  assert(equal(p2.w,  1.0f));

  gen_rotate_matrix_Y(M_PI / 2, rotMat);
  mat4x4_mul_tuple(rotMat, p1, &p2);
  assert(equal(p2.x, 1.0f));
  assert(equal(p2.y, 0.0f));
  assert(equal(p2.z, 0.0f));
  assert(equal(p2.w, 1.0f));
  return 0;
}

// 50 Rotating a point around the y axis
int gen_rotation_matrix_Z_test() {
  Mat4x4 rotMat;
  tuple p1 = create_point(0.0f, 1.0f, 0.0f);
  tuple p2 = create_point(7.0f, 8.0f, 9.0f);
  gen_rotate_matrix_Z(M_PI / 4, rotMat);
  mat4x4_mul_tuple(rotMat, p1, &p2);
  assert(equal(p2.x, -sqrt(2.0f) / 2.0f));
  assert(equal(p2.y, sqrt(2.0f) / 2.0f));
  assert(equal(p2.z, 0.0f));
  assert(equal(p2.w, 1.0f));

  gen_rotate_matrix_Z(M_PI / 2, rotMat);
  mat4x4_mul_tuple(rotMat, p1, &p2);
  assert(equal(p2.x, -1.0f));
  assert(equal(p2.y, 0.0f));
  assert(equal(p2.z, 0.0f));
  assert(equal(p2.w, 1.0f));
  return 0;
}

// 52 Shearing transformation moves x in proportion to y
int gen_shear_matrix_test() {
  Mat4x4 shearMat;
  gen_shear_matrix(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, shearMat);
  tuple p1 = create_point(2.0f, 3.0f, 4.0f);
  tuple p2 = create_point(7.0f, 8.0f, 9.0f);
  mat4x4_mul_tuple(shearMat, p1, &p2);
  assert(equal(p2.x, 5.0f));
  assert(equal(p2.y, 3.0f));
  assert(equal(p2.z, 4.0f));
  assert(equal(p2.w, 1.0f));

  // moves x in proportion to y
  gen_shear_matrix(0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, shearMat);
  p2.x = 2.0f; p2.y = 3.0f; p2.z = 4.0f;
  mat4x4_mul_tuple(shearMat, p1, &p2);
  assert(equal(p2.x, 6.0f));
  assert(equal(p2.y, 3.0f));
  assert(equal(p2.z, 4.0f));
  assert(equal(p2.w, 1.0f));

  // moves y in proportion to x
  gen_shear_matrix(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, shearMat);
  p2.x = 2.0f; p2.y = 3.0f; p2.z = 4.0f;
  mat4x4_mul_tuple(shearMat, p1, &p2);
  assert(equal(p2.x, 2.0f));
  assert(equal(p2.y, 5.0f));
  assert(equal(p2.z, 4.0f));
  assert(equal(p2.w, 1.0f));

  // moves y in proportion to z
  gen_shear_matrix(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, shearMat);
  p2.x = 2.0f; p2.y = 3.0f; p2.z = 4.0f;
  mat4x4_mul_tuple(shearMat, p1, &p2);
  assert(equal(p2.x, 2.0f));
  assert(equal(p2.y, 7.0f));
  assert(equal(p2.z, 4.0f));
  assert(equal(p2.w, 1.0f));

  // moves x in proportion to x
  gen_shear_matrix(0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, shearMat);
  p2.x = 2.0f; p2.y = 3.0f; p2.z = 4.0f;
  mat4x4_mul_tuple(shearMat, p1, &p2);
  assert(equal(p2.x, 2.0f));
  assert(equal(p2.y, 3.0f));
  assert(equal(p2.z, 6.0f));
  assert(equal(p2.w, 1.0f));

  // moves x in proportion to x
  gen_shear_matrix(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, shearMat);
  p2.x = 2.0f; p2.y = 3.0f; p2.z = 4.0f;
  mat4x4_mul_tuple(shearMat, p1, &p2);
  assert(equal(p2.x, 2.0f));
  assert(equal(p2.y, 3.0f));
  assert(equal(p2.z, 7.0f));
  assert(equal(p2.w, 1.0f));
  return 0;
}

// 54 Individual transormations are applied in sequence
int transform_applied_in_sequence_test() {
  Mat4x4 rotMat;
  Mat4x4 scaleMat;
  Mat4x4 shearMat;
  gen_rotate_matrix_X(M_PI / 2, rotMat);
  gen_scale_matrix(5.0f, 5.0f, 5.0f, scaleMat);
  gen_translate_matrix(10.0f, 5.0f, 7.0f, shearMat);
  tuple p1 = create_point(1.0f, 0.0f, 1.0f);
  tuple p2 = create_point(1.0f, -1.0f, 0.0f);
  tuple p3 = create_point(5.0f, -5.0f, 0.0f);
  tuple p4 = create_point(15.0f, 0.0f, 7.0f);
  mat4x4_mul_tuple(rotMat, p1, &p2);
  assert(equal(p2.x, 1.0f));
  assert(equal(p2.y, -1.0f));
  assert(equal(p2.z, 0.0f));
  assert(equal(p2.w, 1.0f));
  mat4x4_mul_tuple(scaleMat, p2, &p3);
  assert(equal(p3.x, 5.0f));
  assert(equal(p3.y, -5.0f));
  assert(equal(p3.z, 0.0f));
  assert(equal(p3.w, 1.0f));
  mat4x4_mul_tuple(shearMat, p3, &p4);
  assert(equal(p4.x, 15.0f));
  assert(equal(p4.y, 0.0f));
  assert(equal(p4.z, 7.0f));
  assert(equal(p4.w, 1.0f));
  p1.x = 1.0f; p1.y = 0.0f; p1.z = 1.0f;
  mat4x4_mul(shearMat, scaleMat, scaleMat);
  mat4x4_mul(scaleMat, rotMat, rotMat);
  mat4x4_mul_tuple(rotMat, p1, &p2);
  assert(equal(p2.x, 15.0f));
  assert(equal(p2.y, 0.0f));
  assert(equal(p2.z, 7.0f));
  assert(equal(p2.w, 1.0f));
  return 0;
}

// 55
int draw_clock_test() {
  double rotation = 2 * 3.14159 / 12;
  tuple twelve = create_point(0, 0, 1);
  tuple three = create_point(0, 0, 0);
  Mat4x4 rotMat;
  for (int i = 0; i < 12; ++i) {
    gen_rotate_matrix_Y(rotation * i, rotMat);
    mat4x4_mul_tuple(rotMat, twelve, &three);
    three.x = three.x * 40 + 50;  // 40 is radius of circle
                                  // 50 is center in x and z
    three.z = three.z * 40 + 50;
    canvas[(int)three.x][(int)three.z].x = 1.0f;
  }
  canvas[50][50].y = 1.0f;
  return 0;
}

int tuple_copy_test() {
  tuple t1 = { 1.0f, 2.0f, 3.0f, 4.0f };
  tuple t2 = { 0.0f, 0.0f, 0.0f, 0.0f };
  tuple_copy(&t1, &t2);
  assert(equal(t1.x, 1.0f));
  assert(equal(t1.y, 2.0f));
  assert(equal(t1.z, 3.0f));
  assert(equal(t1.w, 4.0f));

  assert(equal(t2.x, 1.0f));
  assert(equal(t2.y, 2.0f));
  assert(equal(t2.z, 3.0f));
  assert(equal(t2.w, 4.0f));

  assert(equal(t1.x, t2.x));
  assert(equal(t1.y, t2.y));
  assert(equal(t1.z, t2.z));
  assert(equal(t1.w, t2.w));
  return 0;
}

int Mat4x4_copy_test() {
  Mat4x4 a = { { 1.0f, 2.0f, 3.0f, 4.0f },{ 5.0f, 6.0f, 7.0f, 8.0f },\
    { 9.0f, 10.0f, 11.0f, 12.0f},{ 13.0f, 14.0f, 15.0f, 16.0f } };
  Mat4x4 b = { { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f },\
    { 0.0f, 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f, 0.0f } };
  Mat4x4_copy(a, b);
  assert(equal(a[0][0], 1.0f));
  assert(equal(a[1][0], 5.0f));
  assert(equal(a[2][0], 9.0f));
  assert(equal(a[3][0], 13.0f));

  assert(equal(a[0][1], 2.0f));
  assert(equal(a[1][1], 6.0f));
  assert(equal(a[2][1], 10.0f));
  assert(equal(a[3][1], 14.0f));

  assert(equal(a[0][2], 3.0f));
  assert(equal(a[1][2], 7.0f));
  assert(equal(a[2][2], 11.0f));
  assert(equal(a[3][2], 15.0f));

  assert(equal(a[0][3], 4.0f));
  assert(equal(a[1][3], 8.0f));
  assert(equal(a[2][3], 12.0f));
  assert(equal(a[3][3], 16.0f));

  assert(equal(b[0][0], 1.0f));
  assert(equal(b[1][0], 5.0f));
  assert(equal(b[2][0], 9.0f));
  assert(equal(b[3][0], 13.0f));

  assert(equal(b[0][1], 2.0f));
  assert(equal(b[1][1], 6.0f));
  assert(equal(b[2][1], 10.0f));
  assert(equal(b[3][1], 14.0f));

  assert(equal(b[0][2], 3.0f));
  assert(equal(b[1][2], 7.0f));
  assert(equal(b[2][2], 11.0f));
  assert(equal(b[3][2], 15.0f));

  assert(equal(b[0][3], 4.0f));
  assert(equal(b[1][3], 8.0f));
  assert(equal(b[2][3], 12.0f));
  assert(equal(b[3][3], 16.0f));
  return 0;
}

// 58 Creating and quering a ray
int create_ray_test() {
  ray r = create_ray(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f);

  assert(equal(r.originPoint.x, 1.0f));
  assert(equal(r.originPoint.y, 2.0f));
  assert(equal(r.originPoint.z, 3.0f));
  assert(equal(r.originPoint.w, 1.0f));

  assert(equal(r.directionVector.x, 4.0f));
  assert(equal(r.directionVector.y, 5.0f));
  assert(equal(r.directionVector.z, 6.0f));
  assert(equal(r.directionVector.w, 0.0f));

  return 0;
}

int create_sphere_test() {
    sphere* s = create_sphere();

    assert(s->next == NULL);
    assert(equal(s->t, 1.0f));

    assert(equal(s->location.x, 0.0f));
    assert(equal(s->location.y, 0.0f));
    assert(equal(s->location.z, 0.0f));
    assert(equal(s->location.w, 0.0f));

    assert(equal(s->transform[0][0], 1.0f));
    assert(equal(s->transform[1][0], 0.0f));
    assert(equal(s->transform[2][0], 0.0f));
    assert(equal(s->transform[3][0], 0.0f));

    assert(equal(s->transform[0][1], 0.0f));
    assert(equal(s->transform[1][1], 1.0f));
    assert(equal(s->transform[2][1], 0.0f));
    assert(equal(s->transform[3][1], 0.0f));

    assert(equal(s->transform[0][2], 0.0f));
    assert(equal(s->transform[1][2], 0.0f));
    assert(equal(s->transform[2][2], 1.0f));
    assert(equal(s->transform[3][2], 0.0f));

    assert(equal(s->transform[0][3], 0.0f));
    assert(equal(s->transform[1][3], 0.0f));
    assert(equal(s->transform[2][3], 0.0f));
    assert(equal(s->transform[3][3], 1.0f));

    assert(equal(s->material.color.x, 1.0f));
    assert(equal(s->material.color.y, 1.0f));
    assert(equal(s->material.color.z, 1.0f));
    assert(equal(s->material.color.w, 0.0f));

    assert(equal(s->material.ambient, 0.1f));
    assert(equal(s->material.diffuse, 0.9f));
    assert(equal(s->material.specular, 0.9f));
    assert(equal(s->material.shininess, 200.0f));
    return 0;
}

int create_intersections_test() {
    intersections intersects  = create_intersections();

    assert(intersects.count == 0);
    for (int i = 0; i < INTERSECTIONS_SIZE; ++i) {
        assert(equal(intersects.itersection[i].t, 0.0f));
        assert(intersects.itersection[i].object_id == NULL);
    }
    return 0;
}

// 58 Computing a point from a distance
int position_test() {
    ray r = create_ray(2.0f, 3.0f, 4.0f, 1.0f, 0.0f, 0.0f);
    tuple p1 = position(r, 0.0f);
    assert(equal(p1.x, 2.0f));
    assert(equal(p1.y, 3.0f));
    assert(equal(p1.z, 4.0f));
    assert(equal(p1.w, 1.0f));

    tuple p2 = position(r, 1.0f);
    assert(equal(p2.x, 3.0f));
    assert(equal(p2.y, 3.0f));
    assert(equal(p2.z, 4.0f));
    assert(equal(p2.w, 1.0f));

    tuple p3 = position(r, -1.0f);
    assert(equal(p3.x, 1.0f));
    assert(equal(p3.y, 3.0f));
    assert(equal(p3.z, 4.0f));
    assert(equal(p3.w, 1.0f));

    tuple p4 = position(r, 2.5f);
    assert(equal(p4.x, 4.5f));
    assert(equal(p4.y, 3.0f));
    assert(equal(p4.z, 4.0f));
    assert(equal(p4.w, 1.0f));

    return 0;
}

// 59 A ray intersects a sphere at two points
int ray_intersect_sphere_two_point_test() {
    ray r = create_ray(0.0f, 0.0f, -5.0f, 0.0f, 0.0f, 1.0f);
    sphere* s = create_sphere();
    intersections inter = create_intersections();
    intersect(s, &r, &inter);
    assert(inter.count == 2);
    assert(equal(inter.itersection[0].t, 4.0f));
    assert(equal(inter.itersection[1].t, 6.0f));
    return 0;
}

// 60 A ray intersects a sphere at a tangent
int ray_intersect_sphere_tangent_test() {
    ray r = create_ray(0.0f, 1.0f, -5.0f, 0.0f, 0.0f, 1.0f);
    sphere* s = create_sphere();
    intersections inter = create_intersections();
    intersect(s, &r, &inter);
    assert(inter.count == 2);
    assert(equal(inter.itersection[0].t, 5.0f));
    assert(equal(inter.itersection[1].t, 5.0f));
    return 0;
}

// 60 A ray misses a sphere
int ray_misses_sphere_test() {
    ray r = create_ray(0.0f, 2.0f, -5.0f, 0.0f, 0.0f, 1.0f);
    sphere* s = create_sphere();
    intersections inter = create_intersections();
    intersect(s, &r, &inter);
    assert(inter.count == 0);
    assert(equal(inter.itersection[0].t, 0.0f)); // might as well check
    assert(equal(inter.itersection[1].t, 0.0f));

    assert(inter.itersection[0].object_id == NULL);
    assert(inter.itersection[1].object_id == NULL);

    return 0;
}

// 61 A ray originates inside a sphere
int ray_originates_inside_sphere_test() {
    ray r = create_ray(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    sphere* s = create_sphere();
    intersections inter = create_intersections();
    intersect(s, &r, &inter);
    assert(inter.count == 2);
    assert(equal(inter.itersection[0].t, -1.0f));
    assert(equal(inter.itersection[1].t, 1.0f));
    return 0;
}

// 62 A sphere is behind a ray
int sphere_is_behind_ray_test() {
    ray r = create_ray(0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 1.0f);
    sphere* s = create_sphere();
    intersections inter = create_intersections();
    intersect(s, &r, &inter);
    assert(inter.count == 2);
    assert(equal(inter.itersection[0].t, -6.0f));
    assert(equal(inter.itersection[1].t, -4.0f));
    return 0;
}

// 63 An intersection encapsulates t and object
// Test Not Required Due To Design Of Application

// 64 Aggegating intersections
int aggregating_intersections_test() {
    intersections intersects = create_intersections();
    sphere* s = create_sphere();
    add_intersection_to_list(&intersects, 1.0, s);
    add_intersection_to_list(&intersects, 2.0, s);
    assert(intersects.count == 2);
    assert(equal(intersects.itersection[0].t, 1.0f));
    assert(equal(intersects.itersection[1].t, 2.0f));
    return 0;
}

// 64 Intersect sets the object on the intersection
int intersect_sets_object_on_intersection_test() {
    ray r = create_ray(0.0f, 0.0f, -5.0f, 0.0f, 0.0f, 1.0f);
    sphere* sp = create_sphere();
    intersections inter = create_intersections();
    intersect(sp, &r, &inter);
    assert(inter.count == 2);
    assert(inter.itersection[0].object_id == sp);
    assert(inter.itersection[1].object_id == sp);
    return 0;
}

// Clear Intersections List
// NOTE: Needed for testing
int clear_intersections_test() {
    intersections intersects = create_intersections();
    sphere* sp = create_sphere();
    add_intersection_to_list(&intersects, 9.0f, sp);
    clear_intersections(&intersects);
    assert(intersects.count == 0);
    for (int i = 0; i < INTERSECTIONS_SIZE; ++i) {
        assert(equal(intersects.itersection[i].t, DBL_MIN));
        assert(intersects.itersection[i].object_id == NULL);
    }
    return 0;
}

// 64  NOTE: All hit tests have been put together
int hit_tests(){
    intersections intersects = create_intersections();
    sphere* sp = create_sphere();

    // 65 The hit when all intersections have positive t
    add_intersection_to_list(&intersects, 1.0, sp);
    add_intersection_to_list(&intersects, 2.0, sp);
    assert(intersects.count == 2);
    intersection* intersect1 = hit(&intersects);
    assert(intersect1->object_id == sp);
    assert(equal(intersect1->t, 1.0f));

    //65 The hit when some intersections have a negative t
    clear_intersections(&intersects);
    assert(intersects.count == 0);
    add_intersection_to_list(&intersects, -1.0, sp);
    add_intersection_to_list(&intersects, 1.0, sp);
    assert(intersects.count == 2);
    intersect1 = hit(&intersects);
    assert(intersect1->object_id == sp);
    assert(equal(intersect1->t, 1.0f));

    // 65 The hit when all intersections have negative t
    clear_intersections(&intersects);
    assert(intersects.count == 0);
    add_intersection_to_list(&intersects, -2.0, sp);
    add_intersection_to_list(&intersects, -1.0, sp);
    assert(intersects.count == 2);
    intersect1 = hit(&intersects);
    assert(intersect1 == NULL);

    // 66 The hit is always the lowest nonnegative intersection
    clear_intersections(&intersects);
    assert(intersects.count == 0);
    add_intersection_to_list(&intersects, 5.0, sp);
    add_intersection_to_list(&intersects, 7.0, sp);
    add_intersection_to_list(&intersects, -3.0, sp);
    add_intersection_to_list(&intersects, 2.0, sp);
    assert(intersects.count == 4);
    intersect1 = hit(&intersects);
    assert(intersect1->object_id == sp);
    assert(equal(intersect1->t, 2.0f));
    return 0;
}

// 69 Translating a ray
int translating_ray_test() {
    ray r1 = create_ray(1.0f, 2.0f, 3.0f, 0.0f, 1.0f, 0.0f);
    Mat4x4 transMat;
    gen_translate_matrix(3.0f, 4.0f, 5.0f, transMat);

    ray r2 = transform(&r1, transMat);
    assert(&r1 != &r2);
    assert(equal(r2.originPoint.x, 4.0f));
    assert(equal(r2.originPoint.y, 6.0f));
    assert(equal(r2.originPoint.z, 8.0f));
    assert(equal(r2.originPoint.w, 1.0f));

    assert(equal(r2.directionVector.x, 0.0f));
    assert(equal(r2.directionVector.y, 1.0f));
    assert(equal(r2.directionVector.z, 0.0f));
    assert(equal(r2.directionVector.w, 0.0f));
    return 0;
}

// 69 Scaling a ray
int scaling_ray_test() {
    ray r1 = create_ray(1.0f, 2.0f, 3.0f, 0.0f, 1.0f, 0.0f);
    Mat4x4 scaleMat;
    gen_scale_matrix(2.0f, 3.0f, 4.0f, scaleMat);
    ray r2 = transform(&r1, scaleMat);

    assert(&r1 != &r2);

    assert(equal(r2.originPoint.x, 2.0f));
    assert(equal(r2.originPoint.y, 6.0f));
    assert(equal(r2.originPoint.z, 12.0f));
    assert(equal(r2.originPoint.w, 1.0f));

    assert(equal(r2.directionVector.x, 0.0f));
    assert(equal(r2.directionVector.y, 3.0f));
    assert(equal(r2.directionVector.z, 0.0f));
    assert(equal(r2.directionVector.w, 0.0f));
    return 0;
}

// 69 Sphere default transformation
int sphere_default_transformation_test() {
    sphere* sp = create_sphere();
    Mat4x4 identMat;
    Mat4x4_set_ident(identMat);
    assert(mat4x4_equal(sp->transform, identMat) == true);
    return 0;
}

// 69 Changing a sphere's transformation
int change_sphere_transform_test() {
    sphere* sp = create_sphere();
    Mat4x4 transMat;
    gen_translate_matrix(2.0f, 3.0f, 4.0f, transMat);
    set_transform(sp, transMat);
    assert(mat4x4_equal(sp->transform, transMat) == true);
    return 0;
}

int set_transform_test() {
    sphere* sp = create_sphere();
   
    Mat4x4 identMat;
    Mat4x4_set_ident(identMat);
    // does sphere have identity as transform?
    assert(mat4x4_equal(sp->transform, identMat));

    Mat4x4 transMat;
    gen_translate_matrix(2.0f, 3.0f, 4.0f, transMat);

    // is correct translate matrix?
    assert(equal(transMat[0][3], 2.0f));
    assert(equal(transMat[1][3], 3.0f));
    assert(equal(transMat[2][3], 4.0f));
    assert(equal(transMat[3][3], 1.0f));

    set_transform(sp, transMat);
    // has it been copied correctly?
    assert(mat4x4_equal(sp->transform, transMat));
    // two seperate matrixes
    assert(&sp->transform != &identMat);
    return 0;
}

// 69 Intersecting a scaled sphere with a ray
int intersect_scaled_sphere_test() {
    ray r1 = create_ray(0.0f, 0.0f, -5.0f, 0.0f, 0.0f, 1.0f);
    sphere* sp = create_sphere();
    Mat4x4 scaleMat;
    gen_scale_matrix(2.0f, 2.0f, 2.0f, scaleMat);
    set_transform(sp, scaleMat);
    assert(mat4x4_equal(sp->transform, scaleMat) == true);
    assert(&sp->transform != &scaleMat);

    intersections inter = create_intersections();
    intersect(sp, &r1, &inter);

    assert(inter.count == 2);
    assert(inter.itersection[0].object_id == sp);
    assert(equal(inter.itersection[0].t, 3.0f));

    assert(inter.itersection[1].object_id == sp);
    assert(equal(inter.itersection[1].t, 7.0f));
    return 0;
}

// 70 Intersecting a translated sphere with a ray
int intersecting_translated_sphere_test() {
    ray r1 = create_ray(0.0f, 0.0f, -5.0f, 0.0f, 0.0f, 1.0f);
    sphere* sp = create_sphere();
    Mat4x4 transMat;
    gen_translate_matrix(5.0f, 0.0f, 0.0f, transMat);
    set_transform(sp, transMat);

    Mat4x4 invScaleMat;
    Mat4x4_set_ident(invScaleMat);
    mat4x4_inverse(sp->transform, invScaleMat);
    ray r2 = transform(&r1, invScaleMat);
    intersections inter = create_intersections();
    intersect(sp, &r2, &inter);
    assert(inter.count == 0);
    return 0;
}

int normals_test() {
    sphere* sphere1 = create_sphere();
    // 78 The normal on a sphere at a point on the X axis.
    tuple location1 = create_point(1.0f, 0.0f, 0.0f);
    tuple n = normal_at(sphere1, location1);
    assert(equal(n.x, 1.0f));
    assert(equal(n.y, 0.0f));
    assert(equal(n.z, 0.0f));
    assert(equal(n.w, 0.0f));

    // 78 The normal on a sphere at a point on the Y axis.
    tuple location2 = create_point(0.0f, 1.0f, 0.0f);
    n = normal_at(sphere1, location2);
    assert(equal(n.x, 0.0f));
    assert(equal(n.y, 1.0f));
    assert(equal(n.z, 0.0f));
    assert(equal(n.w, 0.0f));

    // 78 The normal on a sphere at a point on the Z axis.
    tuple location3 = create_point(0.0f, 0.0f, 1.0f);
    n = normal_at(sphere1, location3);
    assert(equal(n.x, 0.0f));
    assert(equal(n.y, 0.0f));
    assert(equal(n.z, 1.0f));
    assert(equal(n.w, 0.0f));

    // 78 The normal on a sphere at a nonaxial point.
    double nonaxial = sqrt(3) / 3.0f;
    tuple location4 = create_point(nonaxial, nonaxial, nonaxial);
    n = normal_at(sphere1, location4);
    assert(equal(n.x, nonaxial));
    assert(equal(n.y, nonaxial));
    assert(equal(n.z, nonaxial));
    assert(equal(n.w, 0.0f));
    return 0;
}

// 78 The normal is a normalized vector.
int normal_is_normal_test() {
    sphere* sp = create_sphere();
    double nonaxial = sqrt(3) / 3.0f;
    tuple location1 = create_point(nonaxial, nonaxial, nonaxial);
    tuple n = normal_at(sp, location1);
    tuple nn = norm_vec(n);
    assert(equal(n.x, nn.x));
    assert(equal(n.y, nn.y));
    assert(equal(n.z, nn.z));
    assert(equal(n.w, 0.0f));
    return 0;
}

// 80 Computing the normal on a translated sphere
int compute_normal_on_sphere_test() {
    tuple vec1 = create_vector(0.0f, sqrt(2) / 2, -sqrt(2) / 2);
    sphere* sp = create_sphere();
    gen_translate_matrix(0.0f, 1.0f, 0.0f, sp->transform);
    tuple n = normal_at(sp, vec1);
    assert(equal(n.x, 0.0f));
    assert(equal(n.y, sqrt(2) / 2));
    assert(equal(n.z, -sqrt(2) / 2));
    assert(equal(n.w, 0.0f));
    return 0;
}
 
// 80 Computing the normal on a transformed sphere
int compute_normal_on_transformed_sphere_test(){
    sphere* sp = create_sphere();
    Mat4x4 scaleMat;
    gen_scale_matrix(1.0f, 0.5f, 1.0f, scaleMat);
    Mat4x4 rotMat;
    gen_rotate_matrix_Z(M_PI / 5.0f, rotMat);
    Mat4x4 translateMat;
    mat4x4_mul(scaleMat, rotMat, translateMat);
    set_transform(sp, translateMat);
    tuple point = create_point(0.0f, sqrt(2) / 2.0f, -sqrt(2) / 2);
    tuple norm_at = normal_at(sp, point);
    assert(equal(norm_at.x, 0.0f));
    assert(equal(norm_at.y, 0.97014250014533188f));
    assert(equal(norm_at.z, -0.24253562503633294f));
    return 0;
}

// 83 Reflecting a vector approaching at 45deg
int reflect_vector_approach_at_45_deg_test() {
    tuple v = create_vector(1.0f, -1.0f, 0.0f);
    tuple n = create_vector(0.0f, 1.0f, 0.0f);
    tuple r = reflect(v, n);
    assert(equal(r.x, 1.0f));
    assert(equal(r.y, 1.0f));
    assert(equal(r.z, 0.0f));
    return 0;
}

// 83 Reflecting a vector off a slanted surface
int reflect_vector_off_slanted_surf_test() {
    tuple v = create_vector(0.0f, -1.0f, 0.0f);
    tuple n = create_vector(sqrt(2)/2, sqrt(2)/2, 0.0f);
    tuple r = reflect(v, n);
    assert(equal(r.x, 1.0f));
    assert(equal(r.y, 0.0f));
    assert(equal(r.z, 0.0f));
    return 0;
}

// 84 A point light has a position and intensity
int point_light_position_intensity_test() {
    tuple intensity = create_point(1.0f, 2.0f, 3.0f);
    tuple position1 = create_point(4.0f, 5.0f, 6.0f);
    point_light pl = create_point_light(position1, intensity);
    assert(pl.next == NULL);
    assert(equal(pl.intensity.x, intensity.x));
    assert(equal(pl.intensity.y, intensity.y));
    assert(equal(pl.intensity.z, intensity.z));
    assert(equal(pl.intensity.w, intensity.w));

    assert(equal(pl.position.x, position1.x));
    assert(equal(pl.position.y, position1.y));
    assert(equal(pl.position.z, position1.z));
    assert(equal(pl.position.w, position1.w));
    return 0;
}

// 85 The default material
int default_material_test() {
    // typedef struct { tuple color; double ambient; double diffuse; double specualar; double shininess; } material;
    tuple color_white = create_vector(1.0f, 1.0f, 1.0f);
    material m1 = create_material(color_white, 0.1f, 0.9f, 0.9f, 200.0f);

    assert(equal(m1.color.x, 1.0f));
    assert(equal(m1.color.y, 1.0f));
    assert(equal(m1.color.z, 1.0f));
    assert(equal(m1.color.w, 0.0f));

    assert(equal(m1.ambient, 0.1f));
    assert(equal(m1.diffuse, 0.9f));
    assert(equal(m1.specular, 0.9f));
    assert(equal(m1.shininess, 200.0f));

    material m2 = create_material_default();

    assert(equal(m2.color.x, 1.0f));
    assert(equal(m2.color.y, 1.0f));
    assert(equal(m2.color.z, 1.0f));
    assert(equal(m2.color.w, 0.0f));

    assert(equal(m2.ambient, 0.1f));
    assert(equal(m2.diffuse, 0.9f));
    assert(equal(m2.specular, 0.9f));
    assert(equal(m2.shininess, 200.0f));

    return 0;
}

// 85 Sphere has a default material
int sphere_has_default_material_test() {
    sphere* sp = create_sphere();
    material m1 = create_material_default();

    assert(equal(m1.color.x, sp->material.color.x));
    assert(equal(m1.color.y, sp->material.color.y));
    assert(equal(m1.color.z, sp->material.color.z));
    assert(equal(m1.color.w, sp->material.color.w));

    assert(equal(m1.ambient, 0.1f));
    assert(equal(m1.diffuse, 0.9f));
    assert(equal(m1.specular, 0.9f));
    assert(equal(m1.shininess, 200.0f));
    return 0;
}

// 86 Lighting with the eye between the light and the surface
int lighting_with_eye_between_light_and_surface_test() {
    material m = create_material_default();
    tuple position1 = create_vector(0.0f, 0.0f, 0.0f);
    tuple eyev = create_vector(0.0f, 0.0f, -1.0f);
    tuple normalv = create_vector(0.0f, 0.0f, -1.0f);
    tuple intensity = create_vector(1.0f, 1.0f, 1.0f);
    tuple p_light_position = create_point(0.0f, 0.0f, -10.0f);
    point_light p_light = create_point_light(p_light_position, intensity);
    tuple light1 = lighting(m, p_light, position1, eyev, normalv);
    assert(equal(light1.x, 1.9f));
    assert(equal(light1.y, 1.9f));
    assert(equal(light1.z, 1.9f));
    return 0;
}

// 86 Lighting with the eye between light and surface, eye offset 45 deg
int lighting_with_eye_between_light_and_surface_eye_offset_test() {
    material m = create_material_default();
    tuple position1 = create_point(0.0f, 0.0f, 0.0f);
    tuple eyev = create_vector(0.0f, sqrt(2)/2, -sqrt(2)/2);
    tuple normalv = create_vector(0.0f, 0.0f, -1.0f);
    tuple intensity = create_vector(1.0f, 1.0f, 1.0f);
    tuple p_light_position = create_point(0.0f, 0.0f, -10.0f);
    point_light p_light = create_point_light(p_light_position, intensity);
    tuple light1 = lighting(m, p_light, position1, eyev, normalv);
    assert(equal(light1.x, 1.0f));
    assert(equal(light1.y, 1.0f));
    assert(equal(light1.z, 1.0f));
    return 0;
}

// 87 Lighting with the eye opposite surface, light offset 45 deg
int lighting_with_eye_opposite_surface_test() {
    material m = create_material_default();
    tuple position1 = create_point(0.0f, 0.0f, 0.0f);
    tuple eyev = create_vector(0.0f, 0.0f, -1.0f);
    tuple normalv = create_vector(0.0f, 0.0f, -1.0f);
    tuple intensity = create_vector(1.0f, 1.0f, 1.0f);
    tuple p_light_position = create_point(0.0f, 10.0f, -10.0f);
    point_light p_light = create_point_light(p_light_position, intensity);
    tuple light1 = lighting(m, p_light, position1, eyev, normalv);
    assert(equal(light1.x, 0.73639608769926945f));
    assert(equal(light1.y, 0.73639608769926945f));
    assert(equal(light1.z, 0.73639608769926945f));
    return 0;
}

// 87 Lighting with the eye in the path of the reflection vector
int lighting_with_eye_in_path_of_reflect_vector_test() {
    material m = create_material_default();
    tuple position1 = create_point(0.0f, 0.0f, 0.0f);
    tuple eyev = create_vector(0.0f, -sqrt(2) / 2, -sqrt(2) / 2);
    tuple normalv = create_vector(0.0f, 0.0f, -1.0f);
    tuple intensity = create_vector(1.0f, 1.0f, 1.0f);
    tuple p_light_position = create_point(0.0f, 10.0f, -10.0f);
    point_light p_light = create_point_light(p_light_position, intensity);
    tuple light1 = lighting(m, p_light, position1, eyev, normalv);
    assert(equal(light1.x, 1.6363960638574115f));
    assert(equal(light1.y, 1.6363960638574115f));
    assert(equal(light1.z, 1.6363960638574115f));
    return 0;
}

// 88 Lighting with the light behind the surface
int lighting_with_the_light_behind_surface_test() {
    material m = create_material_default();
    tuple position1 = create_point(0.0f, 0.0f, 0.0f);
    tuple eyev = create_vector(0.0f, 0.0f, 1.0f);
    tuple normalv = create_vector(0.0f, 0.0f, -1.0f);
    tuple intensity = create_vector(1.0f, 1.0f, 1.0f);
    tuple p_light_position = create_point(0.0f, 0.0f, 10.0f);
    point_light p_light = create_point_light(p_light_position, intensity);
    tuple light1 = lighting(m, p_light, position1, eyev, normalv);
    assert(equal(light1.x, 0.1f));
    assert(equal(light1.y, 0.1f));
    assert(equal(light1.z, 0.1f));
    return 0;
}



int intersect_compare_test() {
    sphere* sp1 = create_sphere();
    sphere* sp2 = create_sphere();
    sphere* sp3 = create_sphere();
    intersection i1 = { 0.0f, sp1 };
    intersection i2 = { 1.0f, sp2 };
    intersection i3 = { -1.0f, sp3 };

    assert(intersect_compare(&i1, &i2) == -1);
    assert(intersect_compare(&i1, &i1) == 0);
    assert(intersect_compare(&i2, &i1) == 1);

    assert(intersect_compare(&i1, &i3) == 1);
    assert(intersect_compare(&i3, &i3) == 0);
    assert(intersect_compare(&i3, &i1) == -1);
    return 0;
}



int sort_intersects_test() {
    intersections intersects = create_intersections();
    sphere* sp1 = create_sphere();
    sphere* sp2 = create_sphere();
    sphere* sp3 = create_sphere();
    sphere* sp4 = create_sphere();

    add_intersection_to_list(&intersects, 1.0, sp1);
    add_intersection_to_list(&intersects, 2.0, sp2);

    sort_intersects(&intersects);

    assert(intersects.count == 2);
    assert(equal(intersects.itersection[0].t, 1.0f));
    assert(equal(intersects.itersection[1].t, 2.0f));

    clear_intersections(&intersects);

    add_intersection_to_list(&intersects, 4.0, sp1);
    add_intersection_to_list(&intersects, 3.0, sp2);
    add_intersection_to_list(&intersects, 2.0, sp3);
    add_intersection_to_list(&intersects, 1.0, sp4);

    sort_intersects(&intersects);

    assert(intersects.count == 4);
    assert(equal(intersects.itersection[0].t, 1.0f));
    assert(intersects.itersection[0].object_id == sp4);
    assert(equal(intersects.itersection[1].t, 2.0f));
    assert(intersects.itersection[1].object_id == sp3);
    assert(equal(intersects.itersection[2].t, 3.0f));
    assert(intersects.itersection[2].object_id == sp2);
    assert(equal(intersects.itersection[3].t, 4.0f));
    assert(intersects.itersection[3].object_id == sp1);

    clear_intersections(&intersects);

    add_intersection_to_list(&intersects, -6.0, sp1);
    add_intersection_to_list(&intersects, 1.0, sp2);
    add_intersection_to_list(&intersects, 1.0, sp2);
    add_intersection_to_list(&intersects, 57.0, sp3);
    add_intersection_to_list(&intersects, -90.0, sp4);

    sort_intersects(&intersects);

    assert(intersects.count == 5);
    assert(equal(intersects.itersection[0].t, -90.0f));
    assert(intersects.itersection[0].object_id == sp4);
    assert(equal(intersects.itersection[1].t, -6.0f));
    assert(intersects.itersection[1].object_id == sp1);
    assert(equal(intersects.itersection[2].t, 1.0f));
    assert(intersects.itersection[2].object_id == sp2);
    assert(equal(intersects.itersection[3].t, 1.0f));
    assert(intersects.itersection[3].object_id == sp2);
    assert(equal(intersects.itersection[4].t, 57.0f));
    assert(intersects.itersection[4].object_id == sp3);
    return 0;
}

// 92 Creating A World
int creating_a_world_test() {
    world w = create_world();
    assert(w.lights  == NULL);
    assert(w.objects == NULL);
    return 0;
}

// 92 Default World
int default_world_test() {
    world w = default_world();
    // w.light = light
    assert(equal(w.lights->intensity.x,  1.0f));
    assert(equal(w.lights->intensity.y,  1.0f));
    assert(equal(w.lights->intensity.z,  1.0f));
    assert(equal(w.lights->position.x, -10.0f));
    assert(equal(w.lights->position.y,  10.0f));
    assert(equal(w.lights->position.z, -10.0f));
    assert(w.objects != NULL);
    assert(w.objects->next != NULL);
    assert(w.objects->next->next == NULL);
    assert(equal(w.objects->location.x, 0.0f));
    assert(equal(w.objects->location.y, 0.0f));
    assert(equal(w.objects->location.z, 0.0f));
    Mat4x4 ident;
    Mat4x4_set_ident(ident);
    
    // w contains s1
    sphere* sp = w.objects;
    assert(mat4x4_equal(sp->transform, ident));

    // w contains s2
    sp = sp->next;
    assert(equal(sp->transform[0][0], 0.5f));
    assert(equal(sp->transform[1][1], 0.5f));
    assert(equal(sp->transform[2][2], 0.5f));
    assert(equal(sp->transform[3][3], 1.0f));
    return 0;
}

// 92 Intersect a world with a ray
int intersect_world_with_ray_test() {
    world w = default_world();
    ray r = create_ray(0.0f, 0.0f, -5.0f, 0.0f, 0.0f, 1.0f);
    intersections inter = create_intersections();
    intersect_world(&w, &r, &inter);
    assert(inter.count == 4);
    assert(equal(inter.itersection[0].t, 4.0f));
    assert(equal(inter.itersection[1].t, 4.5f));
    assert(equal(inter.itersection[2].t, 5.5f));
    assert(equal(inter.itersection[3].t, 6.0f));
    return 0;
}

#endif

// 72 Hint #4
void render_sphere() {
  
  tuple ray_origin = create_point(0.0f, 0.0f, -5.0f);

  double wall_z = 10.0f;
  double wall_size = 7.0f;
  double pixel_size = wall_size / WIDTH;
  double half = wall_size / 2.0f;

  material m = create_material_default();
  m.color.x = 1.0f; m.color.y = 0.2f; m.color.z = 1.0f;
  sphere* sphere1 = create_sphere();
  sphere1->material = m;
  sphere1->material.ambient = 0.15f;
  sphere1->material.color.x = 0.254901;
  sphere1->material.color.y = 0.423529;
  sphere1->material.color.z = 0.58823;
  sphere1->material.shininess = 100.0f;

  tuple l_color = create_vector(1.0f, 1.0f, 1.0f);
  tuple l_position = create_point(-10.0f, -10.0f, -10.0f);
  point_light p_light = create_point_light(l_position, l_color);

  for (int y = 0; y < WIDTH; ++y) {
    double world_y = half - pixel_size * y;
    for (int x = 0; x < HEIGHT; ++x) {
      double world_x = -half + pixel_size * x;
      tuple position1 = create_point(world_x, world_y, wall_z);
      tuple posRayOrigin = tuple_sub(position1, ray_origin);
      tuple normRayOrigin = norm_vec(posRayOrigin);
      ray ray_to_draw = create_ray(ray_origin.x, ray_origin.y, ray_origin.z, normRayOrigin.x, normRayOrigin.y, normRayOrigin.z );
      ray_to_draw.directionVector = norm_vec(ray_to_draw.directionVector);
      intersections inter = create_intersections();
      intersect(sphere1, &ray_to_draw, &inter);
      intersection* hit_intersection = hit(&inter);
      if (hit_intersection) {
        tuple point2 = position(ray_to_draw, hit_intersection->t);
        tuple normal = normal_at(hit_intersection->object_id, point2);
        tuple eye = tuple_negate(ray_to_draw.directionVector);
        tuple pix_color = lighting(hit_intersection->object_id->material, p_light, point2, eye, normal);
        //assert(pix_color.x <= 255 && pix_color.x >= 0);
        //assert(pix_color.y <= 255 && pix_color.y >= 0);
        //assert(pix_color.z <= 255 && pix_color.z >= 0);
        write_pixel(x, y, pix_color);
      }
    }
  }
}

int main() {
#if defined _DEBUG
  unit_test("Create Point Test", create_point_test());
  unit_test("Create Vector Test", create_vector_test());
  unit_test("Tuple With 0 Is A Point Test", tuple_with_W_0_is_point_test());
  unit_test("Tuple Add Test", tuple_add_test());
  unit_test("Tuple Subtract Test", tuple_sub_test());
  unit_test("Subtract Vector From A Point Test", subtract_vector_from_point_test());
  unit_test("Subtract Two Vectors Test", subtract_two_vectors_test());
  unit_test("Subtract Vector From Zero Vector Test", subtract_vector_from_zero_vector_test());
  unit_test("Negative Tuple Test", negating_tuple_test());
  unit_test("Tuple Multiplication Scalar Test", tuple_mult_scalar_test());
  unit_test("Tuple Multiplication Scalar Fraction Test", tuple_mult_scalar_fraction_test());
  unit_test("Tuple Division Scalar Test", tuple_div_scalar_test());
  unit_test("Tuple Magnigude Vector Test", tuple_mag_vec_test());
  unit_test("Normal Vector Test", norm_vec_test());
  unit_test("Dot Product Test", dot_prod_test());
  unit_test("Cross Product Test", cross_prod_test());
  unit_test("Hadamard Product Test", hadamard_product_test());
  //unitTest("Write Pixel Test", writePixelTest());
  unit_test("Color Conversion Test", color_convert_test());
  unit_test("Matrix Equality Test", mat_equal_test());
  unit_test("4x4 Matrix Multiply Test", mat4x4_mul_test());
  unit_test("4x4 Matrix Multiply In Place Test", mat4x4_mul_in_place_test());
  unit_test("4x4 Matrix Multiply By Tuple Test", mat4x4_mul_tuple_test());
  unit_test("4x4 Matrix Multiply By Identity Test", mat4x4_mult_ident_test());
  unit_test("4x4 Matrix Transposition Test", mat4x4_transpose_test());
  unit_test("2x2 Matrix Determinant Test", mat2x2_det_test());
  unit_test("2x2 Submatrix From 3x3 Matrix Test",mat3x3_submat_2x2_test());
  unit_test("3x3 Submatrix From 4x4 Matrix Test", mat4x4_submat_3x3_test());
  unit_test("3x3 Matrix Minor Test", mat3x3_minor_test());
  unit_test("3x3 Matrix Cofactor Test", mat3x3_cofactor_test());
  unit_test("3x3 Matrix Determinant Test", mat3x3_det_test());
  unit_test("4x4 Matrix Determinant Test", mat4x4_det_test());
  unit_test("Invertable Matrix Test", invertable_matrix_test());
  unit_test("4x4 Matrix Invert Test", inverse_matrix_test());
  unit_test("Multiply Product By Its Inverse Test", mult_prod_by_inverse_test());
  unit_test("Multiply By Translation Matrix Test", point_trans_test());
  unit_test("Multiply By Inverse Of Translation Matrix Test", point_mult_inverse_translation_test());
  unit_test("Vector Translation Has No Effect Test", vector_translation_has_no_effect_test());
  unit_test("Scaling Matrix Applied To A Point Test", point_scale_Mat4x4_test());
  unit_test("Scaling Matrix Applied To A Vector Test", vec_scale_Mat4x4_test());
  unit_test("Multiply Inverse Of Scaling Matrix Test", mult_inverse_scale_matrix_test());
  unit_test("Reflection Scaling Negative Value Test", reflection_scaling_neg_value_test());
  unit_test("Generate Rotation Matrix X Test", gen_rotation_matrix_X_test());
  unit_test("Generate  Rotation Matrix X Reverse Test", gen_rotation_matrix_reverse_test());
  unit_test("Generate Rotation Matrix Y Test", gen_rotation_matrix_Y_test());
  unit_test("Generate Rotation Matrix Z Test", gen_rotation_matrix_Z_test());
  unit_test("Generate Sheer Matrix Test", gen_shear_matrix_test());
  unit_test("Transformations Applied In Sequence Test", transform_applied_in_sequence_test());
  //unitTest("Draw Clock Test", drawClockTest());
  unit_test("Tuple Copy Test", tuple_copy_test());
  unit_test("Create Ray Test", create_ray_test());
  unit_test("Create Sphere Test", create_sphere_test());
  unit_test("Create Intersections Test", create_intersections_test());
  unit_test("Position Test", position_test());
  unit_test("Ray Intersect Sphere At Two Points Test", ray_intersect_sphere_two_point_test());
  unit_test("Ray Intersect Sphere Tangent Test", ray_intersect_sphere_tangent_test());
  unit_test("Ray Misses Sphere Test", ray_misses_sphere_test());
  unit_test("Ray Originates Inside Sphere Test", ray_originates_inside_sphere_test());
  unit_test("Sphere Is Behind Ray Test", sphere_is_behind_ray_test());
  unit_test("Aggregating Intersections Test", aggregating_intersections_test());
  unit_test("Intersect Sets Object On Intersection Test", intersect_sets_object_on_intersection_test());
  unit_test("Clear Intersections Test", clear_intersections_test());
  unit_test("Hit Test", hit_tests());
  unit_test("Change Sphere Transform Test", change_sphere_transform_test());
  unit_test("Intersect Scaled Sphere With Ray Test", intersect_scaled_sphere_test());
  unit_test("Translating A Ray Test", translating_ray_test());
  unit_test("Scaling A Ray Test", scaling_ray_test());
  unit_test("Sphere Default Transformation Test", sphere_default_transformation_test());
  unit_test("Set Transform Test", set_transform_test());
  unit_test("Intersecting Translated Sphere With Ray Test", intersecting_translated_sphere_test());
  unit_test("Normal Is Normal Test", normal_is_normal_test());
  unit_test("Normals Test", normals_test());
  unit_test("Compute Normal On Sphere Test", compute_normal_on_sphere_test());
  unit_test("Compute Normal On Transformed Sphere Test", compute_normal_on_transformed_sphere_test());
  unit_test("Reflect Vector Approach At 45 Deg Test", reflect_vector_approach_at_45_deg_test());
  unit_test("Reflect Vector Off Slanted Surface Test", reflect_vector_off_slanted_surf_test());
  unit_test("Point Light Position Intensity Test", point_light_position_intensity_test());
  unit_test("Default Material Test", default_material_test());
  unit_test("Sphere Has A Default Material Test", sphere_has_default_material_test());
  unit_test("Lighting With Eye Between Light And Surface Test", lighting_with_eye_between_light_and_surface_test());
  unit_test("Lighting With Eye Between Light And Surface Eye Offset 45 Degrees Test", lighting_with_eye_between_light_and_surface_eye_offset_test());
  unit_test("Lighting With Eye Opposite Surface, Light Offset 45 Degrees Test", lighting_with_eye_opposite_surface_test());
  unit_test("Lighting With Eye In Path Of Reflect Vector Test", lighting_with_eye_in_path_of_reflect_vector_test());
  unit_test("Lighting With The Light Behind Surface Test", lighting_with_the_light_behind_surface_test());
  unit_test("Intersect Compare Test", intersect_compare_test());
  unit_test("Sort Intersects Test", sort_intersects_test());
  unit_test("Create New World Test", creating_a_world_test());
  unit_test("Default World Test", default_world_test());
  unit_test("Intersect World With Ray Test", intersect_world_with_ray_test());
#endif
  render_sphere();
  write_canvas_to_file();
  return 0;
}