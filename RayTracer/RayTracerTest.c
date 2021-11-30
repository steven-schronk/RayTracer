/*

Ray Tracer - By Steven Schronk

Generates PPM ray traced image.

Compile For Linux Raspberry Pi
    gcc RayTracer.c -lm -lrt -O3 -mcpu=cortex-a7 -mfpu=neon-vfpv4 -o RayTracer

Copyright 2021 Steven Ray Schronk

*/

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
// #include <vld.h>  // C:\Program Files (x86)\Visual Leak Detector

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EPSILON 0.000001

// remaining number of iterations when calculating reflection
#define RECURSION_DEPTH 5

#define VERTICAL_SIZE   480
#define HORIZONTAL_SIZE 480

typedef double Mat2x2[2][2];
typedef double Mat3x3[3][3];
typedef double Mat4x4[4][4];

typedef struct { double x, y, z, w; } tuple;

enum pattern_type { TEST, STRIPE, GRADIANT, RING, CHECKER };

typedef struct pattern { tuple from; tuple to; Mat4x4 transform; tuple(*pattern_at_fn_ptr)(struct pattern* pat, tuple* point); enum pattern_type type; } pattern;

typedef struct { tuple color; double ambient; double diffuse; double specular; double shininess; double reflective; double transparency; double refractive_index; bool has_pattern; pattern pattern; } material;

typedef struct { tuple position; tuple intensity; struct point_light* next; } point_light;

enum shape_type { SHAPE, PLANE, SPHERE };

typedef struct _shape { tuple location; double t; Mat4x4 transform; material material; struct _shape* next; enum shape_type type; } shape;

typedef struct { tuple origin_point; tuple direction_vector; } ray;

typedef struct { double t; shape* object_id; } intersection;

typedef struct { struct _shape* objects; point_light* lights; } world;

typedef struct { double t; shape* object; tuple point; tuple eyev; tuple normalv; bool inside; tuple over_point; tuple under_point; tuple reflectv; double n1; double n2; } comps;

typedef struct { double hsize; double vsize; double field_of_view; double pixel_size; double half_width; double half_height; Mat4x4 view_transform; } camera;

#define INTERSECTIONS_SIZE 100

typedef struct { intersection itersection[INTERSECTIONS_SIZE]; int count; } intersections;

#define CONTAINERS_SIZE 100

typedef struct { shape* shape_ptr[CONTAINERS_SIZE]; } containers;

containers contents;

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

void clear_intersections(intersections* intersection_list) {
    assert(intersection_list != NULL);
    intersection_list->count = 0;
    for (int i = 0; i < INTERSECTIONS_SIZE; ++i) {
        intersection_list->itersection[i].t = DBL_MIN;
        intersection_list->itersection[i].object_id = NULL;
    }
}

int intersect_compare(const intersection* a, const intersection* b) {
    if (a->t > b->t) { return  1; }
    if (a->t < b->t) { return -1; }
    return 0;
}

void sort_intersects(intersections* intersects) {
    qsort(intersects->itersection, intersects->count, sizeof(intersection), intersect_compare);
}

intersection* hit(intersections* intersection_list) {
  assert(intersection_list != NULL);
  if (0 == intersection_list->count) return NULL;
  intersection* intersect1 = NULL;
  double t = DBL_MAX;
  for (int i = 0; i < INTERSECTIONS_SIZE; ++i) {
      if (intersection_list->itersection[i].t == DBL_MIN) { break; } // intersections list empty
      if (intersection_list->itersection[i].t < 0) { continue; }
      if(intersection_list->itersection[i].t < t) {
          t = intersection_list->itersection[i].t;
          intersect1 = &intersection_list->itersection[i];
      }
  }
  return intersect1;
}

bool add_intersection(intersections* intersection_list, double t, shape* sp ) {
  assert(intersection_list != NULL && "Call to add insertion to list cannot contain null intersection list");
  if (intersection_list->count == INTERSECTIONS_SIZE)
  {
      assert("Intersection list too small.\nIncreate INTERSECTIONS_SIZE to fix.");
      return false;
  }
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
  //sort_intersects(intersection_list); // TODO: This might need to come out.
  return true;
}

containers containers_create() {
    containers conts;
    for (int i = 0; i < CONTAINERS_SIZE; ++i) {
        conts.shape_ptr[i] = NULL;
    }
    return conts;
}

void containers_clear(containers* conts) {
    assert(conts != NULL);
    for (int i = 0; i < CONTAINERS_SIZE; ++i) {
        conts->shape_ptr[i] = NULL;
    }
}

bool containers_is_empty(containers* conts) {
    assert(conts != NULL);
    if (conts->shape_ptr[0] == NULL) { return true; }
    return false;
}

shape* container_last(containers* conts) {
    assert(conts != NULL);
    if (conts->shape_ptr[0] == NULL) { return NULL; }
    int i = 0;
    shape* last_cont = NULL;
    while (conts->shape_ptr[i] != NULL) {
        last_cont = conts->shape_ptr[i];
        ++i;
    }
    return last_cont;
}

bool containers_includes(containers* conts, shape* comp) {
    assert(conts != NULL);
    assert(comp != NULL);
    for (int i = 0; i < CONTAINERS_SIZE; ++i) {
        if (conts->shape_ptr[i] == comp) { return true; }
    }
    return false;
}

void container_append(containers* conts, shape* comp) {
    assert(conts != NULL);
    assert(comp != NULL);
    for (int i = 0; i < CONTAINERS_SIZE; i++) {
        if (conts->shape_ptr[i] == NULL) {
            conts->shape_ptr[i] = comp;
            break;
        }
    }
}

bool container_remove(containers* conts, shape* comp) {
    assert(conts != NULL);
    assert(comp != NULL);
    int i = 0;
    bool found = false;
    while (i < CONTAINERS_SIZE && conts->shape_ptr[i] != NULL) {
        if (conts->shape_ptr[i] == comp) {
            found = true;
            break;
        }
        i++;
    }
    if (found) {
        while (i < CONTAINERS_SIZE - 1 && conts->shape_ptr[i] != NULL) {
            conts->shape_ptr[i] = conts->shape_ptr[i + 1];
            conts->shape_ptr[i + 1] = NULL;
            i++;
        }
    }
    return found;
}

tuple canvas[HORIZONTAL_SIZE][VERTICAL_SIZE];

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
  tuple t = { x, y, z, 1.0 };
  return t;
}

tuple create_vector(double x, double y, double z) {
  tuple t = { x, y, z, 0.0 };
  return t;
}

ray create_ray(double origin_point_x, double origin_point_y, double origin_point_z, double dirVector_x, double dirVector_y, double dirVector_z) {
    ray r;
    r.origin_point.x = origin_point_x;
    r.origin_point.y = origin_point_y;
    r.origin_point.z = origin_point_z;
    r.origin_point.w = 1.0;

    r.direction_vector.x = dirVector_x;
    r.direction_vector.y = dirVector_y;
    r.direction_vector.z = dirVector_z;
    r.direction_vector.w = 0.0;
    return r;
}

bool tuple_is_point(tuple t) { return t.w == 1.0 ? true : false; }

bool tuple_is_vector(tuple t) { return t.w == 0.0 ? true : false; }

void tuple_copy(tuple* t1, tuple* t2) {
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
  tuple t3 = { t1.x - t2.x, t1.y - t2.y, t1.z - t2.z, t1.w - t2.w};
  return t3;
}

tuple tuple_negate(tuple t) {
  tuple neg = { 0.0, 0.0, 0.0, 0.0 };
  tuple ret = tuple_sub(neg, t);
  return ret;
}

tuple tuple_mult_scalar(tuple t, double s) {
  tuple ret = { t.x * s, t.y * s, t.z * s, t.w * s };
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
  double magw = pow(t.w, 2);
  double mag = sqrt( magx + magy + magz + magw);
  return mag;
}

tuple tuple_normalize(tuple t) {
  double mag = tuple_mag_vec(t);
  tuple ret = { t.x / mag, t.y / mag, t.z / mag, t.w / mag};
  return ret;
}

double tuple_dot(tuple t1, tuple t2) {
  double prod1 = t1.x * t2.x;
  double prod2 = t1.y * t2.y;
  double prod3 = t1.z * t2.z;
  double prod4 = t1.w * t2.w;
  double dot = prod1 + prod2 + prod3 + prod4;
  return dot;
}

tuple tuple_cross(tuple a, tuple b) {
  double x = a.y * b.z - a.z * b.y;
  double y = a.z * b.x - a.x * b.z;
  double z = a.x * b.y - a.y * b.x;
  tuple cross = create_vector(x, y, z);
  return cross;
}

tuple hadamard_product(tuple c1, tuple c2) {
  tuple color = { c1.x * c2.x, c1.y * c2.y, c1.z * c2.z, 1.0 };
  return color;
}

void mat4x4_copy(Mat4x4 m1, Mat4x4 m2) {
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

void mat4x4_mul_in_place(const Mat4x4 a, const Mat4x4 b, Mat4x4 m) {
    Mat4x4 orig;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            orig[row][col] =
                a[row][0] * b[0][col] +
                a[row][1] * b[1][col] +
                a[row][2] * b[2][col] +
                a[row][3] * b[3][col];
        }
    }
    mat4x4_copy(orig, m);
}

void mat4x4_mul_tuple(const Mat4x4 a, const tuple b, tuple* c) {
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

// TODO: All these print statements should be in debug
void tuple_print(tuple t) {
  printf("{ %+10.5f, %+10.5f, %+10.5f, %+10.5f }\n", t.x, t.y, t.z, t.w);
}

void mat4x4_print(Mat4x4 mat) {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      printf("%+15.5f", mat[i][j]);
    }
  }
}

void camera_print(camera* c) {
    printf("\nCAMERA\n");
    printf("field_of_view: %+10.5f\n", c->field_of_view);
    printf("half_height:   %+10.5f\n", c->half_height);
    printf("half_width:    %+10.5f\n", c->half_width);
    printf("hsize:         %+10.5f\n", c->hsize);
    printf("pixel_size:    %+10.5f\n", c->pixel_size);
    printf("vsize:         %+10.5f\n", c->vsize);
    printf("view_transform:\n");
    mat4x4_print(c->view_transform);
    printf("\n");
}

void material_print(material* mat) {
    printf("\nMATERIAL\n");
    printf("ambient:          %+10.5f\n", mat->ambient);
    printf("color:");
    tuple_print(mat->color);
    printf("diffuse:          %+10.5f\n", mat->diffuse);
    printf("reflective:       %+10.5f\n", mat->reflective);
    printf("refractive_index: %+10.5f\n", mat->refractive_index);
    printf("shininess:        %+10.5f\n", mat->shininess);
    printf("specular:         %+10.5f\n", mat->specular);
    printf("transparency:     %+10.5f\n", mat->transparency);
    printf("has_pattern:      %+10.5d\n", mat->has_pattern);
}

void object_print(shape* s) {
    printf("\nOBJECT    \n");
    printf("location:");
    tuple_print(s->location);
    printf("transform:  \n");
    mat4x4_print(s->transform);
    material_print(&s->material);
    printf("shape_type: ");
    switch (s->type)
    {
    case SHAPE: printf("SHAPE");
    case PLANE: printf("PLANE");
    case SPHERE: printf("SPHERE");
    }
    printf("\n");
}

void light_print(point_light *pl) {
    printf("\nLIGHT\n");
    printf("intensity:");
    tuple_print(pl->intensity);
    printf("position: ");
    tuple_print(pl->position);
    printf("\n");
}

void world_print(world w) {
    printf("\nWORLD\n");
    shape* obj = w.objects;
    while (obj != NULL) {
        object_print(obj);
        obj = obj->next;
    }
    point_light* pl = w.lights;
    while (pl != NULL) {
        light_print(pl);
        pl = pl->next;
    }
    printf("\n");
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
  Mat2x2 b = { { 0.0, 0.0 }, { 0.0, 0.0 } };
  mat3x3_submat2x2(a, b, row, col);
  return mat2x2_det(b);
}

double mat4x4_minor(Mat4x4 a, int row, int col) {
  Mat3x3 b = { { 0.0, 0.0 }, { 0.0, 0.0 }, { 0.0, 0.0 } };
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
  double det_val = 0.0;
  for (int column = 0; column < size; ++column) {
    double mat3_cof = mat4x4_cofactor(m, 0, column);
    det_val = det_val + m[0][column] * mat3_cof;
  }
  return det_val;
}

bool mat4x4_invertable(Mat4x4 m) {
  if(equal(mat4x4_det(m, 4),0)) { return false; }
  return true;
}

bool mat4x4_inverse(Mat4x4 a, Mat4x4 b) {
  bool invert = mat4x4_invertable(a);
  if (!invert) { return false; }
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double c = mat4x4_cofactor(a, i, j);
      b[j][i] = c / mat4x4_det(a, 4);
    }
  }
  return true;
}

void mat4x4_set_ident(Mat4x4 m) {
  m[0][0] = 1.0; m[0][1] = 0.0; m[0][2] = 0.0; m[0][3] = 0.0;
  m[1][0] = 0.0; m[1][1] = 1.0; m[1][2] = 0.0; m[1][3] = 0.0;
  m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 1.0; m[2][3] = 0.0;
  m[3][0] = 0.0; m[3][1] = 0.0; m[3][2] = 0.0; m[3][3] = 1.0;
}

void gen_translate_matrix(const double x, const double y, const double z, Mat4x4 m) {
  m[0][0] = 1.0; m[0][1] = 0.0; m[0][2] = 0.0; m[0][3] = x;
  m[1][0] = 0.0; m[1][1] = 1.0; m[1][2] = 0.0; m[1][3] = y;
  m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 1.0; m[2][3] = z;
  m[3][0] = 0.0; m[3][1] = 0.0; m[3][2] = 0.0; m[3][3] = 1.0;
}

void gen_scale_matrix(const double x, const double y, const double z, Mat4x4 m) {
  m[0][0] = x;   m[0][1] = 0.0; m[0][2] = 0.0; m[0][3] = 0.0;
  m[1][0] = 0.0; m[1][1] = y;   m[1][2] = 0.0; m[1][3] = 0.0;
  m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = z;   m[2][3] = 0.0;
  m[3][0] = 0.0; m[3][1] = 0.0; m[3][2] = 0.0; m[3][3] = 1.0;
}

void gen_rotate_matrix_X(const double rad, Mat4x4 m) {
  m[0][0] = 1.0; m[0][1] = 0.0;      m[0][2] = 0.0;       m[0][3] = 0.0;
  m[1][0] = 0.0; m[1][1] = cos(rad); m[1][2] = -sin(rad); m[1][3] = 0.0;
  m[2][0] = 0.0; m[2][1] = sin(rad); m[2][2] = cos(rad);  m[2][3] = 0.0;
  m[3][0] = 0.0; m[3][1] = 0.0;      m[3][2] = 0.0;       m[3][3] = 1.0;
}

void gen_rotate_matrix_Y(const double rad, Mat4x4 m) {
  m[0][0] = cos(rad);  m[0][1] = 0.0; m[0][2] = sin(rad); m[0][3] = 0.0;
  m[1][0] = 0.0;       m[1][1] = 1.0; m[1][2] = 0.0;      m[1][3] = 0.0;
  m[2][0] = -sin(rad); m[2][1] = 0.0; m[2][2] = cos(rad); m[2][3] = 0.0;
  m[3][0] = 0.0;       m[3][1] = 0.0; m[3][2] = 0.0;      m[3][3] = 1.0;
}

void gen_rotate_matrix_Z(const double rad, Mat4x4 m) {
  m[0][0] = cos(rad); m[0][1] = -sin(rad); m[0][2] = 0.0; m[0][3] = 0.0;
  m[1][0] = sin(rad); m[1][1] = cos(rad);  m[1][2] = 0.0; m[1][3] = 0.0;
  m[2][0] = 0.0;      m[2][1] = 0.0;       m[2][2] = 1.0; m[2][3] = 0.0;
  m[3][0] = 0.0;      m[3][1] = 0.0;       m[3][2] = 0.0; m[3][3] = 1.0;
}

void gen_shear_matrix(const double xy, const double xz, const double yx,\
  const double yz, const double zx, const double zy, Mat4x4 m) {
  m[0][0] = 1.0;  m[0][1] = xy;   m[0][2] = xz;   m[0][3] = 0.0;
  m[1][0] = yx;   m[1][1] = 1.0;  m[1][2] = yz;   m[1][3] = 0.0;
  m[2][0] = zx;   m[2][1] = zy;   m[2][2] = 1.0;  m[2][3] = 0.0;
  m[3][0] = 0.0;  m[3][1] = 0.0;  m[3][2] = 0.0;  m[3][3] = 1.0;
}

// TODO: Make these more generic
void mat2x2_reset_to_zero(Mat2x2 mat) {
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            mat[i][j] = 0.0;
        }
    }
}

void mat3x3_reset_to_zero(Mat3x3 mat) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat[i][j] = 0.0;
        }
    }
}

void mat4x4_reset_to_zero(Mat4x4 mat) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            mat[i][j] = 0.0;
        }
    }
}

material create_material_default() {
    material m;
    m.color.x = 1.0; m.color.y = 1.0; m.color.z = 1.0; m.color.w = 0.0;
    m.ambient = 0.1;
    m.diffuse = 0.9;
    m.specular = 0.9;
    m.shininess = 200.0;
    m.reflective = 0.0;
    m.transparency = 0.0;
    m.refractive_index = 1.0;
    m.pattern.to = create_point(1.0, 1.0, 1.0);
    m.pattern.from = create_point(0.0, 0.0, 0.0);
    mat4x4_set_ident(m.pattern.transform);
    m.has_pattern = false;
    return m;
}

shape* create_shape(enum shape_type type) {
    shape* s = (shape*)malloc(sizeof(shape));
    if (!s) { return NULL; }
    s->t = 1.0;
    s->location.x = 0.0;
    s->location.y = 0.0;
    s->location.z = 0.0;
    s->location.w = 0.0;
    mat4x4_set_ident(s->transform);
    s->material = create_material_default();
    s->next = NULL;
    s->type = type;
    return s;
}

void delete_shape(shape* s) {
    free(s);
}

shape* create_glass_sphere() {
    shape* sp = create_shape(SPHERE); // TODO: Might need to make adjustment for SPHERE type
    material mt = create_material_default();
    mt.transparency = 1.0;
    mt.refractive_index = 1.5;
    sp->material = mt;
    return sp;
}

tuple position(ray r, double t) {
    tuple pos = tuple_mult_scalar(r.direction_vector, t); 
    pos = tuple_add(r.origin_point, pos);
    return pos;
}

ray transform(ray* r, Mat4x4 m) {
    ray ray_out = *r;
    mat4x4_mul_tuple(m, r->origin_point, &ray_out.origin_point);
    mat4x4_mul_tuple(m, r->direction_vector, &ray_out.direction_vector);
    return ray_out;
}

void intersect(shape* sp, ray* r, intersections* intersects) {
    assert(sp != NULL);
    assert(r != NULL);
    assert(intersects != NULL);

    Mat4x4 inv_scale_mat;
    mat4x4_set_ident(inv_scale_mat);
    mat4x4_inverse(sp->transform, inv_scale_mat);
    ray r2 = transform(r, inv_scale_mat);

    switch (sp->type) {
    case PLANE:
        if (fabs(r2.direction_vector.y) < EPSILON) {
            return;
        } else {
            double t = (-r2.origin_point.y) / r2.direction_vector.y;
            add_intersection(intersects, t, sp);
            return;
        }
    }

    tuple origin = create_point(0.0, 0.0, 0.0);
    tuple sphere_to_ray = tuple_sub(r2.origin_point, origin);
    double a = tuple_dot(r2.direction_vector, r2.direction_vector);
    double b = 2 * tuple_dot(r2.direction_vector, sphere_to_ray);
    double c = tuple_dot(sphere_to_ray, sphere_to_ray) - 1.0;
    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) { return; }
    double t1 = (-b - sqrt(discriminant)) / (2 * a);
    double t2 = (-b + sqrt(discriminant)) / (2 * a);
    if (t1 < t2) {
        add_intersection(intersects, t1, sp);
        add_intersection(intersects, t2, sp);
    } else {
        add_intersection(intersects, t2, sp);
        add_intersection(intersects, t1, sp);
    }
    return;
}

bool intersects_in_order_test(intersections* intersects); // wanted test to be in debug block

void intersect_world(world* w, ray* r, intersections* intersects) {
    assert(w != NULL);
    assert(r != NULL);
    assert(intersects != NULL);
    shape* sp = w->objects;
    while (sp != NULL) {
        intersect(sp, r, intersects);
        sp = sp->next;
    }
    sort_intersects(intersects);
    assert(intersects_in_order_test(intersects));
    return;
}

void set_transform(shape* sp, Mat4x4 m) {
    mat4x4_copy(m, sp->transform);
}

void set_pattern_transform(pattern* p, Mat4x4 m) {
    mat4x4_copy(m, p->transform);
}

tuple normal_at(shape* shape, tuple point) {
    // local_point <- inverse(shape.transform)*point
    tuple local_point = create_point(0.0, 0.0, 0.0);
    Mat4x4 inverse_shape_transform;
    mat4x4_inverse(shape->transform, inverse_shape_transform);
    mat4x4_mul_tuple(inverse_shape_transform, point, &local_point);

    // local_normal <- local_normal_at(shape, local_point)
    tuple local_normal = { 0.0, 0.0, 0.0, 0.0 };

    // local_normal_at function substitute
    switch (shape->type) {
    case SHAPE:
    case SPHERE:
        local_normal = create_vector(local_point.x, local_point.y, local_point.z);
        break;
    case PLANE:
        local_normal = create_vector(0.0, 1.0, 0.0);
        break;
    default:
        assert(0 && "Shape Type Is Not Set For Shape.");
    }

    // world_normal <- trnaspose(inverse(shape.transform)) * local_normal;
    tuple world_normal;
    mat4x4_transpose(inverse_shape_transform);
    mat4x4_mul_tuple(inverse_shape_transform, local_normal, &world_normal);

    // world_normal.w <- 0
    world_normal.w = 0.0;

    return tuple_normalize(world_normal);
}

tuple tuple_reflect(tuple in, tuple normal) {
    tuple norm_two = tuple_mult_scalar(normal, 2.0);
    double dot_n_normal = tuple_dot(in, normal);
    return tuple_sub(in, tuple_mult_scalar(norm_two, dot_n_normal));
}

world create_world() {
    world w;
    w.lights = (point_light*)malloc(sizeof(point_light));
    w.objects = NULL;
    return w;
}

void add_shape_to_world(shape * sh, world* w) {
    assert(sh != NULL);
    assert(w != NULL);
    shape* next_shape = w->objects;
    if (next_shape == NULL) {
        w->objects = sh;
        return;
    }
    while (next_shape->next != NULL) {
        next_shape = next_shape->next;
    }
    next_shape->next = sh;
}

world create_default_world() {
    world w;
    w.lights = NULL;
    w.objects = NULL;
    tuple light_pos = create_point(-10.0, 10.0, -10.0);
    tuple light_intensity = create_point(1.0, 1.0, 1.0);
    point_light light = create_point_light(light_pos, light_intensity);
    w.lights = (point_light*)malloc(sizeof(point_light));
    assert(w.lights != NULL);
    w.lights->intensity.x = light.intensity.x;
    w.lights->intensity.y = light.intensity.y;
    w.lights->intensity.z = light.intensity.z;
    w.lights->intensity.w = 1.0;

    w.lights->position.x = light.position.x;
    w.lights->position.y = light.position.y;
    w.lights->position.z = light.position.z;
    w.lights->position.w = 1.0;

    w.lights->next = NULL;
    shape* s1 = create_shape(SPHERE);
    tuple material_color = create_point(0.8f, 1.0, 0.6);
    material matl = create_material_default();
    matl.color = material_color;
    matl.diffuse = 0.7;
    matl.specular = 0.2;
    s1->material = matl;
    w.objects = s1;
    w.objects->next = NULL;

    shape* s2 = create_shape(SPHERE);
    gen_scale_matrix(0.5, 0.5, 0.5, s2->transform);
    w.objects->next = s2;
    return w;
}

void free_default_world(world* w) {
    assert(w);
    free(w->objects->next);
    free(w->objects);
    free(w->lights);
}

camera* create_camera(double hsize, double vsize, double field_of_view) {
    double half_view = tan(field_of_view / 2.0);
    double half_height = 0.0;
    double half_width = 0.0;
    double aspect = hsize / vsize;
    if (aspect >= 1) {
        half_width = half_view;
        half_height = half_view / aspect;
    } else {
        half_width = half_view * aspect;
        half_height = half_view;
    }
    double pixel_size = (half_width * 2.0) / hsize;

    camera* c = (camera*)malloc(sizeof(camera));
    if (!c) { return NULL; }
    mat4x4_set_ident(c->view_transform); // render method sets this later
    c->hsize = hsize;
    c->vsize = vsize;
    c->half_height = half_height;
    c->half_width = half_width;
    c->pixel_size = pixel_size;
    c->field_of_view = field_of_view;
    return c;
}

tuple test_pattern_at(struct pattern* pat, tuple* point) {
    return create_point(point->x, point->y, point->z);
}

tuple pattern_at(struct pattern* pat, tuple* point) {
    tuple color = create_point(0.0, 0.0, 0.0);
    tuple distance;
    switch (pat->type) {
    case TEST:
        return create_point(point->x, point->y, point->z);
    case STRIPE:
        if ((int)floor(point->x) % 2 == 0) {
            color.x = pat->from.x;
            color.y = pat->from.y;
            color.z = pat->from.z;
        } else {
            color.x = pat->to.x;
            color.y = pat->to.y;
            color.z = pat->to.z;
        }
        return color;
    case GRADIANT:
        distance = tuple_sub(pat->to, pat->from);
        double fraction = point->x - floor(point->x);
        return tuple_add(pat->from, tuple_mult_scalar(distance, fraction));
    case RING:
        int ring_location = (int)floor(sqrt(point->x * point->x + point->z * point->z)) % 2 == 0;
        if (ring_location) {
            return pat->from;
        }
        return pat->to;
    case CHECKER:
        if ((int)(floor(point->x) + floor(point->y) + floor(point->z)) % 2 == 0) {
            return pat->from;
        }
        return pat->to;
    default:
        assert(false && "Pattern must have a pattern_type");
    }
    return create_point(0.0, 0.0, 0.0);
}

// TODO: Merge these three functions together
pattern test_pattern(tuple from, tuple to) {
    pattern pat;
    pat.from = from;
    pat.to = to;
    pat.pattern_at_fn_ptr = test_pattern_at;
    pat.type = TEST;
    mat4x4_set_ident(pat.transform);
    return pat;
}

pattern stripe_pattern(tuple from, tuple to) {
    pattern pat;
    pat.from = from;
    pat.to = to;
    pat.pattern_at_fn_ptr = pattern_at; /* TODO: Start using these function pointers. */
    pat.type = STRIPE;
    mat4x4_set_ident(pat.transform);
    return pat;
}

pattern gradiant_pattern(tuple from, tuple to) {
    pattern pat;
    pat.from = from;
    pat.to = to;
    pat.pattern_at_fn_ptr = pattern_at; /* TODO: Start using these function pointers. */
    pat.type = GRADIANT;
    mat4x4_set_ident(pat.transform);
    return pat;
}

pattern ring_pattern(tuple from, tuple to) {
    pattern pat;
    pat.from = from;
    pat.to = to;
    pat.pattern_at_fn_ptr = pattern_at; /* TODO: Start using these function pointers. */
    pat.type = RING;
    mat4x4_set_ident(pat.transform);
    return pat;
}

pattern checkers_pattern(tuple from, tuple to) {
    pattern pat;
    pat.from = from;
    pat.to = to;
    pat.pattern_at_fn_ptr = pattern_at; /* TODO: Start using these function pointers. */
    pat.type = CHECKER;
    mat4x4_set_ident(pat.transform);
    return pat;
}

tuple stripe_at_object(pattern pat, shape* sp, tuple* world_point){
    tuple object_point = create_point(0.0, 0.0, 0.0);
    tuple pattern_point = create_point(0.0, 0.0, 0.0);
    Mat4x4 inverse_object_transform;
    mat4x4_inverse(sp->transform, inverse_object_transform);
    mat4x4_mul_tuple(inverse_object_transform, *world_point, &object_point);

    Mat4x4 inverse_pattern_transform;
    mat4x4_inverse(pat.transform, inverse_pattern_transform);
    mat4x4_mul_tuple(inverse_pattern_transform, object_point, &pattern_point);

    return pattern_at(&pat, &pattern_point);
}

tuple lighting(material mat, shape* sh, point_light* light, tuple point, tuple eyev, tuple normalv, bool in_shadow) {

    tuple color_at_point;
    if (mat.has_pattern == true) {
        color_at_point = stripe_at_object(mat.pattern, sh, &point);
    } else {
        color_at_point = mat.color;
    }

    tuple effective_color = tuple_mult_tuple(color_at_point, light->intensity);
    tuple diffuse;
    tuple specular;
    tuple ambient;

    tuple color_black = create_vector(0.0, 0.0, 0.0);
    tuple light_sub_point = tuple_sub(light->position, point);
    tuple lightv = tuple_normalize(light_sub_point);

    ambient = tuple_mult_scalar(effective_color, mat.ambient);

    double light_dot_normal = tuple_dot(lightv, normalv);

    if (light_dot_normal < 0) {
        diffuse = color_black;
        specular = color_black;
    } else {
        diffuse = tuple_mult_scalar( tuple_mult_scalar(effective_color, mat.diffuse), light_dot_normal);

        tuple reflectv = tuple_reflect( tuple_negate(lightv), normalv);
        double reflect_dot_eye = tuple_dot(reflectv, eyev);

        if (reflect_dot_eye <= 0) {
            specular = color_black;
        } else {
            double factor = pow(reflect_dot_eye, mat.shininess);
            specular = tuple_mult_scalar(tuple_mult_scalar(light->intensity, mat.specular), factor);
        }
    }
    if (in_shadow) {
        return ambient;
    } else {
        return tuple_add(tuple_add(ambient, specular), diffuse);
    }
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
    fprintf(fp, "%d %d\n255\n", HORIZONTAL_SIZE, VERTICAL_SIZE);
    for (int col = 0; col < VERTICAL_SIZE; ++col) {
        for (int row = 0; row < HORIZONTAL_SIZE; ++row) {
            int color = color_convert(canvas[row][col].x);
            fprintf(fp, "%d ", color);
            color = color_convert(canvas[row][col].y);
            fprintf(fp, "%d ", color);
            color = color_convert(canvas[row][col].z);
            fprintf(fp, "%d \n", color);
        }
    }
    return 1;
}

comps create_comp() {
    comps comp;
    comp.eyev = create_vector(0.0, 0.0, 0.0);
    comp.normalv = create_vector(0.0, 0.0, 0.0);
    comp.object = NULL;
    comp.point = create_point(0.0, 0.0, 0.0);
    comp.t = 0.0;
    return comp;
}

comps prepare_computations(intersection* hit, ray* r, intersections* xs) {
    comps comps = create_comp();
    comps.t = hit->t;
    comps.object = hit->object_id;
    comps.point = position(*r, comps.t);
    comps.eyev = tuple_negate(r->direction_vector);
    comps.normalv = normal_at(comps.object, comps.point);
    if (tuple_dot(comps.normalv, comps.eyev) < 0.0f) {
        comps.inside = true;
        comps.normalv = tuple_negate(comps.normalv);
    }
    else {
        comps.inside = false;
    }
    comps.over_point = tuple_add(comps.point, tuple_mult_scalar(comps.normalv, EPSILON));
    comps.under_point = tuple_sub(comps.point, tuple_mult_scalar(comps.normalv, EPSILON));
    comps.reflectv = tuple_reflect(r->direction_vector, comps.normalv);

    // refraction section
    containers_clear(&contents);
    for (int count = 0; count < xs->count; count++) {
        intersection* i = &xs->itersection[count];
        if (i->t == hit->t) {
            if (containers_is_empty(&contents)) {
                comps.n1 = 1.0f;
            }
            else {
                comps.n1 = container_last(&contents)->material.refractive_index;
            }
        }
        if (containers_includes(&contents, i->object_id)) {
            container_remove(&contents, i->object_id);
        }
        else {
            container_append(&contents, i->object_id);
        }

        if (i->t == hit->t) {
            if (containers_is_empty(&contents)) {
                comps.n2 = 1.0f;
            }
            else {
                comps.n2 = container_last(&contents)->material.refractive_index;
            }
            break;
        }
    }
    return comps;
}

bool is_shadowed(world* world, tuple* point) {
    tuple v = tuple_sub(world->lights->position, *point);
    double distance = tuple_mag_vec(v);
    tuple direction = tuple_normalize(v);
    ray r = create_ray(point->x, point->y, point->z, direction.x, direction.y, direction.z);
    intersections inter = create_intersections();
    intersect_world(world, &r, &inter);
    intersection* h = hit(&inter);
    if (h != NULL && h->t < distance) {
        return true;
    }
    return false;
}

tuple shade_hit(world* w, comps* comp, int remaining);

tuple color_at(world* w, ray* r, int remaining) {
    assert(w);
    assert(r);
    intersections inter = create_intersections();
    intersect_world(w, r, &inter);
    assert(intersects_in_order_test(&inter));
    intersection* hit1 = hit(&inter);
    if (hit1 == NULL) {
        return create_point(0.0, 0.0, 0.0);
    } else {
        comps comp = prepare_computations(hit1, r, &inter);
        return shade_hit(w, &comp, remaining);
    }
}

// spawns new ray for reflection
tuple reflected_color(world* w, comps* comp, int remaining) {
    if(remaining < 1 || equal(comp->object->material.reflective,0.0)) {
        return create_point(0.0, 0.0, 0.0);
    }
    ray reflect_ray = create_ray(comp->over_point.x, comp->over_point.y, comp->over_point.z, comp->reflectv.x, comp->reflectv.y, comp->reflectv.z);
    remaining -= 1;
    tuple color = color_at(w, &reflect_ray, remaining);
    return tuple_mult_scalar(color, comp->object->material.reflective);
}

tuple refracted_color(world* w, comps* comp, int remaining) {
    double n_ratio = comp->n1 / comp->n2;
    double cos_i = tuple_dot(comp->eyev, comp->normalv);
    double sin2_t = pow(n_ratio,2) * (1.0 - pow(cos_i,2));
    if (remaining <= 0 || equal(comp->object->material.transparency,0.0) || sin2_t > 1.0) {
        return create_point(0.0, 0.0, 0.0);
    }
    double cos_t = sqrt(1.0 - sin2_t);
    double n_ratio_cos = n_ratio * cos_i - cos_t;
    tuple eyev_n_ratio = tuple_mult_scalar(comp->eyev, n_ratio);
    tuple direction = tuple_sub( tuple_mult_scalar(comp->normalv, n_ratio_cos), eyev_n_ratio);
    ray refract_ray = create_ray(comp->under_point.x, comp->under_point.y, comp->under_point.z, direction.x, direction.y, direction.z);
    tuple color_at_value = color_at(w, &refract_ray, remaining - 1);
    tuple final_color = tuple_mult_scalar(color_at_value,comp->object->material.transparency);
    return final_color;
}

double schlick(comps* comp) {
    double cosine = tuple_dot(comp->eyev, comp->normalv);
    if (comp->n1 > comp->n2) {
        double n = comp->n1 / comp->n2;
        double sin2_t = pow(n,2) * (1.0 - pow(cosine,2));
        if (sin2_t > 1.0) {
            return 1.0;
        }
        double cos_t = sqrt(1.0 - sin2_t);
        cosine = cos_t;
    }
    double r0 = pow(((comp->n1 - comp->n2) / (comp->n1 + comp->n2)), 2);
    return r0 + (1.0 - r0) * pow((1.0 - cosine), 5);
}

tuple shade_hit(world* w, comps* comp, int remaining) {
    assert(w->lights);
    assert(w->objects);
    bool shadowed = is_shadowed(w, &comp->over_point);
    tuple surface = lighting(comp->object->material,w->objects, w->lights, comp->over_point, comp->eyev, comp->normalv, shadowed);
    tuple reflected = reflected_color(w, comp, remaining);
    tuple refracted = refracted_color(w, comp, remaining);

    material mater = comp->object->material;
    if (mater.reflective > 0.0 && mater.transparency > 0.0) {
        double reflectance = schlick(comp);
        return tuple_add(surface, tuple_add(tuple_mult_scalar(reflected, reflectance),tuple_mult_scalar(refracted, (1.0 - reflectance))));
    }
    return tuple_add(tuple_add(surface, reflected), refracted);
}

void view_transform(tuple from, tuple to, tuple up, Mat4x4 m) {
    tuple forward = tuple_normalize(tuple_sub(to, from));
    tuple upn = tuple_normalize(up);
    tuple left = tuple_cross(forward, upn);
    tuple true_up = tuple_cross(left, forward);
    Mat4x4 orientation;

    orientation[0][0] = left.x;
    orientation[1][0] = true_up.x;
    orientation[2][0] = -forward.x;
    orientation[3][0] = 0.0;

    orientation[0][1] = left.y;
    orientation[1][1] = true_up.y;
    orientation[2][1] = -forward.y;
    orientation[3][1] = 0.0;

    orientation[0][2] = left.z;
    orientation[1][2] = true_up.z;
    orientation[2][2] = -forward.z;
    orientation[3][2] = 0.0;

    orientation[0][3] = 0.0;
    orientation[1][3] = 0.0;
    orientation[2][3] = 0.0;
    orientation[3][3] = 1.0;

    Mat4x4 translate;
    gen_translate_matrix(-from.x, -from.y, -from.z, translate);

    mat4x4_mul_in_place(orientation, translate, m);
    return;
}

ray ray_for_pixel(camera* camera, double px, double py) {
    
    // the offset from the edge of the canvas tp the pixel's center
    double x_offset = camera->pixel_size * (px + 0.5);
    double y_offset = camera->pixel_size * (py + 0.5);

    // the untransformed coordinates of the pixel in world space
    // (remember that the camera looks toward -z sor +x is to the **left**)
    double world_x = camera->half_width - x_offset;
    double world_y = camera->half_height - y_offset;

    // using the camera matrix, transform the canvas point and the origin,
    // and then compute the ray's direction vector.
    // remeber that the canvas is at x=-1

    // pixel
    tuple pixel = create_point(0.0, 0.0, 0.0);
    Mat4x4 inverse;
    mat4x4_inverse(camera->view_transform, inverse);
    tuple point = create_point(world_x, world_y, -1);
    mat4x4_mul_tuple(inverse, point, &pixel);

    // origin
    tuple origin = create_point(0.0, 0.0, 0.0);
    tuple temp = create_point(0.0, 0.0, 0.0);
    Mat4x4 inverse2;
    mat4x4_set_ident(inverse2);
    mat4x4_inverse(camera->view_transform, inverse2);
    mat4x4_mul_tuple(inverse2, temp, &origin);

    //direction
    tuple direction = tuple_normalize(tuple_sub(pixel, origin));
    
    ray r = create_ray(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    r.direction_vector = direction;
    r.origin_point = origin;
    return r;
}

void render(camera* c, world* w) {
//#pragma omp parallel shared(c,w)
    for (int y = 0; y < VERTICAL_SIZE; ++y) {
        for (int x = 0; x < HORIZONTAL_SIZE; ++x) {
            ray r = ray_for_pixel(c, x, y);

            //printf("[%d][%d] o.x=%4.1f o.y=%4.1f o.z=%4.1f d.x=%4.1f d.y=%4.1f d.z=%4.1f\n", y, x, r.origin_point.x, r.origin_point.y, r.origin_point.z, r.direction_vector.x, r.direction_vector.y, r.direction_vector.z);

            tuple color = color_at(w, &r, RECURSION_DEPTH);
            //color.x = r.direction_vector.x;
            //color.y = r.direction_vector.x;
            //color.z = r.direction_vector.x;
            write_pixel(x, y, color);
        }
    }
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
  tuple t = create_point(4.0, -4.0, 3.0);
  assert(equal(t.x, 4.0));
  assert(equal(t.y, -4.0));
  assert(equal(t.z, 3.0));
  assert(equal(t.w, 1.0));
  return 0;
}

// 4 creates tuples with w=0
int create_vector_test() {
  tuple t = create_vector(4.0, -4.0, 3.0);
  assert(equal(t.x, 4.0));
  assert(equal(t.y, -4.0));
  assert(equal(t.z, 3.0));
  assert(equal(t.w, 0.0));
  return 0;
}

// 4 A tuple with w=1.0 is a point
int tuple_with_W_0_is_point_test()
{
  tuple a = { 4.3, -4.2, 3.1, 1.0 };
  assert(equal(a.x,  4.3));
  assert(equal(a.y, -4.2));
  assert(equal(a.z,  3.1));
  assert(equal(a.w,  1.0));
  assert(tuple_is_point(a)  == true);
  assert(tuple_is_vector(a) == false);

  tuple b = { 4.3, -4.2, 3.1, 0.0 };
  assert(equal(b.x,  4.3));
  assert(equal(b.y, -4.2));
  assert(equal(b.z,  3.1));
  assert(equal(b.w,  0.0));
  assert(tuple_is_point(b)  == false);
  assert(tuple_is_vector(b) == true);
  return 0;
}

// 6 Adding two tuples
int tuple_add_test() {
  tuple a = { 3.0, -2.0, 5.0, 1.0 };
  tuple b = { -2.0, 3.0, 1.0, 0.0 };
  tuple c = tuple_add(a, b);
  assert(equal(c.x, 1.0));
  assert(equal(c.y, 1.0));
  assert(equal(c.z, 6.0));
  assert(equal(c.w, 1.0));
  return 0;
}

// 6 Subtracting two points
int tuple_sub_test() {
  tuple a = { 3.0, 2.0, 1.0 };
  tuple b = { 5.0, 6.0, 7.0 };
  tuple c = tuple_sub(a, b);
  assert(equal(c.x, -2.0));
  assert(equal(c.y, -4.0));
  assert(equal(c.z, -6.0));
  assert(tuple_is_vector(c) == true);
  return 0;
}

// 6 Subtracting vector from a point
int subtract_vector_from_point_test() {
  tuple pt = create_point(3.0, 2.0, 1.0);
  tuple vec = create_vector(5.0, 6.0, 7.0);
  tuple ans = tuple_sub(pt, vec);
  assert(equal(ans.x, -2.0));
  assert(equal(ans.y, -4.0));
  assert(equal(ans.z, -6.0));
  return 0;
}

// 7 Subtracting two vectors
int subtract_two_vectors_test() {
  tuple vec1 = create_vector(3.0, 2.0, 1.0);
  tuple vec2 = create_vector(5.0, 6.0, 7.0);
  tuple vec3 = tuple_sub(vec1, vec2);
  assert(equal(vec3.x, -2.0));
  assert(equal(vec3.y, -4.0));
  assert(equal(vec3.z, -6.0));
  assert(tuple_is_vector(vec3) == true);
  return 0;
}

// 7 Subtracting a vector from zero vector
int subtract_vector_from_zero_vector_test() {
  tuple zero = create_vector(0.0, 0.0, 0.0);
  tuple vec1 = create_vector(1.0, -2.0, 3.0);
  tuple vec2 = tuple_sub(zero, vec1);
  assert(equal(vec2.x, -1.0));
  assert(equal(vec2.y,  2.0));
  assert(equal(vec2.z, -3.0));
  assert(tuple_is_vector(vec2) == true);
  return 0;
}

// 7 Negating a tuple
int negating_tuple_test() {
  tuple vec1 = { 1.0, -2.0, 3.0, -4.0 };
  vec1 = tuple_negate(vec1);
  assert(equal(vec1.x, -1.0));
  assert(equal(vec1.y,  2.0));
  assert(equal(vec1.z, -3.0));
  assert(equal(vec1.w, 4.0));
  return 0;
}

// 8 Multiply tuple by a scalar
int tuple_mult_scalar_test() {
  tuple vec1 = { 1.0, -2.0, 3.0, -4.0 };
  double scalar = 3.5;
  vec1 = tuple_mult_scalar(vec1, scalar);
  assert(equal(vec1.x,   3.5));
  assert(equal(vec1.y,  -7.0));
  assert(equal(vec1.z,  10.5));
  assert(equal(vec1.w, -14.0));
  return 0;
}

// 8 Multiply tuple by a fraction
int tuple_mult_scalar_fraction_test() {
  tuple vec1 = { 1.0, -2.0, 3.0, -4.0 };
  double scalar = 0.5;
  vec1 = tuple_mult_scalar(vec1, scalar);
  assert(equal(vec1.x, 0.5));
  assert(equal(vec1.y, -1.0));
  assert(equal(vec1.z, 1.5));
  assert(equal(vec1.w, -2.0));
  return 0;
}

// 8 Divide a tuple by a scalar
int tuple_div_scalar_test() {
  tuple vec1 = { 1.0, -2.0, 3.0, -4.0 };
  double scalar = 2.0;
  vec1 = tuple_div_scalar(vec1, scalar);
  assert(equal(vec1.x, 0.5));
  assert(equal(vec1.y, -1.0));
  assert(equal(vec1.z, 1.5));
  assert(equal(vec1.w, -2.0));
  return 0;
}

// 8 Computing the magnitude of vector(1, 0, 0)
int tuple_mag_vec_test() {
  tuple vec1 = create_vector(1.0, 0.0, 0.0);
  double mag = tuple_mag_vec(vec1);
  assert(equal(mag, 1.0));

  tuple vec2 = create_vector(0.0, 1.0, 0.0);
  mag = tuple_mag_vec(vec2);
  assert(equal(mag, 1.0));

  tuple vec3 = create_vector(0.0, 0.0, 1.0);
  mag = tuple_mag_vec(vec3);
  assert(equal(mag, 1.0));

  tuple vec4 = create_vector(1.0, 2.0, 3.0);
  mag = tuple_mag_vec(vec4);
  assert(equal(mag, sqrt(14.0)));

  tuple vec5 = create_vector(-1.0, -2.0, -3.0);
  mag = tuple_mag_vec(vec5);
  assert(equal(mag, sqrt(14.0)));
  return 0;
}

// 10 Normalizing vector(4,0,0) gives (1,0,0)
int vec_norm_test() {
  tuple vec1 = create_vector(4.0, 0.0, 0.0);
  tuple norm = tuple_normalize(vec1);
  assert(equal(norm.x, 1.0));
  assert(equal(norm.y, 0.0));
  assert(equal(norm.z, 0.0));

  tuple vec2 = create_vector(1.0, 2.0, 3.0);
  norm = tuple_normalize(vec2);
  double ans1 = 1 / sqrt(14);
  double ans2 = 2 / sqrt(14);
  double ans3 = 3 / sqrt(14);
  assert(equal(norm.x, ans1));
  assert(equal(norm.y, ans2));
  assert(equal(norm.z, ans3));

  tuple vec3 = create_vector(1.0, 2.0, 3.0);
  norm = tuple_normalize(vec3);
  double mag = tuple_mag_vec(norm);
  assert(equal(mag, 1.0));
  return 0;
}

// 10 dot rpoduct of two tuples
int dot_prod_test() {
  tuple vec1 = create_vector(1.0, 2.0, 3.0);
  tuple vec2 = create_vector(2.0, 3.0, 4.0);
  double dot_prod = tuple_dot(vec1, vec2);
  assert(equal(dot_prod, 20.0));
  return 0;
}

// 11 cross product of two vectors
int cross_prod_test() {
  tuple vec1 = create_vector(1.0, 2.0, 3.0);
  tuple vec2 = create_vector(2.0, 3.0, 4.0);
  tuple cross1 = tuple_cross(vec1, vec2);
  assert(equal(cross1.x, -1.0));
  assert(equal(cross1.y,  2.0));
  assert(equal(cross1.z, -1.0));
  assert(equal(cross1.w,  0.0));
  tuple cross2 = tuple_cross(vec2, vec1);
  assert(equal(cross2.x,  1.0));
  assert(equal(cross2.y, -2.0));
  assert(equal(cross2.z,  1.0));
  assert(equal(cross2.w,  0.0));
  return 0;
}

// 18 Hadamard product
int hadamard_product_test() {
  tuple col1 = create_vector(1.0, 0.2, 0.4);
  tuple col2 = create_vector(0.9, 1.0, 0.1);
  tuple col3 = hadamard_product(col1, col2);
  assert(equal(col3.x, 0.899999976));
  assert(equal(col3.y, 0.2));
  assert(equal(col3.z, 0.04));
  assert(equal(col3.w, 1.0));
  return 0;
}

int write_pixel_test() {
  tuple red = create_vector(1.0, 0.0, 0.0);
  write_pixel(0, 0, red);

  // horizonatal axis
  tuple green = create_vector(0.0, 1.0, 0.0);
  write_pixel(0, 1, green);

  tuple blue = create_vector(0.0, 0.0, 1.0);
  write_pixel(0, 2, blue);

  // vertical axis
  tuple sky = create_vector(0.3, 0.6, 0.9);
  write_pixel(1, 1, sky);

  tuple orange = create_vector(1.0, 0.5, 0.25);
  write_pixel(1, 2, orange);

  assert(equal(canvas[0][0].x, 1.0));
  assert(equal(canvas[0][0].y, 0.0));
  assert(equal(canvas[0][0].z, 0.0));

  assert(equal(canvas[0][1].x, 0.0));
  assert(equal(canvas[0][1].y, 1.0));
  assert(equal(canvas[0][1].z, 0.0));

  assert(equal(canvas[0][2].x, 0.0));
  assert(equal(canvas[0][2].y, 0.0));
  assert(equal(canvas[0][2].z, 1.0));

  assert(equal(canvas[1][1].x, 0.3));
  assert(equal(canvas[1][1].y, 0.6));
  assert(equal(canvas[1][1].z, 0.9));

  assert(equal(canvas[1][2].x, 1.0));
  assert(equal(canvas[1][2].y, 0.5));
  assert(equal(canvas[1][2].z, 0.25));
  return 0;
}

int color_convert_test() {
  int color = color_convert(0.0);
  assert(color == 0);
  color = color_convert(0.5);
  assert(color == 127);
  color = color_convert(1.0);
  assert(color == 255);
  color = color_convert(2.0);
  assert(color == 255);
  color = color_convert(-2.0);
  assert(color == 0);
  return 0;
}

int mat_equal_test() {
  double old_value;
  Mat2x2 mat2x2a = { { 0.0, 1.0 }, { 2.0, 3.0 } };
  Mat2x2 mat2x2b = { { 0.0, 1.0 }, { 2.0, 3.0 } };

  bool test1 = mat2x2_equal(mat2x2a, mat2x2b);
  assert(true == test1);

  // set each element of the first array different one at a time.
  // verify that method catches it
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      old_value = mat2x2a[i][j];
      mat2x2a[i][j] = 9.0;
      test1 = mat2x2_equal(mat2x2a, mat2x2b);
      assert(false == test1);
      mat2x2a[i][j] = old_value;
    }
  }

  Mat3x3 mat3x3a = { { 0.0, 1.0, 2.0 }, { 3.0, 4.0, 5.0 }, { 6.0, 7.0, 8.0 } };
  Mat3x3 mat3x3b = { { 0.0, 1.0, 2.0 }, { 3.0, 4.0, 5.0 }, { 6.0, 7.0, 8.0 } };
  bool test2 = mat3x3_equal(mat3x3a, mat3x3b);
  assert(true == test2);

  // set each element of the first array different one at a time.
  // verify that method catches it
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      old_value = mat3x3a[i][j];
      mat3x3a[i][j] = 9.0;
      test2 = mat3x3_equal(mat3x3a, mat3x3b);
      assert(false == test2);
      mat3x3a[i][j] = old_value;
    }
  }

  Mat4x4 mat4x4a = { { 0.0, 1.0, 2.0 }, { 3.0, 4.0, 5.0 },\
    { 6.0, 7.0, 8.0 }, { 9.0, 10.0, 11.0 } };
  Mat4x4 mat4x4b = { { 0.0, 1.0, 2.0 }, { 3.0, 4.0, 5.0 },\
    { 6.0, 7.0, 8.0 }, { 9.0, 10.0, 11.0 } };
  bool test3 = mat4x4_equal(mat4x4a, mat4x4b);
  assert(true == test3);

  // set each element of the first array different one at a time.
  // verify that method catches it
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      old_value = mat4x4a[i][j];
      mat4x4a[i][j] = 12.0;
      test3 = mat4x4_equal(mat4x4a, mat4x4b);
      assert(false == test3);
      mat4x4a[i][j] = old_value;
    }
  }
  return 0;
}

int mat4x4_mul_test() {
  Mat4x4 a = { { 1.0, 2.0, 3.0, 4.0 }, { 5.0, 6.0, 7.0, 8.0 },\
    { 9.0, 8.0, 7.0, 6.0 }, { 5.0, 4.0, 3.0, 2.0 } };
  Mat4x4 b = { { -2.0, 1.0, 2.0, 3.0 }, { 3.0, 2.0, 1.0, -1.0 },\
    { 4.0, 3.0, 6.0, 5.0 }, { 1.0, 2.0, 7.0, 8.0 } };
  Mat4x4 m;

  mat4x4_mul_in_place(a, b, m);
  assert(equal(m[0][0],  20.0));
  assert(equal(m[0][1],  22.0));
  assert(equal(m[0][2],  50.0));
  assert(equal(m[0][3],  48.0));

  assert(equal(m[1][0],  44.0));
  assert(equal(m[1][1],  54.0));
  assert(equal(m[1][2], 114.0));
  assert(equal(m[1][3], 108.0));

  assert(equal(m[2][0],  40.0));
  assert(equal(m[2][1],  58.0));
  assert(equal(m[2][2], 110.0));
  assert(equal(m[2][3], 102.0));

  assert(equal(m[3][0],  16.0));
  assert(equal(m[3][1],  26.0));
  assert(equal(m[3][2],  46.0));
  assert(equal(m[3][3],  42.0));

  mat4x4_mul_in_place(b, a, m);

  assert(equal(m[0][0], 36.0));
  assert(equal(m[0][1], 30.0));
  assert(equal(m[0][2], 24.0));
  assert(equal(m[0][3], 18.0));

  assert(equal(m[1][0], 17.0));
  assert(equal(m[1][1], 22.0));
  assert(equal(m[1][2], 27.0));
  assert(equal(m[1][3], 32.0));

  assert(equal(m[2][0], 98.0));
  assert(equal(m[2][1], 94.0));
  assert(equal(m[2][2], 90.0));
  assert(equal(m[2][3], 86.0));

  assert(equal(m[3][0], 114.0));
  assert(equal(m[3][1], 102.0));
  assert(equal(m[3][2], 90.0));
  assert(equal(m[3][3], 78.0));

  return 0;
}

int mat4x4_mul_in_place_test() {
    Mat4x4 a = { { 1.0, 2.0, 3.0, 4.0 }, { 5.0, 6.0, 7.0, 8.0 },\
      { 9.0, 8.0, 7.0, 6.0 }, { 5.0, 4.0, 3.0, 2.0 } };
    Mat4x4 b = { { -2.0, 1.0, 2.0, 3.0 }, { 3.0, 2.0, 1.0, -1.0 },\
      { 4.0, 3.0, 6.0, 5.0 }, { 1.0, 2.0, 7.0, 8.0 } };

    mat4x4_mul_in_place(a, b, b);

    assert(equal(a[0][0], 1.0));
    assert(equal(a[0][1], 2.0));
    assert(equal(a[0][2], 3.0));
    assert(equal(a[0][3], 4.0));
    assert(equal(a[1][0], 5.0));
    assert(equal(a[1][1], 6.0));
    assert(equal(a[1][2], 7.0));
    assert(equal(a[1][3], 8.0));
    assert(equal(a[2][0], 9.0));
    assert(equal(a[2][1], 8.0));
    assert(equal(a[2][2], 7.0));
    assert(equal(a[2][3], 6.0));
    assert(equal(a[3][0], 5.0));
    assert(equal(a[3][1], 4.0));
    assert(equal(a[3][2], 3.0));
    assert(equal(a[3][3], 2.0));

    assert(equal(b[0][0], 20.0));
    assert(equal(b[0][1], 22.0));
    assert(equal(b[0][2], 50.0));
    assert(equal(b[0][3], 48.0));
    assert(equal(b[1][0], 44.0));
    assert(equal(b[1][1], 54.0));
    assert(equal(b[1][2], 114.0));
    assert(equal(b[1][3], 108.0));
    assert(equal(b[2][0], 40.0));
    assert(equal(b[2][1], 58.0));
    assert(equal(b[2][2], 110.0));
    assert(equal(b[2][3], 102.0));
    assert(equal(b[3][0], 16.0));
    assert(equal(b[3][1], 26.0));
    assert(equal(b[3][2], 46.0));
    assert(equal(b[3][3], 42.0));
    return 0;
}

// 30 Matrix multipled by a tuple
int mat4x4_mul_tuple_test() {
  Mat4x4 a = { { 1.0, 2.0, 3.0, 4.0 }, { 2.0, 4.0, 4.0, 2.0 },\
    { 8.0, 6.0, 4.0, 1.0 }, { 0.0, 0.0, 0.0, 1.0 } };
  tuple b = create_point(1.0, 2.0, 3.0);
  tuple c = create_point(0.0, 0.0, 0.0);
  mat4x4_mul_tuple(a, b, &c);
  assert(equal(c.x, 18.0));
  assert(equal(c.y, 24.0));
  assert(equal(c.z, 33.0));
  assert(equal(c.w,  1.0));
  return 0;
}

// 32 Multiply matrix by identity matrix
int mat4x4_mult_ident_test() {
  Mat4x4 ident = { { 1.0, 0.0, 0.0, 0.0 },{ 0.0, 1.0, 0.0, 0.0 },\
    { 0.0, 0.0, 1.0, 0.0}, { 0.0, 0.0, 0.0, 1.0 } };
  Mat4x4 a = { { 0.0, 1.0, 2.0, 4.0 }, { 1.0, 2.0, 4.0, 8.0 },\
    { 2.0, 4.0, 8.0, 16.0 }, { 4.0, 8.0, 16.0, 32.0 } };  
  Mat4x4 b = { { 0.0, 1.0, 2.0, 4.0 }, { 1.0, 2.0, 4.0, 8.0 },\
    { 2.0, 4.0, 8.0, 16.0 }, { 4.0, 8.0, 16.0, 32.0 } };
  mat4x4_mul_in_place(a, ident, a);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      assert(equal(a[i][j], b[i][j]));
    }
  }
  return 0;
}

// 33 Transpose a matrix
int mat4x4_transpose_test() {
  Mat4x4 a = { { 0.0, 9.0, 3.0, 0.0 },{ 9.0, 8.0, 0.0, 8.0 },\
    { 1.0, 8.0, 5.0, 3.0}, { 0.0, 0.0, 5.0, 8.0 } };
  Mat4x4 b = { { 0.0, 9.0, 1.0, 0.0 },{ 9.0, 8.0, 8.0, 0.0 },\
    { 3.0, 0.0, 5.0, 5.0}, { 0.0, 8.0, 3.0, 8.0 } };
  mat4x4_transpose(a);
  assert(mat4x4_equal(a, b));
  return 0;
}

// 34 Calculating the determinant of a 2x2 matrix
int mat2x2_det_test() {
  Mat2x2 a = { { 1.0, 5.0 },{ -3.0, 2.0 } };
  double det = mat2x2_det(a);
  assert(equal(det, 17.0));

  Mat2x2 b = { { 5.0, 0.0 },{ -1.0, 5.0 } };
  det = mat2x2_det(b);
  assert(equal(det, 25.0));

  mat2x2_reset_to_zero(b);
  det = mat2x2_det(b);
  assert(equal(det, 0.0));

  Mat2x2 c = { { 1.0, 0.0 },{ 0.0, -1.0 } };
  det = mat2x2_det(c);
  assert(equal(det, -1.0));

  Mat2x2 d = { { -1.0, -1.0 },{ -1.0, -1.0 } };
  det = mat2x2_det(d);
  assert(equal(det, 0.0));

  Mat2x2 e = { { 1.0, 2.0 },{ 3.0, 4.0 } };
  det = mat2x2_det(e);
  assert(equal(det, -2.0));
  return 0;
}

// 35 Submatrix of 3x3 matrix is a 2x2 matrix
int mat3x3_submat_2x2_test() {
  Mat3x3 a = { { 1.0, 2.0, 3.0 },{ 4.0, 5.0, 6.0 },{ 7.0, 8.0, 9.0 } };
  Mat2x2 b = { { 0.0, 0.0 },{ 0.0, 0.0 } };
  mat3x3_submat2x2(a, b, 0, 0);
  assert(equal(b[0][0], 5.0));
  assert(equal(b[0][1], 6.0));
  assert(equal(b[1][0], 8.0));
  assert(equal(b[1][1], 9.0));

  mat2x2_reset_to_zero(b);
  mat3x3_submat2x2(a, b, 0, 2);
  assert(equal(b[0][0], 4.0));
  assert(equal(b[0][1], 5.0));
  assert(equal(b[1][0], 7.0));
  assert(equal(b[1][1], 8.0));

  mat2x2_reset_to_zero(b);
  mat3x3_submat2x2(a, b, 1, 1);
  assert(equal(b[0][0], 1.0));
  assert(equal(b[0][1], 3.0));
  assert(equal(b[1][0], 7.0));
  assert(equal(b[1][1], 9.0));
  return 0;
}

// 35 Submatrix of 4x4 matrix is a 3x3 matrix
int mat4x4_submat_3x3_test() {
  Mat4x4 a = { { 1.0, 2.0, 3.0, 4.0 },{ 5.0, 6.0, 7.0, 8.0 },\
    { 9.0, 10.0, 11.0, 12.0},{ 13.0, 14.0, 15.0, 16.0 } };
  Mat3x3 b = { { 0.0, 0.0, 0.0 },{ 0.0, 0.0, 0.0 },{ 0.0, 0.0, 0.0 } };

  mat4x4_submat3x3(a, b, 0, 0);
  assert(equal(b[0][0], 6.0));
  assert(equal(b[0][1], 7.0));
  assert(equal(b[0][2], 8.0));
  assert(equal(b[1][0], 10.0));
  assert(equal(b[1][1], 11.0));
  assert(equal(b[1][2], 12.0));
  assert(equal(b[2][0], 14.0));
  assert(equal(b[2][1], 15.0));
  assert(equal(b[2][2], 16.0));

  mat3x3_reset_to_zero(b);
  mat4x4_submat3x3(a, b, 2, 1);
  assert(equal(b[0][0],  1.0));
  assert(equal(b[0][1],  3.0));
  assert(equal(b[0][2],  4.0));
  assert(equal(b[1][0],  5.0));
  assert(equal(b[1][1],  7.0));
  assert(equal(b[1][2],  8.0));
  assert(equal(b[2][0], 13.0));
  assert(equal(b[2][1], 15.0));
  assert(equal(b[2][2], 16.0));

  mat3x3_reset_to_zero(b);
  mat4x4_submat3x3(a, b, 3, 3);
  assert(equal(b[0][0], 1.0));
  assert(equal(b[0][1], 2.0));
  assert(equal(b[0][2], 3.0));
  assert(equal(b[1][0], 5.0));
  assert(equal(b[1][1], 6.0));
  assert(equal(b[1][2], 7.0));
  assert(equal(b[2][0], 9.0));
  assert(equal(b[2][1], 10.0));
  assert(equal(b[2][2], 11.0));
  return 0;
}

// 35 Calculating a minor of a 3x3 matrix
int mat3x3_minor_test() {
  Mat3x3 a = { { 3.0, 5.0, 0.0 },{ 2.0, -1.0, -7.0 },{ 6.0, -1.0, 5.0 } };
  double minor = mat3x3_minor(a, 1, 0);
  assert(equal(minor, 25.0));
  return 0;
}

// 36 Calculating a cofactor of a 3x3 matrix
int mat3x3_cofactor_test() {
  Mat3x3 a = { { 3.0, 5.0, 0.0 },{ 2.0, -1.0, -7.0 },{ 6.0, -1.0, 5.0 } };
  double minor = mat3x3_minor(a, 0, 0);
  assert(equal(minor, -12.0));

  double cofactor = mat3x3_cofactor(a, 0, 0);
  assert(equal(cofactor, -12.0));

  minor = mat3x3_minor(a, 1, 0);
  assert(equal(minor, 25.0));

  cofactor = mat3x3_cofactor(a, 1, 0);
  assert(equal(cofactor, -25.0));
  return 0;
}

// 37 Calculating the determinant of a 3x3 matrix
int mat3x3_det_test() {
  Mat3x3 a = { { 1.0, 2.0, 6.0 },{ -5.0, 8.0, -4.0 },{ 2.0, 6.0, 4.0 } };
  double cofactor = mat3x3_cofactor(a, 0, 0);
  assert(equal(cofactor, 56.0));

  cofactor = mat3x3_cofactor(a, 0, 1);
  assert(equal(cofactor, 12.0));

  cofactor = mat3x3_cofactor(a, 0, 2);
  assert(equal(cofactor, -46.0));

  double det = mat3x3_det(a);
  assert(equal(det, -196.0));
  return 0;
}

// 37 Calculating the determinant of a 4x4 matrix
int mat4x4_det_test() {
  Mat4x4 a = { { -2.0, -8.0, 3.0, 5.0 },{ -3.0, 1.0, 7.0, 3.0 },\
    { 1.0, 2.0, -9.0, 6.0},{ -6.0, 7.0, 7.0, -9.0 } };

  double cofactor = mat4x4_cofactor(a, 0, 0);
  assert(equal(cofactor, 690.0));
  cofactor = mat4x4_cofactor(a, 0, 1);
  assert(equal(cofactor, 447.0));
  cofactor = mat4x4_cofactor(a, 0, 2);
  assert(equal(cofactor, 210.0));
  cofactor = mat4x4_cofactor(a, 0, 3);
  assert(equal(cofactor, 51.0));
  double det = mat4x4_det(a, 4);
  assert(equal(det, -4071.0));
  return 0;
}

// 39 Testing an invertable matrix for invertability
int invertable_matrix_test() {
  Mat4x4 a = { { 6.0, 4.0, 4.0, 4.0 },{ 5.0, 5.0, 7.0, 6.0 },\
    { 4.0, -9.0, 3.0, -7.0},{ 9.0, 1.0, 7.0, -6.0 } };
  bool inv = mat4x4_invertable(a);
  assert(inv == true);

  Mat4x4 b = { { -4.0, 2.0, -2.0, -3.0 },{ 9.0, 6.0, 2.0, 6.0 },\
    { 0.0, -5.0, 1.0, -5.0},{ 0.0, 0.0, 0.0, 0.0 } };
  inv = mat4x4_invertable(b);
  assert(inv == false);
  return 0;
}

// 39 Calculating the inverse of a matrix
int inverse_matrix_test() {
  Mat4x4 a = { { -5.0, 2.0, 6.0, -8.0 },{ 1.0, -5.0, 1.0, 8.0 },\
      { 7.0, 7.0, -6.0, -7.0},{ 1.0, -3.0, 7.0, 4.0 } };
  Mat4x4 b = { { 0.0, 0.0, 0.0, 0.0 },{ 0.0, 0.0, 0.0, 0.0 },\
    { 0.0, 0.0, 0.0, 0.0 },{ 0.0, 0.0, 0.0, 0.0 } };
  Mat4x4 c = { { 0.21804512, 0.45112783, 0.24060151, -0.04511278f },{ -0.80827069, -1.45676696, -0.44360903, 0.52067667 },\
    { -0.07894737, -0.22368421, -0.05263158f, 0.19736843},{ -0.52255636, -0.81390977, -0.30075186, 0.30639097 } };

  bool inversable = mat4x4_inverse(a, b);
  assert(inversable == true);
  double det = mat4x4_det(a, 4);
  assert(equal(det, 532.0));
  double cof = mat4x4_cofactor(a, 2, 3);
  assert(equal(cof, -160.0));
  assert(equal(b[3][2], -160.0/532.0));
  cof = mat4x4_cofactor(a, 3, 2);
  assert(equal(cof, 105.0));
  assert(equal(b[2][3], 105.0/532.0));
  assert(mat4x4_equal(b, c) == true);

  Mat4x4 d = { { 8.0, -5.0, 9.0, 2.0 },{ 7.0, 5.0, 6.0, 1.0 },\
        { -6.0, 0.0, 9.0, 6.0},{ -3.0, 0.0, -9.0, -4.0 } };
  Mat4x4 e = { { 0.0, 0.0, 0.0, 0.0 },{ 0.0, 0.0, 0.0, 0.0 },\
    { 0.0, 0.0, 0.0, 0.0 },{ 0.0, 0.0, 0.0, 0.0 } };
  Mat4x4 f = { { -0.15384616, -0.15384616, -0.28205130, -0.53846157 },{ -0.07692308f, 0.12307692, 0.02564103, 0.03076923 },\
      { 0.35897437, 0.35897437, 0.43589744, 0.92307693},{ -0.69230771, -0.69230771, -0.76923078f, -1.92307687 } };

  inversable = mat4x4_inverse(d, e);
  assert(inversable == true);
  assert(mat4x4_equal(e, f) == true);

  Mat4x4 g = { { 9.0, 3.0, 0.0, 9.0 },{ -5.0, -2.0, -6.0, -3.0 },\
        { -4.0, 9.0, 6.0, 4.0},{ -7.0, 6.0, 6.0, 2.0 } };
  Mat4x4 h = { { 0.0, 0.0, 0.0, 0.0 },{ 0.0, 0.0, 0.0, 0.0 },\
    { 0.0, 0.0, 0.0, 0.0 },{ 0.0, 0.0, 0.0, 0.0 } };

  inversable = mat4x4_inverse(g, h);
  assert(inversable == true);
  assert(mat4x4_equal(e, f) == true);
  return 0;
}

// 41 Multiply product by its inverse
int mult_prod_by_inverse_test() {
  Mat4x4 a = { { 3.0, -9.0, 7.0, 3.0 },{ 3.0, -8.0, 2.0, -9.0 },\
      { -4.0, 4.0, 4.0, 1.0},{ -6.0, 5.0, -1.0, 1.0 } };
  Mat4x4 b = { { 8.0, 2.0, 2.0, 2.0 },{ 3.0, -1.0, 7.0, 0.0 },\
    { 7.0, 0.0, 5.0, 4.0 },{ 6.0, -2.0, 0.0, 5.0 } };
  Mat4x4 c = { { 0.0, 0.0, 0.0, 0.0 },{ 0.0, 0.0, 0.0, 0.0 },\
    { 0.0, 0.0, 0.0, 0.0 },{ 0.0, 0.0, 0.0, 0.0 } };

  mat4x4_mul_in_place(a, b, c);
  Mat4x4 t;
  bool inversable = mat4x4_inverse(b, t);
  Mat4x4 u;
  mat4x4_mul_in_place(c, t, u);
  assert(inversable == true);
  assert(equal(u[0][0], a[0][0]));
  assert(equal(u[0][1], a[0][1]));
  assert(equal(u[0][2], a[0][2]));
  assert(equal(u[0][3], a[0][3]));
  return 0;
}

// 45 Multiply by a translation matrix
int point_trans_test() {
  tuple point1 = create_point(-3.0, 4.0, 5.0);
  tuple point2 = create_point( 0.0, 0.0, 0.0);
  Mat4x4 trans;
  gen_translate_matrix(5.0, -3.0, 2.0, trans);
  mat4x4_mul_tuple(trans, point1, &point2);
  assert(equal(point2.x, 2.0));
  assert(equal(point2.y, 1.0));
  assert(equal(point2.z, 7.0));
  assert(equal(point2.w, 1.0));
  return 0;
}

// 45 Multiply by the inverse of a traslation matrix
int point_mult_inverse_translation_test() {
  Mat4x4 trans;
  Mat4x4 trans_inverse;
  gen_translate_matrix(5.0, -3.0, 2.0, trans);
  mat4x4_inverse(trans, trans_inverse);
  tuple p1 = create_point(-3.0, 4.0, 5.0);
  tuple p2 = create_point(0.0, 0.0, 0.0);
  mat4x4_mul_tuple(trans_inverse, p1, &p2);
  assert(equal(p2.x, -8.0));
  assert(equal(p2.y,  7.0));
  assert(equal(p2.z,  3.0));
  assert(equal(p2.w,  1.0));
  return 0;
}

// 45 Translation does not affect vectors
int vector_translation_has_no_effect_test() {
  Mat4x4 trans;
  gen_translate_matrix(5.0, -3.0, 2.0, trans);
  tuple v1 = create_vector(-3.0, 4.0, 5.0);
  tuple v2 = create_point(0.0, 0.0, 0.0);
  mat4x4_mul_tuple(trans, v1, &v2);
  assert(equal(v2.x, -3.0));
  assert(equal(v2.y,  4.0));
  assert(equal(v2.z,  5.0));
  assert(equal(v2.w,  0.0));
  return 0;
}

// 46 Scaling matrix applied to a point
int point_scale_mat4x4_test() {
  tuple p1 = create_point(-4.0, 6.0, 8.0);
  tuple p2 = create_point(0.0, 0.0, 0.0);
  Mat4x4 scale_mat;
  gen_scale_matrix(2.0, 3.0, 4.0, scale_mat);
  mat4x4_mul_tuple(scale_mat, p1, &p2);
  assert(equal(p2.x, -8.0));
  assert(equal(p2.y, 18.0));
  assert(equal(p2.z, 32.0));
  assert(equal(p2.w,  1.0));
  return 0;
}

// 46 Scaling matrix applied to a vector
int vec_scale_mat4x4_test() {
  tuple p1 = create_vector(-4.0, 6.0, 8.0);
  tuple p2 = create_vector(0.0, 0.0, 0.0);
  Mat4x4 scale_mat;
  gen_scale_matrix(2.0, 3.0, 4.0, scale_mat);
  mat4x4_mul_tuple(scale_mat, p1, &p2);
  assert(equal(p2.x, -8.0));
  assert(equal(p2.y, 18.0));
  assert(equal(p2.z, 32.0));
  assert(equal(p2.w,  0.0));
  return 0;
}

// 46 Multiply inverse of scaling matrix
int mult_inverse_scale_matrix_test() {
  Mat4x4 scale_mat;
  Mat4x4 scale_mat_inv;
  tuple p1 = create_vector(-4.0, 6.0, 8.0);
  tuple p2 = create_vector(0.0, 0.0, 0.0);
  gen_scale_matrix(2.0, 3.0, 4.0, scale_mat);
  mat4x4_inverse(scale_mat, scale_mat_inv);
  mat4x4_mul_tuple(scale_mat_inv, p1, &p2);
  assert(equal(p2.x, -2.0));
  assert(equal(p2.y,  2.0));
  assert(equal(p2.z,  2.0));
  assert(equal(p2.w,  0.0));
  return 0;
}

// 47 Reflection is scaling by a negative value
int reflection_scaling_neg_value_test() {
    Mat4x4 scale_mat;
    gen_scale_matrix(-1.0, 1.0, 1.0, scale_mat);
    tuple p1 = create_point(2.0, 3.0, 4.0);
    tuple p2 = create_point(0.0, 0.0, 0.0);
    mat4x4_mul_tuple(scale_mat, p1, &p2);
    assert(equal(p2.x, -2.0));
    assert(equal(p2.y, 3.0));
    assert(equal(p2.z, 4.0));
    return 0;
}

// 48 Rotating a point around the x axis
int gen_rotation_matrix_X_test() {
  Mat4x4 rot_mat;
  tuple p1 = create_point(0.0, 1.0, 0.0);
  tuple p2 = create_point(7.0, 8.0, 9.0);
  gen_rotate_matrix_X(M_PI / 4, rot_mat);
  mat4x4_mul_tuple(rot_mat, p1, &p2);
  assert(equal(p2.x, 0.0));
  assert(equal(p2.y, sqrt(2.0)/2.0));
  assert(equal(p2.z, sqrt(2.0)/2.0));
  assert(equal(p2.w, 1.0));

  gen_rotate_matrix_X(M_PI / 2, rot_mat);
  mat4x4_mul_tuple(rot_mat, p1, &p2);
  assert(equal(p2.x, 0.0));
  assert(equal(p2.y, 0.0));
  assert(equal(p2.z, 1.0));
  assert(equal(p2.w, 1.0));
  return 0;
}

// 49 Inverse of an x rotation rotates the opposite direction
int gen_rotation_matrix_reverse_test() {
  Mat4x4 rot_mat;
  Mat4x4 rot_mat_inv;
  tuple p1 = create_point(0.0, 1.0, 0.0);
  tuple p2 = create_point(7.0, 8.0, 9.0);
  gen_rotate_matrix_X(M_PI / 4, rot_mat);
  mat4x4_inverse(rot_mat, rot_mat_inv);
  mat4x4_mul_tuple(rot_mat_inv, p1, &p2);
  assert(equal(p2.x, 0.0));
  assert(equal(p2.y, sqrt(2.0) / 2.0));
  assert(equal(p2.z, -sqrt(2.0) / 2.0));
  assert(equal(p2.w, 1.0));
  return 0;
}

// 50 Rotating a point around the y axis
int gen_rotation_matrix_Y_test() {
  Mat4x4 rot_mat;
  tuple p1 = create_point(0.0, 0.0, 1.0);
  tuple p2 = create_point(7.0, 8.0, 9.0);
  gen_rotate_matrix_Y(M_PI / 4, rot_mat);
  mat4x4_mul_tuple(rot_mat, p1, &p2);
  assert(equal(p2.x, sqrt(2.0) / 2.0));
  assert(equal(p2.y,  0.0));
  assert(equal(p2.z, sqrt(2.0) / 2.0));
  assert(equal(p2.w,  1.0));

  gen_rotate_matrix_Y(M_PI / 2, rot_mat);
  mat4x4_mul_tuple(rot_mat, p1, &p2);
  assert(equal(p2.x, 1.0));
  assert(equal(p2.y, 0.0));
  assert(equal(p2.z, 0.0));
  assert(equal(p2.w, 1.0));
  return 0;
}

// 50 Rotating a point around the y axis
int gen_rotation_matrix_Z_test() {
  Mat4x4 rot_mat;
  tuple p1 = create_point(0.0, 1.0, 0.0);
  tuple p2 = create_point(7.0, 8.0, 9.0);
  gen_rotate_matrix_Z(M_PI / 4, rot_mat);
  mat4x4_mul_tuple(rot_mat, p1, &p2);
  assert(equal(p2.x, -sqrt(2.0) / 2.0));
  assert(equal(p2.y, sqrt(2.0) / 2.0));
  assert(equal(p2.z, 0.0));
  assert(equal(p2.w, 1.0));

  gen_rotate_matrix_Z(M_PI / 2, rot_mat);
  mat4x4_mul_tuple(rot_mat, p1, &p2);
  assert(equal(p2.x, -1.0));
  assert(equal(p2.y, 0.0));
  assert(equal(p2.z, 0.0));
  assert(equal(p2.w, 1.0));
  return 0;
}

// 52 Shearing transformation moves x in proportion to y
int gen_shear_matrix_test() {
  Mat4x4 shear_mat;
  gen_shear_matrix(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, shear_mat);
  tuple p1 = create_point(2.0, 3.0, 4.0);
  tuple p2 = create_point(7.0, 8.0, 9.0);
  mat4x4_mul_tuple(shear_mat, p1, &p2);
  assert(equal(p2.x, 5.0));
  assert(equal(p2.y, 3.0));
  assert(equal(p2.z, 4.0));
  assert(equal(p2.w, 1.0));

  // moves x in proportion to y
  gen_shear_matrix(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, shear_mat);
  p2.x = 2.0; p2.y = 3.0; p2.z = 4.0;
  mat4x4_mul_tuple(shear_mat, p1, &p2);
  assert(equal(p2.x, 6.0));
  assert(equal(p2.y, 3.0));
  assert(equal(p2.z, 4.0));
  assert(equal(p2.w, 1.0));

  // moves y in proportion to x
  gen_shear_matrix(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, shear_mat);
  p2.x = 2.0; p2.y = 3.0; p2.z = 4.0;
  mat4x4_mul_tuple(shear_mat, p1, &p2);
  assert(equal(p2.x, 2.0));
  assert(equal(p2.y, 5.0));
  assert(equal(p2.z, 4.0));
  assert(equal(p2.w, 1.0));

  // moves y in proportion to z
  gen_shear_matrix(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, shear_mat);
  p2.x = 2.0; p2.y = 3.0; p2.z = 4.0;
  mat4x4_mul_tuple(shear_mat, p1, &p2);
  assert(equal(p2.x, 2.0));
  assert(equal(p2.y, 7.0));
  assert(equal(p2.z, 4.0));
  assert(equal(p2.w, 1.0));

  // moves x in proportion to x
  gen_shear_matrix(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, shear_mat);
  p2.x = 2.0; p2.y = 3.0; p2.z = 4.0;
  mat4x4_mul_tuple(shear_mat, p1, &p2);
  assert(equal(p2.x, 2.0));
  assert(equal(p2.y, 3.0));
  assert(equal(p2.z, 6.0));
  assert(equal(p2.w, 1.0));

  // moves x in proportion to x
  gen_shear_matrix(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, shear_mat);
  p2.x = 2.0; p2.y = 3.0; p2.z = 4.0;
  mat4x4_mul_tuple(shear_mat, p1, &p2);
  assert(equal(p2.x, 2.0));
  assert(equal(p2.y, 3.0));
  assert(equal(p2.z, 7.0));
  assert(equal(p2.w, 1.0));
  return 0;
}

// 54 Individual transormations are applied in sequence
int transform_applied_in_sequence_test() {
  Mat4x4 rot_mat;
  Mat4x4 scale_mat;
  Mat4x4 shear_mat;
  gen_rotate_matrix_X(M_PI / 2, rot_mat);
  gen_scale_matrix(5.0, 5.0, 5.0, scale_mat);
  gen_translate_matrix(10.0, 5.0, 7.0, shear_mat);
  tuple p1 = create_point(1.0, 0.0, 1.0);
  tuple p2 = create_point(1.0, -1.0, 0.0);
  tuple p3 = create_point(5.0, -5.0, 0.0);
  tuple p4 = create_point(15.0, 0.0, 7.0);
  mat4x4_mul_tuple(rot_mat, p1, &p2);
  assert(equal(p2.x, 1.0));
  assert(equal(p2.y, -1.0));
  assert(equal(p2.z, 0.0));
  assert(equal(p2.w, 1.0));
  mat4x4_mul_tuple(scale_mat, p2, &p3);
  assert(equal(p3.x, 5.0));
  assert(equal(p3.y, -5.0));
  assert(equal(p3.z, 0.0));
  assert(equal(p3.w, 1.0));
  mat4x4_mul_tuple(shear_mat, p3, &p4);
  assert(equal(p4.x, 15.0));
  assert(equal(p4.y, 0.0));
  assert(equal(p4.z, 7.0));
  assert(equal(p4.w, 1.0));
  p1.x = 1.0; p1.y = 0.0; p1.z = 1.0;
  mat4x4_mul_in_place(shear_mat, scale_mat, scale_mat);
  mat4x4_mul_in_place(scale_mat, rot_mat, rot_mat);
  mat4x4_mul_tuple(rot_mat, p1, &p2);
  assert(equal(p2.x, 15.0));
  assert(equal(p2.y, 0.0));
  assert(equal(p2.z, 7.0));
  assert(equal(p2.w, 1.0));
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
    canvas[(int)three.x][(int)three.z].x = 1.0;
  }
  return 0;
}

int tuple_copy_test() {
  tuple t1 = { 1.0, 2.0, 3.0, 4.0 };
  tuple t2 = { 0.0, 0.0, 0.0, 0.0 };
  tuple_copy(&t1, &t2);
  assert(equal(t1.x, 1.0));
  assert(equal(t1.y, 2.0));
  assert(equal(t1.z, 3.0));
  assert(equal(t1.w, 4.0));

  assert(equal(t2.x, 1.0));
  assert(equal(t2.y, 2.0));
  assert(equal(t2.z, 3.0));
  assert(equal(t2.w, 4.0));

  assert(equal(t1.x, t2.x));
  assert(equal(t1.y, t2.y));
  assert(equal(t1.z, t2.z));
  assert(equal(t1.w, t2.w));
  return 0;
}

int mat4x4_copy_test() {
  Mat4x4 a = { { 1.0, 2.0, 3.0, 4.0 },{ 5.0, 6.0, 7.0, 8.0 },\
    { 9.0, 10.0, 11.0, 12.0},{ 13.0, 14.0, 15.0, 16.0 } };
  Mat4x4 b = { { 0.0, 0.0, 0.0, 0.0 },{ 0.0, 0.0, 0.0, 0.0 },\
    { 0.0, 0.0, 0.0, 0.0 },{ 0.0, 0.0, 0.0, 0.0 } };
  mat4x4_copy(a, b);
  assert(equal(a[0][0], 1.0));
  assert(equal(a[1][0], 5.0));
  assert(equal(a[2][0], 9.0));
  assert(equal(a[3][0], 13.0));

  assert(equal(a[0][1], 2.0));
  assert(equal(a[1][1], 6.0));
  assert(equal(a[2][1], 10.0));
  assert(equal(a[3][1], 14.0));

  assert(equal(a[0][2], 3.0));
  assert(equal(a[1][2], 7.0));
  assert(equal(a[2][2], 11.0));
  assert(equal(a[3][2], 15.0));

  assert(equal(a[0][3], 4.0));
  assert(equal(a[1][3], 8.0));
  assert(equal(a[2][3], 12.0));
  assert(equal(a[3][3], 16.0));

  assert(equal(b[0][0], 1.0));
  assert(equal(b[1][0], 5.0));
  assert(equal(b[2][0], 9.0));
  assert(equal(b[3][0], 13.0));

  assert(equal(b[0][1], 2.0));
  assert(equal(b[1][1], 6.0));
  assert(equal(b[2][1], 10.0));
  assert(equal(b[3][1], 14.0));

  assert(equal(b[0][2], 3.0));
  assert(equal(b[1][2], 7.0));
  assert(equal(b[2][2], 11.0));
  assert(equal(b[3][2], 15.0));

  assert(equal(b[0][3], 4.0));
  assert(equal(b[1][3], 8.0));
  assert(equal(b[2][3], 12.0));
  assert(equal(b[3][3], 16.0));
  return 0;
}

// 58 Creating and quering a ray
int create_ray_test() {
  ray r = create_ray(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

  assert(equal(r.origin_point.x, 1.0));
  assert(equal(r.origin_point.y, 2.0));
  assert(equal(r.origin_point.z, 3.0));
  assert(equal(r.origin_point.w, 1.0));

  assert(equal(r.direction_vector.x, 4.0));
  assert(equal(r.direction_vector.y, 5.0));
  assert(equal(r.direction_vector.z, 6.0));
  assert(equal(r.direction_vector.w, 0.0));

  return 0;
}

int create_shape_test() {
    shape* sp = create_shape(SHAPE);

    assert(sp->next == NULL);
    assert(equal(sp->t, 1.0));

    assert(equal(sp->location.x, 0.0));
    assert(equal(sp->location.y, 0.0));
    assert(equal(sp->location.z, 0.0));
    assert(equal(sp->location.w, 0.0));

    assert(equal(sp->transform[0][0], 1.0));
    assert(equal(sp->transform[1][0], 0.0));
    assert(equal(sp->transform[2][0], 0.0));
    assert(equal(sp->transform[3][0], 0.0));

    assert(equal(sp->transform[0][1], 0.0));
    assert(equal(sp->transform[1][1], 1.0));
    assert(equal(sp->transform[2][1], 0.0));
    assert(equal(sp->transform[3][1], 0.0));

    assert(equal(sp->transform[0][2], 0.0));
    assert(equal(sp->transform[1][2], 0.0));
    assert(equal(sp->transform[2][2], 1.0));
    assert(equal(sp->transform[3][2], 0.0));

    assert(equal(sp->transform[0][3], 0.0));
    assert(equal(sp->transform[1][3], 0.0));
    assert(equal(sp->transform[2][3], 0.0));
    assert(equal(sp->transform[3][3], 1.0));

    assert(equal(sp->material.color.x, 1.0));
    assert(equal(sp->material.color.y, 1.0));
    assert(equal(sp->material.color.z, 1.0));
    assert(equal(sp->material.color.w, 0.0));

    assert(equal(sp->material.ambient, 0.1));
    assert(equal(sp->material.diffuse, 0.9));
    assert(equal(sp->material.specular, 0.9));
    assert(equal(sp->material.shininess, 200.0));
    free(sp);
    return 0;
}

int create_intersections_test() {
    intersections intersects  = create_intersections();
    assert(intersects.count == 0);
    for (int i = 0; i < INTERSECTIONS_SIZE; ++i) {
        assert(equal(intersects.itersection[i].t, 0.0));
        assert(intersects.itersection[i].object_id == NULL);
    }
    return 0;
}

// 58 Computing a point from a distance
int position_test() {
    ray r = create_ray(2.0, 3.0, 4.0, 1.0, 0.0, 0.0);
    tuple p1 = position(r, 0.0);
    assert(equal(p1.x, 2.0));
    assert(equal(p1.y, 3.0));
    assert(equal(p1.z, 4.0));
    assert(equal(p1.w, 1.0));

    tuple p2 = position(r, 1.0);
    assert(equal(p2.x, 3.0));
    assert(equal(p2.y, 3.0));
    assert(equal(p2.z, 4.0));
    assert(equal(p2.w, 1.0));

    tuple p3 = position(r, -1.0);
    assert(equal(p3.x, 1.0));
    assert(equal(p3.y, 3.0));
    assert(equal(p3.z, 4.0));
    assert(equal(p3.w, 1.0));

    tuple p4 = position(r, 2.5);
    assert(equal(p4.x, 4.5));
    assert(equal(p4.y, 3.0));
    assert(equal(p4.z, 4.0));
    assert(equal(p4.w, 1.0));

    return 0;
}

// 59 A ray intersects a sphere at two points
int ray_intersect_sphere_two_point_test() {
    ray r = create_ray(0.0, 0.0, -5.0, 0.0, 0.0, 1.0);
    shape* sp = create_shape(SHAPE);
    intersections inter = create_intersections();
    intersect(sp, &r, &inter);
    assert(inter.count == 2);
    assert(equal(inter.itersection[0].t, 4.0));
    assert(equal(inter.itersection[1].t, 6.0));
    free(sp);
    return 0;
}

// 60 A ray intersects a sphere at a tangent
int ray_intersect_sphere_tangent_test() {
    ray r = create_ray(0.0, 1.0, -5.0, 0.0, 0.0, 1.0);
    shape* sp = create_shape(SHAPE);
    intersections inter = create_intersections();
    intersect(sp, &r, &inter);
    assert(inter.count == 2);
    assert(equal(inter.itersection[0].t, 5.0));
    assert(equal(inter.itersection[1].t, 5.0));
    free(sp);
    return 0;
}

// 60 A ray misses a sphere
int ray_misses_sphere_test() {
    ray r = create_ray(0.0, 2.0, -5.0, 0.0, 0.0, 1.0);
    shape* sp = create_shape(SHAPE);
    intersections inter = create_intersections();
    intersect(sp, &r, &inter);
    assert(inter.count == 0);
    assert(equal(inter.itersection[0].t, 0.0)); // might as well check
    assert(equal(inter.itersection[1].t, 0.0));

    assert(inter.itersection[0].object_id == NULL);
    assert(inter.itersection[1].object_id == NULL);
    free(sp);
    return 0;
}

// 61 A ray originates inside a sphere
int ray_originates_inside_sphere_test() {
    ray r = create_ray(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    shape* sp = create_shape(SHAPE);
    intersections inter = create_intersections();
    intersect(sp, &r, &inter);
    assert(inter.count == 2);
    assert(equal(inter.itersection[0].t, -1.0));
    assert(equal(inter.itersection[1].t, 1.0));
    free(sp);
    return 0;
}

// 62 A sphere is behind a ray
int sphere_is_behind_ray_test() {
    ray r = create_ray(0.0, 0.0, 5.0, 0.0, 0.0, 1.0);
    shape* sp = create_shape(SHAPE);
    intersections inter = create_intersections();
    intersect(sp, &r, &inter);
    assert(inter.count == 2);
    assert(equal(inter.itersection[0].t, -6.0));
    assert(equal(inter.itersection[1].t, -4.0));
    free(sp);
    return 0;
}

// 63 An intersection encapsulates t and object
// Test Not Required Due To Design Of Application

// 64 Aggegating intersections
int aggregating_intersections_test() {
    intersections intersects = create_intersections();
    shape* sp = create_shape(SHAPE);
    add_intersection(&intersects, 1.0, sp);
    add_intersection(&intersects, 2.0, sp);
    assert(intersects.count == 2);
    assert(equal(intersects.itersection[0].t, 1.0));
    assert(equal(intersects.itersection[1].t, 2.0));
    free(sp);
    return 0;
}

// 64 Intersect sets the object on the intersection
int intersect_sets_object_on_intersection_test() {
    ray r = create_ray(0.0, 0.0, -5.0, 0.0, 0.0, 1.0);
    shape* sp = create_shape(SHAPE);
    intersections inter = create_intersections();
    intersect(sp, &r, &inter);
    assert(inter.count == 2);
    assert(inter.itersection[0].object_id == sp);
    assert(inter.itersection[1].object_id == sp);
    free(sp);
    return 0;
}

// Clear Intersections List
// NOTE: Needed for testing
int clear_intersections_test() {
    intersections intersects = create_intersections();
    shape* sp = create_shape(SHAPE);
    add_intersection(&intersects, 9.0, sp);
    clear_intersections(&intersects);
    assert(intersects.count == 0);
    for (int i = 0; i < INTERSECTIONS_SIZE; ++i) {
        assert(equal(intersects.itersection[i].t, DBL_MIN));
        assert(intersects.itersection[i].object_id == NULL);
    }
    free(sp);
    return 0;
}

int too_many_intersections_test() {
    shape* sp = create_shape(SHAPE);
    intersections inter = create_intersections();
    // fill up all intersections
    bool insert_status = false;
    for (int i = 0; i < INTERSECTIONS_SIZE; ++i) {
        insert_status = add_intersection(&inter, 9.0, sp);
        assert(insert_status == true);
    }
    // then add one more
    insert_status = add_intersection(&inter, 9.0, sp);
    assert(insert_status == false);
    return 0;
}

// 64  NOTE: All hit tests have been put together
int hit_tests(){
    intersections intersects = create_intersections();
    shape* sp = create_shape(SHAPE);

    // 65 The hit when all intersections have positive t
    add_intersection(&intersects, 1.0, sp);
    add_intersection(&intersects, 2.0, sp);
    assert(intersects.count == 2);
    intersection* intersect1 = hit(&intersects);
    assert(intersect1->object_id == sp);
    assert(equal(intersect1->t, 1.0));

    //65 The hit when some intersections have a negative t
    clear_intersections(&intersects);
    assert(intersects.count == 0);
    add_intersection(&intersects, -1.0, sp);
    add_intersection(&intersects, 1.0, sp);
    assert(intersects.count == 2);
    intersect1 = hit(&intersects);
    assert(intersect1->object_id == sp);
    assert(equal(intersect1->t, 1.0));

    // 65 The hit when all intersections have negative t
    clear_intersections(&intersects);
    assert(intersects.count == 0);
    add_intersection(&intersects, -2.0, sp);
    add_intersection(&intersects, -1.0, sp);
    assert(intersects.count == 2);
    intersect1 = hit(&intersects);
    assert(intersect1 == NULL);

    // 66 The hit is always the lowest nonnegative intersection
    clear_intersections(&intersects);
    assert(intersects.count == 0);
    add_intersection(&intersects, 5.0, sp);
    add_intersection(&intersects, 7.0, sp);
    add_intersection(&intersects, -3.0, sp);
    add_intersection(&intersects, 2.0, sp);
    assert(intersects.count == 4);
    intersect1 = hit(&intersects);
    assert(intersect1->object_id == sp);
    assert(equal(intersect1->t, 2.0));
    free(sp);
    return 0;
}

// 69 Translating a ray
int translating_ray_test() {
    ray r1 = create_ray(1.0, 2.0, 3.0, 0.0, 1.0, 0.0);
    Mat4x4 trans_mat;
    gen_translate_matrix(3.0, 4.0, 5.0, trans_mat);

    ray r2 = transform(&r1, trans_mat);
    assert(&r1 != &r2);
    assert(equal(r2.origin_point.x, 4.0));
    assert(equal(r2.origin_point.y, 6.0));
    assert(equal(r2.origin_point.z, 8.0));
    assert(equal(r2.origin_point.w, 1.0));

    assert(equal(r2.direction_vector.x, 0.0));
    assert(equal(r2.direction_vector.y, 1.0));
    assert(equal(r2.direction_vector.z, 0.0));
    assert(equal(r2.direction_vector.w, 0.0));
    return 0;
}

// 69 Scaling a ray
int scaling_ray_test() {
    ray r1 = create_ray(1.0, 2.0, 3.0, 0.0, 1.0, 0.0);
    Mat4x4 scale_mat;
    gen_scale_matrix(2.0, 3.0, 4.0, scale_mat);
    ray r2 = transform(&r1, scale_mat);

    assert(&r1 != &r2);

    assert(equal(r2.origin_point.x, 2.0));
    assert(equal(r2.origin_point.y, 6.0));
    assert(equal(r2.origin_point.z, 12.0));
    assert(equal(r2.origin_point.w, 1.0));

    assert(equal(r2.direction_vector.x, 0.0));
    assert(equal(r2.direction_vector.y, 3.0));
    assert(equal(r2.direction_vector.z, 0.0));
    assert(equal(r2.direction_vector.w, 0.0));
    return 0;
}

// 69 Sphere default transformation
int sphere_default_transformation_test() {
    shape* sp = create_shape(SHAPE);
    Mat4x4 ident_mat;
    mat4x4_set_ident(ident_mat);
    assert(mat4x4_equal(sp->transform, ident_mat) == true);
    free(sp);
    return 0;
}

// 69 Changing a sphere's transformation
int change_sphere_transform_test() {
    shape* sp = create_shape(SHAPE);
    Mat4x4 trans_mat;
    gen_translate_matrix(2.0, 3.0, 4.0, trans_mat);
    set_transform(sp, trans_mat);
    assert(mat4x4_equal(sp->transform, trans_mat) == true);
    free(sp);
    return 0;
}

int set_transform_test() {
    shape* sp = create_shape(SHAPE);
   
    Mat4x4 ident_mat;
    mat4x4_set_ident(ident_mat);
    // does sphere have identity as transform?
    assert(mat4x4_equal(sp->transform, ident_mat));

    Mat4x4 trans_mat;
    gen_translate_matrix(2.0, 3.0, 4.0, trans_mat);

    // is correct translate matrix?
    assert(equal(trans_mat[0][3], 2.0));
    assert(equal(trans_mat[1][3], 3.0));
    assert(equal(trans_mat[2][3], 4.0));
    assert(equal(trans_mat[3][3], 1.0));

    set_transform(sp, trans_mat);
    // has it been copied correctly?
    assert(mat4x4_equal(sp->transform, trans_mat));
    // two seperate matrixes
    assert(&sp->transform != &ident_mat);
    free(sp);
    return 0;
}

// 69 Intersecting a scaled sphere with a ray
int intersect_scaled_sphere_test() {
    ray r1 = create_ray(0.0, 0.0, -5.0, 0.0, 0.0, 1.0);
    shape* sp = create_shape(SHAPE);
    Mat4x4 scale_mat;
    gen_scale_matrix(2.0, 2.0, 2.0, scale_mat);
    set_transform(sp, scale_mat);
    assert(mat4x4_equal(sp->transform, scale_mat) == true);
    assert(&sp->transform != &scale_mat);

    intersections inter = create_intersections();
    intersect(sp, &r1, &inter);

    assert(inter.count == 2);
    assert(inter.itersection[0].object_id == sp);
    assert(equal(inter.itersection[0].t, 3.0));

    assert(inter.itersection[1].object_id == sp);
    assert(equal(inter.itersection[1].t, 7.0));
    free(sp);
    return 0;
}

// 70 Intersecting a translated sphere with a ray
int intersecting_translated_sphere_test() {
    ray r1 = create_ray(0.0, 0.0, -5.0, 0.0, 0.0, 1.0);
    shape* sp = create_shape(SHAPE);
    Mat4x4 trans_mat;
    gen_translate_matrix(5.0, 0.0, 0.0, trans_mat);
    set_transform(sp, trans_mat);

    Mat4x4 inv_scale_mat;
    mat4x4_set_ident(inv_scale_mat);
    mat4x4_inverse(sp->transform, inv_scale_mat);
    ray r2 = transform(&r1, inv_scale_mat);
    intersections inter = create_intersections();
    intersect(sp, &r2, &inter);
    assert(inter.count == 0);
    free(sp);
    return 0;
}

int normals_test() {
    shape* sphere1 = create_shape(SHAPE);
    // 78 The normal on a sphere at a point on the X axis.
    tuple location1 = create_point(1.0, 0.0, 0.0);
    tuple n = normal_at(sphere1, location1);
    assert(equal(n.x, 1.0));
    assert(equal(n.y, 0.0));
    assert(equal(n.z, 0.0));
    assert(equal(n.w, 0.0));

    // 78 The normal on a sphere at a point on the Y axis.
    tuple location2 = create_point(0.0, 1.0, 0.0);
    n = normal_at(sphere1, location2);
    assert(equal(n.x, 0.0));
    assert(equal(n.y, 1.0));
    assert(equal(n.z, 0.0));
    assert(equal(n.w, 0.0));

    // 78 The normal on a sphere at a point on the Z axis.
    tuple location3 = create_point(0.0, 0.0, 1.0);
    n = normal_at(sphere1, location3);
    assert(equal(n.x, 0.0));
    assert(equal(n.y, 0.0));
    assert(equal(n.z, 1.0));
    assert(equal(n.w, 0.0));

    // 78 The normal on a sphere at a nonaxial point.
    double nonaxial = sqrt(3) / 3.0;
    tuple location4 = create_point(nonaxial, nonaxial, nonaxial);
    n = normal_at(sphere1, location4);
    assert(equal(n.x, nonaxial));
    assert(equal(n.y, nonaxial));
    assert(equal(n.z, nonaxial));
    assert(equal(n.w, 0.0));
    free(sphere1);
    return 0;
}

// 78 The normal is a normalized vector.
int normal_is_normal_test() {
    shape* sp = create_shape(SHAPE);
    double nonaxial = sqrt(3) / 3.0;
    tuple location1 = create_point(nonaxial, nonaxial, nonaxial);
    tuple n = normal_at(sp, location1);
    tuple nn = tuple_normalize(n);
    assert(equal(n.x, nn.x));
    assert(equal(n.y, nn.y));
    assert(equal(n.z, nn.z));
    assert(equal(n.w, 0.0));
    free(sp);
    return 0;
}

// 80 Computing the normal on a translated sphere
int compute_normal_on_sphere_test() {
    tuple vec1 = create_vector(0.0, sqrt(2) / 2, -sqrt(2) / 2);
    shape* sp = create_shape(SHAPE);
    gen_translate_matrix(0.0, 1.0, 0.0, sp->transform);
    tuple n = normal_at(sp, vec1);
    assert(equal(n.x, 0.0));
    assert(equal(n.y, sqrt(2) / 2));
    assert(equal(n.z, -sqrt(2) / 2));
    assert(equal(n.w, 0.0));
    free(sp);
    return 0;
}
 
// 80 Computing the normal on a transformed sphere
int compute_normal_on_transformed_sphere_test(){
    shape* sp = create_shape(SHAPE);
    Mat4x4 scale_mat;
    gen_scale_matrix(1.0, 0.5, 1.0, scale_mat);
    Mat4x4 rot_mat;
    gen_rotate_matrix_Z(M_PI / 5.0, rot_mat);
    Mat4x4 translate_mat;
    mat4x4_mul_in_place(scale_mat, rot_mat, translate_mat);
    set_transform(sp, translate_mat);
    tuple point = create_point(0.0, sqrt(2) / 2.0, -sqrt(2) / 2);
    tuple norm_at = normal_at(sp, point);
    assert(equal(norm_at.x, 0.0));
    assert(equal(norm_at.y, 0.97014250014533188f));
    assert(equal(norm_at.z, -0.24253562503633294));
    free(sp);
    return 0;
}

// 83 Reflecting a vector approaching at 45deg
int reflect_vector_approach_at_45_deg_test() {
    tuple v = create_vector(1.0, -1.0, 0.0);
    tuple n = create_vector(0.0, 1.0, 0.0);
    tuple r = tuple_reflect(v, n);
    assert(equal(r.x, 1.0));
    assert(equal(r.y, 1.0));
    assert(equal(r.z, 0.0));
    return 0;
}

// 83 Reflecting a vector off a slanted surface
int reflect_vector_off_slanted_surf_test() {
    tuple v = create_vector(0.0, -1.0, 0.0);
    tuple n = create_vector(sqrt(2)/2, sqrt(2)/2, 0.0);
    tuple r = tuple_reflect(v, n);
    assert(equal(r.x, 1.0));
    assert(equal(r.y, 0.0));
    assert(equal(r.z, 0.0));
    return 0;
}

// 84 A point light has a position and intensity
int point_light_position_intensity_test() {
    tuple intensity = create_point(1.0, 2.0, 3.0);
    tuple position1 = create_point(4.0, 5.0, 6.0);
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
    material m1 = create_material_default();

    assert(equal(m1.color.x, 1.0));
    assert(equal(m1.color.y, 1.0));
    assert(equal(m1.color.z, 1.0));
    assert(equal(m1.color.w, 0.0));

    assert(equal(m1.ambient, 0.1));
    assert(equal(m1.diffuse, 0.9));
    assert(equal(m1.specular, 0.9));
    assert(equal(m1.shininess, 200.0));
    assert(equal(m1.reflective, 0.0));
    assert(equal(m1.transparency, 0.0));
    assert(equal(m1.refractive_index, 1.0));
    return 0;
}

// 85 Sphere has a default material
int sphere_has_default_material_test() {
    shape* sp = create_shape(SHAPE);
    material m1 = create_material_default();

    assert(equal(m1.color.x, sp->material.color.x));
    assert(equal(m1.color.y, sp->material.color.y));
    assert(equal(m1.color.z, sp->material.color.z));
    assert(equal(m1.color.w, sp->material.color.w));

    assert(equal(m1.ambient, 0.1));
    assert(equal(m1.diffuse, 0.9));
    assert(equal(m1.specular, 0.9));
    assert(equal(m1.shininess, 200.0));

    assert(equal(m1.reflective, 0.0));
    assert(equal(sp->material.reflective, 0.0));

    assert(equal(m1.transparency, 0.0));
    assert(equal(sp->material.transparency, 0.0));

    assert(equal(m1.refractive_index, 1.0));
    assert(equal(sp->material.refractive_index, 1.0));

    free(sp);
    return 0;
}

// 86 Lighting with the eye between the light and the surface
int lighting_with_eye_between_light_and_surface_test() {
    shape* sh = create_shape(SHAPE);
    material m = create_material_default();
    tuple position1 = create_vector(0.0, 0.0, 0.0);
    tuple eyev = create_vector(0.0, 0.0, -1.0);
    tuple normalv = create_vector(0.0, 0.0, -1.0);
    tuple p_light_color = create_point(1.0, 1.0, 1.0);
    tuple p_light_position = create_vector(0.0, 0.0, -10.0);
    point_light p_light = create_point_light(p_light_position, p_light_color);
    tuple light1 = lighting(m, sh, &p_light, position1, eyev, normalv, false);
    assert(equal(light1.x, 1.9));
    assert(equal(light1.y, 1.9));
    assert(equal(light1.z, 1.9));
    return 0;
}

// 86 Lighting with the eye between light and surface, eye offset 45 deg
int lighting_with_eye_between_light_and_surface_eye_offset_test() {
    shape* sh = create_shape(SHAPE);
    material m = create_material_default();
    tuple position1 = create_point(0.0, 0.0, 0.0);
    tuple eyev = create_vector(0.0, sqrt(2)/2, -sqrt(2)/2);
    tuple normalv = create_vector(0.0, 0.0, -1.0);
    tuple intensity = create_vector(1.0, 1.0, 1.0);
    tuple p_light_position = create_point(0.0, 0.0, -10.0);
    point_light p_light = create_point_light(p_light_position, intensity);
    tuple light1 = lighting(m, sh, &p_light, position1, eyev, normalv, false);
    assert(equal(light1.x, 1.0));
    assert(equal(light1.y, 1.0));
    assert(equal(light1.z, 1.0));
    return 0;
}

// 87 Lighting with the eye opposite surface, light offset 45 deg
int lighting_with_eye_opposite_surface_test() {
    shape* sh = create_shape(SHAPE);
    material m = create_material_default();
    tuple position1 = create_point(0.0, 0.0, 0.0);
    tuple eyev = create_vector(0.0, 0.0, -1.0);
    tuple normalv = create_vector(0.0, 0.0, -1.0);
    tuple intensity = create_vector(1.0, 1.0, 1.0);
    tuple p_light_position = create_point(0.0, 10.0, -10.0);
    point_light p_light = create_point_light(p_light_position, intensity);
    tuple light1 = lighting(m, sh, &p_light, position1, eyev, normalv, false);
    assert(equal(light1.x, 0.73639608769926945));
    assert(equal(light1.y, 0.73639608769926945));
    assert(equal(light1.z, 0.73639608769926945));
    return 0;
}

// 87 Lighting with the eye in the path of the reflection vector
int lighting_with_eye_in_path_of_reflect_vector_test() {
    shape* sh = create_shape(SHAPE);
    material m = create_material_default();
    tuple position1 = create_point(0.0, 0.0, 0.0);
    tuple eyev = create_vector(0.0, -sqrt(2) / 2, -sqrt(2) / 2);
    tuple normalv = create_vector(0.0, 0.0, -1.0);
    tuple intensity = create_vector(1.0, 1.0, 1.0);
    tuple p_light_position = create_point(0.0, 10.0, -10.0);
    point_light p_light = create_point_light(p_light_position, intensity);
    tuple light1 = lighting(m, sh, &p_light, position1, eyev, normalv, false);
    assert(equal(light1.x, 1.6363960638574115));
    assert(equal(light1.y, 1.6363960638574115));
    assert(equal(light1.z, 1.6363960638574115));
    return 0;
}

// 88 Lighting with the light behind the surface
int lighting_with_the_light_behind_surface_test() {
    shape* sh = create_shape(SHAPE);
    material m = create_material_default();
    tuple position1 = create_point(0.0, 0.0, 0.0);
    tuple eyev = create_vector(0.0, 0.0, 1.0);
    tuple normalv = create_vector(0.0, 0.0, -1.0);
    tuple intensity = create_vector(1.0, 1.0, 1.0);
    tuple p_light_position = create_point(0.0, 0.0, 10.0);
    point_light p_light = create_point_light(p_light_position, intensity);
    tuple light1 = lighting(m, sh, &p_light, position1, eyev, normalv, true);
    assert(equal(light1.x, 0.1));
    assert(equal(light1.y, 0.1));
    assert(equal(light1.z, 0.1));
    return 0;
}

int intersect_compare_test() {
    shape* sp1 = create_shape(SHAPE);
    shape* sp2 = create_shape(SHAPE);
    shape* sp3 = create_shape(SHAPE);
    intersection i1 = { 0.0, sp1 };
    intersection i2 = { 1.0, sp2 };
    intersection i3 = { -1.0, sp3 };

    assert(intersect_compare(&i1, &i2) == -1);
    assert(intersect_compare(&i1, &i1) == 0);
    assert(intersect_compare(&i2, &i1) == 1);

    assert(intersect_compare(&i1, &i3) == 1);
    assert(intersect_compare(&i3, &i3) == 0);
    assert(intersect_compare(&i3, &i1) == -1);
    free(sp1);
    free(sp2);
    free(sp3);

    return 0;
}

int sort_intersects_test() {
    intersections intersects = create_intersections();

    intersects.itersection[0].t = -22.23821;
    intersects.itersection[1].t = -1.3210377;
    intersects.itersection[2].t = -1.8887736;
    intersects.itersection[3].t = -0.567737;
    intersects.itersection[4].t = -1.058888f;

    intersects.count = 5;

    // testing quicksort setup
    qsort(intersects.itersection, intersects.count, sizeof(intersection), intersect_compare);

    assert(equal(intersects.itersection[0].t, -22.23821));
    assert(equal(intersects.itersection[1].t, -1.8887736));
    assert(equal(intersects.itersection[2].t, -1.3210377));
    assert(equal(intersects.itersection[3].t, -1.058888f));
    assert(equal(intersects.itersection[4].t, -0.567737));
    assert(equal(intersects.itersection[5].t, DBL_MIN));

    intersection i1 = { -22.23821, NULL };
    intersection i2 = { -1.3210377, NULL };
    intersection i3 = { -1.8887736, NULL };
    intersection i4 = { -0.567737, NULL };
    intersection i5 = { -1.058888f, NULL };
    intersection i6 = { 0.0, NULL };
    intersection i7 = { 0.0000000000001, NULL };
    intersection i8 = { 0.0000000000002, NULL };
    intersection i9 = { -0.0000000000001, NULL };

    assert(intersect_compare(&i1, &i1) == 0);
    assert(intersect_compare(&i2, &i2) == 0);
    assert(intersect_compare(&i6, &i6) == 0);

    assert(intersect_compare(&i6, &i7) == -1);
    assert(intersect_compare(&i7, &i6) ==  1);
    assert(intersect_compare(&i7, &i8) == -1);
    assert(intersect_compare(&i8, &i7) ==  1);

    assert(intersect_compare(&i6, &i9) ==  1);
    assert(intersect_compare(&i9, &i6) == -1);

    clear_intersections(&intersects);

    assert(intersects_in_order_test(&intersects) == true);

    intersects.itersection[0].t = -22.23821;
    intersects.itersection[1].t = -1.3210377;
    intersects.itersection[2].t = -1.8887736;
    intersects.itersection[3].t = -0.567737;
    intersects.itersection[4].t = -1.058888f;

    intersects.count = 5;

    assert(intersects_in_order_test(&intersects) == false);

    clear_intersections(&intersects);

    intersects.itersection[0].t = 0.0;
    intersects.itersection[1].t = 0.0;
    intersects.itersection[2].t = 0.0;
    intersects.itersection[3].t = 0.0;
    intersects.itersection[4].t = 0.0;

    intersects.count = 5;

    assert(intersects_in_order_test(&intersects) == true);

    clear_intersections(&intersects);

    intersects.itersection[0].t = 0.0001;
    intersects.itersection[1].t = 0.0001;
    intersects.itersection[2].t = 0.0002;
    intersects.itersection[3].t = 0.0001;
    intersects.itersection[4].t = 0.0001;

    intersects.count = 5;

    assert(intersects_in_order_test(&intersects) == false);

    clear_intersections(&intersects);

    intersects.itersection[0].t = -0.004;
    intersects.itersection[1].t = -0.003;
    intersects.itersection[2].t = -0.002;
    intersects.itersection[3].t = -0.001;
    intersects.itersection[4].t =  0.000;

    intersects.count = 5;

    assert(intersects_in_order_test(&intersects) == true);

    clear_intersections(&intersects);

    intersects.itersection[0].t = -0.004;
    intersects.itersection[1].t = -0.003;
    intersects.itersection[2].t = -0.002;
    intersects.itersection[3].t = -0.000;  // flipped at end of array
    intersects.itersection[4].t = -0.001;

    intersects.count = 5;

    assert(intersects_in_order_test(&intersects) == false);

    clear_intersections(&intersects);

    intersects.itersection[0].t = -0.003;
    intersects.itersection[1].t = -0.004; // flipped at beginning of array
    intersects.itersection[2].t = -0.002;
    intersects.itersection[3].t = -0.001;
    intersects.itersection[4].t = -0.000;

    intersects.count = 5;

    assert(intersects_in_order_test(&intersects) == false);

    clear_intersections(&intersects);

    intersects.itersection[0].t = -0.0002;
    intersects.itersection[1].t = -0.0001; // positive and negative close to zero
    intersects.itersection[2].t = -0.0000;
    intersects.itersection[3].t =  0.0001;
    intersects.itersection[4].t =  0.0002;

    intersects.count = 5;

    assert(intersects_in_order_test(&intersects) == true);

    clear_intersections(&intersects);

    intersects.itersection[0].t = -0.0001;
    intersects.itersection[1].t = -0.0002; // positive and negative close to zero
    intersects.itersection[2].t = -0.0000; // flipped at beginning
    intersects.itersection[3].t =  0.0001;
    intersects.itersection[4].t =  0.0002;

    intersects.count = 5;

    assert(intersects_in_order_test(&intersects) == false);

    clear_intersections(&intersects);

    intersects.itersection[0].t = -0.0002;
    intersects.itersection[1].t = -0.0001; // positive and negative close to zero
    intersects.itersection[2].t = -0.0000; // flipped at end
    intersects.itersection[3].t =  0.0002;
    intersects.itersection[4].t =  0.0001;

    intersects.count = 5;

    assert(intersects_in_order_test(&intersects) == false);

    clear_intersections(&intersects);

    intersects.itersection[0].t = -0.00000002;
    intersects.itersection[1].t = -0.00000001; // very large and small
    intersects.itersection[2].t = -0.0;
    intersects.itersection[3].t = 1000000000.0;
    intersects.itersection[4].t = 2000000000.0;

    intersects.count = 5;

    assert(intersects_in_order_test(&intersects) == true);

    clear_intersections(&intersects);

    intersects.itersection[0].t = 1000000000.0;
    intersects.itersection[1].t = 2000000000.0;
    intersects.itersection[2].t = -0.00000002;
    intersects.itersection[3].t = -0.00000001; // very large and small
    intersects.itersection[4].t = -0.0;

    intersects.count = 5;

    assert(intersects_in_order_test(&intersects) == false);

    clear_intersections(&intersects);

    shape* sp1 = create_shape(SHAPE);
    shape* sp2 = create_shape(SHAPE);
    shape* sp3 = create_shape(SHAPE);
    shape* sp4 = create_shape(SHAPE);
    shape* sp5 = create_shape(SHAPE);

    add_intersection(&intersects, 1.0, sp1);
    add_intersection(&intersects, 2.0, sp2);

    sort_intersects(&intersects);

    assert(intersects.count == 2);
    assert(equal(intersects.itersection[0].t, 1.0));
    assert(equal(intersects.itersection[1].t, 2.0));
    assert(intersects.itersection[2].t == DBL_MIN);

    clear_intersections(&intersects);

    add_intersection(&intersects, 4.0, sp1);
    add_intersection(&intersects, 3.0, sp2);
    add_intersection(&intersects, 2.0, sp3);
    add_intersection(&intersects, 1.0, sp4);

    sort_intersects(&intersects);

    assert(intersects.count == 4);
    assert(equal(intersects.itersection[0].t, 1.0));
    assert(intersects.itersection[0].object_id == sp4);
    assert(equal(intersects.itersection[1].t, 2.0));
    assert(intersects.itersection[1].object_id == sp3);
    assert(equal(intersects.itersection[2].t, 3.0));
    assert(intersects.itersection[2].object_id == sp2);
    assert(equal(intersects.itersection[3].t, 4.0));
    assert(intersects.itersection[3].object_id == sp1);
    assert(intersects.itersection[4].t == DBL_MIN);

    clear_intersections(&intersects);

    add_intersection(&intersects,  -6.0, sp1);
    add_intersection(&intersects,   1.0, sp2);
    add_intersection(&intersects,   1.0, sp2);
    add_intersection(&intersects,  57.0, sp3);
    add_intersection(&intersects, -90.0, sp4);

    sort_intersects(&intersects);

    assert(intersects.count == 5);
    assert(equal(intersects.itersection[0].t, -90.0));
    assert(intersects.itersection[0].object_id == sp4);
    assert(equal(intersects.itersection[1].t, -6.0));
    assert(intersects.itersection[1].object_id == sp1);
    assert(equal(intersects.itersection[2].t, 1.0));
    assert(intersects.itersection[2].object_id == sp2);
    assert(equal(intersects.itersection[3].t, 1.0));
    assert(intersects.itersection[3].object_id == sp2);
    assert(equal(intersects.itersection[4].t, 57.0));
    assert(intersects.itersection[4].object_id == sp3);
    assert(intersects.itersection[5].t == DBL_MIN);

    clear_intersections(&intersects);

    add_intersection(&intersects, -22.23821,  sp1);
    add_intersection(&intersects, -1.3210377, sp2);
    add_intersection(&intersects, -1.8887736, sp3);
    add_intersection(&intersects, -0.567737,  sp4);
    add_intersection(&intersects, -1.058888f,  sp5);

    sort_intersects(&intersects);

    assert(intersects.count == 5);
    assert(equal(intersects.itersection[0].t, -22.23821));
    assert(intersects.itersection[0].object_id == sp1);
    assert(equal(intersects.itersection[1].t, -1.8887736));
    assert(intersects.itersection[1].object_id == sp3);
    assert(equal(intersects.itersection[2].t, -1.3210377));
    assert(intersects.itersection[2].object_id == sp2);
    assert(equal(intersects.itersection[3].t, -1.058888f));
    assert(intersects.itersection[3].object_id == sp5);
    assert(equal(intersects.itersection[4].t, -0.567737));
    assert(intersects.itersection[4].object_id == sp4);
    assert(intersects.itersection[5].t == DBL_MIN);

    free(sp1);
    free(sp2);
    free(sp3);
    free(sp4);
    return 0;
}

// 92 Creating A World
int creating_a_world_test() {
    world w = create_world();
    assert(w.lights  != NULL);
    assert(w.objects == NULL);
    return 0;
}

// 92 Default World
int default_world_test() {
    world w = create_default_world();
    assert(equal(w.lights->intensity.x,  1.0));
    assert(equal(w.lights->intensity.y,  1.0));
    assert(equal(w.lights->intensity.z,  1.0));
    assert(equal(w.lights->position.x, -10.0));
    assert(equal(w.lights->position.y,  10.0));
    assert(equal(w.lights->position.z, -10.0));
    assert(w.objects != NULL);
    assert(w.objects->next != NULL);
    assert(w.objects->next->next == NULL);
    assert(equal(w.objects->location.x, 0.0));
    assert(equal(w.objects->location.y, 0.0));
    assert(equal(w.objects->location.z, 0.0));
    Mat4x4 ident;
    mat4x4_set_ident(ident);
    
    // w contains s1
    shape* sp = w.objects;
    assert(mat4x4_equal(sp->transform, ident));

    // w contains s2
    sp = sp->next;
    assert(equal(sp->transform[0][0], 0.5));
    assert(equal(sp->transform[1][1], 0.5));
    assert(equal(sp->transform[2][2], 0.5));
    assert(equal(sp->transform[3][3], 1.0));
    free_default_world(&w);
    return 0;
}

// 92 Intersect a world with a ray
int intersect_world_with_ray_test() {
    world w = create_default_world();
    ray r = create_ray(0.0, 0.0, -5.0, 0.0, 0.0, 1.0);
    intersections inter = create_intersections();
    intersect_world(&w, &r, &inter);
    assert(inter.count == 4);
    assert(equal(inter.itersection[0].t, 4.0));
    assert(equal(inter.itersection[1].t, 4.5));
    assert(equal(inter.itersection[2].t, 5.5));
    assert(equal(inter.itersection[3].t, 6.0));
    free_default_world(&w);
    return 0;
}

// 93 Precomputing the state of an intersection
int prepare_computations_test() {
    ray r = create_ray(0.0, 0.0, -5.0, 0.0, 0.0, 1.0);
    shape* sp1 = create_shape(SHAPE);
    intersection inter = { 4.0, sp1 };
    intersections intersects = create_intersections();
    comps comp = prepare_computations(&inter, &r, &intersects);
    assert(equal(comp.t, inter.t));
    assert(comp.object == sp1);
    assert(equal(comp.point.x, 0.0));
    assert(equal(comp.point.y, 0.0));
    assert(equal(comp.point.z, -1.0));

    assert(equal(comp.eyev.x, 0.0));
    assert(equal(comp.eyev.y, 0.0));
    assert(equal(comp.eyev.z, -1.0));

    assert(equal(comp.normalv.x, 0.0));
    assert(equal(comp.normalv.y, 0.0));
    assert(equal(comp.normalv.z, -1.0));
    free(sp1);
    return 0;
}

// 94 The hit when an intersection occurs on the outside
int hit_when_intersect_on_outside_test() {
    ray r = create_ray(0.0, 0.0, -5.0, 0.0, 0.0, 1.0);
    shape* sp1 = create_shape(SHAPE);
    intersection inter = { 4.0, sp1 };
    intersections intersects = create_intersections();
    comps comp = prepare_computations(&inter, &r, &intersects);
    assert(comp.inside == false);
    free(sp1);
    return 0;
}

// 95 The hit when an intersection occurs on the inside
int hit_when_intersect_occurs_on_inside_test() {
    ray r = create_ray(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    shape* sp1 = create_shape(SHAPE);
    intersection inter = { 1.0, sp1 };

    intersections intersects = create_intersections();
    comps comp = prepare_computations(&inter, &r, &intersects);

    assert(equal(comp.point.x, 0.0));
    assert(equal(comp.point.y, 0.0));
    assert(equal(comp.point.z, 1.0));

    assert(equal(comp.eyev.x, 0.0));
    assert(equal(comp.eyev.y, 0.0));
    assert(equal(comp.eyev.z, -1.0));

    assert(equal(comp.normalv.x, 0.0));
    assert(equal(comp.normalv.y, 0.0));
    assert(equal(comp.normalv.z, -1.0));
    assert(comp.inside == true);

    // testing if prepare_computations overwrote stack
    assert(equal(sp1->location.x, 0.0));
    assert(equal(sp1->location.y, 0.0));
    assert(equal(sp1->location.z, 0.0));
    assert(equal(sp1->t, 1.0));

    assert(equal(r.direction_vector.x, 0.0));
    assert(equal(r.direction_vector.y, 0.0));
    assert(equal(r.direction_vector.z, 1.0));
    assert(equal(r.origin_point.x, 0.0));
    assert(equal(r.origin_point.y, 0.0));
    assert(equal(r.origin_point.z, 0.0));
    assert(equal(inter.t, 1.0));
    assert(inter.object_id == sp1);
    free(sp1);
    return 0;
}

// 95 Shading an intersection
int shading_an_intersection_test() {
    world w = create_default_world();
    ray r = create_ray(0.0, 0.0, -5.0, 0.0, 0.0, 1.0);
    // shape <- the first object in w
    intersection inter = { 4.0, w.objects };
    intersections intersects = create_intersections();
    comps comp = prepare_computations(&inter, &r, &intersects);

    // testing if prepare_computations overwrote stack
    assert(equal(r.direction_vector.x, 0.0));
    assert(equal(r.direction_vector.y, 0.0));
    assert(equal(r.direction_vector.z, 1.0));
    assert(equal(r.origin_point.x, 0.0));
    assert(equal(r.origin_point.y, 0.0));
    assert(equal(r.origin_point.z, -5.0));
    assert(equal(inter.t, 4.0));

    Mat4x4 ident;
    mat4x4_set_ident(ident);

    // w contains s1
    shape* sp = w.objects;
    assert(mat4x4_equal(sp->transform, ident));

    // w contains s2
    sp = sp->next;
    assert(equal(sp->transform[0][0], 0.5));
    assert(equal(sp->transform[1][1], 0.5));
    assert(equal(sp->transform[2][2], 0.5));
    assert(equal(sp->transform[3][3], 1.0));

    assert(equal(w.lights->intensity.x, 1.0));
    assert(equal(w.lights->intensity.y, 1.0));
    assert(equal(w.lights->intensity.z, 1.0));
    assert(equal(w.lights->position.x, -10.0));
    assert(equal(w.lights->position.y, 10.0));
    assert(equal(w.lights->position.z, -10.0));
    assert(w.objects != NULL);
    assert(w.objects->next != NULL);
    assert(w.objects->next->next == NULL);
    assert(equal(w.objects->location.x, 0.0));
    assert(equal(w.objects->location.y, 0.0));
    assert(equal(w.objects->location.z, 0.0));

    // w contains s1
    sp = w.objects;
    assert(mat4x4_equal(sp->transform, ident));

    // w contains s2
    sp = sp->next;
    assert(equal(sp->transform[0][0], 0.5));
    assert(equal(sp->transform[1][1], 0.5));
    assert(equal(sp->transform[2][2], 0.5));
    assert(equal(sp->transform[3][3], 1.0));

    assert(equal(w.lights->intensity.x, 1.0));
    assert(equal(w.lights->intensity.y, 1.0));
    assert(equal(w.lights->intensity.z, 1.0));
    assert(equal(w.lights->position.x, -10.0));
    assert(equal(w.lights->position.y, 10.0));
    assert(equal(w.lights->position.z, -10.0));
    assert(w.objects != NULL);
    assert(w.objects->next != NULL);
    assert(w.objects->next->next == NULL);
    assert(equal(w.objects->location.x, 0.0));
    assert(equal(w.objects->location.y, 0.0));
    assert(equal(w.objects->location.z, 0.0));

    // continue normal testing
    tuple color = shade_hit(&w, &comp, RECURSION_DEPTH);
    assert(equal(color.x, 0.38066119994542108f));
    assert(equal(color.y, 0.47582649284140904));
    assert(equal(color.z, 0.28549590704943306));
    free_default_world(&w);
    return 0;
}

// 95 Shading an intersection from the inside
int shading_intersection_from_inside_test() {
    world w = create_default_world();
    tuple light_pos = create_point(0.0, 0.25, 0.0);
    tuple light_color = create_point(1.0, 1.0, 1.0);

    *w.lights = create_point_light(light_pos, light_color);

    ray r = create_ray(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    intersection inter = { 0.5, w.objects->next };
    intersections intersects = create_intersections();
    comps comp = prepare_computations(&inter, &r, &intersects);

    // testing if prepare_computations overwrote stack
    assert(equal(r.direction_vector.x, 0.0));
    assert(equal(r.direction_vector.y, 0.0));
    assert(equal(r.direction_vector.z, 1.0));
    assert(equal(r.origin_point.x, 0.0));
    assert(equal(r.origin_point.y, 0.0));
    assert(equal(r.origin_point.z, 0.0));
    assert(equal(inter.t, 0.5));

    Mat4x4 ident;
    mat4x4_set_ident(ident);

    // w contains s1
    shape* sp = w.objects;
    assert(mat4x4_equal(sp->transform, ident));

    // w contains s2
    sp = sp->next;
    assert(equal(sp->transform[0][0], 0.5));
    assert(equal(sp->transform[1][1], 0.5));
    assert(equal(sp->transform[2][2], 0.5));
    assert(equal(sp->transform[3][3], 1.0));

    assert(equal(w.lights->intensity.x, 1.0));
    assert(equal(w.lights->intensity.y, 1.0));
    assert(equal(w.lights->intensity.z, 1.0));
    assert(equal(w.lights->position.x, 0.0));
    assert(equal(w.lights->position.y, 0.25));
    assert(equal(w.lights->position.z, 0.0));
    assert(w.objects != NULL);
    assert(w.objects->next != NULL);
    assert(w.objects->next->next == NULL);
    assert(equal(w.objects->location.x, 0.0));
    assert(equal(w.objects->location.y, 0.0));
    assert(equal(w.objects->location.z, 0.0));

    // w contains s1
    sp = w.objects;
    assert(mat4x4_equal(sp->transform, ident));

    // w contains s2
    sp = sp->next;
    assert(equal(sp->transform[0][0], 0.5));
    assert(equal(sp->transform[1][1], 0.5));
    assert(equal(sp->transform[2][2], 0.5));
    assert(equal(sp->transform[3][3], 1.0));

    assert(equal(w.lights->intensity.x, 1.0));
    assert(equal(w.lights->intensity.y, 1.0));
    assert(equal(w.lights->intensity.z, 1.0));
    assert(equal(w.lights->position.x, 0.0));
    assert(equal(w.lights->position.y, 0.25));
    assert(equal(w.lights->position.z, 0.0));
    assert(w.objects != NULL);
    assert(w.objects->next != NULL);
    assert(w.objects->next->next == NULL);
    assert(equal(w.objects->location.x, 0.0));
    assert(equal(w.objects->location.y, 0.0));
    assert(equal(w.objects->location.z, 0.0));

    // continue normal testing
    tuple color = shade_hit(&w, &comp, RECURSION_DEPTH);
    assert(equal(color.x, 0.90498445224856761));
    assert(equal(color.y, 0.90498445224856761));
    assert(equal(color.z, 0.90498445224856761));
    free_default_world(&w);
    return 0;
}

// 96 Color when a ray misses
int color_when_ray_misses_test() {
    world w = create_default_world();
    ray r = create_ray(0.0, 0.0, -5.0, 0.0, 1.0, 0.0);
    tuple color = color_at(&w, &r, RECURSION_DEPTH);
    assert(equal(color.x, 0.0));
    assert(equal(color.y, 0.0));
    assert(equal(color.z, 0.0));
    free_default_world(&w);
    return 0;
}

// 96 Color when a ray hits
int color_when_ray_hits_test() {
    world w = create_default_world();
    ray r = create_ray(0.0, 0.0, -5.0, 0.0, 0.0, 1.0);
    tuple color = color_at(&w, &r, RECURSION_DEPTH);
    assert(equal(color.x, 0.38066119994542108f));
    assert(equal(color.y, 0.47582649284140904));
    assert(equal(color.z, 0.28549590704943306));
    free_default_world(&w);
    return 0;
}

// 97 Color with an intersection behind the ray
int color_with_intersect_behind_ray_test() {
    world w = create_default_world();
    w.objects->material.ambient = 1.0;
    w.objects->next->material.ambient = 1.0;
    ray r = create_ray(0.0, 0.0, 0.75, 0.0, 0.0, -1.0);
    tuple color = color_at(&w, &r, RECURSION_DEPTH);
    assert(equal(w.objects->next->material.color.x, color.x));
    assert(equal(w.objects->next->material.color.y, color.y));
    assert(equal(w.objects->next->material.color.z, color.z));
    free_default_world(&w);
    return 0;
}

// 98 The transformation matrix for the default orientation
int transformation_for_default_orientation_test() {
    tuple from = create_point(0.0, 0.0, 0.0);
    tuple to = create_point(0.0, 0.0, -1.0);
    tuple up = create_vector(0.0, 1.0, 0.0);
    Mat4x4 view;
    view_transform(from, to, up, view);
    Mat4x4 ident;
    mat4x4_set_ident(ident);
    assert(mat4x4_equal(view, ident) == true);
    return 0;
}

// 98 A view transformation matrix looking in positive z direction
int view_transform_mat_looking_positive_z_dir_test() {
    tuple from = create_point(0.0, 0.0, 0.0);
    tuple to = create_point(0.0, 0.0, 1.0);
    tuple up = create_vector(0.0, 1.0, 0.0);
    Mat4x4 view;
    view_transform(from, to, up, view);
    Mat4x4 scaling;
    gen_scale_matrix(-1.0, 1.0, -1.0, scaling);
    assert(mat4x4_equal(view, scaling) == true);
    return 0;
}

// 99 The view transformation moves the world
int view_transform_moves_world_test() {
    tuple from = create_point(0.0, 0.0, 8.0);
    tuple to = create_point(0.0, 0.0, 0.0);
    tuple up = create_vector(0.0, 1.0, 0.0);
    Mat4x4 view;
    view_transform(from, to, up, view);
    Mat4x4 translate;
    gen_translate_matrix(0.0, 0.0, -8.0, translate);
    assert(mat4x4_equal(view, translate) == true);
    return 0;
}

// 99 The arbitrary view transformation
int arbitrary_view_transform_test() {
    tuple from = create_point(1.0, 3.0, 2.0);
    tuple to = create_point(4.0, -2.0, 8.0);
    tuple up = create_vector(1.0, 1.0, 0.0);
    Mat4x4 view;
    view_transform(from, to, up, view);
    assert(equal(view[0][0], -0.50709255283710986));
    assert(equal(view[1][0], 0.76771593385968007));
    assert(equal(view[2][0], -0.35856858280031806));
    assert(equal(view[3][0], 0.0));

    assert(equal(view[0][1], 0.50709255283710986));
    assert(equal(view[1][1], 0.60609152673132627));
    assert(equal(view[2][1], 0.59761430466719678f));
    assert(equal(view[3][1], 0.0));

    assert(equal(view[0][2], 0.67612340378281321));
    assert(equal(view[1][2], 0.12121830534626524));
    assert(equal(view[2][2], -0.71713716560063612));
    assert(equal(view[3][2], 0.0));

    assert(equal(view[0][3], -2.3664319132398459));
    assert(equal(view[1][3], -2.8284271247461894));
    assert(equal(view[2][3], 0.0));
    assert(equal(view[3][3], 1.0));
    return 0;
}

// 101 Constructing a camera
int constructing_camera_test() {
    camera* c = create_camera(160.0, 120.0, M_PI / 2.0);
    assert(equal(c->hsize, 160.0));
    assert(equal(c->vsize, 120.0));
    assert(equal(c->field_of_view, M_PI / 2.0));
    Mat4x4 view;
    mat4x4_set_ident(view);
    assert(mat4x4_equal(c->view_transform, view));
    free(c);
    return 0;
}

// 101 The pixel size for a horizontal canvas
int pixel_size_for_horizontal_canvas_test() {
    camera* c = create_camera(200.0, 125.0, M_PI / 2.0);
    assert(equal(c->pixel_size, 0.01));
    free(c);
    return 0;
}

// 101 The pixel size for a vertical canvas
int pixel_size_for_vertical_canvas_test() {
    camera* c = create_camera(125.0, 200.0, M_PI / 2.0);
    assert(equal(c->pixel_size, 0.01));
    free(c);
    return 0;
}

// 103 Constructing a ray through the center of the canvas
int const_a_ray_through_center_of_canvas() {
    camera* c = create_camera(201.0, 101.0, M_PI / 2.0);
    ray r = ray_for_pixel(c, 100.0, 50.0);
    assert(equal(r.origin_point.x, 0.0));
    assert(equal(r.origin_point.y, 0.0));
    assert(equal(r.origin_point.z, 0.0));
    assert(equal(r.direction_vector.x, 0.0));
    assert(equal(r.direction_vector.y, 0.0));
    assert(equal(r.direction_vector.z, -1.0));
    free(c);
    return 0;
}

// 103 Constructing a ray through a corner of the canvas
int const_a_ray_through_corner_of_canvas() {
    camera* c = create_camera(201.0, 101.0, M_PI / 2.0);
    ray r = ray_for_pixel(c, 0.0, 0.0);
    assert(equal(r.origin_point.x, 0.0));
    assert(equal(r.origin_point.y, 0.0));
    assert(equal(r.origin_point.z, 0.0));
    assert(equal(r.direction_vector.x, 0.66518642611945078f));
    assert(equal(r.direction_vector.y, 0.33259321305972539));
    assert(equal(r.direction_vector.z, -0.66851235825004807));
    free(c);
    return 0;
}

// 103 Constructng a ray when the camera is transformed
int const_a_ray_when_camera_is_transformed() {
    camera* c = create_camera(201.0, 101.0, M_PI / 2.0);
    Mat4x4 rotate;
    Mat4x4 translate;
    gen_rotate_matrix_Y(M_PI / 4.0, rotate);
    gen_translate_matrix(0.0, -2.0, 5.0, translate);
    mat4x4_mul_in_place(rotate, translate, c->view_transform);
    ray r = ray_for_pixel(c, 100.0, 50.0);
    assert(equal(r.origin_point.x, 0.0));
    assert(equal(r.origin_point.y, 2.0));
    assert(equal(r.origin_point.z, -5.0));
    assert(equal(r.direction_vector.x, sqrt(2)/2));
    assert(equal(r.direction_vector.y, 0.0));
    assert(equal(r.direction_vector.z, -sqrt(2)/2));
    free(c);
    return 0;
}

// 104 Rendering a world with a camera
int render_a_world_with_camera_test() {
    world w = create_default_world();
    camera* c = create_camera(11.0, 11.0, M_PI / 2);
    tuple from = create_point(0.0, 0.0, -5.0);
    tuple to = create_point(0.0, 0.0, 0.0);
    tuple up = create_vector(0.0, 1.0, 0.0);
    view_transform(from, up, to, c->view_transform);
    render(c, &w);
    assert(equal(canvas[5][5].x, 0.38066119994542108f));
    assert(equal(canvas[5][5].y, 0.47582649284140904));
    assert(equal(canvas[5][5].z, 0.28549590704943306));
    free_default_world(&w);
    free(c);
    return 0;
}

// extra
bool intersects_in_order_test(intersections* intersects) {
    assert(intersects != NULL);
    if (intersects->count <= 1) { return true; }
    for (int i = 0; i < intersects->count-1; i++) {
        if (intersects->itersection[i].t > intersects->itersection[i + 1].t) {
            return false;
        }
    }
    return true;
}

// 110 Lighting with the surface in shadow
int lighting_with_surface_in_shadow_test() {
    shape* sh = create_shape(SHAPE);
    tuple eyev = create_vector(0.0, 0.0, -1.0);
    tuple normalv = create_vector(0.0, 0.0, -1.0);
    tuple light_pos = create_point(0.0, 0.0, -10.0);
    tuple light_color = create_point(1.0, 1.0, 1.0);
    point_light light = create_point_light(light_pos, light_color);
    bool in_shadow = true;
    material mat = create_material_default();
    tuple result = lighting(mat, sh, &light, light_pos, eyev, normalv, in_shadow);
    assert(equal(result.x, 0.1));
    assert(equal(result.y, 0.1));
    assert(equal(result.z, 0.1));
    return 0;
}

// 111 There is no shadow when nothing is collinear with point and light
int no_shadow_when_not_collinear_point_light_test() {
    world w = create_default_world();
    tuple point = create_point(0.0, 10.0, 0.0);
    bool shadow = is_shadowed(&w, &point);
    assert(shadow == false);
    free_default_world(&w);
    return 0;
}

// 112 The shadow when an object is between the point and the light
int no_shadow_when_object_between_point_and_light_test() {
    world w = create_default_world();
    tuple point = create_point(10.0, -10.0, 10.0);
    bool shadow = is_shadowed(&w, &point);
    assert(shadow == true);
    free_default_world(&w);
    return 0;
}

// 112 The is no shadow when an object is behind the light
int no_shadow_when_object_behind_light_test() {
    world w = create_default_world();
    tuple point = create_point(-20.0, 20.0, -20.0);
    bool shadow = is_shadowed(&w, &point);
    assert(shadow == false);
    free_default_world(&w);
    return 0;
}

// 112 There is no shadow when an object is behind the point
int no_shadow_when_object_behind_point_test() {
    world w = create_default_world();
    tuple point = create_point(-2.0, 2.0, -2.0);
    bool shadow = is_shadowed(&w, &point);
    assert(shadow == false);
    free_default_world(&w);
    return 0;
}

// 114 Shade hit is given an intersection in shadow
int shade_hit_given_intersection_in_shadow_test() {
    world w = create_default_world();
    tuple light_pos = create_point(0.0, 0.0, -10.0);
    tuple light_color = create_point(1.0, 1.0, 1.0);
    point_light light = create_point_light(light_pos, light_color);
    free(w.lights);
    w.lights = &light;
    shape* sp1 = create_shape(SHAPE);
    shape* sp2 = create_shape(SHAPE);
    Mat4x4 trans_matrix;
    gen_translate_matrix(0.0, 0.0, 10.0, trans_matrix);
    mat4x4_mul_in_place(sp2->transform, trans_matrix, trans_matrix);
    mat4x4_copy(trans_matrix, sp2->transform);
    sp2->next = NULL;
    sp1->next = sp2;
    w.objects = sp1;
    ray r = create_ray(0.0, 0.0, 5.0, 0.0, 0.0, 1.0);
    intersection i = { 4.0, sp2 };
    intersections intersects = create_intersections();
    comps comp = prepare_computations(&i, &r, &intersects);
    tuple c = shade_hit(&w, &comp, RECURSION_DEPTH);
    assert(equal(c.x, 0.1));
    assert(equal(c.y, 0.1));
    assert(equal(c.z, 0.1));
    free(sp1);
    free(sp2);
    return 0;
}

// 115 Hit should offset the point
int hit_should_offset_point_test() {
    ray r = create_ray(0.0, 0.0, -5.0, 0.0, 0.0, 1.0);
    shape* sp1 = create_shape(SHAPE);
    Mat4x4 trans_matrix;
    gen_translate_matrix(0.0, 0.0, 10.0, trans_matrix);
    mat4x4_copy(trans_matrix, sp1->transform);
    intersection i = { 5.0, sp1 };
    intersections intersects = create_intersections();
    comps comp = prepare_computations(&i, &r, &intersects);
    assert(comp.over_point.z < -EPSILON / 2);
    assert(comp.point.z > comp.over_point.z);
    free(sp1);
    return 0;
}

// 119 The default transformation
int default_transformation_of_shape() {
    shape* s = create_shape(SHAPE);
    Mat4x4 ident;
    mat4x4_set_ident(ident);
    assert(mat4x4_equal(ident, s->transform));
    free(s);
    return 0;
}

// 119 Assigning a transform
int assign_transformation_of_shape() {
    shape* s = create_shape(SHAPE);
    Mat4x4 trans;
    gen_translate_matrix(2.0, 3.0, 4.0, trans);
    set_transform(s, trans);
    assert(mat4x4_equal(trans, s->transform));
    free(s);
    return 0;
}

// 119 The default material
int default_material_of_shape() {
    shape* sp = create_shape(SHAPE);

    material m1 = create_material_default();

    assert(equal(m1.color.x, 1.0));
    assert(equal(m1.color.y, 1.0));
    assert(equal(m1.color.z, 1.0));
    assert(equal(m1.color.w, 0.0));

    assert(equal(m1.ambient, 0.1));
    assert(equal(m1.diffuse, 0.9));
    assert(equal(m1.specular, 0.9));
    assert(equal(m1.shininess, 200.0));

    assert(equal(m1.color.x, sp->material.color.x));
    assert(equal(m1.color.y, sp->material.color.y));
    assert(equal(m1.color.z, sp->material.color.z));
    assert(equal(m1.color.w, sp->material.color.w));

    assert(equal(m1.ambient,   sp->material.ambient));
    assert(equal(m1.diffuse,   sp->material.diffuse));
    assert(equal(m1.specular,  sp->material.specular));
    assert(equal(m1.shininess, sp->material.shininess));

    free(sp);
    return 0;
}

// 119 Assigning a material
int assigning_material_to_a_shape() {
    shape* sp = create_shape(SHAPE);

    material m1 = create_material_default();
    m1.color.x = 9.0; m1.color.y = 8.0; m1.color.z = 7.0;
    m1.ambient = 2.5; m1.diffuse = 3.6; m1.specular = 4.6; m1.shininess = 5.3;

    sp->material = m1;

    assert(equal(sp->material.color.x, 9.0));
    assert(equal(sp->material.color.y, 8.0));
    assert(equal(sp->material.color.z, 7.0));
    assert(equal(sp->material.color.w, 0.0));

    assert(equal(sp->material.ambient, 2.5));
    assert(equal(sp->material.diffuse, 3.6));
    assert(equal(sp->material.specular, 4.6));
    assert(equal(sp->material.shininess, 5.3));

    assert(equal(m1.color.x, sp->material.color.x));
    assert(equal(m1.color.y, sp->material.color.y));
    assert(equal(m1.color.z, sp->material.color.z));
    assert(equal(m1.color.w, sp->material.color.w));

    assert(equal(m1.ambient,   sp->material.ambient));
    assert(equal(m1.diffuse,   sp->material.diffuse));
    assert(equal(m1.specular,  sp->material.specular));
    assert(equal(m1.shininess, sp->material.shininess));

    free(sp);
    return 0;
}

// 120 Intersecting a scaled sphere with a ray
// see intersect_scaled_sphere_test()

// 120 Intersecting a translated shape with a ray
// see intersecting_translated_sphere_test()

// 121 Computing the normal on a translated shape
// see compute_normal_on_sphere_test()

// 121 Computing the normal on a transformed shape
// see compute_normal_on_transformed_sphere_test()

// 122 The normal of a plane is constant everywhere
int normal_of_plane_is_const_everywhere_test() {
    shape* pl = create_shape(PLANE);
    tuple point = create_point(0.0, 0.0, 0.0);
    tuple n1 = normal_at(pl, point);
    assert(equal(n1.x, 0.0));
    assert(equal(n1.y, 1.0));
    assert(equal(n1.z, 0.0));
    
    point.x = 10.0;
    point.z = -10.0;
    tuple n2 = normal_at(pl, point);
    assert(equal(n2.x, 0.0));
    assert(equal(n2.y, 1.0));
    assert(equal(n2.z, 0.0));

    point.x = -5.0;
    point.z = 150;
    tuple n3 = normal_at(pl, point);
    assert(equal(n3.x, 0.0));
    assert(equal(n3.y, 1.0));
    assert(equal(n3.z, 0.0));

    delete_shape(pl);
    return 0;
}

// 123 Intersect with a ray parallel to the plane
int intersect_ray_parallel_to_plane_test() {
    shape* pl = create_shape(PLANE);
    ray r = create_ray(0.0, 10.0, 0.0, 0.0, 0.0, 1.0);
    intersections inter = create_intersections();
    intersect(pl, &r, &inter);
    assert(inter.count == 0);
    free(pl);
    return 0;
}

// 123 Intersect with coplanar ray
int intersect_coplanar_ray_test() {
    shape* pl = create_shape(PLANE);
    ray r = create_ray(0.0, 00.0, 0.0, 0.0, 0.0, 1.0);
    intersections inter = create_intersections();
    intersect(pl, &r, &inter);
    assert(inter.count == 0);
    free(pl);
    return 0;
}

// 123 A ray intersecting a plane from above
int intersect_ray_plane_above_test() {
    shape* pl = create_shape(PLANE);
    ray r = create_ray(0.0, 1.0, 0.0, 0.0, -1.0, 0.0);
    intersections inter = create_intersections();
    intersect(pl, &r, &inter);
    assert(inter.count == 1);
    assert(equal(inter.itersection[0].t, 1.0));
    assert(inter.itersection[0].object_id == pl);
    free(pl);
    return 0;
}

// 123 A ray intersecting a plane from below
int intersect_ray_plane_below_test() {
    shape* pl = create_shape(PLANE);
    ray r = create_ray(0.0, -1.0, 0.0, 0.0, 1.0, 0.0);
    intersections inter = create_intersections();
    intersect(pl, &r, &inter);
    assert(inter.count == 1);
    assert(equal(inter.itersection[0].t, 1.0));
    assert(inter.itersection[0].object_id == pl);
    free(pl);
    return 0;
}

// 128 Creating a stripe pattern
int creating_a_stripe_pattern_test() {
    tuple white = create_point(1.0, 1.0, 1.0);
    tuple black = create_point(0.0, 0.0, 0.0);
    pattern pat = stripe_pattern(white, black);
    assert(equal(white.x, pat.from.x));
    assert(equal(white.y, pat.from.y));
    assert(equal(white.z, pat.from.z));

    assert(equal(black.x, pat.to.x));
    assert(equal(black.y, pat.to.y));
    assert(equal(black.z, pat.to.z));
    return 0;
}

// 129 A stripe pattern is constant in y
int stripe_pattern_is_const_in_y_test() {
    tuple white = create_point(1.0, 1.0, 1.0);
    tuple black = create_point(0.0, 0.0, 0.0);
    pattern pat = stripe_pattern(white, black);
    tuple point = create_point(0.0, 0.0, 0.0);
    tuple  color = pattern_at(&pat, &point);
    assert(equal(white.x, color.x));
    assert(equal(white.y, color.y));
    assert(equal(white.z, color.z));

    point.y = 1.0;
    color = pattern_at(&pat, &point);
    assert(equal(white.x, color.x));
    assert(equal(white.y, color.y));
    assert(equal(white.z, color.z));

    point.y = 2.0;
    color = pattern_at(&pat, &point);
    assert(equal(white.x, color.x));
    assert(equal(white.y, color.y));
    assert(equal(white.z, color.z));
    return 0;
}

// 129 A stripe pattern is constant in z
int stripe_pattern_is_const_in_z_test() {
    tuple white = create_point(1.0, 1.0, 1.0);
    tuple black = create_point(0.0, 0.0, 0.0);
    pattern pat = stripe_pattern(white, black);
    tuple point = create_point(0.0, 0.0, 0.0);
    tuple  color = pattern_at(&pat, &point);
    assert(equal(white.x, color.x));
    assert(equal(white.y, color.y));
    assert(equal(white.z, color.z));

    point.z = 1.0;
    color = pattern_at(&pat, &point);
    assert(equal(white.x, color.x));
    assert(equal(white.y, color.y));
    assert(equal(white.z, color.z));

    point.z = 2.0;
    color = pattern_at(&pat, &point);
    assert(equal(white.x, color.x));
    assert(equal(white.y, color.y));
    assert(equal(white.z, color.z));
    return 0;
}

// 129 Stripe pattern alternates in x
int stripe_pattern_alternates_in_x_test() {
    tuple white = create_point(1.0, 1.0, 1.0);
    tuple black = create_point(0.0, 0.0, 0.0);
    pattern pat = stripe_pattern(white, black);
    tuple point = create_point(0.0, 0.0, 0.0);
    tuple  color = pattern_at(&pat, &point);
    assert(equal(white.x, color.x));
    assert(equal(white.y, color.y));
    assert(equal(white.z, color.z));

    point.x = 0.9;
    color = pattern_at(&pat, &point);
    assert(equal(white.x, color.x));
    assert(equal(white.y, color.y));
    assert(equal(white.z, color.z));

    point.x = 1.0;
    color = pattern_at(&pat, &point);
    assert(equal(black.x, color.x));
    assert(equal(black.y, color.y));
    assert(equal(black.z, color.z));

    point.x = -0.1;
    color = pattern_at(&pat, &point);
    assert(equal(black.x, color.x));
    assert(equal(black.y, color.y));
    assert(equal(black.z, color.z));

    point.x = -1.0;
    color = pattern_at(&pat, &point);
    assert(equal(black.x, color.x));
    assert(equal(black.y, color.y));
    assert(equal(black.z, color.z));

    point.x = -1.1;
    color = pattern_at(&pat, &point);
    assert(equal(white.x, color.x));
    assert(equal(white.y, color.y));
    assert(equal(white.z, color.z));
    return 0;
}

// 129 Lighting with a pattern applied
int lighting_with_pattern_applied() {
    shape* sh = create_shape(SHAPE);
    tuple white = create_point(1.0, 1.0, 1.0);
    tuple black = create_point(0.0, 0.0, 0.0);
    pattern pat = stripe_pattern(white, black);
    material m = create_material_default();
    m.pattern = pat;
    m.has_pattern = true;
    m.ambient = 1.0;
    m.diffuse = 0.0;
    m.specular = 0.0;
    
    tuple eyev = create_vector(0.0, 0.0, -1.0);
    tuple normalv = create_vector(0.0, 0.0, -1.0);
    point_light light = create_point_light(create_point(0.0, 0.0, 0.0), create_point(1.0, 1.0, 1.0));
    tuple c1 = lighting(m, sh, &light, create_point(0.9, 0.0, 0.0), eyev, normalv, false);
    tuple c2 = lighting(m, sh, &light, create_point(1.1, 0.0, 0.0), eyev, normalv, false);

    assert(equal(white.x, c1.x));
    assert(equal(white.y, c1.y));
    assert(equal(white.z, c1.z));

    assert(equal(black.x, c2.x));
    assert(equal(black.y, c2.y));
    assert(equal(black.z, c2.z));

    return 0;
}

// 131 Stripes with an object transformation
int stripes_with_object_transformation_test() {
    shape* sp = create_shape(SHAPE);
    Mat4x4 scale_mat;
    gen_scale_matrix(2.0, 2.0, 2.0, scale_mat);
    set_transform(sp, scale_mat);
    tuple white = create_point(1.0, 1.0, 1.0);
    tuple black = create_point(0.0, 0.0, 0.0);
    pattern pat = stripe_pattern(white, black);
    tuple world_point = create_point(1.5, 0.0, 0.0);
    tuple c1 = stripe_at_object(pat, sp, &world_point);
    assert(equal(white.x, c1.x));
    assert(equal(white.y, c1.y));
    assert(equal(white.z, c1.z));
    return 0;
}

// 131 Stripes with a pattern transformation
int stripes_with_pattern_transform_test() {
    shape* sp = create_shape(SHAPE);
    tuple white = create_point(1.0, 1.0, 1.0);
    tuple black = create_point(0.0, 0.0, 0.0);
    pattern pat = stripe_pattern(white, black);
    Mat4x4 scale_mat;
    gen_scale_matrix(2.0, 2.0, 2.0, scale_mat);
    set_pattern_transform(&pat, scale_mat);
    tuple world_point = create_point(1.5, 0.0, 0.0);
    tuple c1 = stripe_at_object(pat, sp, &world_point);
    assert(equal(white.x, c1.x));
    assert(equal(white.y, c1.y));
    assert(equal(white.z, c1.z));
    return 0;
}

// 131 Stripes with both an object and a pattern transformation
int stripes_with_both_object_and_pattern_transform_test() {
    shape* sp = create_shape(SHAPE);
    Mat4x4 sphere_scale;
    set_transform(sp, sphere_scale);
    tuple white = create_point(1.0, 1.0, 1.0);
    tuple black = create_point(0.0, 0.0, 0.0);
    pattern pat = stripe_pattern(white, black);
    Mat4x4 trans_mat;
    gen_translate_matrix(0.5, 0.0, 0.0, trans_mat);
    set_pattern_transform(&pat, trans_mat);
    tuple world_point = create_point(1.5, 0.0, 0.0);
    tuple c1 = stripe_at_object(pat, sp, &world_point);
    assert(equal(white.x, c1.x));
    assert(equal(white.y, c1.y));
    assert(equal(white.z, c1.z));
    return 0;
}

// 135 A gradient linearly interpolates between colors
int gradiant_linearly_interpolates_between_colors_test() {
    tuple white = create_point(1.0, 1.0, 1.0);
    tuple black = create_point(0.0, 0.0, 0.0);
    pattern pat = gradiant_pattern(white, black);
    tuple point1 = create_point(0.0, 0.0, 0.0);
    tuple color1 = pattern_at(&pat, &point1);
    assert(equal(white.x, color1.x));
    assert(equal(white.y, color1.y));
    assert(equal(white.z, color1.z));
    tuple point2 = create_point(0.25, 0.0, 0.0);
    tuple color2 = pattern_at(&pat, &point2);
    assert(equal(color2.x, 0.75));
    assert(equal(color2.y, 0.75));
    assert(equal(color2.z, 0.75));
    tuple point3 = create_point(0.5, 0.0, 0.0);
    tuple color3 = pattern_at(&pat, &point3);
    assert(equal(color3.x, 0.5));
    assert(equal(color3.y, 0.5));
    assert(equal(color3.z, 0.5));
    tuple point4 = create_point(0.75, 0.0, 0.0);
    tuple color4 = pattern_at(&pat, &point4);
    assert(equal(color4.x, 0.25));
    assert(equal(color4.y, 0.25));
    assert(equal(color4.z, 0.25));
    return 0;
}

// 136 A ring should extend in both x and z
int ring_pattern_should_extend_in_x_and_y_test() {
    tuple white = create_point(1.0, 1.0, 1.0);
    tuple black = create_point(0.0, 0.0, 0.0);
    pattern pat = ring_pattern(white, black);

    tuple point1 = create_point(0.0, 0.0, 0.0);
    tuple color1 = pattern_at(&pat, &point1);
    assert(equal(white.x, color1.x));
    assert(equal(white.y, color1.y));
    assert(equal(white.z, color1.z));
    tuple point2 = create_point(1.0, 0.0, 0.0);
    tuple color2 = pattern_at(&pat, &point2);
    assert(equal(black.x, color2.x));
    assert(equal(black.y, color2.y));
    assert(equal(black.z, color2.z));
    tuple point3 = create_point(0.0, 0.0, 1.0);
    tuple color3 = pattern_at(&pat, &point3);
    assert(equal(black.x, color3.x));
    assert(equal(black.y, color3.y));
    assert(equal(black.z, color3.z));
    tuple point4 = create_point(0.708f, 0.0, 0.708f);
    tuple color4 = pattern_at(&pat, &point4);
    assert(equal(black.x, color4.x));
    assert(equal(black.y, color4.y));
    assert(equal(black.z, color4.z));
    return 0;
}

// 137 Checkers should repeat in X
int checkers_pattern_should_repeat_in_x_test() {
    tuple white = create_point(1.0, 1.0, 1.0);
    tuple black = create_point(0.0, 0.0, 0.0);
    pattern pat = checkers_pattern(white, black);
    tuple point1 = create_point(0.0, 0.0, 0.0);
    tuple color1 = pattern_at(&pat, &point1);
    assert(equal(white.x, color1.x));
    assert(equal(white.y, color1.y));
    assert(equal(white.z, color1.z));
    tuple point2 = create_point(0.99, 0.0, 0.0);
    tuple color2 = pattern_at(&pat, &point2);
    assert(equal(white.x, color2.x));
    assert(equal(white.y, color2.y));
    assert(equal(white.z, color2.z));
    tuple point3 = create_point(1.01, 0.0, 0.0);
    tuple color3 = pattern_at(&pat, &point3);
    assert(equal(black.x, color3.x));
    assert(equal(black.y, color3.y));
    assert(equal(black.z, color3.z));
    return 0;
}

// 137 Checkers should repeat in Y
int checkers_pattern_should_repeat_in_y_test() {
    tuple white = create_point(1.0, 1.0, 1.0);
    tuple black = create_point(0.0, 0.0, 0.0);
    pattern pat = checkers_pattern(white, black);
    tuple point1 = create_point(0.0, 0.0, 0.0);
    tuple color1 = pattern_at(&pat, &point1);
    assert(equal(white.x, color1.x));
    assert(equal(white.y, color1.y));
    assert(equal(white.z, color1.z));
    tuple point2 = create_point(0.0, 0.99, 0.0);
    tuple color2 = pattern_at(&pat, &point2);
    assert(equal(white.x, color2.x));
    assert(equal(white.y, color2.y));
    assert(equal(white.z, color2.z));
    tuple point3 = create_point(0.01, 1.01, 0.0);
    tuple color3 = pattern_at(&pat, &point3);
    assert(equal(black.x, color3.x));
    assert(equal(black.y, color3.y));
    assert(equal(black.z, color3.z));
    return 0;
}

// 137 Checkers should repeat in Z
int checkers_pattern_should_repeat_in_z_test() {
    tuple white = create_point(1.0, 1.0, 1.0);
    tuple black = create_point(0.0, 0.0, 0.0);
    pattern pat = checkers_pattern(white, black);
    tuple point1 = create_point(0.0, 0.0, 0.0);
    tuple color1 = pattern_at(&pat, &point1);
    assert(equal(white.x, color1.x));
    assert(equal(white.y, color1.y));
    assert(equal(white.z, color1.z));
    tuple point2 = create_point(0.0, 0.0, 0.99);
    tuple color2 = pattern_at(&pat, &point2);
    assert(equal(white.x, color2.x));
    assert(equal(white.y, color2.y));
    assert(equal(white.z, color2.z));
    tuple point3 = create_point(0.01, 0.0, 1.01);
    tuple color3 = pattern_at(&pat, &point3);
    assert(equal(black.x, color3.x));
    assert(equal(black.y, color3.y));
    assert(equal(black.z, color3.z));
    return 0;
}

// 143 Precomputing the reflection vector
int precompute_reflection_vector_test() {
    shape* sp = create_shape(PLANE);
    ray r = create_ray(0.0, 1.0, -1.0, 0.0, -sqrt(2)/2, sqrt(2)/2);
    intersections intersects = create_intersections();
    add_intersection(&intersects, sqrt(2.0), sp);
    comps comp = prepare_computations(&intersects.itersection[0], &r, &intersects);
    assert(equal(comp.reflectv.x, 0.0));
    assert(equal(comp.reflectv.y, sqrt(2) / 2));
    assert(equal(comp.reflectv.z, sqrt(2) / 2));
    return 0;
}

// 144 reflected color for a nonreflective material
int reflected_color_for_non_reflective_material_test() {
    world w = create_default_world();
    ray r = create_ray(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    shape* sp = create_shape(SHAPE);
    material sp_material = create_material_default();
    sp_material.ambient = 1.0;
    sp->material = sp_material;
    w.objects->next = sp; // sphere is 2nd object in world
    intersections intersects = create_intersections();
    add_intersection(&intersects, 1.0, sp);
    comps comp = prepare_computations(&intersects.itersection[0], &r, &intersects);
    tuple color = reflected_color(&w, &comp, RECURSION_DEPTH);
    assert(equal(0.0, color.x));
    assert(equal(0.0, color.y));
    assert(equal(0.0, color.z));
    return 0;
}

// 144 reflected color for a reflective material
int reflected_color_for_reflective_material_test() {
    world w = create_default_world();
    shape* pl = create_shape(PLANE);
    material plane_material = create_material_default();
    plane_material.reflective = 0.5;
    pl->material = plane_material;
    Mat4x4 trans_mat;
    gen_translate_matrix(0.0, -1.0, 0.0, trans_mat);
    mat4x4_copy(trans_mat, pl->transform);
    add_shape_to_world(pl, &w);
    ray r = create_ray(0.0, 0.0, -3.0, 0.0, -sqrt(2)/2, sqrt(2)/2);
    intersections intersects = create_intersections();
    add_intersection(&intersects, sqrt(2.0), pl);
    comps comp = prepare_computations(&intersects.itersection[0], &r, &intersects);
    tuple color = reflected_color(&w, &comp, RECURSION_DEPTH);
    assert(equal(0.19033077235655871, color.x));
    assert(equal(0.23791346190051155, color.y));
    assert(equal(0.14274808281260587, color.z));
    return 0;
}

// 145 shade_hit with a reflective material
int shade_hit_with_reflective_material_test() {
    world w = create_default_world();
    shape* pl = create_shape(PLANE);
    material pl_material = create_material_default();
    pl_material.reflective = 0.5;
    pl->material = pl_material;
    Mat4x4 trans_mat;
    gen_translate_matrix(0.0, -1.0, 0.0, trans_mat);
    mat4x4_copy(trans_mat, pl->transform);
    add_shape_to_world(pl, &w);
    ray r = create_ray(0.0, 0.0, -3.0, 0.0, -sqrt(2)/2, sqrt(2)/2);
    intersections intersects = create_intersections();
    add_intersection(&intersects, sqrt(2.0), pl);
    comps comp = prepare_computations(&intersects.itersection[0], &r, &intersects);
    tuple color = shade_hit(&w, &comp, RECURSION_DEPTH);
    assert(equal(0.87675614729320861, color.x));
    assert(equal(0.92433883683716145, color.y));
    assert(equal(0.82917345774925577, color.z));
    return 0;
}

// 146 color_at with mutually reflective surfaces
int color_at_with_mutually_reflective_surfaces_test() {
    world w = create_world();
    point_light plight = create_point_light(create_point(0.0, 0.0, 0.0), create_point(1.0, 1.0, 1.0));
    w.lights = &plight;

    shape* lower = create_shape(PLANE);
    material lower_material = create_material_default();
    lower_material.reflective = 1.0;
    lower->material = lower_material;
    Mat4x4 trans_mat1;
    gen_translate_matrix(0.0, -1.0, 0.0, trans_mat1);
    mat4x4_copy(trans_mat1, lower->transform);
    add_shape_to_world(lower, &w);

    shape* upper = create_shape(PLANE);
    material upper_material = create_material_default();
    upper_material.reflective = 1.0;
    upper->material = upper_material;
    Mat4x4 trans_mat2;
    gen_translate_matrix(0.0, 1.0, 0.0, trans_mat2);
    mat4x4_copy(trans_mat2, upper->transform);
    add_shape_to_world(upper, &w);

    ray r = create_ray(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    color_at(&w, &r, RECURSION_DEPTH);
    // NOTE: This test should eventually exit to pass
    return 0;
}

// 147 reflected color at the maximum recursive depth
int reflected_color_at_max_recursive_depth_test() {
    world w = create_default_world();
    shape* pl = create_shape(PLANE);
    material pl_material = create_material_default();
    pl_material.reflective = 0.5;
    pl->material = pl_material;
    Mat4x4 trans_mat;
    gen_translate_matrix(0.0, -1.0, 0.0, trans_mat);
    mat4x4_copy(trans_mat, pl->transform);
    add_shape_to_world(pl, &w);
    ray r = create_ray(0.0, 0.0, -3.0, 0.0, -sqrt(2)/2, sqrt(2)/2);
    
    intersections intersects = create_intersections();
    add_intersection(&intersects, sqrt(2.0), pl);
    comps comp = prepare_computations(&intersects.itersection[0], &r, &intersects);
    tuple color = reflected_color(&w, &comp, 0);
    assert(equal(0.0, color.x));
    assert(equal(0.0, color.y));
    assert(equal(0.0, color.z));
    return 0;
}

// 151 A helper for producing a sphere with a glassy material
int helper_for_producing_sphere_with_glassy_material_test() {
    shape* glass_sphere = create_glass_sphere();
    Mat4x4 ident;
    mat4x4_set_ident(ident);
    mat4x4_equal(glass_sphere->transform, ident);
    assert(equal(glass_sphere->material.transparency, 1.0));
    assert(equal(glass_sphere->material.refractive_index, 1.5));
    return 0;
}

int containers_tests() {
    containers test_containers = containers_create();

    shape* pl1 = create_shape(PLANE);
    shape* pl2 = create_shape(PLANE);

    comps comp1 = create_comp();
    comp1.object = pl1;
    comps comp2 = create_comp();
    comp2.object = pl2;

    for (int i = 0; i < CONTAINERS_SIZE; i++) {
        assert(test_containers.shape_ptr[i] == NULL);
    }

    assert(containers_is_empty(&test_containers) == true);
    // clearing empty container does not crash
    containers_clear(&test_containers);
    assert(containers_is_empty(&test_containers) == true);

    for (int i = 0; i < CONTAINERS_SIZE; i++) {
        assert(test_containers.shape_ptr[i] == NULL);
    }

    // empty container should return null for empty set
    assert(container_last(&test_containers) == NULL);

    assert(containers_includes(&test_containers, comp1.object) == false);
    assert(containers_includes(&test_containers, comp2.object) == false);

    container_append(&test_containers, comp1.object);
    assert(containers_includes(&test_containers, comp1.object) == true);
    assert(container_last(&test_containers) == comp1.object);

    assert(test_containers.shape_ptr[0] == comp1.object);
    assert(test_containers.shape_ptr[1] == NULL);

    container_append(&test_containers, comp2.object);
    assert(containers_includes(&test_containers, comp1.object) == true);
    assert(containers_includes(&test_containers, comp2.object) == true);
    assert(container_last(&test_containers) == comp2.object);

    assert(test_containers.shape_ptr[0] == comp1.object);
    assert(test_containers.shape_ptr[1] == comp2.object);
    assert(test_containers.shape_ptr[2] == NULL);

    assert(containers_includes(&test_containers, comp1.object) == true);
    assert(containers_includes(&test_containers, comp2.object) == true);

    assert(container_remove(&test_containers, comp1.object) == true);
    assert(container_last(&test_containers) == comp2.object);

    assert(test_containers.shape_ptr[0] == comp2.object);
    assert(test_containers.shape_ptr[1] == NULL);
    assert(test_containers.shape_ptr[2] == NULL);

    container_append(&test_containers, comp1.object);
    assert(container_last(&test_containers) == comp1.object);

    assert(test_containers.shape_ptr[0] == comp2.object);
    assert(test_containers.shape_ptr[1] == comp1.object);
    assert(test_containers.shape_ptr[2] == NULL);

    assert(containers_includes(&test_containers, comp1.object) == true);
    assert(containers_includes(&test_containers, comp2.object) == true);

    containers_clear(&test_containers);

    for (int i = 0; i < CONTAINERS_SIZE; i++) {
        assert(test_containers.shape_ptr[i] == NULL);
    }

    assert(containers_includes(&test_containers, comp1.object) == false);
    assert(containers_includes(&test_containers, comp2.object) == false);
    assert(containers_is_empty(&test_containers) == true);
    return 0;
}

// 152 Finding n1 and n2 at various intersections
int finding_n1_and_n2_at_various_intersections_test() {
    shape* glass_sphere_a = create_glass_sphere();
    Mat4x4 scale_mat;
    gen_scale_matrix(2.0, 2.0, 2.0, scale_mat);
    mat4x4_copy(scale_mat, glass_sphere_a->transform);
    glass_sphere_a->material.refractive_index = 1.5;

    shape* glass_sphere_b = create_glass_sphere();
    Mat4x4 translate_mat_b;
    gen_translate_matrix(0.0, 0.0, -0.25, translate_mat_b);
    mat4x4_copy(translate_mat_b, glass_sphere_b->transform);
    glass_sphere_b->material.refractive_index = 2.0;

    shape* glass_sphere_c = create_glass_sphere();
    Mat4x4 translate_mat_c;
    gen_translate_matrix(0.0, 0.0, 0.25, translate_mat_c);
    mat4x4_copy(translate_mat_c, glass_sphere_c->transform);
    glass_sphere_c->material.refractive_index = 2.5;

    ray r = create_ray(0.0, 0.0, -4.0, 0.0, 0.0, 1.0);
    intersections intersects = create_intersections();
    add_intersection(&intersects, 2.0,  glass_sphere_a);
    add_intersection(&intersects, 2.75, glass_sphere_b);
    add_intersection(&intersects, 3.25, glass_sphere_c);
    add_intersection(&intersects, 4.75, glass_sphere_b);
    add_intersection(&intersects, 5.25, glass_sphere_c);
    add_intersection(&intersects, 6.0,  glass_sphere_a);

    comps glass_comps = prepare_computations(&intersects.itersection[0], &r, &intersects);
    assert(equal(glass_comps.n1, 1.0));
    assert(equal(glass_comps.n2, 1.5));
    glass_comps = prepare_computations(&intersects.itersection[1], &r, &intersects);
    assert(equal(glass_comps.n1, 1.5));
    assert(equal(glass_comps.n2, 2.0));
    glass_comps = prepare_computations(&intersects.itersection[2], &r, &intersects);
    assert(equal(glass_comps.n1, 2.0));
    assert(equal(glass_comps.n2, 2.5));
    glass_comps = prepare_computations(&intersects.itersection[3], &r, &intersects);
    assert(equal(glass_comps.n1, 2.5));
    assert(equal(glass_comps.n2, 2.5)); 
    glass_comps = prepare_computations(&intersects.itersection[4], &r, &intersects);
    assert(equal(glass_comps.n1, 2.5));
    assert(equal(glass_comps.n2, 1.5));
    glass_comps = prepare_computations(&intersects.itersection[5], &r, &intersects);
    assert(equal(glass_comps.n1, 1.5));
    assert(equal(glass_comps.n2, 1.0));
    return 0;
}

// 154 The under point is offset below the surface
int under_point_is_offset_below_the_suface_test() {
    ray r = create_ray(0.0, 0.0, -5.0, 0.0, 0.0, 1.0);

    shape* glass_sphere_a = create_glass_sphere();
    Mat4x4 translate_mat_b;
    gen_translate_matrix(0.0, 0.0, 1.0, translate_mat_b);
    mat4x4_copy(translate_mat_b, glass_sphere_a->transform);
 
    intersections intersects = create_intersections();
    add_intersection(&intersects, 5.0, glass_sphere_a);

    comps glass_comps = prepare_computations(&intersects.itersection[0], &r, &intersects);
    assert(glass_comps.under_point.z > EPSILON / 2);
    assert(glass_comps.point.z < glass_comps.under_point.z);
    return 0;
}

// 155 The refracted color with an opaque surface
int refracted_color_with_opaque_surface_test() {
    world w = create_default_world();
    shape* sh = create_shape(SHAPE);
    w.objects = sh;
    ray r = create_ray(0.0, 0.0, -5.0, 0.0, 0.0, 1.0);
    intersections intersects = create_intersections();
    add_intersection(&intersects, 4.0, sh);
    add_intersection(&intersects, 6.0, sh);
    comps comp = prepare_computations(&intersects.itersection[0], &r, &intersects);
    tuple color = refracted_color(&w, &comp, 5);
    assert(equal(0.0, color.x));
    assert(equal(0.0, color.y));
    assert(equal(0.0, color.z));
    return 0;
}

// 156 The refracted color at maximum recursive depth
int refracted_color_with_maximum_recursive_depth_test() {
    world w = create_default_world();
    shape* sh = create_shape(SHAPE);

    material sh_material = create_material_default();
    sh_material.transparency = 1.0;
    sh_material.refractive_index = 1.5;
    sh->material = sh_material;
    w.objects = sh;

    ray r = create_ray(0.0, 0.0, -5.0, 0.0, 0.0, 1.0);
    intersections intersects = create_intersections();
    add_intersection(&intersects, 4.0, sh);
    add_intersection(&intersects, 6.0, sh);
    comps comp = prepare_computations(&intersects.itersection[0], &r, &intersects);
    tuple color = refracted_color(&w, &comp, 0);
    assert(equal(0.0, color.x));
    assert(equal(0.0, color.y));
    assert(equal(0.0, color.z));
    return 0;
}

// 157 The reflected color under total internal reflection
int reflected_color_under_total_internal_reflection_test() {
    world w = create_default_world();
    shape* sh = create_shape(SHAPE);

    material sh_material = create_material_default();
    sh_material.transparency = 1.0;
    sh_material.refractive_index = 1.5;
    sh->material = sh_material;
    w.objects = sh;

    ray r = create_ray(0.0, 0.0, sqrt(2)/2, 0.0, 1.0, 0.0);
    intersections intersects = create_intersections();
    add_intersection(&intersects, -sqrt(2)/2, sh);
    add_intersection(&intersects, sqrt(2)/2, sh);
    comps comp = prepare_computations(&intersects.itersection[1], &r, &intersects);
    tuple color = refracted_color(&w, &comp, 5);
    assert(equal(0.0, color.x));
    assert(equal(0.0, color.y));
    assert(equal(0.0, color.z));
    return 0;
}

// 158 The refracted color with a refracted ray
int refracted_color_with_refracted_ray_test() {
    world w = create_default_world();

    tuple white = create_point(1.0, 1.0, 1.0);
    pattern pat = test_pattern(white, white);
    w.objects->material.pattern = pat;
    w.objects->material.has_pattern = true;
    w.objects->material.ambient = 1.0;


    w.objects->next->material.transparency = 1.0;
    w.objects->next->material.refractive_index = 1.5;

    ray r = create_ray(0.0, 0.0, 0.1, 0.0, 1.0, 0.0);
    intersections intersects = create_intersections();
    add_intersection(&intersects, -0.9899, w.objects);
    add_intersection(&intersects, -0.4899, w.objects->next);
    add_intersection(&intersects, 0.4899, w.objects->next);
    add_intersection(&intersects, 0.9899, w.objects);
    comps comp = prepare_computations(&intersects.itersection[2], &r, &intersects);
    tuple color = refracted_color(&w, &comp, 5);
    assert(equal(0.0, color.x));
    assert(equal(0.99888366962741248f, color.y));
    assert(equal(0.047216676637331971, color.z));
    return 0;
}

// 159 shade_hit with a transparent material
int shade_hit_with_transparent_material_test() {
    world w = create_default_world();

    shape* floor = create_shape(PLANE);
    Mat4x4 floor_transform;
    gen_translate_matrix(0.0, -1.0, 0.0, floor_transform);
    mat4x4_copy(floor_transform, floor->transform);

    material floor_material = create_material_default();
    floor_material.transparency = 0.5;
    floor_material.refractive_index = 1.5;
    floor->material = floor_material;
    w.objects = floor;

    shape* ball = create_shape(SPHERE);
    Mat4x4 ball_transform;
    gen_translate_matrix(0.0, -3.5, -0.5, ball_transform);
    mat4x4_copy(ball_transform, ball->transform);

    material ball_material = create_material_default();
    ball_material.color = create_point(1.0, 0.0, 0.0);
    ball_material.ambient = 0.5;
    ball->material = ball_material;
    w.objects->next = ball;

    ray r = create_ray(0.0, 0.0, -3.0, 0.0, -sqrt(2.0)/2.0, sqrt(2.0)/2.0);
    intersections intersects = create_intersections();
    add_intersection(&intersects, sqrt(2), w.objects);
    comps comp = prepare_computations(&intersects.itersection[0], &r, &intersects);
    tuple color = shade_hit(&w, &comp, 5);
    assert(equal(0.93642537493664990, color.x));
    assert(equal(0.68642537493664990, color.y));
    assert(equal(0.68642537493664990, color.z));
    return 0;
}

// 161 The Schlick approximation under total internal reflection
int schlick_approximation_under_total_internal_reflection_test() {
    shape* glass_sphere = create_glass_sphere();
    ray r = create_ray(0.0, 0.0, sqrt(2.0) / 2.0, 0.0, 1.0, 0.0);
    intersections intersects = create_intersections();
    add_intersection(&intersects, -sqrt(2)/2.0, glass_sphere);
    add_intersection(&intersects, sqrt(2)/2.0, glass_sphere);
    comps comp = prepare_computations(&intersects.itersection[1], &r, &intersects);
    double reflectance = schlick(&comp);
    assert(equal(reflectance, 1.0));
    return 0;
}

// 162 The Schlick approximation with a perpendicular viewing angle
int schlick_approximation_with_perpedicular_viewing_angle_test() {
    shape* glass_sphere = create_glass_sphere();
    ray r = create_ray(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    intersections intersects = create_intersections();
    add_intersection(&intersects, -1.0, glass_sphere);
    add_intersection(&intersects, 1.0, glass_sphere);
    comps comp = prepare_computations(&intersects.itersection[1], &r, &intersects);
    double reflectance = schlick(&comp);
    assert(equal(reflectance, 0.04));
    return 0;
}

// 163 The Schlick approximation with a small angle and n2 > n1
int schlick_approximation_with_small_angle_n2_gt_n1_test() {
    shape* glass_sphere = create_glass_sphere();
    ray r = create_ray(0.0, 0.99, -2.0, 0.0, 0.0, 1.0);
    intersections intersects = create_intersections();
    add_intersection(&intersects, 1.8589, glass_sphere);
    comps comp = prepare_computations(&intersects.itersection[0], &r, &intersects);
    double reflectance = schlick(&comp);
    assert(equal(reflectance, 0.48873));
    return 0;
}

// Extra tests for add_shape_to_world
int add_shape_to_world_tests() {
    world w = create_default_world();

    shape* sh1 = create_shape(PLANE);
    shape* sh2 = create_shape(PLANE);

    shape* default1 = w.objects;
    shape* default2 = w.objects->next;

    assert(w.objects->next->next == NULL);

    add_shape_to_world(sh1, &w);
    assert(w.objects == default1);
    assert(w.objects->next = default2);
    assert(w.objects->next->next == sh1);

    add_shape_to_world(sh2, &w);
    assert(w.objects == default1);
    assert(w.objects->next = default2);
    assert(w.objects->next->next == sh1);
    assert(w.objects->next->next->next == sh2);
    return 0;
}

// 164 shade_hit() with reflective, transparent material
int shade_hit_with_reflective_transparent_material_test() {
    world w = create_default_world();
    ray r = create_ray(0.0, 0.0, -3.0, 0.0, -sqrt(2.0) / 2.0, sqrt(2.0) / 2.0);

    shape* floor = create_shape(PLANE);
    Mat4x4 floor_transform;
    gen_translate_matrix(0.0, -1.0, 0.0, floor_transform);
    mat4x4_copy(floor_transform, floor->transform);

    material floor_material = create_material_default();
    floor_material.reflective = 0.5;
    floor_material.transparency = 0.5;
    floor_material.refractive_index = 1.5;
    floor->material = floor_material;

    add_shape_to_world(floor, &w);

    shape* ball = create_shape(SPHERE);

    material ball_material = create_material_default();
    ball_material.color = create_point(1.0, 0.0, 0.0);
    ball_material.ambient = 0.5;
    ball->material = ball_material;

    Mat4x4 ball_transform;
    gen_translate_matrix(0.0, -3.5, -0.5, ball_transform);
    mat4x4_copy(ball_transform, ball->transform);

    add_shape_to_world(ball, &w);

    intersections intersects = create_intersections();
    add_intersection(&intersects, sqrt(2), floor);
    assert(intersects.count == 1);
    comps comp = prepare_computations(&intersects.itersection[0], &r, &intersects);
    tuple color = shade_hit(&w, &comp, 5);
    assert(equal(0.933915102, color.x));
    assert(equal(0.69643419, color.y));
    assert(equal(0.69243009, color.z));
    return 0;
}

#endif

/*------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------*/

// 72 Hint #4
void render_sphere() {
    tuple ray_origin = create_point(0.0, 0.0, -5.0);

    double wall_z = 10.0;
    double wall_size = 7.0;
    double pixel_size = wall_size / HORIZONTAL_SIZE;
    double half = wall_size / 2.0;

    material m = create_material_default();
    m.color.x = 1.0; m.color.y = 0.2; m.color.z = 1.0;
    shape* sphere1 = create_shape(SHAPE);
    sphere1->material = m;
    sphere1->material.ambient = 0.15;
    sphere1->material.color.x = 0.254901;
    sphere1->material.color.y = 0.423529;
    sphere1->material.color.z = 0.58823;
    sphere1->material.shininess = 100.0;

    tuple l_color = create_vector(1.0, 1.0, 1.0);
    tuple l_position = create_point(-10.0, -10.0, -10.0);
    point_light p_light = create_point_light(l_position, l_color);
    shape* sh = create_shape(SHAPE);

    for (int y = 0; y < HORIZONTAL_SIZE; ++y) {
        double world_y = half - pixel_size * y;
        for (int x = 0; x < VERTICAL_SIZE; ++x) {
            double world_x = -half + pixel_size * x;
            tuple position1 = create_point(world_x, world_y, wall_z);
            tuple posRayOrigin = tuple_sub(position1, ray_origin);
            tuple normRayOrigin = tuple_normalize(posRayOrigin);
            ray ray_to_draw = create_ray(ray_origin.x, ray_origin.y, ray_origin.z, normRayOrigin.x, normRayOrigin.y, normRayOrigin.z);
            ray_to_draw.direction_vector = tuple_normalize(ray_to_draw.direction_vector);
            intersections inter = create_intersections();
            intersect(sphere1, &ray_to_draw, &inter);
            intersection* hit_intersection = hit(&inter);
            if (hit_intersection) {
                tuple point2 = position(ray_to_draw, hit_intersection->t);
                tuple normal = normal_at(hit_intersection->object_id, point2);
                tuple eye = tuple_negate(ray_to_draw.direction_vector);
                tuple pix_color = lighting(hit_intersection->object_id->material, sh, &p_light, point2, eye, normal, true);
                assert(pix_color.x <= 255 && pix_color.x >= 0);
                assert(pix_color.y <= 255 && pix_color.y >= 0);
                assert(pix_color.z <= 255 && pix_color.z >= 0);
                write_pixel(x, y, pix_color);
            }
        }
    }
}

// 106 Render complete world
void render_complete_world() {
    world w = create_default_world();

    // 1. floor extremely flattened sphere with matte texture
    shape* floor = create_shape(SHAPE);
    Mat4x4 floor_transform;
    gen_scale_matrix(10.0, 0.01, 10.0, floor_transform);
    mat4x4_mul_in_place(floor_transform, floor->transform, floor->transform);

    assert(equal(floor->t, 1.0));
    assert(equal(floor->location.x, 0.0));
    assert(equal(floor->location.y, 0.0));
    assert(equal(floor->location.z, 0.0));
    assert(equal(floor->location.w, 0.0));

    assert(equal(floor->transform[0][0], 10.0));
    assert(equal(floor->transform[1][1], 0.01));
    assert(equal(floor->transform[2][2], 10.0));
    assert(equal(floor->transform[3][3], 1.0));

    material floor_material = create_material_default();
    floor_material.color = create_point(0.9, 0.9, 0.9); // gray
    floor_material.specular = 0.0;
    floor->material = floor_material;

    assert(equal(floor->material.color.x, 1.0));
    assert(equal(floor->material.color.y, 0.9));
    assert(equal(floor->material.color.z, 0.9));
    assert(equal(floor->material.specular, 0.0));

    assert(equal(floor->material.ambient, 0.1));
    assert(equal(floor->material.diffuse, 0.9));

    // 2. wall on left has same scale and color but also rotated and translated into place
    shape* left_wall = create_shape(SHAPE);
    Mat4x4 translate_left;

    floor_material.color = create_point(1.0, 0.0, 0.0); // red
    left_wall->material = floor_material;

    gen_translate_matrix(0.0, 0.0, 5.0, translate_left);
    Mat4x4 rotate_y_left;
    gen_rotate_matrix_Y(-M_PI / 4.0, rotate_y_left);
    Mat4x4 rotate_x_left;
    gen_rotate_matrix_X(M_PI / 2.0, rotate_x_left);
    Mat4x4 scale_left;
    gen_scale_matrix(10.0, 0.01, 10.0, scale_left);
    Mat4x4 final_transform_left;
    mat4x4_set_ident(final_transform_left);
    mat4x4_mul_in_place(translate_left, final_transform_left, final_transform_left);

    assert(equal(final_transform_left[0][0], 1.0));
    assert(equal(final_transform_left[1][0], 0.0));
    assert(equal(final_transform_left[2][0], 0.0));
    assert(equal(final_transform_left[3][0], 0.0));

    assert(equal(final_transform_left[0][1], 0.0));
    assert(equal(final_transform_left[1][1], 1.0));
    assert(equal(final_transform_left[2][1], 0.0));
    assert(equal(final_transform_left[3][1], 0.0));

    assert(equal(final_transform_left[0][2], 0.0));
    assert(equal(final_transform_left[1][2], 0.0));
    assert(equal(final_transform_left[2][2], 1.0));
    assert(equal(final_transform_left[3][2], 0.0));

    assert(equal(final_transform_left[0][3], 0.0));
    assert(equal(final_transform_left[1][3], 0.0));
    assert(equal(final_transform_left[2][3], 5.0));
    assert(equal(final_transform_left[3][3], 1.0));

    mat4x4_mul_in_place(final_transform_left, rotate_y_left, final_transform_left);

    assert(equal(final_transform_left[0][0], 0.707106781200000));
    assert(equal(final_transform_left[1][0], 0.0));
    assert(equal(final_transform_left[2][0], 0.707106781200000));
    assert(equal(final_transform_left[3][0], 0.0));

    assert(equal(final_transform_left[0][1], 0.0));
    assert(equal(final_transform_left[1][1], 1.0));
    assert(equal(final_transform_left[2][1], 0.0));
    assert(equal(final_transform_left[3][1], 0.0));

    assert(equal(final_transform_left[0][2], -0.707106781200000));
    assert(equal(final_transform_left[1][2], 0.0));
    assert(equal(final_transform_left[2][2], 0.707106781200000));
    assert(equal(final_transform_left[3][2], 0.0));

    assert(equal(final_transform_left[0][3], 0.0));
    assert(equal(final_transform_left[1][3], 0.0));
    assert(equal(final_transform_left[2][3], 5.0));
    assert(equal(final_transform_left[3][3], 1.0));

    mat4x4_mul_in_place(final_transform_left, rotate_x_left, final_transform_left);

    assert(equal(final_transform_left[0][0], 0.707106781200000));
    assert(equal(final_transform_left[1][0], 0.0));
    assert(equal(final_transform_left[2][0], 0.707106781200000));
    assert(equal(final_transform_left[3][0], 0.0));

    assert(equal(final_transform_left[0][1], -0.707106781200000));
    assert(equal(final_transform_left[1][1], 0.0));
    assert(equal(final_transform_left[2][1], 0.707106781200000));
    assert(equal(final_transform_left[3][1], 0.0));

    assert(equal(final_transform_left[0][2], 0.0));
    assert(equal(final_transform_left[1][2], -1.0));
    assert(equal(final_transform_left[2][2], 0.0));
    assert(equal(final_transform_left[3][2], 0.0));

    assert(equal(final_transform_left[0][3], 0.0));
    assert(equal(final_transform_left[1][3], 0.0));
    assert(equal(final_transform_left[2][3], 5.0));
    assert(equal(final_transform_left[3][3], 1.0));

    mat4x4_mul_in_place(final_transform_left, scale_left, final_transform_left);

    assert(equal(final_transform_left[0][0], 7.071067812000001));
    assert(equal(final_transform_left[1][0], 0.0));
    assert(equal(final_transform_left[2][0], 7.071067812000001));
    assert(equal(final_transform_left[3][0], 0.0));

    assert(equal(final_transform_left[0][1], -.007071067670578643));
    assert(equal(final_transform_left[1][1], 0.0));
    assert(equal(final_transform_left[2][1], .007071067670578643));
    assert(equal(final_transform_left[3][1], 0.0));

    assert(equal(final_transform_left[0][2], 0.0));
    assert(equal(final_transform_left[1][2], -10.0));
    assert(equal(final_transform_left[2][2], 0.0));
    assert(equal(final_transform_left[3][2], 0.0));

    assert(equal(final_transform_left[0][3], 0.0));
    assert(equal(final_transform_left[1][3], 0.0));
    assert(equal(final_transform_left[2][3], 5.0));
    assert(equal(final_transform_left[3][3], 1.0));

    mat4x4_copy(final_transform_left, left_wall->transform);

    assert(mat4x4_equal(final_transform_left, left_wall->transform));

    assert(equal(left_wall->transform[0][0], 7.071067812000001));
    assert(equal(left_wall->transform[1][0], 0.0));
    assert(equal(left_wall->transform[2][0], 7.071067812000001));
    assert(equal(left_wall->transform[3][0], 0.0));

    assert(equal(left_wall->transform[0][1], -.007071067670578643));
    assert(equal(left_wall->transform[1][1], 0.0));
    assert(equal(left_wall->transform[2][1], .007071067670578643));
    assert(equal(left_wall->transform[3][1], 0.0));

    assert(equal(left_wall->transform[0][2], 0.0));
    assert(equal(left_wall->transform[1][2], -10.0));
    assert(equal(left_wall->transform[2][2], 0.0));
    assert(equal(left_wall->transform[3][2], 0.0));

    assert(equal(left_wall->transform[0][3], 0.0));
    assert(equal(left_wall->transform[1][3], 0.0));
    assert(equal(left_wall->transform[2][3], 5.0));
    assert(equal(left_wall->transform[3][3], 1.0));

    left_wall->material = floor_material;

    assert(equal(translate_left[0][3], 0.0));
    assert(equal(translate_left[1][3], 0.0));
    assert(equal(translate_left[2][3], 5.0));
    assert(equal(translate_left[3][3], 1.0));

    assert(equal(rotate_y_left[0][0], cos(-M_PI / 4.0)));
    assert(equal(rotate_y_left[0][2], sin(-M_PI / 4.0)));
    assert(equal(rotate_y_left[2][0], -sin(-M_PI / 4.0)));
    assert(equal(rotate_y_left[2][2], cos(-M_PI / 4.0)));

    assert(equal(rotate_x_left[0][0], 1.0));
    assert(equal(rotate_x_left[1][1], cos(M_PI / 2.0)));
    assert(equal(rotate_x_left[1][2], -sin(M_PI / 2.0)));
    assert(equal(rotate_x_left[2][1], sin(M_PI / 2.0)));
    assert(equal(rotate_x_left[2][2], cos(M_PI / 2.0)));

    assert(equal(scale_left[0][0], 10.0));
    assert(equal(scale_left[1][1], 0.01));
    assert(equal(scale_left[2][2], 10.0));

    // 3. wall on right is identical to left, rotated opposite direction in y
    shape* right_wall = create_shape(SHAPE);

    floor_material.color = create_point(0.0, 0.0, 1.0); // blue
    mat4x4_set_ident(final_transform_left);
    mat4x4_mul_in_place(final_transform_left, translate_left, final_transform_left);
    Mat4x4 rotate_y_right;
    gen_rotate_matrix_Y(M_PI / 4.0, rotate_y_right);
    mat4x4_mul_in_place(final_transform_left, rotate_y_right, final_transform_left);
    mat4x4_mul_in_place(final_transform_left, rotate_x_left, final_transform_left);
    mat4x4_mul_in_place(final_transform_left, scale_left, final_transform_left);
    mat4x4_copy(final_transform_left, right_wall->transform);

    left_wall->material = floor_material;

    assert(equal(right_wall->transform[0][0], 7.071067812000001));
    assert(equal(right_wall->transform[1][0], 0.0));
    assert(equal(right_wall->transform[2][0], -7.071067812000001));
    assert(equal(right_wall->transform[3][0], 0.0));

    assert(equal(right_wall->transform[0][1], .007071067670578643));
    assert(equal(right_wall->transform[1][1], 0.0));
    assert(equal(right_wall->transform[2][1], .007071067670578643));
    assert(equal(right_wall->transform[3][1], 0.0));

    assert(equal(right_wall->transform[0][2], 0.0));
    assert(equal(right_wall->transform[1][2], -10.0));
    assert(equal(right_wall->transform[2][2], 0.0));
    assert(equal(right_wall->transform[3][2], 0.0));

    assert(equal(right_wall->transform[0][3], 0.0));
    assert(equal(right_wall->transform[1][3], 0.0));
    assert(equal(right_wall->transform[2][3], 5.0));
    assert(equal(right_wall->transform[3][3], 1.0));

    // 4. Large sphere in middle is a unit sphere, translated upward slightly and colored green
    shape* middle_sphere = create_shape(SHAPE);
    Mat4x4 middle_transform;
    gen_translate_matrix(-0.5, 1.0, 0.5, middle_transform);

    mat4x4_copy(middle_transform, middle_sphere->transform);

    assert(equal(middle_sphere->transform[0][0], 1.0));
    assert(equal(middle_sphere->transform[1][0], 0.0));
    assert(equal(middle_sphere->transform[2][0], 0.0));
    assert(equal(middle_sphere->transform[3][0], 0.0));

    assert(equal(middle_sphere->transform[0][1], 0.0));
    assert(equal(middle_sphere->transform[1][1], 1.0));
    assert(equal(middle_sphere->transform[2][1], 0.0));
    assert(equal(middle_sphere->transform[3][1], 0.0));

    assert(equal(middle_sphere->transform[0][2], 0.0));
    assert(equal(middle_sphere->transform[1][2], 0.0));
    assert(equal(middle_sphere->transform[2][2], 1.0));
    assert(equal(middle_sphere->transform[3][2], 0.0));

    assert(equal(middle_sphere->transform[0][3], -0.5));
    assert(equal(middle_sphere->transform[1][3], 1.0));
    assert(equal(middle_sphere->transform[2][3], 0.5));
    assert(equal(middle_sphere->transform[3][3], 1.0));

    material middle_material = create_material_default();
    middle_material.color = create_point(0.1, 1.0, 0.5);
    middle_material.diffuse = 0.7;
    middle_material.specular = 0.3;
    middle_sphere->material = middle_material;

    // 5. Smaller green sphere on the right is scaled in half
    shape* right_sphere = create_shape(SHAPE);
    Mat4x4 translate_right_sphere;
    gen_translate_matrix(1.5, 0.5, -0.5, translate_right_sphere);
    Mat4x4 scale_right_sphere;
    gen_scale_matrix(0.5, 0.5, 0.5, scale_right_sphere);
    Mat4x4 final_transform_right_sphere;
    mat4x4_set_ident(final_transform_right_sphere);

    mat4x4_mul_in_place(final_transform_right_sphere, translate_right_sphere, final_transform_right_sphere);
    mat4x4_mul_in_place(final_transform_right_sphere, scale_right_sphere, final_transform_right_sphere);
    mat4x4_copy(final_transform_right_sphere, right_sphere->transform);

    assert(equal(right_sphere->transform[0][0], 0.5));
    assert(equal(right_sphere->transform[1][0], 0.0));
    assert(equal(right_sphere->transform[2][0], 0.0));
    assert(equal(right_sphere->transform[3][0], 0.0));

    assert(equal(right_sphere->transform[0][1], 0.0));
    assert(equal(right_sphere->transform[1][1], 0.5));
    assert(equal(right_sphere->transform[2][1], 0.0));
    assert(equal(right_sphere->transform[3][1], 0.0));

    assert(equal(right_sphere->transform[0][2], 0.0));
    assert(equal(right_sphere->transform[1][2], 0.0));
    assert(equal(right_sphere->transform[2][2], 0.5));
    assert(equal(right_sphere->transform[3][2], 0.0));

    assert(equal(right_sphere->transform[0][3], 1.5));
    assert(equal(right_sphere->transform[1][3], 0.5));
    assert(equal(right_sphere->transform[2][3], -0.5));
    assert(equal(right_sphere->transform[3][3], 1.0));

    material right_sphere_material = create_material_default();
    right_sphere_material.color = create_point(0.5, 1.0, 0.1);
    right_sphere_material.diffuse = 0.7;
    right_sphere_material.specular = 0.3;
    right_sphere->material = right_sphere_material;

    // 6. Smallest sphere is scaled by a tird, before being translated
    shape* small_sphere = create_shape(SHAPE);
    Mat4x4 translate_small_sphere;
    gen_translate_matrix(-1.5, 0.33, -0.75, translate_small_sphere);
    Mat4x4 scale_small_sphere;
    gen_scale_matrix(0.33, 0.33, 0.33, scale_small_sphere);
    Mat4x4 final_transform_small_sphere;
    mat4x4_set_ident(final_transform_small_sphere);
    mat4x4_mul_in_place(final_transform_small_sphere, translate_small_sphere, final_transform_small_sphere);
    mat4x4_mul_in_place(final_transform_small_sphere, scale_small_sphere, final_transform_small_sphere);
    mat4x4_copy(final_transform_small_sphere, small_sphere->transform);

    material small_sphere_material = create_material_default();
    small_sphere_material.color = create_point(1.0, 0.8f, 0.1);
    small_sphere_material.diffuse = 0.7;
    small_sphere_material.specular = 0.3;
    small_sphere->material = small_sphere_material;

    // putting geometry together
    small_sphere->next = NULL;
    right_sphere->next = small_sphere;
    middle_sphere->next = right_sphere;
    right_wall->next = middle_sphere;
    left_wall->next = right_wall;
    floor->next = left_wall;
    w.objects = floor;

    assert(equal(w.objects->material.color.x, 1.0));
    assert(equal(w.objects->material.color.y, 0.9));
    assert(equal(w.objects->material.color.z, 0.9));
    assert(equal(w.objects->material.specular, 0.0));

    assert(equal(w.objects->material.ambient, 0.1));
    assert(equal(w.objects->material.diffuse, 0.9));

    // Ensure the values here have not changed after stringing together the objects

    shape* test_object = w.objects; // floor

    assert(equal(test_object->transform[0][0], 10.0));
    assert(equal(test_object->transform[1][1], 0.01));
    assert(equal(test_object->transform[2][2], 10.0));
    assert(equal(test_object->transform[3][3], 1.0));

    test_object = test_object->next; // left wall

    assert(equal(test_object->transform[0][0], 7.071067812000001));
    assert(equal(test_object->transform[1][0], 0.0));
    assert(equal(test_object->transform[2][0], 7.071067812000001));
    assert(equal(test_object->transform[3][0], 0.0));

    assert(equal(test_object->transform[0][1], -.007071067670578643));
    assert(equal(test_object->transform[1][1], 0.0));
    assert(equal(test_object->transform[2][1], .007071067670578643));
    assert(equal(test_object->transform[3][1], 0.0));

    assert(equal(test_object->transform[0][2], 0.0));
    assert(equal(test_object->transform[1][2], -10.0));
    assert(equal(test_object->transform[2][2], 0.0));
    assert(equal(test_object->transform[3][2], 0.0));

    assert(equal(test_object->transform[0][3], 0.0));
    assert(equal(test_object->transform[1][3], 0.0));
    assert(equal(test_object->transform[2][3], 5.0));
    assert(equal(test_object->transform[3][3], 1.0));

    // lighting
    tuple light_position = create_point(-10.0, 10.0, -10.0);
    tuple light_intensity = create_point(1.0, 1.0, 1.0);
    *w.lights = create_point_light(light_position, light_intensity);

    camera* c = create_camera(HORIZONTAL_SIZE, VERTICAL_SIZE, M_PI / 3.0);
    tuple from = create_point(0.0, 1.5, -5.0);
    tuple to = create_point(0.0, 1.0, 0.0);
    tuple up = create_vector(0.0, 1.0, 0.0);
    view_transform(from, to, up, c->view_transform);

    render(c, &w);

    free(c);
    free(floor);
    free(left_wall);
    free(right_wall);
    free(middle_sphere);
    free(right_sphere);
    free(small_sphere);
}

// 125 Render complete world with plane
void render_complete_world_with_plane() {
    world w = create_default_world();

    shape* floor = create_shape(PLANE);
    material floor_material = create_material_default();
    floor_material.color = create_point(1.0, 1.0, 1.0); // white
    floor_material.specular = 0.0;
    //floor_material.pattern = ring_pattern(create_point(0.8f, 1.0, 0.8f), create_point(1.0, 1.0, 1.0));
    //floor_material.has_pattern = true;
    floor_material.reflective = 0.0;
    floor_material.transparency = 0.0;
    floor->material = floor_material;

    shape* right_wall = create_shape(PLANE);
    Mat4x4 rot_wall_mat;
    gen_rotate_matrix_Z(M_PI / 2.0, rot_wall_mat);

    Mat4x4 trans_wall_mat;
    gen_translate_matrix(0.0, -8.25, 0.0, trans_wall_mat);
    //mat4x4_mul_in_place(trans_wall_mat, rot_wall_mat, trans_wall_mat);
    mat4x4_copy(rot_wall_mat, right_wall->transform);
    floor_material.color = create_point(0.0, 0.0, 1.0); // blue
    right_wall->material = floor_material;

    shape* left_wall = create_shape(PLANE);
    floor_material.color = create_point(1.0, 0.0, 0.0); // red
    gen_rotate_matrix_X(-M_PI / 2.0, rot_wall_mat);

    gen_translate_matrix(0.0, 0.0, -2.75, trans_wall_mat);
    mat4x4_mul_in_place(rot_wall_mat, trans_wall_mat, rot_wall_mat);
    mat4x4_copy(rot_wall_mat, left_wall->transform);
    left_wall->material = floor_material;

    shape* glass_sphere = create_glass_sphere();
    Mat4x4 front_transform;
    gen_translate_matrix(-7.0, 1.0, -8.0, front_transform);

    mat4x4_copy(front_transform, glass_sphere->transform);

    glass_sphere->material.diffuse = 0.0;
    glass_sphere->material.refractive_index = 1.5;
    glass_sphere->material.transparency = 0.9;
    glass_sphere->material.reflective = 0.2;

    //material front_material = create_material_default();
    //front_material.color = create_point(0.1, 0.0, 0.45);
    //front_material.ambient = 0.0;
    //front_material.diffuse = 0.0;
    //front_material.specular = 0.0;
    //front_material.shininess = 200.0;
    //front_material.reflective = 0.2;
    //front_material.transparency = 0.8f;
    //front_material.has_pattern = false;
    /*
    m.ambient = 0.1;
    m.diffuse = 0.9;
    m.specular = 0.9;
    m.shininess = 200.0;
    m.reflective = 0.0;
    m.transparency = 0.0;
    m.refractive_index = 1.0;
    */
    //front_material.transparency = 1.0;

    //glass_sphere->material = front_material;

    shape* middle_sphere = create_shape(SHAPE);
    Mat4x4 middle_transform;
    gen_translate_matrix(-3.5, 1.0, -3.0, middle_transform);

    mat4x4_copy(middle_transform, middle_sphere->transform);

    material middle_material = create_material_default();
    middle_material.color = create_point(0.1, 1.0, 0.5);
    middle_material.diffuse = 0.7;
    middle_material.specular = 0.3;
    middle_material.transparency = 0.0;

    tuple light = create_point(1.0, 1.0, 1.0);
    tuple dark = create_point(0.439, 0.305, 0.827); // purple
    pattern pat = stripe_pattern(light, dark);

    Mat4x4 scale_pattern;
    gen_scale_matrix(0.175, 0.175, 0.175, scale_pattern);

    set_pattern_transform(&pat, scale_pattern);
    middle_material.pattern = pat;
    middle_material.has_pattern = true;

    middle_sphere->material = middle_material;

    shape* right_sphere = create_shape(SHAPE);
    Mat4x4 translate_right_sphere;
    gen_translate_matrix(-1.95, 1.0, -5.5, translate_right_sphere);
    Mat4x4 scale_right_sphere;
    gen_scale_matrix(0.5, 0.5, 0.5, scale_right_sphere);
    Mat4x4 final_transform_right_sphere;
    mat4x4_set_ident(final_transform_right_sphere);

    mat4x4_mul_in_place(final_transform_right_sphere, translate_right_sphere, final_transform_right_sphere);
    mat4x4_mul_in_place(final_transform_right_sphere, scale_right_sphere, final_transform_right_sphere);
    mat4x4_copy(final_transform_right_sphere, right_sphere->transform);

    material right_sphere_material = create_material_default();
    right_sphere_material.color = create_point(0.0, 0.0, 0.0);
    right_sphere_material.diffuse = 0.7;
    right_sphere_material.specular = 0.3;
    right_sphere_material.reflective = 1.0;
    right_sphere_material.transparency = 0.0;
    //right_sphere_material.has_pattern = true;
    //tuple white = create_point(1.0, 1.0, 1.0);
    //tuple black = create_point(0.0, 0.0, 0.0);
    //right_sphere_material.pattern = gradiant_pattern(white, black);
    right_sphere->material = right_sphere_material;

    shape* origin_sphere = create_shape(SHAPE);
    origin_sphere->material.color.z = 0.0; // green color
    origin_sphere->material.color.x = 0.0;

    shape* small_sphere = create_shape(SHAPE);
    Mat4x4 translate_small_sphere;
    gen_translate_matrix(-6.5, 0.33, -2.75, translate_small_sphere);
    Mat4x4 scale_small_sphere;
    gen_scale_matrix(0.33, 0.33, 0.33, scale_small_sphere);
    Mat4x4 final_transform_small_sphere;
    mat4x4_set_ident(final_transform_small_sphere);
    mat4x4_mul_in_place(final_transform_small_sphere, translate_small_sphere, final_transform_small_sphere);
    mat4x4_mul_in_place(final_transform_small_sphere, scale_small_sphere, final_transform_small_sphere);
    mat4x4_copy(final_transform_small_sphere, small_sphere->transform);
    material small_sphere_material = create_material_default();
    small_sphere_material.color = create_point(1.0, 0.8f, 0.1);
    small_sphere_material.diffuse = 0.7;
    small_sphere_material.specular = 0.3;
    small_sphere_material.shininess = 100.0;
    tuple small_sphere_light = create_point(0.2, 0.2, 0.2);
    tuple small_sphere_dark = create_point(0.0, 0.0, 0.0);
    pattern small_sphere_pat = stripe_pattern(small_sphere_light, small_sphere_dark);
    Mat4x4 small_sphere_scale_pattern;
    gen_scale_matrix(0.07, 0.07, 0.07, small_sphere_scale_pattern);
    set_pattern_transform(&small_sphere_pat, small_sphere_scale_pattern);
    small_sphere_material.pattern = small_sphere_pat;
    small_sphere_material.has_pattern = true;
    small_sphere->material = small_sphere_material;

    /*
    origin_sphere->next = NULL;
    left_wall->next = origin_sphere;
    right_wall->next = left_wall;
    floor->next = right_wall;
    w.objects = floor;
    */
    // putting geometry together

    origin_sphere->next = NULL;
    small_sphere->next = origin_sphere;
    right_sphere->next = small_sphere;
    middle_sphere->next = right_sphere;
    glass_sphere->next = middle_sphere;
    left_wall->next = glass_sphere;
    right_wall->next = left_wall;
    floor->next = right_wall;
    w.objects = floor;

    // lighting
    tuple light_position = create_point(-10.0, 10.0, -10.0);
    tuple light_intensity = create_point(1.0, 1.0, 1.0);
    *w.lights = create_point_light(light_position, light_intensity);

    camera* c = create_camera(HORIZONTAL_SIZE, VERTICAL_SIZE, M_PI / 3.0);
    tuple from = create_point(-10.0, 1.5, -10.0);
    tuple to = create_point(0.0, 0.0, 0.0);
    tuple up = create_vector(0.0, 1.0, 0.0);
    view_transform(from, to, up, c->view_transform);

    render(c, &w);

    free(c);
    free(floor);
    free(middle_sphere);
    free(right_sphere);
    free(origin_sphere);
}

// extra rendering of dual spheres refracting on floor
void render_dual_spheres_refracting_on_floor() {
    world w = create_world();

    camera* c = create_camera(HORIZONTAL_SIZE, VERTICAL_SIZE, 0.45);
    tuple from = create_point(0.0, 0.0, -5.0);
    tuple to = create_point(0.0, 0.0, 0.0);
    tuple up = create_vector(0.0, 1.0, 0.0);
    view_transform(from, to, up, c->view_transform);

    tuple light_position = create_point(2.0, 10.0, -5.0);
    tuple light_intensity = create_point(0.9, 0.9, 0.9);
    *w.lights = create_point_light(light_position, light_intensity);

    shape* wall = create_shape(PLANE);

    Mat4x4 wall_rotate_transform;
    gen_rotate_matrix_X(M_PI / 2.0, wall_rotate_transform);
    Mat4x4 wall_position_transform;
    gen_translate_matrix(0.0, 0.0, 10.0, wall_position_transform);
    mat4x4_mul_in_place(wall_position_transform, wall_rotate_transform, wall_position_transform);
    mat4x4_copy(wall_position_transform, wall->transform);

    material wall_material = create_material_default();
    tuple from_color = create_point(0.15, 0.15, 0.15);
    tuple to_color = create_point(0.85, 0.85, 0.85);
    pattern checkers = checkers_pattern(from_color, to_color);
    wall_material.has_pattern = true;
    wall_material.pattern = checkers;
    wall_material.ambient = 0.8f;
    wall_material.diffuse = 0.2;
    wall_material.specular = 0.0;

    wall->material = wall_material;

    shape* outer_sphere = create_shape(SPHERE);
    material sphere_material = create_material_default();

    sphere_material.ambient = 0.0;
    sphere_material.color = create_point(1.0, 1.0, 1.0);
    sphere_material.diffuse = 0.0;
    sphere_material.specular = 0.9;
    sphere_material.shininess = 300.0;
    sphere_material.reflective = 0.9;
    sphere_material.transparency = 0.9;
    sphere_material.refractive_index = 1.5;
    outer_sphere->material = sphere_material;

    shape* hollow_center = create_shape(SPHERE);
    Mat4x4 hollow_center_transform;
    gen_scale_matrix(0.5, 0.5, 0.5, hollow_center_transform);
    mat4x4_copy(hollow_center_transform, hollow_center->transform);

    material hollow_sphere_material = create_material_default();

    hollow_sphere_material.ambient = 0.0;
    hollow_sphere_material.color = create_point(1.0, 1.0, 1.0);
    hollow_sphere_material.diffuse = 0.0;
    hollow_sphere_material.specular = 0.9;
    hollow_sphere_material.shininess = 300.0;
    hollow_sphere_material.reflective = 0.9;
    hollow_sphere_material.transparency = 0.9;
    hollow_sphere_material.refractive_index = 1.0000034;
    hollow_center->material = hollow_sphere_material;

    add_shape_to_world(hollow_center, &w);
    add_shape_to_world(outer_sphere, &w);
    add_shape_to_world(wall, &w);
    world_print(w);
    camera_print(c);
    render(c, &w);
}

void render_refraction_scene() {
    world w = create_world();

    camera* c = create_camera(HORIZONTAL_SIZE, VERTICAL_SIZE, 0.5);
    tuple from = create_point(-4.5, 0.85, -4);
    tuple to = create_point(0.0, 0.85, 0.0);
    tuple up = create_vector(0.0, 1.0, 0.0);
    view_transform(from, to, up, c->view_transform);

    tuple light_position = create_point(-4.9, 4.9, 1);
    tuple light_intensity = create_point(1, 1, 1);
    *w.lights = create_point_light(light_position, light_intensity);

    // Wall Material is Reused Several Places
    material wall_material = create_material_default();
    tuple wall_material_from_color = create_point(0.0, 0.0, 0.0);
    tuple wall_material_to_color = create_point(0.75, 0.75, 0.75);
    pattern wall_material_checkers = checkers_pattern(wall_material_from_color, wall_material_to_color);
    wall_material.has_pattern = true;
    wall_material.pattern = wall_material_checkers;
    wall_material.specular = 0;

    shape* floor = create_shape(PLANE);
    Mat4x4 floor_rotate_transform;
    gen_rotate_matrix_Y( 0.31415, floor_rotate_transform);
    mat4x4_copy(floor_rotate_transform, floor->transform);
    material floor_material = create_material_default();
    tuple from_color = create_point(0.0, 0.0, 0.0);
    tuple to_color = create_point(0.75, 0.75, 0.75);
    pattern checkers = checkers_pattern(from_color, to_color);
    floor_material.has_pattern = true;
    floor_material.pattern = checkers;
    floor_material.ambient = 0.5;
    floor_material.diffuse = 0.4;
    floor_material.specular = 0.8;
    floor_material.reflective = 0.5;
    floor->material = floor_material;

    shape* west_wall = create_shape(PLANE);
    Mat4x4 translate_west;
    gen_translate_matrix(-5.0, 0.0, 0.0, translate_west);
    Mat4x4 rotate_y_west;
    gen_rotate_matrix_Y(1.5708, rotate_y_west);
    Mat4x4 rotate_z_west;
    gen_rotate_matrix_Z(1.5708, rotate_z_west);
    Mat4x4 scale_west;
    gen_scale_matrix(10.0, 0.01, 10.0, scale_west);
    Mat4x4 final_transform_west;
    mat4x4_set_ident(final_transform_west);
    mat4x4_mul_in_place(translate_west, final_transform_west, final_transform_west);
    mat4x4_copy(final_transform_west, west_wall->transform);
    west_wall->material = wall_material;

    shape* east_wall = create_shape(PLANE);
    Mat4x4 translate_east;
    gen_translate_matrix(5.0, 0.0, 0.0, translate_east);
    Mat4x4 rotate_y_east;
    gen_rotate_matrix_Y(1.5708, rotate_y_east);
    Mat4x4 rotate_z_east;
    gen_rotate_matrix_Z(1.5708, rotate_z_east);
    Mat4x4 final_transform_east;
    mat4x4_set_ident(final_transform_east);
    mat4x4_mul_in_place(rotate_y_east, final_transform_east, final_transform_east);
    mat4x4_mul_in_place(rotate_z_east, final_transform_east, final_transform_east);
    mat4x4_mul_in_place(translate_east, final_transform_east, final_transform_east);
    mat4x4_copy(final_transform_east, east_wall->transform);
    east_wall->material = wall_material;

    shape* north_wall = create_shape(PLANE);
    Mat4x4 translate_north;
    gen_translate_matrix(0.0, 0.0, 5.0, translate_north);
    Mat4x4 rotate_x_north;
    gen_rotate_matrix_X(1.5708, rotate_x_north);
    Mat4x4 final_transform_north;
    mat4x4_set_ident(final_transform_north);
    mat4x4_mul_in_place(rotate_x_north, final_transform_north, final_transform_north);
    mat4x4_mul_in_place(translate_north, final_transform_north, final_transform_north);
    mat4x4_copy(final_transform_north, north_wall->transform);
    north_wall->material = wall_material;

    shape* south_wall = create_shape(PLANE);
    Mat4x4 translate_south;
    gen_translate_matrix(0.0, 0.0, -5.0, translate_south);
    Mat4x4 rotate_x_south;
    gen_rotate_matrix_X(1.5708, rotate_x_south);
    Mat4x4 final_transform_south;
    mat4x4_set_ident(final_transform_south);
    mat4x4_mul_in_place(rotate_x_south, final_transform_south, final_transform_south);
    mat4x4_mul_in_place(translate_south, final_transform_south, final_transform_south);
    mat4x4_copy(final_transform_south, south_wall->transform);
    south_wall->material = wall_material;

    shape* red_background_ball = create_shape(SHAPE);
    Mat4x4 translate_background_ball;
    gen_translate_matrix(4, 1, 4, translate_background_ball);
    mat4x4_copy(translate_background_ball, red_background_ball->transform);
    material background_ball_material = create_material_default();
    background_ball_material.color = create_point(0.8, 0.1, 0.3);
    red_background_ball->material = background_ball_material;

    shape* blue_background_ball = create_shape(SHAPE);
    Mat4x4 blue_background_ball_translate;
    gen_translate_matrix(2.6, 0.6, 4.4, blue_background_ball_translate);
    Mat4x4 blue_background_ball_scale_transform;
    gen_scale_matrix(0.6, 0.6, 0.6, blue_background_ball_scale_transform);
    mat4x4_mul_in_place(blue_background_ball_translate, blue_background_ball_scale_transform, blue_background_ball_translate);
    mat4x4_copy(blue_background_ball_translate, blue_background_ball->transform);

    background_ball_material = create_material_default();
    background_ball_material.color = create_point(0.2, 0.1, 0.8);
    background_ball_material.shininess = 10;
    background_ball_material.specular = 0.9;
    blue_background_ball->material = background_ball_material;

    shape* green_background_ball = create_shape(SHAPE);
    Mat4x4 green_background_ball_translate;
    gen_translate_matrix(4.6, 0.4, 2.9, green_background_ball_translate);
    Mat4x4 green_background_ball_scale_transform;
    gen_scale_matrix(0.4, 0.4, 0.4, green_background_ball_scale_transform);
    mat4x4_mul_in_place(green_background_ball_translate, green_background_ball_scale_transform, green_background_ball_translate);
    mat4x4_copy(green_background_ball_translate, green_background_ball->transform);

    background_ball_material = create_material_default();
    background_ball_material.color = create_point(0.1, 0.8, 0.2);
    background_ball_material.shininess = 200;
    green_background_ball->material = background_ball_material;

    shape* glass_ball = create_shape(SHAPE);
    Mat4x4 translate_glass_ball;
    gen_translate_matrix(0.10, 1.0, 0, translate_glass_ball);
    Mat4x4 scale_glass_ball;
    gen_scale_matrix(0.85, 0.85, 0.85, scale_glass_ball);
    Mat4x4 final_transform_glass_ball;
    mat4x4_set_ident(final_transform_glass_ball);

    mat4x4_mul_in_place(final_transform_glass_ball, translate_glass_ball, final_transform_glass_ball);
    mat4x4_mul_in_place(final_transform_glass_ball, scale_glass_ball, final_transform_glass_ball);
    mat4x4_copy(final_transform_glass_ball, glass_ball->transform);

    material glass_ball_material = create_material_default();
    glass_ball_material.color = create_point(0.8, 0.8, 0.9);
    glass_ball_material.ambient = 0;
    glass_ball_material.diffuse = 0.2;
    glass_ball_material.specular = 0.9;
    glass_ball_material.shininess = 300;
    glass_ball_material.transparency = 0.8;
    glass_ball_material.refractive_index = 1.57;
    glass_ball->material = glass_ball_material;

    shape* mirror_ball = create_shape(SHAPE);
    Mat4x4 translate_mirror_ball;
    gen_translate_matrix(2.2, 0.6, 0.0, translate_mirror_ball);
    Mat4x4 scale_mirror_ball;
    gen_scale_matrix(0.5, 0.5, 0.5, scale_mirror_ball);
    Mat4x4 final_transform_mirror_ball;
    mat4x4_set_ident(final_transform_mirror_ball);

    mat4x4_mul_in_place(final_transform_mirror_ball, translate_mirror_ball, final_transform_mirror_ball);
    mat4x4_mul_in_place(final_transform_mirror_ball, scale_mirror_ball, final_transform_mirror_ball);
    mat4x4_copy(final_transform_mirror_ball, mirror_ball->transform);

    material mirror_ball_material = create_material_default();
    mirror_ball_material.color = create_point(0.0, 0.0, 0.0);
    mirror_ball_material.ambient = 0;
    mirror_ball_material.diffuse = 0.0;
    mirror_ball_material.specular = 0.9;
    mirror_ball_material.shininess = 300;
    mirror_ball_material.transparency = 0.0;
    mirror_ball_material.refractive_index = 0.0;
    mirror_ball_material.reflective = 1.0;
    mirror_ball->material = mirror_ball_material;

    add_shape_to_world(floor, &w);
    add_shape_to_world(west_wall, &w);
    add_shape_to_world(east_wall, &w);
    add_shape_to_world(north_wall, &w);
    add_shape_to_world(south_wall, &w);
    add_shape_to_world(red_background_ball, &w);
    add_shape_to_world(blue_background_ball, &w);
    add_shape_to_world(green_background_ball, &w);
    add_shape_to_world(glass_ball, &w);
    add_shape_to_world(mirror_ball, &w);

    render(c, &w);

    free(c);
    free(floor);
    free(west_wall);
    free(east_wall);
    free(north_wall);
    free(south_wall);
    free(red_background_ball);
    free(blue_background_ball);
    free(green_background_ball);
    free(glass_ball);
    free(mirror_ball);
}

int main() {

  contents = containers_create();
#if defined _DEBUG
  clock_t start_unit_tests = clock();
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
  unit_test("Tuple Magnitude Vector Test", tuple_mag_vec_test());
  unit_test("Normal Vector Test", vec_norm_test());
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
  unit_test("Scaling Matrix Applied To A Point Test", point_scale_mat4x4_test());
  unit_test("Scaling Matrix Applied To A Vector Test", vec_scale_mat4x4_test());
  unit_test("Multiply Inverse Of Scaling Matrix Test", mult_inverse_scale_matrix_test());
  unit_test("Reflection Scaling Negative Value Test", reflection_scaling_neg_value_test());
  unit_test("Generate Rotation Matrix X Test", gen_rotation_matrix_X_test());
  unit_test("Generate  Rotation Matrix X Reverse Test", gen_rotation_matrix_reverse_test());
  unit_test("Generate Rotation Matrix Y Test", gen_rotation_matrix_Y_test());
  unit_test("Generate Rotation Matrix Z Test", gen_rotation_matrix_Z_test());
  unit_test("Generate Sheer Matrix Test", gen_shear_matrix_test());
  unit_test("Transformations Applied In Sequence Test", transform_applied_in_sequence_test());
  //unitTest("Draw Clock Test", drawClockTest());
  unit_test("4x4 Matrix Copy Test", mat4x4_copy_test());
  unit_test("Tuple Copy Test", tuple_copy_test());
  unit_test("Create Ray Test", create_ray_test());
  unit_test("Create Sphere Test", create_shape_test());
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
  unit_test("Too Many Intersections Test", too_many_intersections_test());
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
  unit_test("Prepare Computations Test", prepare_computations_test());
  unit_test("Hit When Intersection Occurs On Outside Test", hit_when_intersect_on_outside_test());
  unit_test("Hit When Intersect Occurs On Inside Test", hit_when_intersect_occurs_on_inside_test());
  unit_test("Shading An Intersection Test", shading_an_intersection_test());
  unit_test("Shading Intersection From Inside Test", shading_intersection_from_inside_test());
  unit_test("Color When Ray Misses Test", color_when_ray_misses_test());
  unit_test("Color When Ray Hits Test", color_when_ray_hits_test());
  unit_test("Color With Intersect Behind Ray Test", color_with_intersect_behind_ray_test());
  unit_test("Transformation For Default Orientation Test", transformation_for_default_orientation_test());
  unit_test("View Transform Matrix Looking Positive In Z Direction Test", view_transform_mat_looking_positive_z_dir_test());
  unit_test("View Transform Moves World Test", view_transform_moves_world_test());
  unit_test("Arbitrary View Transform Test", arbitrary_view_transform_test());
  unit_test("Constructing New Camera Test", constructing_camera_test());
  unit_test("Pixel Size For Horizontal Canvas Test", pixel_size_for_horizontal_canvas_test());
  unit_test("Pixel Size For Vertical Canvas Test", pixel_size_for_vertical_canvas_test());
  unit_test("Construct A Ray Through Center Of Canvas Test", const_a_ray_through_center_of_canvas());
  unit_test("Construct A Ray Through Corner Of Canvas Test", const_a_ray_through_corner_of_canvas());
  unit_test("Construct A Ray When Camera Is Transformed Test", const_a_ray_when_camera_is_transformed());
  unit_test("Lighting With Surface In Shadow Test", lighting_with_surface_in_shadow_test());
  unit_test("No Shadow When Not Collinear Point Light Test", no_shadow_when_not_collinear_point_light_test());
  unit_test("No Shadow When Object Between Point And Light Test", no_shadow_when_object_between_point_and_light_test());
  unit_test("No Shadow When Object Behind Light Test", no_shadow_when_object_behind_light_test());
  unit_test("No Shadow When Object Behind Point Test", no_shadow_when_object_behind_point_test());
  unit_test("Shade Hit Given Intersection In Shadow Test", shade_hit_given_intersection_in_shadow_test());
  unit_test("Hit Should Offset Point Test", hit_should_offset_point_test());
  unit_test("Default Transformation Of Shape Test", default_transformation_of_shape());
  unit_test("Assign Transformation Of Shape Test", assign_transformation_of_shape());
  unit_test("Default Material Of A Shape Test", default_material_of_shape());
  unit_test("Assigning Material To A Shape Test", assigning_material_to_a_shape());
  unit_test("Normal Of Plane Is Const Everywhere Test", normal_of_plane_is_const_everywhere_test());
  unit_test("Intersect Ray Parallel To Plane Test", intersect_ray_parallel_to_plane_test());
  unit_test("Intersect Coplanar Ray Test", intersect_coplanar_ray_test());
  unit_test("Intersect Ray Plane Above Test", intersect_ray_plane_above_test());
  unit_test("Intersect Ray Plane Below Test", intersect_ray_plane_below_test());
  unit_test("Creating A Stripe Pattern Test", creating_a_stripe_pattern_test());
  unit_test("Stripes With Both Object And Pattern Transform Test", stripes_with_both_object_and_pattern_transform_test());
  unit_test("Stripe Pattern Is Const In Y Test", stripe_pattern_is_const_in_y_test());
  unit_test("Stripe Pattern Is Const In Z Test",stripe_pattern_is_const_in_z_test());
  unit_test("Stripe Pattern Alternates In X Test", stripe_pattern_alternates_in_x_test());
  unit_test("Lighting With Pattern Applied Test", lighting_with_pattern_applied());
  unit_test("Stripes With Object Transformation Test", stripes_with_object_transformation_test());
  unit_test("Stripes With Pattern Transform Test", stripes_with_pattern_transform_test());
  unit_test("Gradiant Linearly Interpolates Between Colors Test", gradiant_linearly_interpolates_between_colors_test());
  unit_test("Ring Pattern Should Extend In X And Y Test", ring_pattern_should_extend_in_x_and_y_test());
  unit_test("Checkers Pattern Should Repeat In X Test", checkers_pattern_should_repeat_in_x_test());
  unit_test("Checkers Pattern Should Repeat In Y Test", checkers_pattern_should_repeat_in_y_test());
  unit_test("Checkers Pattern Should Repeat In Z Test", checkers_pattern_should_repeat_in_z_test());
  unit_test("Precompute Reflection Vector Test", precompute_reflection_vector_test());
  unit_test("Reflected Color For Non Reflective Material Test", reflected_color_for_non_reflective_material_test());
  unit_test("Strike A Non Reflective Surface Test", reflected_color_for_reflective_material_test());
  unit_test("Shade Hit With Reflective Material Test", shade_hit_with_reflective_material_test());
  unit_test("Color At With Mutually Reflective Surfaces Test", color_at_with_mutually_reflective_surfaces_test());
  unit_test("Reflected Color At Max Recursive Depth Test", reflected_color_at_max_recursive_depth_test());
  unit_test("Helper For Producing Sphere With Glassy Material Test", helper_for_producing_sphere_with_glassy_material_test());
  unit_test("Containers Tests", containers_tests());
  unit_test("Finding N1 And N2 At Various Intersections Test", finding_n1_and_n2_at_various_intersections_test());
  unit_test("Under Point Is Offset Below The Suface Test", under_point_is_offset_below_the_suface_test());
  unit_test("Refracted Color With Opaque Surface Test", refracted_color_with_opaque_surface_test());
  unit_test("Refracted Color With Maximum Recursive Depth Test", refracted_color_with_maximum_recursive_depth_test());
  unit_test("Reflected Color Under Total Internal Reflection Test", reflected_color_under_total_internal_reflection_test());
  unit_test("Refracted Color With Refracted Ray Test", refracted_color_with_refracted_ray_test());
  unit_test("Shade Hit With Transparent Material Test", shade_hit_with_transparent_material_test());
  unit_test("Schlick Approximation Under Total Internal Reflection Test", schlick_approximation_under_total_internal_reflection_test());
  unit_test("Schlick Approximation With Perpedicular Viewing Angle Test", schlick_approximation_with_perpedicular_viewing_angle_test());
  unit_test("Schlick Approximation With Small Angle N2 > N1 Test", schlick_approximation_with_small_angle_n2_gt_n1_test());
  unit_test("Add Shape To World Tests", add_shape_to_world_tests());
  unit_test("Shade Hit With Reflective Transparent Material Test", shade_hit_with_reflective_transparent_material_test());
  //unit_test("Render A World With Camera Test", render_a_world_with_camera_test());
  clock_t end_unit_tests = clock();
  float seconds_unit_test = (float)(end_unit_tests - start_unit_tests) / CLOCKS_PER_SEC;
  printf("\nUnit Tests Took %f Seconds\n", seconds_unit_test);
#endif
  clock_t start_render = clock();

  printf("Starting Render Process\n");
  //render_sphere();
  //render_complete_world();
  //render_dual_spheres_refracting_on_floor();
  //render_complete_world_with_plane();
  render_refraction_scene();

  clock_t end_render = clock();
  float seconds_render = (float)(end_render - start_render) / CLOCKS_PER_SEC;
  printf("\nRender Took %f Seconds\n", seconds_render);

  printf("Writing Canvas To File\n");
  write_canvas_to_file();
  return 0;
}