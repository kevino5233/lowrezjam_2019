#include <cassert>
#include <cmath>
// header only math library?
namespace ca {

void ca_memcpy(char * dest, const char * src, size_t N) {
    for (size_t i = 0; i < N; i++) {
        dest[i] = src[i];
    }
}

void ca_memset(char * data, char value, size_t N) {
    for (size_t i = 0; i < N; i++) {
        data[i] = value;
    }
}

struct Mat4f;
struct Mat3f;
struct Quat;
struct Vec4f;
struct Vec4i;
struct Vec3f;
struct Vec3i;

inline float dot(Vec4f a, Vec4f b);
inline float dot(Vec4i a, Vec4i b);
inline float dot(Vec3f a, Vec3f b);
inline float dot(Vec3i a, Vec3i b);

inline Vec3i cross(Vec3i a, Vec3i b);
inline Vec3f cross(Vec3f a, Vec3f b);
inline Quat cross(Quat q1, Quat q2);

inline Vec4f vec_scalar_mult(Vec4f a, int f);
inline Vec4i vec_scalar_mult(Vec4i a, int f);
inline Vec3f vec_scalar_mult(Vec3f a, int f);
inline Vec3i vec_scalar_mult(Vec3i a, int f);

inline Vec4f vec_scalar_mult(Vec4f a, float f);
// inline Vec4i vec_scalar_mult(Vec4i a, float f);
inline Vec3f vec_scalar_mult(Vec3f a, float f);
// inline Vec3i vec_scalar_mult(Vec3i a, float f);

inline Quat vec_sub(Quat a, Quat b);
inline Vec4f vec_sub(Vec4f a, Vec4f b);
inline Vec4i vec_sub(Vec4i a, Vec4i b);
inline Vec3f vec_sub(Vec3f a, Vec3f b);
inline Vec3i vec_sub(Vec3i a, Vec3i b);

inline Quat vec_add(Quat a, Quat b);
inline Vec4f vec_add(Vec4f a, Vec4f b);
inline Vec4i vec_add(Vec4i a, Vec4i b);
inline Vec3f vec_add(Vec3f a, Vec3f b);
inline Vec3i vec_add(Vec3i a, Vec3i b);

inline Vec4f termwise_multiply (Vec4f a, Vec4f b);
inline Vec4i termwise_multiply (Vec4i a, Vec4i b);
inline Vec3f termwise_multiply (Vec3f a, Vec3f b);
inline Vec3i termwise_multiply (Vec3i a, Vec3i b);

inline void normalize_modify(Vec3f& v);
inline Vec3f normalize_copy(Vec3f v);
inline void normalize_modify(Vec4f& v);
inline Vec4f normalize_copy(Vec4f v);

inline Mat3f mat_mult(Mat3f a, Mat3f b);
inline Mat4f mat_mult(Mat4f a, Mat4f b);

inline Vec3f mat_vec_mult(Mat3f a, Vec3f b);
inline Vec4f mat_vec_mult(Mat4f a, Vec4f b);

// TODO explore more stuff with templates and template initialization and stuff
// could be ~cooool~
// TODO how does std::vector take in an initializer list and do stuff with it?
// TODO check orientation on Mat3f and Mat4f
struct Mat4f {
    float x1, x2, x3, x4,
          y1, y2, y3, y4,
          z1, z2, z3, z4,
          w1, w2, w3, w4;
    static Mat4f Identity() {
        return {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
                0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f}; }
    Mat4f operator * (const Mat4f m) {
        return mat_mult(*this, m);
    }
    inline Vec4f operator * (const Vec4f v);
    inline float operator() (int row, int col) {
        assert (row >= 0 && row < 4 && col >= 0 && col < 4);
        float * arr = &x1;
        return arr[row * 4 + col];
    }
    inline bool equals(Mat4f& other) {
        return
            x1 == other.x1 && x2 == other.x2 && x3 == other.x3 && x4 == other.x4
            && y1 == other.y1 && y2 == other.y2 && y3 == other.y3 && y4 == other.y4
            && z1 == other.z1 && z2 == other.z2 && z3 == other.z3 && z4 == other.z4
            && w1 == other.w1 && w2 == other.w2 && w3 == other.w3 && w4 == other.w4;
    }
    inline Vec4f& X();
    inline Vec4f& Y();
    inline Vec4f& Z();
    inline Vec4f& W();
};

struct Mat3f {
    float x1, x2, x3,
          y1, y2, y3,
          z1, z2, z3;
    static Mat3f Identity() { return {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f}; }
    Mat3f operator * (const Mat3f m) {
        return mat_mult(*this, m);
    }
    inline Vec3f operator * (const Vec3f v);
    inline float operator() (int row, int col) {
        assert (row >= 0 && row < 3 && col >= 0 && col < 3);
        float * arr = &x1;
        return arr[row * 3 + col];
    }
    inline Vec3f& X();
    inline Vec3f& Y();
    inline Vec3f& Z();
};

inline const float * raw_data(const Mat4f& m) {
    return &m.x1;
}

inline const float * raw_data(const Mat3f& m) {
    return &m.x1;
}

#define VEC4_EQUALS(other) x == other.x && y == other.y && z == other.z && w == other.w

// TODO verify matrix multiply operations
// TODO rotate quaternion by angle operations
struct Vec4f {
    float x, y, z, w;
    static Vec4f Identity() { return {1.f, 0.f, 0.f, 0.f}; }
    static Vec4f FromVec3f(const Vec3f v3, float w = 0);
        
    Vec4f operator-() { return {-x, -y, -z, -w}; }
    Vec4f operator-(const Vec4f b) const { return vec_sub(*this, b); }
    void operator-=(const Vec4f b) { *this = vec_sub(*this, b); }
    Vec4f operator+(const Vec4f b) const { return vec_add(*this, b); }
    void operator+=(const Vec4f b) { *this = vec_add(*this, b); }
    Vec4f operator * (const float f) { return vec_scalar_mult(*this, f); }
    void operator *= (const float f) { *this = vec_scalar_mult(*this, f); }
    Vec4f operator * (const Vec4f v) { return termwise_multiply(*this, v); }
    void operator *= (const Vec4f v) { *this = termwise_multiply(*this, v); }
    inline bool equals(Vec4f& other) { return VEC4_EQUALS(other); }

};
inline Vec4f Mat4f::operator * (const Vec4f v) {
    return mat_vec_mult(*this, v);
}
inline Vec4f& Mat4f::X() {
    return *(reinterpret_cast<Vec4f *>(this) + 0);
}
inline Vec4f& Mat4f::Y() {
    return *(reinterpret_cast<Vec4f *>(this) + 1);
}
inline Vec4f& Mat4f::Z() {
    return *(reinterpret_cast<Vec4f *>(this) + 2);
}
inline Vec4f& Mat4f::W() {
    return *(reinterpret_cast<Vec4f *>(this) + 3);
}

struct Vec3f {
    float x, y, z;
    static Vec3f Identity() { return {1.f, 0.f, 0.f}; }

    Vec3f operator-() { return {-x, -y, -z}; }
    Vec3f operator-(const Vec3f b) const { return vec_sub(*this, b); }
    void operator-=(const Vec3f b) { *this = vec_sub(*this, b); }
    Vec3f operator+(const Vec3f b) const { return vec_add(*this, b); }
    void operator+=(const Vec3f b) { *this = vec_add(*this, b); }
    Vec3f operator * (const float f) { return vec_scalar_mult(*this, f); }
    void operator *= (const float f) { *this = vec_scalar_mult(*this, f); }
    Vec3f operator * (const Vec3f v) { return termwise_multiply(*this, v); }
    void operator *= (const Vec3f v) { *this = termwise_multiply(*this, v); }
};
inline Vec3f Mat3f::operator * (const Vec3f v) {
    return mat_vec_mult(*this, v);
}
inline Vec3f& Mat3f::X() {
    return *(reinterpret_cast<Vec3f *>(this) + 0);
}
inline Vec3f& Mat3f::Y() {
    return *(reinterpret_cast<Vec3f *>(this) + 1);
}
inline Vec3f& Mat3f::Z() {
    return *(reinterpret_cast<Vec3f *>(this) + 2);
}

inline Quat hamilton_product(const Quat a, const Quat b);
inline Quat cross(const Quat a, const Quat b);

// TODO check taht the subtraction operator should actually do this
struct Quat {
    float x, y, z, w;
    static Quat Identity() { return {0.f, 0.f, 0.f, 1.f}; }

    Vec3f V() const { return {x, y, z}; }
    float S() const { return w; }

    Quat operator-() { return {-x, -y, -z, -w}; }
    Quat operator-(const Quat b) const { return vec_sub(*this, b); }
    void operator-=(const Quat b) { *this = vec_sub(*this, b); }
    Quat operator+(const Quat b) const { return vec_add(*this, b); }
    void operator+=(const Quat b) { *this = vec_add(*this, b); }
    Quat operator* (const Quat f) const { return hamilton_product(*this, f); }
    void operator*= (const Quat f) { *this = hamilton_product(*this, f); }
};

// TODO wrong!!!!
// "For two elements a1 + b1i + c1j + d1k and a2 + b2i + c2j + d2k
// their product, called the Hamilton product (a1 + b1i + c1j + d1k)
// (a2 + b2i + c2j + d2k)" (wikipedia: Quaternion)
inline Quat hamilton_product(const Quat a, const Quat b) {
    Vec3f quat_v = (b.V() * a.S()) + (a.V() * b.S()) + (cross(a.V(), b.V()));
    float quat_s = a.S() * b.S() + dot(a.V(), b.V());
    return {quat_v.x, quat_v.y, quat_v.z, quat_s};
}

// #define STRUCT_VEC4(name, type) \
// struct name {\
// type x, y, z, w;\
// static name Identity() { return {1, 0, 0, 0}; }\
//     name operator+(const name b) {\
//         return vec_add(*this, b);\
//     }\
//     name operator+=(const name b) {\
//         return vec_add(*this, b);\
//     }\
//     name operator * (const int f) {\
//         return vec_scalar_mult(*this, f);\
//     }\
//     name operator *= (const int f) {\
//         return vec_scalar_mult(*this, f);\
//     }\
// }\

struct Vec4i {
    int x, y, z, w;
    static Vec4i Identity() { return {1, 0, 0, 0}; }

    Vec4i operator-() { return {-x, -y, -z, -w}; }
    Vec4i operator-(const Vec4i b) const { return vec_sub(*this, b); }
    void operator-=(const Vec4i b) { *this = vec_sub(*this, b); }
    Vec4i operator+(const Vec4i b) const { return vec_add(*this, b); }
    void operator+=(const Vec4i b) { *this = vec_add(*this, b); }
    Vec4i operator * (const float f) { return vec_scalar_mult(*this, f); }
    void operator *= (const float f) { *this = vec_scalar_mult(*this, f); }
    Vec4i operator * (const Vec4i v) { return termwise_multiply(*this, v); }
    void operator *= (const Vec4i v) { *this = termwise_multiply(*this, v); }
};

inline Quat conjugate(const Quat a) {
    return {-a.x, -a.y, -a.z, a.w};
}

Quat axis_angle_quat(Vec3f v, float angle) {
    float const s = sin(angle * 0.5f);
    return { v.x * s, v.y * s, v.z * s, static_cast<float>(cos(angle * .5f)) };
}

inline Quat rotate_axis_angle(const Quat q, const Vec3f axis, const float angle) {
    return q * axis_angle_quat(axis, angle);
}

Vec4f Vec4f::FromVec3f(const Vec3f v3, float w) {
    Vec4f v4;
    ca_memcpy(reinterpret_cast<char *>(&v4), reinterpret_cast<const char *>(&v3), 3 * sizeof(int));
    v4.w = 0;
    return v4;
}

struct Vec3i {
    int x, y, z;
    static Vec3i Identity() { return {1, 0, 0}; }

    Vec3i operator-() { return {-x, -y, -z}; }
    Vec3i operator-(const Vec3i b) const { return vec_sub(*this, b); }
    void operator-=(const Vec3i b) { *this = vec_sub(*this, b); }
    Vec3i operator+(const Vec3i b) const { return vec_add(*this, b); }
    void operator+=(const Vec3i b) { *this = vec_add(*this, b); }
    Vec3i operator * (const float f) { return vec_scalar_mult(*this, f); }
    void operator *= (const float f) { *this = vec_scalar_mult(*this, f); }
    Vec3i operator * (const Vec3i v) { return termwise_multiply(*this, v); }
    void operator *= (const Vec3i v) { *this = termwise_multiply(*this, v); }
};

#define SUB_VEC4(a, b) { a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w }
#define SUB_VEC3(a, b) { a.x - b.x, a.y - b.y, a.z - b.z }

inline Quat vec_sub(Quat a, Quat b) { return SUB_VEC4(a, b); }
inline Vec4f vec_sub(Vec4f a, Vec4f b) { return SUB_VEC4(a, b); }
inline Vec4i vec_sub(Vec4i a, Vec4i b) { return SUB_VEC4(a, b); }
inline Vec3f vec_sub(Vec3f a, Vec3f b) { return SUB_VEC3(a, b); }
inline Vec3i vec_sub(Vec3i a, Vec3i b) { return SUB_VEC3(a, b); }

#define ADD_VEC4(a, b) { a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w }
#define ADD_VEC3(a, b) { a.x + b.x, a.y + b.y, a.z + b.z }

inline Quat vec_add(Quat a, Quat b) { return ADD_VEC4(a, b); }
inline Vec4f vec_add(Vec4f a, Vec4f b) { return ADD_VEC4(a, b); }
inline Vec4i vec_add(Vec4i a, Vec4i b) { return ADD_VEC4(a, b); }
inline Vec3f vec_add(Vec3f a, Vec3f b) { return ADD_VEC3(a, b); }
inline Vec3i vec_add(Vec3i a, Vec3i b) { return ADD_VEC3(a, b); }

#define TERMWISE_MULT_VEC4(a, b) { a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w }
#define TERMWISE_MULT_VEC3(a, b) { a.x * b.x, a.y * b.y, a.z * b.z }

inline Vec4f termwise_multiply(Vec4f a, Vec4f b) { return TERMWISE_MULT_VEC4(a, b); }
inline Vec4i termwise_multiply(Vec4i a, Vec4i b) { return TERMWISE_MULT_VEC4(a, b); }
inline Vec3f termwise_multiply(Vec3f a, Vec3f b) { return TERMWISE_MULT_VEC3(a, b); }
inline Vec3i termwise_multiply(Vec3i a, Vec3i b) { return TERMWISE_MULT_VEC3(a, b); }

#define DOT_VEC4(a, b) a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w
#define DOT_VEC3(a, b) a.x * b.x + a.y * b.y + a.z * b.z

inline float dot(Vec4f a, Vec4f b) { return DOT_VEC4(a, b); }
inline float dot(Vec4i a, Vec4i b) { return DOT_VEC4(a, b); }
inline float dot(Vec3f a, Vec3f b) { return DOT_VEC3(a, b); }
inline float dot(Vec3i a, Vec3i b) { return DOT_VEC3(a, b); }

#define SCALAR_MULT_VEC4(a, f) {a.x * f, a.y * f, a.z * f, a.w * f}
#define SCALAR_MULT_VEC3(a, f) {a.x * f, a.y * f, a.z * f}

inline Vec4f vec_scalar_mult(Vec4f a, int f) { return SCALAR_MULT_VEC4(a, f); }
inline Vec4i vec_scalar_mult(Vec4i a, int f) { return SCALAR_MULT_VEC4(a, f); }
inline Vec3f vec_scalar_mult(Vec3f a, int f) { return SCALAR_MULT_VEC3(a, f); }
inline Vec3i vec_scalar_mult(Vec3i a, int f) { return SCALAR_MULT_VEC3(a, f); }

inline Vec4f vec_scalar_mult(Vec4f a, float f) { return SCALAR_MULT_VEC4(a, f); }
inline Vec3f vec_scalar_mult(Vec3f a, float f) { return SCALAR_MULT_VEC3(a, f); }

#define LENGTH_VEC3(v) sqrt(DOT_VEC3(v, v))
#define LENGTH_VEC4(v) sqrt(DOT_VEC4(v, v))

#define NORMALIZE_VEC3(v) { float _v_len = LENGTH_VEC3(v); v = {v.x / _v_len, v.y / _v_len, v.z / _v_len}; }
#define NORMALIZE_VEC4(v) { float _v_len = LENGTH_VEC4(v); v = {v.x / _v_len, v.y / _v_len, v.z / _v_len, v.w / _v_len}; }

inline float length(Vec3i v) { return LENGTH_VEC3(v); }
inline float length(Vec3f v) { return LENGTH_VEC3(v); }
inline float length(Vec4i v) { return LENGTH_VEC4(v); }
inline float length(Vec4f v) { return LENGTH_VEC4(v); }

// inline void normalize_modify(Vec3i& v) { NORMALIZE_VEC3(v); }
inline void normalize_modify(Vec3f& v) { NORMALIZE_VEC3(v); }
// inline Vec3i normalize_copy(Vec3i v) { NORMALIZE_VEC3(v); return v; }
inline Vec3f normalize_copy(Vec3f v) { NORMALIZE_VEC3(v); return v; }

// inline void normalize_modify(Vec4i& v) { NORMALIZE_VEC4(v); }
inline void normalize_modify(Vec4f& v) { NORMALIZE_VEC4(v); }
// inline Vec4i normalize_copy(Vec4i v) { NORMALIZE_VEC4(v); return v; }
inline Vec4f normalize_copy(Vec4f v) { NORMALIZE_VEC4(v); return v; }

#define CROSS(a, b) { a.y * b.z - a.z * b.y, -a.x * b.z + a.z * b.x, a.x * b.y - a.y * b.x }
inline Vec3i cross(Vec3i a, Vec3i b) { return CROSS(a, b); }
inline Vec3f cross(Vec3f a, Vec3f b) { return CROSS(a, b); }
inline Quat cross(Quat q1, Quat q2) {
    return {
    q1.x * q2.x - q1.y * q2.y - q1.z * q2.z - q1.w * q2.w,
    q1.x * q2.y + q1.y * q2.x - q1.z * q2.w + q1.w * q2.z,
    q1.x * q2.z + q1.y * q2.w + q1.z * q2.x - q1.w * q2.y,
    q1.x * q2.w - q1.y * q2.z + q1.z * q2.y + q1.w * q2.x};
}

inline float * data(Vec4f& f) {
    return (float *)&f.x;
}

inline float * data(Vec3f& f) {
    return (float *)&f.x;
}

inline Mat4f transpose(Mat4f a) {
    return {
        a.x1, a.y1, a.z1, a.w1,
        a.x2, a.y2, a.z2, a.w2,
        a.x3, a.y3, a.z3, a.w3,
        a.x4, a.y4, a.z4, a.w4};
}

inline Mat3f transpose(Mat3f a) {
    return {
        a.x1, a.y1, a.z1,
        a.x2, a.y2, a.z2,
        a.x3, a.y3, a.z3};
}

// in-place implementation
void transpose(Mat4f * a) {
    int old_y1 = a->y1;
    int old_z1 = a->z1;
    int old_z2 = a->z2;
    int old_w1 = a->w1;
    int old_w2 = a->w2;
    int old_w3 = a->w3;

    a->y1 = a->x2;
    a->z1 = a->x3;
    a->z2 = a->y3;
    a->w1 = a->x4;
    a->w2 = a->y4;
    a->w3 = a->z4;

    a->x2 = old_y1;
    a->x3 = old_z1;
    a->y3 = old_z2;
    a->x4 = old_w1;
    a->y4 = old_w2;
    a->z4 = old_w3;
}

void transpose(Mat3f * a) {
    int old_y1 = a->y1;
    int old_z1 = a->z1;
    int old_z2 = a->z2;

    a->y1 = a->x2;
    a->z1 = a->x3;
    a->z2 = a->y3;

    a->x2 = old_y1;
    a->x3 = old_z1;
    a->y3 = old_z2;
}

inline Mat3f mat_mult(Mat3f a, Mat3f b) {
    Mat3f bt = transpose(b);
    return {
        dot(a.X(), bt.X()), dot(a.X(), bt.Y()), dot(a.X(), bt.Z()),
        dot(a.Y(), bt.X()), dot(a.Y(), bt.Y()), dot(a.Y(), bt.Z()),
        dot(a.Z(), bt.X()), dot(a.Z(), bt.Y()), dot(a.Z(), bt.Z())};
}

inline Mat4f mat_mult(Mat4f a, Mat4f b) {
    Mat4f bt = transpose(b);
    return {
        dot(a.X(), bt.X()), dot(a.X(), bt.Y()), dot(a.X(), bt.Z()), dot(a.X(), bt.W()),
        dot(a.Y(), bt.X()), dot(a.Y(), bt.Y()), dot(a.Y(), bt.Z()), dot(a.Y(), bt.W()),
        dot(a.Z(), bt.X()), dot(a.Z(), bt.Y()), dot(a.Z(), bt.Z()), dot(a.Z(), bt.W()),
        dot(a.W(), bt.X()), dot(a.W(), bt.Y()), dot(a.W(), bt.Z()), dot(a.W(), bt.W())};
}

inline Vec3f mat_vec_mult(Mat3f a, Vec3f b) {
    return { dot(a.X(), b), dot(a.Y(), b), dot(a.Z(), b) };
}
inline Vec4f mat_vec_mult(Mat4f a, Vec4f b) {
    return { dot(a.X(), b), dot(a.Y(), b), dot(a.Z(), b), dot(a.W(), b) };
}

Quat Mat4fToQuat (Mat4f mat) {
    // T fourXSquaredMinus1 = m[0][0] - m[1][1] - m[2][2];
    // T fourYSquaredMinus1 = m[1][1] - m[0][0] - m[2][2];
    // T fourZSquaredMinus1 = m[2][2] - m[0][0] - m[1][1];
    // T fourWSquaredMinus1 = m[0][0] + m[1][1] + m[2][2];

    float s0 = mat.x1 + mat.x2 + mat.z3;
    float s1 = mat.x1 - mat.y2 - mat.z3;
    float s2 = mat.y2 - mat.x1 - mat.z3;
    float s3 = mat.z3 - mat.x1 - mat.y2;

    int biggest_index = 0;
    float biggest_val = s0;
    if(s1 > biggest_val)
    {
        biggest_val = s1;
        biggest_index = 1;
    }
    if(s2 > biggest_val)
    {
        biggest_val = s2;
        biggest_index = 2;
    }
    if(s3 > biggest_val)
    {
        biggest_val = s3;
        biggest_index = 3;
    }

    float s = sqrt(1.f + biggest_val) * 0.5f;
    float mult = 0.25f / s;

    Quat q;
    switch(biggest_index)
    {
    case 0:
        q.w = s;
        q.x = (mat(1, 2) - mat(2, 1)) * mult;
        q.y = (mat(2, 0) - mat(0, 2)) * mult;
        q.z = (mat(0, 1) - mat(1, 0)) * mult;
        break;
    case 1:
        q.w = (mat(1, 2) - mat(2, 1)) * mult;
        q.x = s;
        q.y = (mat(0, 1) + mat(1, 0)) * mult;
        q.z = (mat(2, 0) + mat(0, 2)) * mult;
        break;
    case 2:
        q.w = (mat(2, 0) - mat(0, 2)) * mult;
        q.x = (mat(0, 1) + mat(1, 0)) * mult;
        q.y = s;
        q.z = (mat(1, 2) + mat(2, 1)) * mult;
        break;
    case 3:
        q.w = (mat(0, 1) - mat(1, 0)) * mult;
        q.x = (mat(2, 0) + mat(0, 2)) * mult;
        q.y = (mat(1, 2) + mat(2, 1)) * mult;
        q.z = s;
        break;
        
    default:					// Silence a -Wswitch-default warning in GCC. Should never actually get here. Assert is just for sanity.
        assert(false);
        break;
    }
    return q;
}

Mat4f Mat3fToMat4f (Mat3f mat3) {
    Mat4f mat4;
    ca_memcpy(reinterpret_cast<char *>(&mat4.X()), reinterpret_cast<char *>(&mat3.X()), 3 * sizeof(float));
    ca_memcpy(reinterpret_cast<char *>(&mat4.Y()), reinterpret_cast<char *>(&mat3.Y()), 3 * sizeof(float));
    ca_memcpy(reinterpret_cast<char *>(&mat4.Z()), reinterpret_cast<char *>(&mat3.Z()), 3 * sizeof(float));
    ca_memset(reinterpret_cast<char *>(&mat4.W()), 0, 4 * sizeof(float));
    mat4.x4 = 0.f;
    mat4.y4 = 0.f;
    mat4.z4 = 0.f;
    mat4.w4 = 1.f;
    return mat4;
}

Mat3f RotationMat3f(Quat quat) {
    Mat3f res = Mat3f::Identity();
    float qxx(quat.x * quat.x);
    float qyy(quat.y * quat.y);
    float qzz(quat.z * quat.z);
    float qxz(quat.x * quat.z);
    float qxy(quat.x * quat.y);
    float qyz(quat.y * quat.z);
    float qwx(quat.w * quat.x);
    float qwy(quat.w * quat.y);
    float qwz(quat.w * quat.z);

    // TODO make a pass on correct orientation
    // TODO wut
    res.x1 = 1 - 2 * (qyy +  qzz);
    res.x2 = 2 * (qxy + qwz);
    res.x3 = 2 * (qxz - qwy);

    res.y1 = 2 * (qxy - qwz);
    res.y2 = 1 - 2 * (qxx +  qzz);
    res.y3 = 2 * (qyz + qwx);

    res.z1 = 2 * (qxz + qwy);
    res.z2 = 2 * (qyz - qwx);
    res.z3 = 1 - 2 * (qxx +  qyy);

    return res;
}

Mat4f RotationMat4f(Quat quat) {
    return Mat3fToMat4f(RotationMat3f(quat));
}
Mat4f PerspectiveMatrix(float fovy, float aspect, float zFar, float zNear) {
    // assert(abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));
    float const tanHalfFovy = tan(fovy / 2.f);
    Mat4f persp;
    // Result[0][0] = static_cast<T>(1) / (aspect * tanHalfFovy);
    persp.x1 = 1.f / (aspect * tanHalfFovy);
    // Result[1][1] = static_cast<T>(1) / (tanHalfFovy);
    persp.y2 = 1.f / (tanHalfFovy);
    // Result[2][2] = (zFar + zNear) / (zFar - zNear);
    persp.z3 = (zFar + zNear) / (zFar - zNear);
    // Result[2][3] = 1.f;
    persp.w3 = 1.f;
    // Result[3][2] = -(2.f * zFar * zNear) / (zFar - zNear);
    persp.z4 = -(2.f * zFar * zNear) / (zFar - zNear);
    return persp;
}

Mat4f LookAtMatrix(const Vec3f eye, const Vec3f at, const Vec3f oldup) {
    Vec3f look = normalize_copy(at - eye);
    Vec3f tan = normalize_copy(cross(look, oldup));
    Vec3f up = cross(tan, look);
    Mat4f view_matrix;
    view_matrix.X() = {tan.x, up.x, -look.x, 0.f};
    view_matrix.Y() = {tan.y, up.y, -look.y, 0.f};
    view_matrix.Z() = {tan.z, up.z, -look.z, 0.f};
    view_matrix.W() = {-dot(eye, tan), -dot(eye, up), dot(eye, look), 1.f};
    return view_matrix;
}

}
