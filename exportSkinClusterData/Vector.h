#ifndef VECTOR_H
#define VECTOR_H

#include <assert.h>
#include <math.h>  

#ifdef _DEBUG
#define Assert(expr) assert(expr)
#else
#define Assert(expr) ((void)0)
#endif // NDEBUG

#ifdef _MSC_VER
#include <float.h>  // for _isnan() on VC++
#define isnan(x) _isnan(x)  // VC++ uses _isnan() instead of isnan()
//#else
//#include <math.h>  // for isnan() everywhere else
#endif

class Point;
class Normal;

// Geometry Declarations
class Vector {
public:
	// Vector Public Methods
	Vector() { x = y = z = 0.f; }
	Vector(float xx, float yy, float zz)
		: x(xx), y(yy), z(zz) {
			Assert(!HasNaNs());
	}
	explicit Vector(const Point &p);
	explicit Vector(const Normal &n);

	bool HasNaNs() const { return isnan(x) || isnan(y) || isnan(z); }
	
#ifndef NDEBUG
	// The default versions of these are fine for release builds; for debug
	// we define them so that we can add the Assert checks.
	Vector(const Vector &v) {
		Assert(!v.HasNaNs());
		x = v.x; y = v.y; z = v.z;
	}

	Vector &operator=(const Vector &v) {
		Assert(!v.HasNaNs());
		x = v.x; y = v.y; z = v.z;
		return *this;
	}
#endif // !NDEBUG
	Vector operator+(const Vector &v) const {
		Assert(!v.HasNaNs());
		return Vector(x + v.x, y + v.y, z + v.z);
	}

	Vector& operator+=(const Vector &v) {
		Assert(!v.HasNaNs());
		x += v.x; y += v.y; z += v.z;
		return *this;
	}
	Vector operator-(const Vector &v) const {
		Assert(!v.HasNaNs());
		return Vector(x - v.x, y - v.y, z - v.z);
	}

	Vector& operator-=(const Vector &v) {
		Assert(!v.HasNaNs());
		x -= v.x; y -= v.y; z -= v.z;
		return *this;
	}
	Vector operator*(float f) const { return Vector(f*x, f*y, f*z); }

	Vector &operator*=(float f) {
		Assert(!isnan(f));
		x *= f; y *= f; z *= f;
		return *this;
	}
	Vector operator/(float f) const {
		Assert(f != 0);
		float inv = 1.f / f;
		return Vector(x * inv, y * inv, z * inv);
	}

	Vector &operator/=(float f) {
		Assert(f != 0);
		float inv = 1.f / f;
		x *= inv; y *= inv; z *= inv;
		return *this;
	}
	Vector operator-() const { return Vector(-x, -y, -z); }
	float operator[](int i) const {
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}

	float &operator[](int i) {
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}
	float LengthSquared() const { return x*x + y*y + z*z; }
	float Length() const { return sqrtf(LengthSquared()); }
	

	bool operator==(const Vector &v) const {
		return x == v.x && y == v.y && z == v.z;
	}
	bool operator!=(const Vector &v) const {
		return x != v.x || y != v.y || z != v.z;
	}

	// Vector Public Data
	float x, y, z;
};


class Point {
public:
	// Point Public Methods
	Point() { x = y = z = 0.f; }
	Point(float xx, float yy, float zz)
		: x(xx), y(yy), z(zz) {
			Assert(!HasNaNs());
	}
#ifndef NDEBUG
	Point(const Point &p) {
		Assert(!p.HasNaNs());
		x = p.x; y = p.y; z = p.z;
	}

	Point &operator=(const Point &p) {
		Assert(!p.HasNaNs());
		x = p.x; y = p.y; z = p.z;
		return *this;
	}
#endif // !NDEBUG
	Point operator+(const Vector &v) const {
		Assert(!v.HasNaNs());
		return Point(x + v.x, y + v.y, z + v.z);
	}

	Point &operator+=(const Vector &v) {
		Assert(!v.HasNaNs());
		x += v.x; y += v.y; z += v.z;
		return *this;
	}
	Vector operator-(const Point &p) const {
		Assert(!p.HasNaNs());
		return Vector(x - p.x, y - p.y, z - p.z);
	}

	Point operator-(const Vector &v) const {
		Assert(!v.HasNaNs());
		return Point(x - v.x, y - v.y, z - v.z);
	}

	Point &operator-=(const Vector &v) {
		Assert(!v.HasNaNs());
		x -= v.x; y -= v.y; z -= v.z;
		return *this;
	}
	Point &operator+=(const Point &p) {
		Assert(!p.HasNaNs());
		x += p.x; y += p.y; z += p.z;
		return *this;
	}
	Point operator+(const Point &p) const {
		Assert(!p.HasNaNs());
		return Point(x + p.x, y + p.y, z + p.z);
	}
	Point operator* (float f) const {
		return Point(f*x, f*y, f*z);
	}
	Point &operator*=(float f) {
		x *= f; y *= f; z *= f;
		return *this;
	}
	Point operator/ (float f) const {
		float inv = 1.f/f;
		return Point(inv*x, inv*y, inv*z);
	}
	Point &operator/=(float f) {
		float inv = 1.f/f;
		x *= inv; y *= inv; z *= inv;
		return *this;
	}
	float operator[](int i) const {
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}

	float &operator[](int i) {
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}
	bool HasNaNs() const {
		return isnan(x) || isnan(y) || isnan(z);
	}

	bool operator==(const Point &p) const {
		return x == p.x && y == p.y && z == p.z;
	}
	bool operator!=(const Point &p) const {
		return x != p.x || y != p.y || z != p.z;
	}

	// Point Public Data
	float x, y, z;
};


class Normal {
public:
	// Normal Public Methods
	Normal() { x = y = z = 0.f; }
	Normal(float xx, float yy, float zz)
		: x(xx), y(yy), z(zz) {
			Assert(!HasNaNs());
	}
	Normal operator-() const {
		return Normal(-x, -y, -z);
	}
	Normal operator+ (const Normal &n) const {
		Assert(!n.HasNaNs());
		return Normal(x + n.x, y + n.y, z + n.z);
	}

	Normal& operator+=(const Normal &n) {
		Assert(!n.HasNaNs());
		x += n.x; y += n.y; z += n.z;
		return *this;
	}
	Normal operator- (const Normal &n) const {
		Assert(!n.HasNaNs());
		return Normal(x - n.x, y - n.y, z - n.z);
	}

	Normal& operator-=(const Normal &n) {
		Assert(!n.HasNaNs());
		x -= n.x; y -= n.y; z -= n.z;
		return *this;
	}
	bool HasNaNs() const {
		return isnan(x) || isnan(y) || isnan(z);
	}
	Normal operator*(float f) const {
		return Normal(f*x, f*y, f*z);
	}

	Normal &operator*=(float f) {
		x *= f; y *= f; z *= f;
		return *this;
	}
	Normal operator/(float f) const {
		Assert(f != 0);
		float inv = 1.f/f;
		return Normal(x * inv, y * inv, z * inv);
	}

	Normal &operator/=(float f) {
		Assert(f != 0);
		float inv = 1.f/f;
		x *= inv; y *= inv; z *= inv;
		return *this;
	}
	float LengthSquared() const { return x*x + y*y + z*z; }
	float Length() const        { return sqrtf(LengthSquared()); }

#ifndef NDEBUG
	Normal(const Normal &n) {
		Assert(!n.HasNaNs());
		x = n.x; y = n.y; z = n.z;
	}

	Normal &operator=(const Normal &n) {
		Assert(!n.HasNaNs());
		x = n.x; y = n.y; z = n.z;
		return *this;
	}
#endif // !NDEBUG
	explicit Normal(const Vector &v)
		: x(v.x), y(v.y), z(v.z) {
			Assert(!v.HasNaNs());
	}
	float operator[](int i) const {
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}

	float &operator[](int i) {
		Assert(i >= 0 && i <= 2);
		return (&x)[i];
	}

	bool operator==(const Normal &n) const {
		return x == n.x && y == n.y && z == n.z;
	}
	bool operator!=(const Normal &n) const {
		return x != n.x || y != n.y || z != n.z;
	}

	// Normal Public Data
	float x, y, z;
};

#endif