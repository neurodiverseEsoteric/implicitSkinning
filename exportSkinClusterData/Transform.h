#ifndef TRANSFORM_H
#define TRANSFORM_H

// Matrix4x4 Declarations
class Matrix4x4 {
	// Matrix4x4 Public Methods
	Matrix4x4() {
		m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.f;
		m[0][1] = m[0][2] = m[0][3] = m[1][0] =
			m[1][2] = m[1][3] = m[2][0] = m[2][1] = m[2][3] =
			m[3][0] = m[3][1] = m[3][2] = 0.f;
	}

	Matrix4x4(float mat[4][4]);

	Matrix4x4(float t00, float t01, float t02, float t03,
		float t10, float t11, float t12, float t13,
		float t20, float t21, float t22, float t23,
		float t30, float t31, float t32, float t33);

	bool operator==(const Matrix4x4 &m2) const {
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j)
				if (m[i][j] != m2.m[i][j]) return false;
		return true;
	}

	bool operator!=(const Matrix4x4 &m2) const {
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j)
				if (m[i][j] != m2.m[i][j]) return true;
		return false;
	}

	static Matrix4x4 Mul(const Matrix4x4 &m1, const Matrix4x4 &m2) {
		Matrix4x4 r;
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j)
				r.m[i][j] = m1.m[i][0] * m2.m[0][j] +
				m1.m[i][1] * m2.m[1][j] +
				m1.m[i][2] * m2.m[2][j] +
				m1.m[i][3] * m2.m[3][j];
		return r;
	}
	
	friend Matrix4x4 Inverse(const Matrix4x4 &);
	friend Matrix4x4 Transpose(const Matrix4x4 &);

	float m[4][4];
};



// Transform Declarations
class Transform {
public:
	// Transform Public Methods
	Transform() { }

	Transform(const float mat[4][4]) {
		m = Matrix4x4(mat[0][0], mat[0][1], mat[0][2], mat[0][3],
			mat[1][0], mat[1][1], mat[1][2], mat[1][3],
			mat[2][0], mat[2][1], mat[2][2], mat[2][3],
			mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
		mInv = Inverse(m);
	}

	Transform(const Matrix4x4 &mat)
		: m(mat), mInv(Inverse(mat)) {
	}

	Transform(const Matrix4x4 &mat, const Matrix4x4 &minv)
		: m(mat), mInv(minv) {
	}

	friend Transform Inverse(const Transform &t) {
		return Transform(t.mInv, t.m);
	}

	friend Transform Transpose(const Transform &t) {
		return Transform(Transpose(t.m), Transpose(t.mInv));
	}

	bool operator==(const Transform &t) const {
		return t.m == m && t.mInv == mInv;
	}

	bool operator!=(const Transform &t) const {
		return t.m != m || t.mInv != mInv;
	}

	bool operator<(const Transform &t2) const {
		for (unsigned int i = 0; i < 4; ++i)
			for (unsigned int j = 0; j < 4; ++j) {
				if (m.m[i][j] < t2.m.m[i][j]) return true;
				if (m.m[i][j] > t2.m.m[i][j]) return false;
			}
			return false;
	}
	bool IsIdentity() const {
		return (m.m[0][0] == 1.f && m.m[0][1] == 0.f &&
			m.m[0][2] == 0.f && m.m[0][3] == 0.f &&
			m.m[1][0] == 0.f && m.m[1][1] == 1.f &&
			m.m[1][2] == 0.f && m.m[1][3] == 0.f &&
			m.m[2][0] == 0.f && m.m[2][1] == 0.f &&
			m.m[2][2] == 1.f && m.m[2][3] == 0.f &&
			m.m[3][0] == 0.f && m.m[3][1] == 0.f &&
			m.m[3][2] == 0.f && m.m[3][3] == 1.f);
	}
	const Matrix4x4 &GetMatrix() const { return m; }

	const Matrix4x4 &GetInverseMatrix() const { return mInv; }

	bool HasScale() const {
		float la2 = (*this)(Vector(1,0,0)).LengthSquared();
		float lb2 = (*this)(Vector(0,1,0)).LengthSquared();
		float lc2 = (*this)(Vector(0,0,1)).LengthSquared();

		#define NOT_ONE(x) ((x) < .999f || (x) > 1.001f)
			return (NOT_ONE(la2) || NOT_ONE(lb2) || NOT_ONE(lc2));
		#undef NOT_ONE

	}

	inline Point operator()(const Point &pt) const;
	inline void operator()(const Point &pt, Point *ptrans) const;
	inline Vector operator()(const Vector &v) const;
	inline void operator()(const Vector &v, Vector *vt) const;
	inline Normal operator()(const Normal &) const;
	inline void operator()(const Normal &, Normal *nt) const;
	Transform operator*(const Transform &t2) const;

private:
	// Transform Private Data
	Matrix4x4 m, mInv;
};


Transform Translate(const Vector &delta);
Transform Scale(float x, float y, float z);


// Transform Inline Functions
inline Point Transform::operator()(const Point &pt) const {
	float x = pt.x, y = pt.y, z = pt.z;
	float xp = m.m[0][0]*x + m.m[0][1]*y + m.m[0][2]*z + m.m[0][3];
	float yp = m.m[1][0]*x + m.m[1][1]*y + m.m[1][2]*z + m.m[1][3];
	float zp = m.m[2][0]*x + m.m[2][1]*y + m.m[2][2]*z + m.m[2][3];
	float wp = m.m[3][0]*x + m.m[3][1]*y + m.m[3][2]*z + m.m[3][3];
	Assert(wp != 0);
	if (wp == 1.) return Point(xp, yp, zp);
	else          return Point(xp, yp, zp)/wp;
}


inline void Transform::operator()(const Point &pt,
	Point *ptrans) const {
		float x = pt.x, y = pt.y, z = pt.z;
		ptrans->x = m.m[0][0]*x + m.m[0][1]*y + m.m[0][2]*z + m.m[0][3];
		ptrans->y = m.m[1][0]*x + m.m[1][1]*y + m.m[1][2]*z + m.m[1][3];
		ptrans->z = m.m[2][0]*x + m.m[2][1]*y + m.m[2][2]*z + m.m[2][3];
		float w   = m.m[3][0]*x + m.m[3][1]*y + m.m[3][2]*z + m.m[3][3];
		if (w != 1.) *ptrans /= w;
}


inline Vector Transform::operator()(const Vector &v) const {
	float x = v.x, y = v.y, z = v.z;
	return Vector(m.m[0][0]*x + m.m[0][1]*y + m.m[0][2]*z,
		m.m[1][0]*x + m.m[1][1]*y + m.m[1][2]*z,
		m.m[2][0]*x + m.m[2][1]*y + m.m[2][2]*z);
}


inline void Transform::operator()(const Vector &v,
	Vector *vt) const {
		float x = v.x, y = v.y, z = v.z;
		vt->x = m.m[0][0] * x + m.m[0][1] * y + m.m[0][2] * z;
		vt->y = m.m[1][0] * x + m.m[1][1] * y + m.m[1][2] * z;
		vt->z = m.m[2][0] * x + m.m[2][1] * y + m.m[2][2] * z;
}


inline Normal Transform::operator()(const Normal &n) const {
	float x = n.x, y = n.y, z = n.z;
	return Normal(mInv.m[0][0]*x + mInv.m[1][0]*y + mInv.m[2][0]*z,
		mInv.m[0][1]*x + mInv.m[1][1]*y + mInv.m[2][1]*z,
		mInv.m[0][2]*x + mInv.m[1][2]*y + mInv.m[2][2]*z);
}


inline void Transform::operator()(const Normal &n,
	Normal *nt) const {
		float x = n.x, y = n.y, z = n.z;
		nt->x = mInv.m[0][0] * x + mInv.m[1][0] * y +
			mInv.m[2][0] * z;
		nt->y = mInv.m[0][1] * x + mInv.m[1][1] * y +
			mInv.m[2][1] * z;
		nt->z = mInv.m[0][2] * x + mInv.m[1][2] * y +
			mInv.m[2][2] * z;
}

#endif