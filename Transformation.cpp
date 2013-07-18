/****************************************************************
*																*
* C++ Vector and Matrix Algebra routines						*
* Author: Jean-Francois DOUE									*
* Version 3.1 --- October 1993									*
*																*
****************************************************************/
//
//	From "Graphics Gems IV / Edited by Paul S. Heckbert
//	Academic Press, 1994, ISBN 0-12-336156-9
//	"You are free to use and modify this code in any way 
//	you like." (p. xv)
//
//	Modified by J. Nagle, March 1997
//	-	All functions are inline.
//	-	All functions are const-correct.
//	-	All checking is via the standard "assert" macro.
//	-	Stream I/O is disabled for portability, but can be
//		re-enabled by defining ALGEBRA3IOSTREAMS.
//
//  Modified by Aline Normoyle
// - Added vec4
// - Added Cross Product operator, set/Print functions
// - Add divide by zero length check to Length()

#include "Transformation.h"

/****************************************************************
*																*
*		    vec2 Member functions								*
*																*
****************************************************************/

// CONSTRUCTORS

vec2::vec2() 
{
	n[VX] = 0; n[VY] = 0; 
}

vec2::vec2(const double x, const double y)
{
	n[VX] = x; n[VY] = y; 
}

vec2::vec2(const double d)
{ 
	n[VX] = n[VY] = d; 
}

vec2::vec2(const vec2& v)
{ 
	n[VX] = v.n[VX]; n[VY] = v.n[VY]; 
}

vec2::vec2(const vec3& v) // it is up to caller to avoid divide-by-zero
{ 
	n[VX] = v.n[VX]/v.n[VZ]; n[VY] = v.n[VY]/v.n[VZ]; 
};

vec2::vec2(const vec3& v, int dropAxis) {
	switch (dropAxis) {
	case VX: n[VX] = v.n[VY]; n[VY] = v.n[VZ]; break;
	case VY: n[VX] = v.n[VX]; n[VY] = v.n[VZ]; break;
	default: n[VX] = v.n[VX]; n[VY] = v.n[VY]; break;
	}
}


// ASSIGNMENT OPERATORS

vec2& vec2::operator = (const vec2& v)
{ 
	n[VX] = v.n[VX]; n[VY] = v.n[VY]; return *this; 
}

vec2& vec2::operator += ( const vec2& v )
{ 
	n[VX] += v.n[VX]; n[VY] += v.n[VY]; return *this; 
}

vec2& vec2::operator -= ( const vec2& v )
{ 
	n[VX] -= v.n[VX]; n[VY] -= v.n[VY]; return *this; 
}

vec2& vec2::operator *= ( const double d )
{ 
	n[VX] *= d; n[VY] *= d; return *this; 
}

vec2& vec2::operator /= ( const double d )
{ 
	double d_inv = 1.0f/d; n[VX] *= d_inv; n[VY] *= d_inv; return *this; 
}

double& vec2::operator [] ( int i) 
{
	assert(!(i < VX || i > VY));		// subscript check
	return n[i];
}

double vec2::operator [] ( int i) const 
{
	assert(!(i < VX || i > VY));
	return n[i];
}


// SPECIAL FUNCTIONS

double vec2::Length() const
{ 
	return sqrt(SqrLength()); 
}

double vec2::SqrLength() const
{ 
	return n[VX]*n[VX] + n[VY]*n[VY]; 
}

vec2& vec2::Normalize() // it is up to caller to avoid divide-by-zero
{ 
	double len = Length();
	if (len > 0) *this /= Length(); 
	return *this; 
}

// FRIENDS

vec2 operator - (const vec2& a)
{ 
	return vec2(-a.n[VX],-a.n[VY]); 
}

vec2 operator + (const vec2& a, const vec2& b)
{ 
	return vec2(a.n[VX]+ b.n[VX], a.n[VY] + b.n[VY]); 
}

vec2 operator - (const vec2& a, const vec2& b)
{ 
	return vec2(a.n[VX]-b.n[VX], a.n[VY]-b.n[VY]); 
}

vec2 operator * (const vec2& a, const double d)
{ 
	return vec2(d*a.n[VX], d*a.n[VY]); 
}

vec2 operator * (const double d, const vec2& a)
{ 
	return a*d; 
}

vec2 operator * (const mat3& a, const vec2& v) 
{
	vec3 av;
	av.n[VX] = a.v[0].n[VX]*v.n[VX] + a.v[0].n[VY]*v.n[VY] + a.v[0].n[VZ];
	av.n[VY] = a.v[1].n[VX]*v.n[VX] + a.v[1].n[VY]*v.n[VY] + a.v[1].n[VZ];
	av.n[VZ] = a.v[2].n[VX]*v.n[VX] + a.v[2].n[VY]*v.n[VY] + a.v[2].n[VZ];
	return av;
}

vec2 operator * (const vec2& v, const mat3& a) 
{ 
	return a.Transpose() * v; 
}

double operator * (const vec2& a, const vec2& b)
{ 
	return (a.n[VX]*b.n[VX] + a.n[VY]*b.n[VY]); 
}

vec2 operator / (const vec2& a, const double d)
{ 
	double d_inv = 1.0f/d; return vec2(a.n[VX]*d_inv, a.n[VY]*d_inv); 
}

vec3 operator ^ (const vec2& a, const vec2& b)
{ 
	return vec3(0.0f, 0.0f, a.n[VX] * b.n[VY] - b.n[VX] * a.n[VY]); 
}

int operator == (const vec2& a, const vec2& b)
{ 
	return (a.n[VX] == b.n[VX]) && (a.n[VY] == b.n[VY]); 
}

int operator != (const vec2& a, const vec2& b)
{ 
	return !(a == b); 
}

std::istream& operator >> (std::istream& s, vec2& v)
{
	double x, y;
	s >> x >> y;
	v[VX] = x; v[VY] = y;
	return s;
}

std::ostream& operator << (std::ostream& s, const vec2& v)
{
	s << (float) v.n[VX] << " " << (float) v.n[VY];
	return s;
}

void Swap(vec2& a, vec2& b)
{ 
	vec2 tmp(a); a = b; b = tmp; 
}

vec2 Min(const vec2& a, const vec2& b)
{ 
	return vec2(MIN(a.n[VX], b.n[VX]), MIN(a.n[VY], b.n[VY])); 
}

vec2 Max(const vec2& a, const vec2& b)
{ 
	return vec2(MAX(a.n[VX], b.n[VX]), MAX(a.n[VY], b.n[VY])); 
}

vec2 Prod(const vec2& a, const vec2& b)
{ 
	return vec2(a.n[VX] * b.n[VX], a.n[VY] * b.n[VY]); 
}

double Dot(const vec2& a, const vec2& b)
{
	return a*b;
}

/****************************************************************
*																*
*		    vec3 Member functions								*
*																*
****************************************************************/

// CONSTRUCTORS

vec3::vec3() 
{
    n[VX] = 0; n[VY] = 0; n[VZ] = 0; 
}

vec3::vec3(const double x, const double y, const double z)
{ 
	n[VX] = x; n[VY] = y; n[VZ] = z; 
}

vec3::vec3(const double d)
{ 
	n[VX] = n[VY] = n[VZ] = d; 
}

vec3::vec3(const vec3& v)
{ 
	n[VX] = v.n[VX]; n[VY] = v.n[VY]; n[VZ] = v.n[VZ]; 
}

vec3::vec3(const vec2& v)
{ 
	n[VX] = v.n[VX]; n[VY] = v.n[VY]; n[VZ] = 1.0f; 
}

vec3::vec3(const vec2& v, double d)
{ 
	n[VX] = v.n[VX]; n[VY] = v.n[VY]; n[VZ] = d; 
}

vec3::vec3(const vec4& v) // it is up to caller to avoid divide-by-zero
{ 
	n[VX] = v.n[VX] / v.n[VW]; 
	n[VY] = v.n[VY] / v.n[VW];
	n[VZ] = v.n[VZ] / v.n[VW]; 
}

vec3::vec3(const vec4& v, int dropAxis) 
{
	switch (dropAxis) {
	case VX: n[VX] = v.n[VY]; n[VY] = v.n[VZ]; n[VZ] = v.n[VW]; break;
	case VY: n[VX] = v.n[VX]; n[VY] = v.n[VZ]; n[VZ] = v.n[VW]; break;
	case VZ: n[VX] = v.n[VX]; n[VY] = v.n[VY]; n[VZ] = v.n[VW]; break;
	default: n[VX] = v.n[VX]; n[VY] = v.n[VY]; n[VZ] = v.n[VZ]; break;
	}
}


// ASSIGNMENT OPERATORS

vec3& vec3::operator = (const vec3& v)
{ 
	n[VX] = v.n[VX]; n[VY] = v.n[VY]; n[VZ] = v.n[VZ]; return *this; 
}

vec3& vec3::operator += ( const vec3& v )
{ 
	n[VX] += v.n[VX]; n[VY] += v.n[VY]; n[VZ] += v.n[VZ]; return *this; 
}

vec3& vec3::operator -= ( const vec3& v )
{ 
	n[VX] -= v.n[VX]; n[VY] -= v.n[VY]; n[VZ] -= v.n[VZ]; return *this; 
}

vec3& vec3::operator *= ( const double d )
{ 
	n[VX] *= d; n[VY] *= d; n[VZ] *= d; return *this; 
}

vec3& vec3::operator /= ( const double d )
{ 
	double d_inv = 1.0f/d; n[VX] *= d_inv; n[VY] *= d_inv; n[VZ] *= d_inv;
	return *this; 
}

double& vec3::operator [] ( int i) {
	assert(! (i < VX || i > VZ));
	return n[i];
}

double vec3::operator [] ( int i) const {
	assert(! (i < VX || i > VZ));
	return n[i];
}

void vec3::set(double x, double y, double z)
{
   n[0] = x; n[1] = y; n[2] = z;
}

// SPECIAL FUNCTIONS

double vec3::Length() const
{  
	return sqrt(SqrLength()); 
}

double vec3::SqrLength() const
{  
	return n[VX]*n[VX] + n[VY]*n[VY] + n[VZ]*n[VZ]; 
}

vec3& vec3::Normalize() // it is up to caller to avoid divide-by-zero
{ 
	double len = Length();
	if (len > 0.000001) *this /= Length(); 
	return *this;
}

vec3 vec3::Cross(const vec3 &v) const
{
	vec3 tmp;
	tmp[0] = n[1] * v.n[2] - n[2] * v.n[1];
	tmp[1] = n[2] * v.n[0] - n[0] * v.n[2];
	tmp[2] = n[0] * v.n[1] - n[1] * v.n[0];
	return tmp;
}

void vec3::Print(const char* title) const
{
   printf("%s (%.4f, %.4f, %.4f)\n", title, n[0], n[1], n[2]);
}

// FRIENDS

vec3 operator - (const vec3& a)
{  
	return vec3(-a.n[VX],-a.n[VY],-a.n[VZ]); 
}

vec3 operator + (const vec3& a, const vec3& b)
{ 
	return vec3(a.n[VX]+ b.n[VX], a.n[VY] + b.n[VY], a.n[VZ] + b.n[VZ]); 
}

vec3 operator - (const vec3& a, const vec3& b)
{ 
	return vec3(a.n[VX]-b.n[VX], a.n[VY]-b.n[VY], a.n[VZ]-b.n[VZ]); 
}

vec3 operator * (const vec3& a, const double d)
{ 
	return vec3(d*a.n[VX], d*a.n[VY], d*a.n[VZ]); 
}

vec3 operator * (const double d, const vec3& a)
{ 
	return a*d; 
}

vec3 operator * (const mat3& a, const vec3& v) 
{
#define ROWCOL(i) a.v[i].n[0]*v.n[VX] + a.v[i].n[1]*v.n[VY] \
	+ a.v[i].n[2]*v.n[VZ]
	return vec3(ROWCOL(0), ROWCOL(1), ROWCOL(2));
#undef ROWCOL // (i)
}

vec3 operator * (const mat4& a, const vec3& v)
{ 
	return a * vec4(v); 
}

vec3 operator * (const vec3& v, const mat4& a)
{ 
	return a.Transpose() * v; 
}

double operator * (const vec3& a, const vec3& b)
{ 
	return (a.n[VX]*b.n[VX] + a.n[VY]*b.n[VY] + a.n[VZ]*b.n[VZ]); 
}

vec3 operator / (const vec3& a, const double d)
{ 
	double d_inv = 1.0f/d; 
	return vec3(a.n[VX]*d_inv, a.n[VY]*d_inv, a.n[VZ]*d_inv); 
}

vec3 operator ^ (const vec3& a, const vec3& b) 
{
	return vec3(a.n[VY]*b.n[VZ] - a.n[VZ]*b.n[VY],
		a.n[VZ]*b.n[VX] - a.n[VX]*b.n[VZ],
		a.n[VX]*b.n[VY] - a.n[VY]*b.n[VX]);
}

int operator == (const vec3& a, const vec3& b)
{ 
	return (a.n[VX] == b.n[VX]) && (a.n[VY] == b.n[VY]) && (a.n[VZ] == b.n[VZ]);
}

int operator != (const vec3& a, const vec3& b)
{ 
	return !(a == b); 
}

void Swap(vec3& a, vec3& b)
{ 
	vec3 tmp(a); a = b; b = tmp; 
}

vec3 Min(const vec3& a, const vec3& b)
{ 
	return vec3(MIN(a.n[VX], b.n[VX]), MIN(a.n[VY], b.n[VY]), MIN(a.n[VZ], b.n[VZ])); 
}

vec3 Max(const vec3& a, const vec3& b)
{ 
	return vec3(MAX(a.n[VX], b.n[VX]), MAX(a.n[VY], b.n[VY]), MAX(a.n[VZ], b.n[VZ])); 
}

vec3 Prod(const vec3& a, const vec3& b)
{ 
	return vec3(a.n[VX] * b.n[VX], a.n[VY] * b.n[VY], a.n[VZ] * b.n[VZ]); 
}

double Dot(const vec3& a, const vec3& b)
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

double Distance(const vec3& a, const vec3& b)  // distance
{
   return sqrt( (b[0]-a[0])*(b[0]-a[0]) +
                (b[1]-a[1])*(b[1]-a[1]) +
                (b[2]-a[2])*(b[2]-a[2]));
}

double DistanceSqr(const vec3& a, const vec3& b)  // distance
{
   return ( (b[0]-a[0])*(b[0]-a[0]) +
            (b[1]-a[1])*(b[1]-a[1]) +
            (b[2]-a[2])*(b[2]-a[2]));
}

std::istream& operator>>(std::istream& s, vec3& v)
{
	double x, y, z;
	s >> x >> y >> z;
	v = vec3(x, y, z);
	return s;
}

std::ostream& operator<<(std::ostream& s, const vec3& v)
{
	s << (float) v[VX] << " " << (float) v[VY] << " " << (float) v[VZ];
	return s;
}


/****************************************************************
*																*
*		    vec4 Member functions								*
*																*
****************************************************************/

// CONSTRUCTORS

vec4::vec4() 
{
	n[VX] = 0; n[VY] = 0; n[VZ] = 0; n[VW] = 0; 
}

vec4::vec4(const double x, const double y, const double z, const double w)
{ 
	n[VX] = x; n[VY] = y; n[VZ] = z; n[VW] = w; 
}

vec4::vec4(const double d)
{  
	n[VX] = n[VY] = n[VZ] = n[VW] = d; 
}

vec4::vec4(const vec4& v)
{ 
	n[VX] = v.n[VX]; n[VY] = v.n[VY]; n[VZ] = v.n[VZ]; n[VW] = v.n[VW]; 
}

vec4::vec4(const vec3& v)
{ 
	n[VX] = v.n[VX]; n[VY] = v.n[VY]; n[VZ] = v.n[VZ]; n[VW] = 1.0f; 
}

vec4::vec4(const vec3& v, const double d)
{ 
	n[VX] = v.n[VX]; n[VY] = v.n[VY]; n[VZ] = v.n[VZ];  n[VW] = d; 
}


// ASSIGNMENT OPERATORS

vec4& vec4::operator = (const vec4& v)
{ 
	n[VX] = v.n[VX]; n[VY] = v.n[VY]; n[VZ] = v.n[VZ]; n[VW] = v.n[VW];
	return *this; 
}

vec4& vec4::operator += ( const vec4& v )
{ 
	n[VX] += v.n[VX]; n[VY] += v.n[VY]; n[VZ] += v.n[VZ]; n[VW] += v.n[VW];
	return *this; 
}

vec4& vec4::operator -= ( const vec4& v )
{ 
	n[VX] -= v.n[VX]; n[VY] -= v.n[VY]; n[VZ] -= v.n[VZ]; n[VW] -= v.n[VW];
	return *this; 
}

vec4& vec4::operator *= ( const double d )
{ 
	n[VX] *= d; n[VY] *= d; n[VZ] *= d; n[VW] *= d; 
	return *this; 
}

vec4& vec4::operator /= ( const double d )
{ 
	double d_inv = 1.0f/d; 
	n[VX] *= d_inv; n[VY] *= d_inv; n[VZ] *= d_inv; n[VW] *= d_inv; 
	return *this; 
}

double& vec4::operator [] ( int i) 
{
	assert(! (i < VX || i > VW));
	return n[i];
}

double vec4::operator [] ( int i) const 
{
	assert(! (i < VX || i > VW));
	return n[i];
}

void vec4::set(double x, double y, double z, double w)
{
   n[0] = x; n[1] = y; n[2] = z; n[3] = w;
}

// SPECIAL FUNCTIONS

double vec4::Length() const
{ 
	return sqrt(SqrLength()); 
}

double vec4::SqrLength() const
{ 
	return n[VX]*n[VX] + n[VY]*n[VY] + n[VZ]*n[VZ] + n[VW]*n[VW]; 
}

vec4& vec4::Normalize() // it is up to caller to avoid divide-by-zero
{ 
	double len = Length();
	if (len > 0) *this /= Length(); 
	return *this;
}

// FRIENDS

vec4 operator - (const vec4& a)
{ 
	return vec4(-a.n[VX],-a.n[VY],-a.n[VZ],-a.n[VW]); 
}

vec4 operator + (const vec4& a, const vec4& b)
{ 
	return vec4(a.n[VX] + b.n[VX], a.n[VY] + b.n[VY], a.n[VZ] + b.n[VZ], a.n[VW] + b.n[VW]); 
}

vec4 operator - (const vec4& a, const vec4& b)
{  
	return vec4(a.n[VX] - b.n[VX], a.n[VY] - b.n[VY], a.n[VZ] - b.n[VZ], a.n[VW] - b.n[VW]); 
}

vec4 operator * (const vec4& a, const double d)
{ 
	return vec4(d*a.n[VX], d*a.n[VY], d*a.n[VZ], d*a.n[VW] ); 
}

vec4 operator * (const double d, const vec4& a)
{ 
	return a*d; 
}

vec4 operator * (const mat4& a, const vec4& v) 
{
#define ROWCOL(i) a.v[i].n[0]*v.n[VX] + a.v[i].n[1]*v.n[VY] \
	+ a.v[i].n[2]*v.n[VZ] + a.v[i].n[3]*v.n[VW]
	return vec4(ROWCOL(0), ROWCOL(1), ROWCOL(2), ROWCOL(3));
#undef ROWCOL // (i)
}

vec4 operator * (const vec4& v, const mat4& a)
{ 
	return a.Transpose() * v; 
}

double operator * (const vec4& a, const vec4& b)
{ 
	return (a.n[VX]*b.n[VX] + a.n[VY]*b.n[VY] + a.n[VZ]*b.n[VZ] + a.n[VW]*b.n[VW]); 
}

vec4 operator / (const vec4& a, const double d)
{ 
	double d_inv = 1.0f/d; 
	return vec4(a.n[VX]*d_inv, a.n[VY]*d_inv, a.n[VZ]*d_inv, a.n[VW]*d_inv); 
}

int operator == (const vec4& a, const vec4& b)
{ 
	return (a.n[VX] == b.n[VX]) && (a.n[VY] == b.n[VY]) && (a.n[VZ] == b.n[VZ]) && (a.n[VW] == b.n[VW]); 
}

int operator != (const vec4& a, const vec4& b)
{ 
	return !(a == b); 
}

void Swap(vec4& a, vec4& b)
{ 
	vec4 tmp(a); a = b; b = tmp; 
}

vec4 Min(const vec4& a, const vec4& b)
{ 
	return vec4(MIN(a.n[VX], b.n[VX]), MIN(a.n[VY], b.n[VY]), MIN(a.n[VZ], b.n[VZ]), MIN(a.n[VW], b.n[VW])); 
}

vec4 Max(const vec4& a, const vec4& b)
{ 
	return vec4(MAX(a.n[VX], b.n[VX]), MAX(a.n[VY], b.n[VY]), MAX(a.n[VZ], b.n[VZ]), MAX(a.n[VW], b.n[VW])); 
}

vec4 Prod(const vec4& a, const vec4& b)
{ 
	return vec4(a.n[VX] * b.n[VX], a.n[VY] * b.n[VY], a.n[VZ] * b.n[VZ], a.n[VW] * b.n[VW]); 
}

std::istream& operator >> (std::istream& s, vec4& v)
{
	double x, y, z, w;
	s >> x >> y >> z >> w;
	v = vec4(x, y, z, w);
	return s;
}

std::ostream& operator << (std::ostream& s, const vec4& v)
{
	s << (float) v[VX] << "" << (float) v[VY] << "" << (float) v[VZ] << "" << (float) v[VW];
	return s;
}

/****************************************************************
*																*
*		    mat3 member functions								*
*																*
****************************************************************/

// CONSTRUCTORS

mat3::mat3() 
{
	v[0] = vec3(0.0f,0.0f,0.0f);
	v[1] = v[2] = v[0];
}

mat3::mat3(const vec3& v0, const vec3& v1, const vec3& v2)
{ 
	v[0] = v0; v[1] = v1; v[2] = v2; 
}

mat3::mat3(const double d)
{ 
	v[0] = v[1] = v[2] = vec3(d); 
}

mat3::mat3(const mat3& m)
{ 
	v[0] = m.v[0]; v[1] = m.v[1]; v[2] = m.v[2]; 
}

mat3::mat3(const mat4& m){ 
	v[0] = vec3(m[0][0], m[0][1], m[0][2]);
	v[1] = vec3(m[1][0], m[1][1], m[1][2]);
	v[2] = vec3(m[2][0], m[2][1], m[2][2]);
}

mat3 mat3::Translation2D(const vec2& v)
{
	return mat3(vec3(1.0f, 0.0f, v[VX]),
		vec3(0.0f, 1.0f, v[VY]),
		vec3(0.0f, 0.0f, 1.0f));
}

mat3 mat3::Rotation2DDeg(const vec2& center, const double angleDeg)
{
	double  angleRad = angleDeg * Deg2Rad;
	return Rotation2DRad(center, angleRad);
}

mat3 mat3::Rotation2DRad(const vec2& center, const double angleRad)
{
	double  c = cos(angleRad), s = sin(angleRad);

	return mat3(vec3(c, -s, center[VX] * (1.0f-c) + center[VY] * s),
		vec3(s, c, center[VY] * (1.0f-c) - center[VX] * s),
		vec3(0.0f, 0.0f, 1.0f));
}

mat3 mat3::Scaling2D(const vec2& scaleVector)
{
	return mat3(vec3(scaleVector[VX], 0.0f, 0.0f),
		vec3(0.0f, scaleVector[VY], 0.0f),
		vec3(0.0f, 0.0f, 1.0f));
}

mat3 mat3::Rotation3DDeg(const vec3& axis, const double angleDeg)
{
	double  angleRad = angleDeg * Deg2Rad;
	return Rotation3DRad(axis, angleRad);
}

mat3 mat3::Rotation3DRad(const vec3& axis, const double angleRad)
{
	double c = cos(angleRad), s = sin(angleRad), t = 1.0f - c;
	vec3 Axis = axis;
	Axis.Normalize();
	return mat3(vec3(t * Axis[VX] * Axis[VX] + c,
		t * Axis[VX] * Axis[VY] - s * Axis[VZ],
		t * Axis[VX] * Axis[VZ] + s * Axis[VY]),
		vec3(t * Axis[VX] * Axis[VY] + s * Axis[VZ],
		t * Axis[VY] * Axis[VY] + c,
		t * Axis[VY] * Axis[VZ] - s * Axis[VX]),
		vec3(t * Axis[VX] * Axis[VZ] - s * Axis[VY],
		t * Axis[VY] * Axis[VZ] + s * Axis[VX],
		t * Axis[VZ] * Axis[VZ] + c)
		);
}

mat3 mat3::Rotation3DDeg(const int Axis, const double angleDeg)
{
	double  angleRad = angleDeg * Deg2Rad;
	return Rotation3DRad(Axis, angleRad);
}

mat3 mat3::Rotation3DRad(const int Axis, const double angleRad)
{
	mat3 m;
	switch(Axis)
	{
	case VX: m = Rotation3DRad(axisX, angleRad);
		break;
	case VY: m = Rotation3DRad(axisY, angleRad);
		break;
	case VZ: m = Rotation3DRad(axisZ, angleRad);
		break;
	}
	return m;
}

mat3 mat3::Slerp(const mat3& rot0, const mat3& rot1, const double& fPerc)
{
	Quaternion q0, q1;
	q0.FromRotation(rot0);
	q1.FromRotation(rot1);
	Quaternion q = Quaternion::Slerp(fPerc, q0, q1);
	mat3 rot;
	rot = q.ToRotation();
	return rot;
}

mat3 mat3::Lerp(const mat3& rot0, const mat3& rot1, const double& fPerc)
{
	Quaternion q0, q1;
	q0.FromRotation(rot0);
	q1.FromRotation(rot1);
	if (q0.W() < 0.0f)
	{
		q0 = -q0;
	}
	if (q1.W() < 0.0f)
	{
		q1 = -q1;
	}
	Quaternion q = q0 * (1.0f - fPerc) +  q1 * fPerc;
	q.Normalize();
	mat3 rot;
	rot.FromQuaternion(q);
	return rot;
}

// Rotation operations, matrix must be orthonomal
bool mat3::ToEulerAnglesXYZ(vec3& angleRad) const
{
	angleRad[VY] = asin(v[0][2]);
	if ( angleRad[VY] > -M_PI_2 + EPSILON )
	{
		if ( angleRad[VY] < M_PI_2 - EPSILON )
		{
			angleRad[VX] = atan2(-v[1][2], v[2][2]);
			angleRad[VZ] = atan2(-v[0][1], v[0][0]);
			return true;
		}
		else
		{
			// WARNING.  Not a unique solution.
			angleRad[VZ] = 0.0f;
			angleRad[VX] = atan2(v[1][0], v[1][1]);
			return false;
		}
	}
	else
	{
		// WARNING.  Not a unique solution.
		angleRad[VZ] = 0.0f;  // any angle works
		angleRad[VX] = -atan2(v[1][0], v[1][1]);
		return false;
	}
}

bool mat3::ToEulerAnglesXZY(vec3& angleRad) const
{
	angleRad[VZ] = asin(-v[0][1]);
	if ( angleRad[VZ] > -M_PI_2 + EPSILON )
	{
		if ( angleRad[VZ] < M_PI_2 - EPSILON )
		{
			angleRad[VX] = atan2(v[2][1], v[1][1]);
			angleRad[VY] = atan2(v[0][2], v[0][0]);
			return true;
		}
		else
		{
			// WARNING.  Not a unique solution.
			angleRad[VY] = 0.0f;  // any angle works
			angleRad[VX] = atan2(v[2][0], v[2][2]);
			return false;
		}
	}
	else
	{
		// WARNING.  Not a unique solution.
		angleRad[VY] = 0.0f;
		angleRad[VX] = -atan2(v[2][0], v[2][2]);
		return false;
	}
}

bool mat3::ToEulerAnglesYXZ(vec3& angleRad) const
{
	angleRad[VX] = asin(-v[1][2]);
	if ( angleRad[VX] > -M_PI_2 + EPSILON )
	{
		if ( angleRad[VX] < M_PI_2 - EPSILON )
		{
			angleRad[VY] = atan2(v[0][2], v[2][2]);
			angleRad[VZ] = atan2(v[1][0], v[1][1]);
			return true;
		}
		else
		{
			// WARNING.  Not a unique solution.
			angleRad[VZ] = 0.0f;
			angleRad[VY] = atan2(v[0][1], v[0][0]);
			return false;
		}
	}
	else
	{
		// WARNING.  Not a unique solution.
		angleRad[VZ] = 0.0f;
		angleRad[VY] = -atan2(v[0][1], v[0][0]);
		return false;
	}
}

bool mat3::ToEulerAnglesYZX(vec3& angleRad) const
{
	angleRad[VZ] = asin(v[1][0]);
	if ( angleRad[VZ] > -M_PI_2 + EPSILON )
	{
		if ( angleRad[VZ] < M_PI_2 - EPSILON )
		{
			angleRad[VY] = atan2(-v[2][0], v[0][0]);
			angleRad[VX] = atan2(-v[1][2], v[1][1]);
			return true;
		}
		else
		{
			// WARNING.  Not a unique solution.
			angleRad[VX] = 0.0f;
			angleRad[VY] = atan2(v[2][1], v[2][2]);
			return false;
		}
	}
	else
	{
		// WARNING.  Not a unique solution.
		angleRad[VX] = 0.0f;
		angleRad[VY] = -atan2(v[2][1], v[2][2]);
		return false;
	}	
}

bool mat3::ToEulerAnglesZXY(vec3& angleRad) const
{
	angleRad[VX] = asin(v[2][1]);
	if ( angleRad[VX] > -M_PI_2 + EPSILON )
	{
		if ( angleRad[VX] < M_PI_2 - EPSILON )
		{
			angleRad[VZ] = atan2(-v[0][1], v[1][1]);
			angleRad[VY] = atan2(-v[2][0], v[2][2]);
			return true;
		}
		else
		{
			// WARNING.  Not a unique solution.
			angleRad[VY] = 0.0f;
			angleRad[VZ] = atan2(v[0][2], v[0][0]);
			return false;
		}
	}
	else
	{
		// WARNING.  Not a unique solution.
		angleRad[VY] = 0.0f;
		angleRad[VZ] = -atan2(v[0][2], v[0][0]);
		return false;
	}
}

bool mat3::ToEulerAnglesZYX(vec3& angleRad) const
{
	angleRad[VY] = -asin(v[2][0]);
	if ( angleRad[VY] > -M_PI_2 + EPSILON )
	{
		if ( angleRad[VY] < M_PI_2 - EPSILON )
		{
			angleRad[VZ] = atan2(v[1][0], v[0][0]);
			angleRad[VX] = atan2(v[2][1], v[2][2]);
			return true;
		}
		else
		{
			// WARNING.  Not a unique solution.
			angleRad[VX] = 0.0f;
			angleRad[VZ] = atan2(-v[0][1], v[0][2]);
			return false;
		}
	}
	else
	{
		// WARNING.  Not a unique solution.
		angleRad[VX] = 0.0f;
		angleRad[VZ] = atan2(v[0][1], v[0][2]);
		return false;
	}
}

mat3 mat3::FromEulerAnglesXYZ(const vec3& angleRad)
{
	mat3 m = mat3::Rotation3DRad(axisX, angleRad[VX])
		* mat3::Rotation3DRad(axisY, angleRad[VY])
		* mat3::Rotation3DRad(axisZ, angleRad[VZ]);
	*this = m;
	return m;
}

mat3 mat3::FromEulerAnglesXZY(const vec3& angleRad)
{
	mat3 m = mat3::Rotation3DRad(axisX, angleRad[VX])
		* mat3::Rotation3DRad(axisZ, angleRad[VZ])
		* mat3::Rotation3DRad(axisY, angleRad[VY]);
	*this = m;
	return m;
}

mat3 mat3::FromEulerAnglesYXZ(const vec3& angleRad)
{
	mat3 m = mat3::Rotation3DRad(axisY, angleRad[VY])
		* mat3::Rotation3DRad(axisX, angleRad[VX])
		* mat3::Rotation3DRad(axisZ, angleRad[VZ]);
	*this = m;
	return m;
}

mat3 mat3::FromEulerAnglesYZX(const vec3& angleRad)
{
	mat3 m = mat3::Rotation3DRad(axisY, angleRad[VY])
		* mat3::Rotation3DRad(axisZ, angleRad[VZ])
		* mat3::Rotation3DRad(axisX, angleRad[VX]);
	*this = m;
	return m;
}

mat3 mat3::FromEulerAnglesZXY(const vec3& angleRad)
{
	mat3 m = mat3::Rotation3DRad(axisZ, angleRad[VZ])
		* mat3::Rotation3DRad(axisX, angleRad[VX])
		* mat3::Rotation3DRad(axisY, angleRad[VY]);
	*this = m;
	return m;
}

mat3 mat3::FromEulerAnglesZYX(const vec3& angleRad)
{
	mat3 m = mat3::Rotation3DRad(axisZ, angleRad[VZ])
		* mat3::Rotation3DRad(axisY, angleRad[VY])
		* mat3::Rotation3DRad(axisX, angleRad[VX]);
	*this = m;
	return m;
}

bool mat3::Reorthogonalize()
{
	// Factor M = QR where Q is orthogonal and R is upper triangular.
	// Algorithm uses Gram-Schmidt orthogonalization (the QR algorithm).
	//
	// If M = [ m0 | m1 | m2 ] and Q = [ q0 | q1 | q2 ], then
	//
	//   q0 = m0/|m0|
	//   q1 = (m1-(q0*m1)q0)/|m1-(q0*m1)q0|
	//   q2 = (m2-(q0*m2)q0-(q1*m2)q1)/|m2-(q0*m2)q0-(q1*m2)q1|
	//
	// where |V| indicates length of vector V and A*B indicates dot
	// product of vectors A and B.  The matrix R has entries
	//
	//   r00 = q0*m0  r01 = q0*m1  r02 = q0*m2
	//   r10 = 0      r11 = q1*m1  r12 = q1*m2
	//   r20 = 0      r21 = 0      r22 = q2*m2
	//
	// The reorthogonalization replaces current matrix by computed Q.

	const double fEpsilon = 1e-05f;

	// unitize column 0
	double fLength = sqrt(v[0][0] * v[0][0] + v[1][0] * v[1][0] + v[2][0] * v[2][0]);
	if ( fLength < fEpsilon )
		return false;
	double fInvLength = 1.0f / fLength;
	v[0][0] *= fInvLength;
	v[1][0] *= fInvLength;
	v[2][0] *= fInvLength;

	// project out column 0 from column 1
	double fDot = v[0][0] * v[0][1] + v[1][0] * v[1][1] + v[2][0] * v[2][1];
	v[0][1] -= fDot * v[0][0];
	v[1][1] -= fDot * v[1][0];
	v[2][1] -= fDot * v[2][0];

	// unitize column 1
	fLength = sqrt(v[0][1] * v[0][1] + v[1][1] * v[1][1] + v[2][1] * v[2][1]);
	if ( fLength < fEpsilon )
		return false;
	fInvLength = 1.0f/fLength;
	v[0][1] *= fInvLength;
	v[1][1] *= fInvLength;
	v[2][1] *= fInvLength;

	// project out column 0 from column 2
	fDot = v[0][0] * v[0][2] + v[1][0] * v[1][2] + v[2][0] * v[2][2];
	v[0][2] -= fDot * v[0][0];
	v[1][2] -= fDot * v[1][0];
	v[2][2] -= fDot * v[2][0];

	// project out column 1 from column 2
	fDot = v[0][1] * v[0][2] + v[1][1] * v[1][2] + v[2][1] * v[2][2];
	v[0][2] -= fDot * v[0][1];
	v[1][2] -= fDot * v[1][1];
	v[2][2] -= fDot * v[2][1];

	// unitize column 2
	fLength = sqrt(v[0][2] * v[0][2] + v[1][2] * v[1][2] + v[2][2] * v[2][2]);
	if ( fLength < fEpsilon )
		return false;
	fInvLength = 1.0f / fLength;
	v[0][2] *= fInvLength;
	v[1][2] *= fInvLength;
	v[2][2] *= fInvLength;

	return true;
}

// Conversion with Quaternion
Quaternion mat3::ToQuaternion() const
{
	Quaternion q;
    q.FromRotation(*this);
    return q;
}

void mat3::FromQuaternion(const Quaternion& q)
{
	(*this) = q.ToRotation();
}


void mat3::ToAxisAngle(vec3& axis, double& angleRad) const
{
	// Let (x,y,z) be the unit-length axis and let A be an angle of rotation.
	// The rotation matrix is R = I + sin(A)*P + (1-cos(A))*P^2 where
	// I is the identity and
	//
	//       +-        -+
	//   P = |  0 +z -y |
	//       | -z  0 +x |
	//       | +y -x  0 |
	//       +-        -+
	//
	// Some algebra will show that
	//
	//   cos(A) = (trace(R)-1)/2  and  R - R^t = 2*sin(A)*P

	double fTrace = v[0][0] + v[1][1] + v[2][2];
	angleRad = acos( 0.5f * (fTrace - 1.0f));

	axis[VX] = v[1][2] - v[2][1];
	axis[VY] = v[2][0] - v[0][2];
	axis[VZ] = v[0][1] - v[1][0];
	double fLength = axis.Length();
	const double fEpsilon = 1e-06f;
	if ( fLength > fEpsilon )
	{
		double fInvLength = 1.0f / fLength;
		axis *= -fInvLength;
	}
	else  // angle is 0 or pi
	{
		if ( angleRad > 1.0f )  // any number strictly between 0 and pi works
		{
			// angle must be pi
			axis[VX] = sqrt(0.5f * (1.0f + v[0][0]));
			axis[VY] = sqrt(0.5f * (1.0f + v[1][1]));
			axis[VZ] = sqrt(0.5f * (1.0f + v[2][2]));

			// determine signs of axis components
			double tx, ty, tz;
			tx = v[0][0] * axis[VX] + v[0][1] * axis[VY] + v[0][2] * axis[VZ] - axis[VX];
			ty = v[1][0] * axis[VX] + v[1][1] * axis[VY] + v[1][2] * axis[VZ] - axis[VY];
			tz = v[2][0] * axis[VX] + v[2][1] * axis[VY] + v[2][2] * axis[VZ] - axis[VZ];
			fLength = tx * tx + ty * ty + tz * tz;
			if ( fLength < fEpsilon )
			{
				axis = -axis;
				return;
			}

			axis[VZ] = -axis[VZ];
			tx = v[0][0] * axis[VX] + v[0][1] * axis[VY] + v[0][2] * axis[VZ] - axis[VX];
			ty = v[1][0] * axis[VX] + v[1][1] * axis[VY] + v[1][2] * axis[VZ] - axis[VY];
			tz = v[2][0] * axis[VX] + v[2][1] * axis[VY] + v[2][2] * axis[VZ] - axis[VZ];
			fLength = tx * tx + ty * ty + tz * tz;
			if ( fLength < fEpsilon )
			{
				axis = -axis;
				return;
			}

			axis[VY] = -axis[VY];
			tx = v[0][0] * axis[VX] + v[0][1] * axis[VY] + v[0][2] * axis[VZ] - axis[VX];
			ty = v[1][0] * axis[VX] + v[1][1] * axis[VY] + v[1][2] * axis[VZ] - axis[VY];
			tz = v[2][0] * axis[VX] + v[2][1] * axis[VY] + v[2][2] * axis[VZ] - axis[VZ];
			fLength = tx * tx + ty * ty + tz * tz;
			if ( fLength < fEpsilon )
			{
				axis = -axis;
				return;
			}
		}
		else
		{
			// Angle is zero, matrix is the identity, no unique axis, so
			// return (0,1,0) for as good a guess as any.
			angleRad = 0.0f;
			axis[VX] = 0.0f;
			axis[VY] = 1.0f;
			axis[VZ] = 0.0f;
		}
	}
}

mat3 mat3::Inverse() const    // Gauss-Jordan elimination with partial pivoting
{
	mat3 a(*this),	    // As a evolves from original mat into identity
		b(identity3D);   // b evolves from identity into inverse(a)
	int	 i, j, i1;

	// Loop over cols of a from left to right, eliminating above and below diag
	for (j=0; j<3; j++) {   // Find largest pivot in column j among rows j..2
		i1 = j;		    // Row with largest pivot candidate
		for (i=j+1; i<3; i++)
			if (fabs(a.v[i].n[j]) > fabs(a.v[i1].n[j]))
				i1 = i;

		// Swap rows i1 and j in a and b to put pivot on diagonal
		Swap(a.v[i1], a.v[j]);
		Swap(b.v[i1], b.v[j]);

		// Scale row j to have a unit diagonal
		if (a.v[j].n[j]==0.)
			ALGEBRA_ERROR("mat3::inverse: singular matrix; can't invert\n");
		b.v[j] /= a.v[j].n[j];
		a.v[j] /= a.v[j].n[j];

		// Eliminate off-diagonal elements in col j of a, doing identical ops to b
		for (i=0; i<3; i++)
			if (i!=j) 
			{
				b.v[i] -= a.v[i].n[j]*b.v[j];
				a.v[i] -= a.v[i].n[j]*a.v[j];
			}
	}
	return b;
}

void mat3::FromAxisAngle(const vec3& axis, const double& angleRad)
{
	*this = Rotation3DRad(axis, angleRad);
}


// ASSIGNMENT OPERATORS

mat3& mat3::operator = ( const mat3& m )
{ 
	v[0] = m.v[0]; v[1] = m.v[1]; v[2] = m.v[2]; 
	return *this; 
}

mat3& mat3::operator += ( const mat3& m )
{ 
	v[0] += m.v[0]; v[1] += m.v[1]; v[2] += m.v[2]; 
	return *this; 
}

mat3& mat3::operator -= ( const mat3& m )
{ 
	v[0] -= m.v[0]; v[1] -= m.v[1]; v[2] -= m.v[2]; 
	return *this; 
}

mat3& mat3::operator *= ( const double d )
{ 
	v[0] *= d; v[1] *= d; v[2] *= d; 
	return *this; 
}

mat3& mat3::operator /= ( const double d )
{ 
	v[0] /= d; v[1] /= d; v[2] /= d; 
	return *this; 
}

vec3& mat3::operator [] ( int i) 
{
	assert(! (i < VX || i > VZ));
	return v[i];
}

const vec3& mat3::operator [] ( int i) const 
{
	assert(!(i < VX || i > VZ));
	return v[i];
}

// SPECIAL FUNCTIONS

mat3 mat3::Transpose() const 
{
	return mat3(vec3(v[0][0], v[1][0], v[2][0]),
		vec3(v[0][1], v[1][1], v[2][1]),
		vec3(v[0][2], v[1][2], v[2][2]));
}


void mat3::WriteToGLMatrix(double* m)
{
	m[0] = v[0][0]; m[4] = v[0][1]; m[8] = v[0][2];  m[12] = 0.0f;
	m[1] = v[1][0]; m[5] = v[1][1]; m[9] = v[1][2];  m[13] = 0.0f;
	m[2] = v[2][0]; m[6] = v[2][1]; m[10] = v[2][2]; m[14] = 0.0f;
	m[3] = 0.0f;    m[7] = 0.0f;    m[11] = 0.0f;    m[15] = 1.0f;
}

void mat3::ReadFromGLMatrix(double* m)
{
	v[0][0] = m[0]; v[0][1] = m[4]; v[0][2] = m[8];
	v[1][0] = m[1]; v[1][1] = m[5]; v[1][2] = m[9];
	v[2][0] = m[2]; v[2][1] = m[6]; v[2][2] = m[10];
}

vec3 mat3::GetRow(unsigned int axis) const
{
	vec3 rowVec = v[axis];
	return rowVec;
}

vec3 mat3::GetCol(unsigned int axis) const
{
	vec3 colVec;
	colVec[0] = v[0][axis]; colVec[1] = v[1][axis]; colVec[2] = v[2][axis];
	return colVec;
}

void mat3::SetRow(unsigned int axis, const vec3& rowVec)
{
	v[axis] = rowVec;
}

void mat3::SetCol(unsigned int axis, const vec3& colVec)
{
	v[0][axis] = colVec[0]; v[1][axis] = colVec[1]; v[2][axis] = colVec[2];
}

vec3 mat3::GetYawPitchRoll(unsigned int leftAxis, unsigned int upAxis, unsigned int frontAxis) const
{
	// Assume world coordinates: Y up, X left, Z front.

	vec3 leftVect, upVect, frontVect, dVect, angles, frontVect2, leftVect2;
	double t, value, x, y;
	leftVect = GetCol(leftAxis);
	upVect = GetCol(upAxis);
	frontVect = GetCol(frontAxis);

	// Compute yaw angle
	if (frontVect[VY] >= 0.0f && upVect[VY] >= 0.0f)
	{
		frontVect2 = frontVect;
		dVect = -upVect - frontVect2;
	}else if (frontVect[VY] < 0.0f && upVect[VY] < 0.0f)
	{
		frontVect2 = -frontVect;
		dVect = upVect - frontVect2;
	}else if (frontVect[VY] >= 0.0f && upVect[VY] < 0.0f)
	{
		frontVect2 = -frontVect;
		dVect = -upVect - frontVect2;

	}else if (frontVect[VY] < 0.0f && upVect[VY] >= 0.0f)
	{
		frontVect2 = frontVect;
		dVect = upVect - frontVect2;
	}
	t = -frontVect2[VY] / dVect[VY];
	x = frontVect2[VZ] + t * dVect[VZ];
	y = frontVect2[VX] + t * dVect[VX];
	angles[0] = atan2(y, x);
	frontVect2 = vec3(y, 0.0f, x);
	frontVect2.Normalize();
	leftVect2 = vec3(0.0f, 1.0f, 0.0f);
	leftVect2 = leftVect2.Cross(frontVect2);

	// Compute pitch angle
	double v = acos(frontVect * frontVect2);
	if (frontVect[VY] >= 0.0f)
	{
		value = -v;
	}else
	{
		value = v;
	}
	angles[1] = value;

	// Compute roll angle
	v = acos(leftVect * leftVect2);
	if (leftVect[VY] >= 0.0f)
	{
		value = -v;
	}else
	{
		value = v;
	}
	angles[2] = value;

	return angles;
}

void mat3::ToGLMatrix( double* pData )
{
	pData[0] = v[0][0]; pData[4] = v[0][1]; pData[8]  = v[0][2]; pData[12] = 0.0f;
	pData[1] = v[1][0]; pData[5] = v[1][1]; pData[9]  = v[1][2]; pData[13] = 0.0f;
	pData[2] = v[2][0]; pData[6] = v[2][1]; pData[10] = v[2][2]; pData[14] = 0.0f;
	pData[3] = 0.0f;    pData[7] = 0.0f;    pData[11] = 0.0f;    pData[15] = 1.0f;
}
// FRIENDS

mat3 operator - (const mat3& a)
{ 
	return mat3(-a.v[0], -a.v[1], -a.v[2]); 
}

mat3 operator + (const mat3& a, const mat3& b)
{ 
	return mat3(a.v[0] + b.v[0], a.v[1] + b.v[1], a.v[2] + b.v[2]); 
}

mat3 operator - (const mat3& a, const mat3& b)
{ 
	return mat3(a.v[0] - b.v[0], a.v[1] - b.v[1], a.v[2] - b.v[2]); 
}

mat3 operator * (const mat3& a, const mat3& b) 
{
#define ROWCOL(i, j) \
	a.v[i].n[0]*b.v[0][j] + a.v[i].n[1]*b.v[1][j] + a.v[i].n[2]*b.v[2][j]
	return mat3(vec3(ROWCOL(0,0), ROWCOL(0,1), ROWCOL(0,2)),
		vec3(ROWCOL(1,0), ROWCOL(1,1), ROWCOL(1,2)),
		vec3(ROWCOL(2,0), ROWCOL(2,1), ROWCOL(2,2)));
#undef ROWCOL // (i, j)
}

mat3 operator * (const mat3& a, const double d)
{ 
	return mat3(a.v[0] * d, a.v[1] * d, a.v[2] * d); 
}

mat3 operator * (const double d, const mat3& a)
{ 
	return a*d; 
}

mat3 operator / (const mat3& a, const double d)
{ 
	return mat3(a.v[0] / d, a.v[1] / d, a.v[2] / d); 
}

int operator == (const mat3& a, const mat3& b)
{ 
	return (a.v[0] == b.v[0]) && (a.v[1] == b.v[1]) && (a.v[2] == b.v[2]); 
}

int operator != (const mat3& a, const mat3& b)
{ 
	return !(a == b); 
}

void Swap(mat3& a, mat3& b)
{ 
	mat3 tmp(a); a = b; b = tmp; 
}

std::istream& operator >> (std::istream& s, mat3& v)
{
	double value;
	for (unsigned int i = 0; i < 3; i++)
		for (unsigned int j = 0; j < 3; j++)
		{
			s >> value;
			v[i][j] = value;
		}
	return s;
}

std::ostream& operator << (std::ostream& s, const mat3& v)
{
	for (unsigned int i = 0; i < 3; i++)
	{
		for (unsigned int j = 0; j < 2; j++)
		{
			s << (float) v[i][j] << " ";
		}
		s << (float) v[i][2] << std::endl;
	}
	return s;
}

/****************************************************************
*																*
*		    mat4 member functions								*
*																*
****************************************************************/

// CONSTRUCTORS

mat4::mat4() 
{
}

mat4::mat4(const vec4& v0, const vec4& v1, const vec4& v2, const vec4& v3)
{ 
	v[0] = v0; v[1] = v1; v[2] = v2; v[3] = v3; 
}

mat4::mat4(const double d)
{ 
	v[0] = v[1] = v[2] = v[3] = vec4(d); 
}

mat4::mat4(const mat4& m)
{ 
	v[0] = m.v[0]; v[1] = m.v[1]; v[2] = m.v[2]; v[3] = m.v[3]; 
}

mat4::mat4(const double* d)
{
	v[0] = vec4(d[0], d[4], d[8], d[12]);
	v[1] = vec4(d[1], d[5], d[9], d[13]);
	v[2] = vec4(d[2], d[6], d[10], d[14]);
	v[3] = vec4(d[3], d[7], d[11], d[15]);
}

mat4::mat4(const mat3& m)
{
	v[0] = vec4(m[0], 0.0f);
	v[1] = vec4(m[1], 0.0f);
	v[2] = vec4(m[2], 0.0f);
	v[3] = vec4(0.0f, 0.0f, 0.0f, 1.0f);
}

mat4::mat4(const mat3& m, const vec3& t)
{
	v[0] = vec4(m[0], t[0]);
	v[1] = vec4(m[1], t[1]);
	v[2] = vec4(m[2], t[2]);
	v[3] = vec4(0.0f, 0.0f, 0.0f, 1.0f);	
}

// Static functions
mat4 mat4::Translation3D(const vec3& v)
{
	return mat4(vec4(1.0f, 0.0f, 0.0f, v[VX]),
		vec4(0.0f, 1.0f, 0.0f, v[VY]),
		vec4(0.0f, 0.0f, 1.0f, v[VZ]),
		vec4(0.0f, 0.0f, 0.0f, 1.0f));
}

mat4 mat4::Rotation3DDeg(const vec3& axis, const double angleDeg)
{
	double angleRad = angleDeg * Deg2Rad;
	return Rotation3DRad(axis, angleRad);
}

mat4 mat4::Rotation3DRad(const vec3& axis, const double angleRad)
{
	double  c = cos(angleRad),
		s = sin(angleRad),
		t = 1.0f - c;
	vec3 Axis = axis;
	Axis.Normalize();
	return mat4(vec4(t * Axis[VX] * Axis[VX] + c,
		t * Axis[VX] * Axis[VY] - s * Axis[VZ],
		t * Axis[VX] * Axis[VZ] + s * Axis[VY],
		0.0f),
		vec4(t * Axis[VX] * Axis[VY] + s * Axis[VZ],
		t * Axis[VY] * Axis[VY] + c,
		t * Axis[VY] * Axis[VZ] - s * Axis[VX],
		0.0f),
		vec4(t * Axis[VX] * Axis[VZ] - s * Axis[VY],
		t * Axis[VY] * Axis[VZ] + s * Axis[VX],
		t * Axis[VZ] * Axis[VZ] + c,
		0.0f),
		vec4(0.0f, 0.0f, 0.0f, 1.0f));
}

mat4 mat4::Scaling3D(const vec3& scaleVector)
{
	return mat4(vec4(scaleVector[VX], 0.0f, 0.0f, 0.0f),
		vec4(0.0f, scaleVector[VY], 0.0f, 0.0f),
		vec4(0.0f, 0.0f, scaleVector[VZ], 0.0f),
		vec4(0.0f, 0.0f, 0.0f, 1.0f));
}

mat4 mat4::Perspective3D(const double d)
{
	return mat4(vec4(1.0f, 0.0f, 0.0f, 0.0f),
		vec4(0.0f, 1.0f, 0.0f, 0.0f),
		vec4(0.0f, 0.0f, 1.0f, 0.0f),
		vec4(0.0f, 0.0f, 1.0f/d, 0.0f));
}

// ASSIGNMENT OPERATORS

mat4& mat4::operator = ( const mat4& m )
{ 
	v[0] = m.v[0]; v[1] = m.v[1]; v[2] = m.v[2]; v[3] = m.v[3];
	return *this; 
}

mat4& mat4::operator += ( const mat4& m )
{ 
	v[0] += m.v[0]; v[1] += m.v[1]; v[2] += m.v[2]; v[3] += m.v[3];
	return *this; 
}

mat4& mat4::operator -= ( const mat4& m )
{ 
	v[0] -= m.v[0]; v[1] -= m.v[1]; v[2] -= m.v[2]; v[3] -= m.v[3];
	return *this; 
}

mat4& mat4::operator *= ( const double d )
{ 
	v[0] *= d; v[1] *= d; v[2] *= d; v[3] *= d; 
	return *this; 
}

mat4& mat4::operator /= ( const double d )
{ 
	v[0] /= d; v[1] /= d; v[2] /= d; v[3] /= d; 
	return *this; 
}

vec4& mat4::operator [] ( int i) 
{
	assert(! (i < VX || i > VW));
	return v[i];
}

const vec4& mat4::operator [] ( int i) const 
{
	assert(! (i < VX || i > VW));
	return v[i];
}

// SPECIAL FUNCTIONS;

mat4 mat4::Transpose() const
{
	return mat4(vec4(v[0][0], v[1][0], v[2][0], v[3][0]),
		vec4(v[0][1], v[1][1], v[2][1], v[3][1]),
		vec4(v[0][2], v[1][2], v[2][2], v[3][2]),
		vec4(v[0][3], v[1][3], v[2][3], v[3][3]));
}

void mat4::WriteToGLMatrix(double* m)
{
	m[0] = v[0][0]; m[4] = v[0][1]; m[8] = v[0][2];  m[12] = v[0][3];
	m[1] = v[1][0]; m[5] = v[1][1]; m[9] = v[1][2];  m[13] = v[1][3];
	m[2] = v[2][0]; m[6] = v[2][1]; m[10] = v[2][2]; m[14] = v[2][3];
	m[3] = v[3][0]; m[7] = v[3][1]; m[11] = v[3][2]; m[15] = v[3][3];
}

void mat4::ReadFromGLMatrix(double* m)
{
	v[0][0] = m[0]; v[0][1] = m[4]; v[0][2] = m[8];  v[0][3] = m[12];
	v[1][0] = m[1]; v[1][1] = m[5]; v[1][2] = m[9];  v[1][3] = m[13];
	v[2][0] = m[2]; v[2][1] = m[6]; v[2][2] = m[10]; v[2][3] = m[14];
	v[3][0] = m[3]; v[3][1] = m[7]; v[3][2] = m[11]; v[3][3] = m[15];
}

void mat4::WriteToGLMatrix(float* m)
{
	m[0] = v[0][0]; m[4] = v[0][1]; m[8] = v[0][2];  m[12] = 0.0f;
	m[1] = v[1][0]; m[5] = v[1][1]; m[9] = v[1][2];  m[13] = 0.0f;
	m[2] = v[2][0]; m[6] = v[2][1]; m[10] = v[2][2]; m[14] = 0.0f;
	m[3] = 0.0f;    m[7] = 0.0f;    m[11] = 0.0f;    m[15] = 1.0f;
}

void mat4::ReadFromGLMatrix(float* m)
{
	v[0][0] = m[0]; v[0][1] = m[4]; v[0][2] = m[8];
	v[1][0] = m[1]; v[1][1] = m[5]; v[1][2] = m[9];
	v[2][0] = m[2]; v[2][1] = m[6]; v[2][2] = m[10];
}

// FRIENDS

mat4 operator - (const mat4& a)
{ 
	return mat4(-a.v[0], -a.v[1], -a.v[2], -a.v[3]); 
}

mat4 operator + (const mat4& a, const mat4& b)
{ 
	return mat4(a.v[0] + b.v[0], a.v[1] + b.v[1], a.v[2] + b.v[2], a.v[3] + b.v[3]);
}

mat4 operator - (const mat4& a, const mat4& b)
{ 
	return mat4(a.v[0] - b.v[0], a.v[1] - b.v[1], a.v[2] - b.v[2], a.v[3] - b.v[3]); 
}

mat4 operator * (const mat4& a, const mat4& b) 
{
#define ROWCOL(i, j) a.v[i].n[0]*b.v[0][j] + a.v[i].n[1]*b.v[1][j] + \
	a.v[i].n[2]*b.v[2][j] + a.v[i].n[3]*b.v[3][j]
	return mat4(
		vec4(ROWCOL(0,0), ROWCOL(0,1), ROWCOL(0,2), ROWCOL(0,3)),
		vec4(ROWCOL(1,0), ROWCOL(1,1), ROWCOL(1,2), ROWCOL(1,3)),
		vec4(ROWCOL(2,0), ROWCOL(2,1), ROWCOL(2,2), ROWCOL(2,3)),
		vec4(ROWCOL(3,0), ROWCOL(3,1), ROWCOL(3,2), ROWCOL(3,3))
		);
#undef ROWCOL
}

mat4 operator * (const mat4& a, const double d)
{ 
	return mat4(a.v[0] * d, a.v[1] * d, a.v[2] * d, a.v[3] * d); 
}

mat4 operator * (const double d, const mat4& a)
{ 
	return a*d; 
}

mat4 operator / (const mat4& a, const double d)
{ 
	return mat4(a.v[0] / d, a.v[1] / d, a.v[2] / d, a.v[3] / d); 
}

int operator == (const mat4& a, const mat4& b)
{ 
	return ((a.v[0] == b.v[0]) && (a.v[1] == b.v[1]) && (a.v[2] == b.v[2]) && (a.v[3] == b.v[3])); 
}

int operator != (const mat4& a, const mat4& b)
{ 
	return !(a == b); 
}

void Swap(mat4& a, mat4& b)
{ 
	mat4 tmp(a); a = b; b = tmp; 
}

mat4 mat4::Inverse()	const    // Gauss-Jordan elimination with partial pivoting
{
	mat4 a(*this),	    // As a evolves from original mat into identity
		b(identity4D);   // b evolves from identity into inverse(a)
	int i, j, i1;

	// Loop over cols of a from left to right, eliminating above and below diag
	for (j=0; j<4; j++) {   // Find largest pivot in column j among rows j..3
		i1 = j;		    // Row with largest pivot candidate
		for (i=j+1; i<4; i++)
			if (fabs(a.v[i].n[j]) > fabs(a.v[i1].n[j]))
				i1 = i;

		// Swap rows i1 and j in a and b to put pivot on diagonal
		Swap(a.v[i1], a.v[j]);
		Swap(b.v[i1], b.v[j]);

		// Scale row j to have a unit diagonal
		if (a.v[j].n[j]==0.)
			ALGEBRA_ERROR("mat4::inverse: singular matrix; can't invert\n");
		b.v[j] /= a.v[j].n[j];
		a.v[j] /= a.v[j].n[j];

		// Eliminate off-diagonal elems in col j of a, doing identical ops to b
		for (i=0; i<4; i++)
			if (i!=j) {
				b.v[i] -= a.v[i].n[j]*b.v[j];
				a.v[i] -= a.v[i].n[j]*a.v[j];
			}
	}
	return b;
}

std::istream& operator >> (std::istream& s, mat4& v)
{
	double value;
	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 0; j < 4; j++)
		{
			s >> value;
			v[i][j] = value;
		}
	}
	return s;
}

std::ostream& operator << (std::ostream& s, const mat4& v)
{
	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 0; j < 3; j++)
		{
			s << (float) v[i][j] << " ";
		}
		s << (float) v[i][3] << std::endl;
	}
	return s;
}

/****************************************************************
*																*
*		    Quaternion member functions							*
*																*
****************************************************************/

// CONSTRUCTORS

Quaternion::Quaternion()
{
    n[VW] = 0; n[VX] = 0; n[VY] = 0; n[VZ] = 0;
}

Quaternion::Quaternion(const double w, const double x, const double y, const double z)
{
	n[VW] = w; n[VX] = x; n[VY] = y; n[VZ] = z;
}

Quaternion::Quaternion(const Quaternion& q)
{
	n[VW] = q.n[VW]; n[VX] = q.n[VX]; n[VY] = q.n[VY]; n[VZ] = q.n[VZ];
}

Quaternion::Quaternion(const vec4& v)
{
	n[VW] = v[VW]; n[VX] = v[VX]; n[VY] = v[VY]; n[VZ] = v[VZ];
}

// Static functions

double Quaternion::Dot(const Quaternion& q0, const Quaternion& q1)
{
	return q0.n[VW] * q1.n[VW] + q0.n[VX] * q1.n[VX] + q0.n[VY] * q1.n[VY] + q0.n[VZ] * q1.n[VZ];
}

Quaternion Quaternion::UnitInverse(const Quaternion& q)
{
	return Quaternion(q.n[VW], -q.n[VX], -q.n[VY], -q.n[VZ]);
}

double Quaternion::CounterWarp(double t, double fCos)
{
	const double ATTENUATION = 0.82279687f;
	const double WORST_CASE_SLOPE = 0.58549219f;

	double fFactor = 1.0f - ATTENUATION * fCos;
	fFactor *= fFactor;
	double fK = WORST_CASE_SLOPE * fFactor;

	return t * (fK * t * (2.0f * t - 3.0f) + 1.0f + fK);
}

static const double ISQRT_NEIGHBORHOOD = 0.959066f;
static const double ISQRT_SCALE = 1.000311f;
static const double ISQRT_ADDITIVE_CONSTANT = ISQRT_SCALE / (double)sqrt(ISQRT_NEIGHBORHOOD);
static const double ISQRT_FACTOR = ISQRT_SCALE * (-0.5f / (ISQRT_NEIGHBORHOOD * (double)sqrt(ISQRT_NEIGHBORHOOD)));
double Quaternion::ISqrt_approx_in_neighborhood(double s)
{
	return ISQRT_ADDITIVE_CONSTANT + (s - ISQRT_NEIGHBORHOOD) * ISQRT_FACTOR;	
}

// Assignment operators

Quaternion& Quaternion::operator = (const Quaternion& q)
{
	n[VW] = q.n[VW]; n[VX] = q.n[VX]; n[VY] = q.n[VY]; n[VZ] = q.n[VZ];
	return *this;
}

Quaternion& Quaternion::operator += (const Quaternion& q)
{
	n[VW] += q.n[VW]; n[VX] += q.n[VX]; n[VY] += q.n[VY]; n[VZ] += q.n[VZ];
	return *this;
}

Quaternion& Quaternion::operator -= (const Quaternion& q)
{
	n[VW] -= q.n[VW]; n[VX] -= q.n[VX]; n[VY] -= q.n[VY]; n[VZ] -= q.n[VZ];
	return *this;
}

Quaternion& Quaternion::operator *= (const Quaternion& q)
{
	*this = Quaternion(n[VW] * q.n[VW] - n[VX] * q.n[VX] - n[VY] * q.n[VY] - n[VZ] * q.n[VZ],
		n[VW] * q.n[VX] + n[VX] * q.n[VW] + n[VY] * q.n[VZ] - n[VZ] * q.n[VY],
		n[VW] * q.n[VY] + n[VY] * q.n[VW] + n[VZ] * q.n[VX] - n[VX] * q.n[VZ],
		n[VW] * q.n[VZ] + n[VZ] * q.n[VW] + n[VX] * q.n[VY] - n[VY] * q.n[VX]);
	return *this;
}

Quaternion& Quaternion::operator *= (const double d)
{
	n[VW] *= d; n[VX] *= d;	n[VY] *= d; n[VZ] *= d;
	return *this;
}

Quaternion& Quaternion::operator /= (const double d)
{
	n[VW] /= d; n[VX] /= d;	n[VY] /= d; n[VZ] /= d;
	return *this;
}

// Indexing
double& Quaternion::operator [](int i)
{
	return n[i];
}

double Quaternion::operator [](int i) const
{
	return n[i];
}

double& Quaternion::W()
{
	return n[VW];
}

double Quaternion::W() const
{
	return n[VW];
}

double& Quaternion::X()
{
	return n[VX];
}

double Quaternion::X() const
{
	return n[VX];
}

double& Quaternion::Y()
{
	return n[VY];
}

double Quaternion::Y() const
{
	return n[VY];
}

double& Quaternion::Z()
{
	return n[VZ];
}

double Quaternion::Z() const
{
	return n[VZ];
}

// Friends

Quaternion operator - (const Quaternion& q)
{
	return Quaternion(-q.n[VW], -q.n[VX], -q.n[VY], -q.n[VZ]); 
}

Quaternion operator + (const Quaternion& q0, const Quaternion& q1)
{
	return Quaternion(q0.n[VW] + q1.n[VW], q0.n[VX] + q1.n[VX], q0.n[VY] + q1.n[VY], q0.n[VZ] + q1.n[VZ]);
}

Quaternion operator - (const Quaternion& q0, const Quaternion& q1)
{
	return Quaternion(q0.n[VW] - q1.n[VW], q0.n[VX] - q1.n[VX], q0.n[VY] - q1.n[VY], q0.n[VZ] - q1.n[VZ]);
}

Quaternion operator * (const Quaternion& q, const double d)
{
	return Quaternion(q.n[VW] * d, q.n[VX] * d, q.n[VY] * d, q.n[VZ] * d);
}

Quaternion operator * (const double d, const Quaternion& q)
{
	return Quaternion(q.n[VW] * d, q.n[VX] * d, q.n[VY] * d, q.n[VZ] * d);
}

Quaternion operator * (const Quaternion& q0, const Quaternion& q1)
{
	return Quaternion(q0.n[VW] * q1.n[VW] - q0.n[VX] * q1.n[VX] - q0.n[VY] * q1.n[VY] - q0.n[VZ] * q1.n[VZ],
		q0.n[VW] * q1.n[VX] + q0.n[VX] * q1.n[VW] + q0.n[VY] * q1.n[VZ] - q0.n[VZ] * q1.n[VY],
		q0.n[VW] * q1.n[VY] + q0.n[VY] * q1.n[VW] + q0.n[VZ] * q1.n[VX] - q0.n[VX] * q1.n[VZ],
		q0.n[VW] * q1.n[VZ] + q0.n[VZ] * q1.n[VW] + q0.n[VX] * q1.n[VY] - q0.n[VY] * q1.n[VX]);
}

Quaternion operator / (const Quaternion& q, const double d)
{
	return Quaternion(q.n[VW] / d, q.n[VX] / d, q.n[VY] / d, q.n[VZ] / d);
}

bool operator == (const Quaternion& q0, const Quaternion& q1)
{
	return (q0.n[VW] == q1.n[VW]) && (q0.n[VX] == q1.n[VX]) && (q0.n[VY] == q1.n[VY]) && (q0.n[VZ] == q1.n[VZ]);
}

bool operator != (const Quaternion& q0, const Quaternion& q1)
{
	return !(q0 == q1); 
}

// special functions

double Quaternion::SqrLength() const
{
	return n[VW] * n[VW] + n[VX] * n[VX] + n[VY] * n[VY] + n[VZ] * n[VZ];
}

double Quaternion::Length() const
{
	double l = SqrLength();
	if (l > EPSILON)
		return sqrt(SqrLength());
	else 
		return 0;
}

Quaternion& Quaternion::Normalize()
{
	double l = Length();
	if (l < EPSILON || abs(l) > 1e6)
	{
		FromAxisAngle(vec3(0.0f, 1.0f, 0.0f), 0.0f);
	}else
	{
		*this /= l;
	}

	return *this; 
}

Quaternion& Quaternion::FastNormalize() 
{
	double s = n[VW] * n[VW] + n[VX] * n[VX] + n[VY] * n[VY] + n[VZ] * n[VZ]; // length^2
	double k = ISqrt_approx_in_neighborhood(s);

	if (s <= 0.91521198) {
		k *= ISqrt_approx_in_neighborhood(k * k * s);

		if (s <= 0.65211970) {
			k *= ISqrt_approx_in_neighborhood(k * k * s);
		}
	}

	n[VW] *= k;
	n[VX] *= k;
	n[VY] *= k;
	n[VZ] *= k;

	return * this;
}

Quaternion Quaternion::Conjugate() const
{
	return Quaternion(n[VW], -n[VX], -n[VY], -n[VZ]);
}

Quaternion Quaternion::Inverse() const
{
	return Conjugate() / SqrLength();
}

Quaternion Quaternion::Exp(const Quaternion& q)
{
	// q = A*(x*i+y*j+z*k) where (x,y,z) is unit length
	// exp(q) = cos(A)+sin(A)*(x*i+y*j+z*k)
	double angle = sqrt(q.n[VX] * q.n[VX] + q.n[VY] * q.n[VY] + q.n[VZ] * q.n[VZ]);
	double sn, cs;
	sn = sin(angle);
	cs = cos(angle);

	// When A is near zero, sin(A)/A is approximately 1.  Use
	// exp(q) = cos(A)+A*(x*i+y*j+z*k)
	double coeff = ( abs(sn) < EPSILON ? 1.0f : sn/angle );

	Quaternion result(cs, coeff * q.n[VX], coeff * q.n[VY], coeff * q.n[VZ]);

	return result;
}

Quaternion Quaternion::Log(const Quaternion& q)
{
	// q = cos(A)+sin(A)*(x*i+y*j+z*k) where (x,y,z) is unit length
	// log(q) = A*(x*i+y*j+z*k)
	
	double angle = acos(q.n[VW]);
	double sn = sin(angle);

	// When A is near zero, A/sin(A) is approximately 1.  Use
	// log(q) = sin(A)*(x*i+y*j+z*k)
	double coeff = ( abs(sn) < EPSILON ? 1.0f : angle/sn );

	return Quaternion(0.0f, coeff * q.n[VX], coeff * q.n[VY], coeff * q.n[VZ]);
}

void Quaternion::Zero()
{
	n[VW] = n[VX] = n[VY] = n[VZ] = 0.0f;
}

#define Q_EST(a, b, c) 0.25*(1 + a + b + c)
#define Q_MAX(a, b, c, d) std::max(a, std::max(b, std::max(c, d))) 
void Quaternion::FromRotation (const mat3& rot)
{
	// Replace the following code with your code
   double Wsqr = Q_EST( rot[0][0],  rot[1][1],  rot[2][2]);
   double Xsqr = Q_EST( rot[0][0], -rot[1][1], -rot[2][2]);
   double Ysqr = Q_EST(-rot[0][0],  rot[1][1], -rot[2][2]);
   double Zsqr = Q_EST(-rot[0][0], -rot[1][1],  rot[2][2]);

   double themax = Q_MAX(Wsqr, Xsqr, Ysqr, Zsqr);
   assert (themax >= 0);
   if (Wsqr == themax)
   {
   	n[VW] = sqrt(Wsqr); 
      n[VX] = 0.25*(rot[2][1] - rot[1][2])/n[VW]; 
      n[VY] = 0.25*(rot[0][2] - rot[2][0])/n[VW]; 
      n[VZ] = 0.25*(rot[1][0] - rot[0][1])/n[VW]; 
   }
   else if (Xsqr == themax)
   {
   	n[VX] = sqrt(Xsqr); 
      n[VW] = 0.25*(rot[2][1] - rot[1][2])/n[VX]; 
      n[VY] = 0.25*(rot[0][1] + rot[1][0])/n[VX]; 
      n[VZ] = 0.25*(rot[0][2] + rot[2][0])/n[VX]; 
   }
   else if (Ysqr == themax)
   {
   	n[VY] = sqrt(Ysqr); 
      n[VW] = 0.25*(rot[0][2] - rot[2][0])/n[VY]; 
      n[VX] = 0.25*(rot[0][1] + rot[1][0])/n[VY]; 
      n[VZ] = 0.25*(rot[1][2] + rot[2][1])/n[VY]; 
   }
   else if (Zsqr == themax)
   {
   	n[VZ] = sqrt(Zsqr); 
      n[VW] = 0.25*(rot[1][0] - rot[0][1])/n[VZ]; 
      n[VX] = 0.25*(rot[0][2] + rot[2][0])/n[VZ]; 
      n[VY] = 0.25*(rot[1][2] + rot[2][1])/n[VZ]; 
   }
   Normalize();
}

Quaternion Quaternion::Slerp(double t, const Quaternion& q0, const Quaternion& q1)
{
	// Replace the following code with your code
   Quaternion qTarget = q1;

   double dot = Dot(q0, q1);
   if (dot < 0)
   {
      qTarget = -q1;
      dot = Dot(q0, qTarget); // Ensure interpolation goes over shortest path
   }

	double angle = acos(dot);
   double sinA = sin(angle);

   Quaternion q;
   if (sinA > 0.000001)
   {
      q = (sin(angle*(1-t))*q0+ sin(angle*t)*qTarget)/sinA;      
   }
   else // angle is 0, we're already where we need to be
   {
      q = q0;
   }

	return q;
}
/* Liming's routine
Quaternion Quaternion::Slerp(double t, const Quaternion& q0, const Quaternion& q1)
{
	// assert:  Dot(p,q) >= 0 (guaranteed in NiRotKey::Interpolate methods)
	// (but not necessarily true when coming from a Squad call)

	// This algorithm is Copyright (c) 2002 Jonathan Blow, from his article 
	// "Hacking Quaternions" in Game Developer Magazine, March 2002.

	double fCos = Dot(q0, q1);
	double fTPrime;
	if (t <= 0.5f) {
		fTPrime = CounterWarp(t, fCos);
	} else {
		fTPrime = 1.0f - CounterWarp(1.0f - t, fCos);
	}

	Quaternion kResult(Lerp(q0.n[VW], q1.n[VW], fTPrime),
		Lerp(q0.n[VX], q1.n[VX], fTPrime),
		Lerp(q0.n[VY], q1.n[VY], fTPrime),
		Lerp(q0.n[VZ], q1.n[VZ], fTPrime));

	kResult.FastNormalize();
	return kResult;
}*/

Quaternion Quaternion::Intermediate (const Quaternion& q0, const Quaternion& q1, const Quaternion& q2)
{
	// assert:  q0, q1, q2 are unit quaternions

	Quaternion inv = UnitInverse(q1);
	Quaternion exp = Exp(-0.25f * (Log(inv * q0) + Log(inv * q2)));
	return q1 * exp;	
}

Quaternion Quaternion::Squad(double t, const Quaternion& q0, const Quaternion& a, const Quaternion& b, const Quaternion& q1)
{
	return Slerp(2.0f * t * (1.0f - t), Slerp(t, q0, q1), Slerp(t, a, b));
}

Quaternion Quaternion::ProjectToAxis(const Quaternion& q, vec3& axis)
{
	axis.Normalize();
	vec3 qv = vec3(q[VX], q[VY], q[VZ]);
	double angle = acos(q.n[VW]);
	double sn = sin(angle);
	vec3 qaxis = qv / sn;
	qaxis.Normalize();
	angle = qaxis * axis;
	double halfTheta;
	if (angle < EPSILON)
	{
		halfTheta = 0.0f;
	}else
	{
		double s = axis * qv;
		double c = q[VW];
		halfTheta = atan2(s, c);
	}	
	double cn = cos(halfTheta);
	sn = sin(halfTheta);
	return Quaternion(cn, sn * axis[VX], sn * axis[VY], sn * axis[VZ]); 
}

// Conversion functions
void Quaternion::ToAxisAngle (vec3& axis, double& angleRad) const
{
	double fLength = Length();

	if ( fLength < EPSILON )
	{
		angleRad = 0;
		axis[VX] = 0;
		axis[VY] = 0;
		axis[VZ] = 0;
	}
	else
	{
		angleRad = 2.0f * acos(n[VW]);
		double invLength = 1.0f / fLength;
		axis[VX] = n[VX] * invLength;
		axis[VY] = n[VY] * invLength;
		axis[VZ] = n[VZ] * invLength;
	}
}

void Quaternion::FromAxisAngle (const vec3& axis, double angleRad)
{
	double fHalfAngle = angleRad * 0.5f;
	double sn = sin(fHalfAngle);
	n[VW] = cos(fHalfAngle);
	n[VX] = axis[VX] * sn;
	n[VY] = axis[VY] * sn;
	n[VZ] = axis[VZ] * sn;
}

void Quaternion::FromAxisXAngle(double angleRad)
{
	double fHalfAngle = angleRad * 0.5f;
	n[VW] = cos(fHalfAngle);
	n[VX] = sin(fHalfAngle);
	n[VY] = n[VZ] = 0.0f;
}

void Quaternion::FromAxisYAngle(double angleRad)
{
	double fHalfAngle = angleRad * 0.5f;
	n[VW] = cos(fHalfAngle);
	n[VY] = sin(fHalfAngle);
	n[VX] = n[VZ] = 0.0f;
}

void Quaternion::FromAxisZAngle(double angleRad)
{
	double fHalfAngle = angleRad * 0.5f;
	n[VW] = cos(fHalfAngle);
	n[VZ] = sin(fHalfAngle);
	n[VX] = n[VY] = 0.0f;
}

mat3 Quaternion::ToRotation () const
{
	// operations (*,+,-) = 24
	double tx  = 2.0f * n[VX];
	double ty  = 2.0f * n[VY];
	double tz  = 2.0f * n[VZ];
	double twx = tx * n[VW];
	double twy = ty * n[VW];
	double twz = tz * n[VW];
	double txx = tx * n[VX];
	double txy = ty * n[VX];
	double txz = tz * n[VX];
	double tyy = ty * n[VY];
	double tyz = tz * n[VY];
	double tzz = tz * n[VZ];

	mat3 m;
	m[0][0] = 1.0f - tyy - tzz;
	m[0][1] = txy - twz;
	m[0][2] = txz + twy;
	m[1][0] = txy + twz;
	m[1][1] = 1.0f - txx - tzz;
	m[1][2] = tyz - twx;
	m[2][0] = txz - twy;
	m[2][1] = tyz + twx;
	m[2][2] = 1.0f - txx - tyy;
	return m;
}

std::istream& operator >> (std::istream& s, Quaternion& v)
{
	double x, y, z, w;
	s >> w >> x >> y >> z;
	v[VX] = x;
	v[VY] = y;
	v[VZ] = z;
	v[VW] = w;
	return s;
}

std::ostream& operator << (std::ostream& s, const Quaternion& v)
{
	s << (float) v[VW] << " " << (float) v[VX] << " " << (float) v[VY] << " " << (float) v[VZ];
	return s;
}

/****************************************************************
*																*
*		    Transform member functions							*
*																*
****************************************************************/

Transform::Transform()
{
	m_translation = axisZero;
	m_rotation = identity3D;	
}

Transform::Transform(const vec3& translation, const mat3& rotation)
{
	m_translation = translation;
	m_rotation = rotation;
}

Transform::Transform(const vec3& translation)
{
	m_translation = translation;
	m_rotation = identity3D;
}

Transform::Transform(const mat3& rotation)
{
	m_translation = axisZero;
	m_rotation = rotation;
}

Transform::Transform(const Transform& transform)
{
	m_translation = transform.m_translation;
	m_rotation = transform.m_rotation;
}

Transform Transform::Inverse() const
{
	Transform tmp;
	tmp.m_rotation = m_rotation.Transpose();
	tmp.m_translation = tmp.m_rotation * m_translation * - 1.0f;
	return tmp;
}

Transform& Transform::operator = (const Transform& source)
{
	m_translation = source.m_translation;
	m_rotation = source.m_rotation;
	return *this;
}

Transform Transform::Lerp(const double fPerc, const Transform& t0, const Transform& t1)
{
	Transform result;
	result.m_translation = t0.m_translation * (1.0f - fPerc) + t1.m_translation * fPerc;

	Quaternion q0, q1, q;
	q0.FromRotation(t0.m_rotation);
	q1.FromRotation(t1.m_rotation);
	double d = Quaternion::Dot(q0, q1);
	if (d < 0)
		q1 = -q1;
	q = Quaternion::Slerp(fPerc, q0, q1);
	result.m_rotation = q.ToRotation();

	return result;
}

void Transform::ToGLMatrix(double* pData)
{
	pData[0] = m_rotation[0][0]; pData[4] = m_rotation[0][1]; pData[8]  = m_rotation[0][2]; pData[12] = m_translation[0];
	pData[1] = m_rotation[1][0]; pData[5] = m_rotation[1][1]; pData[9]  = m_rotation[1][2]; pData[13] = m_translation[1];
	pData[2] = m_rotation[2][0]; pData[6] = m_rotation[2][1]; pData[10] = m_rotation[2][2]; pData[14] = m_translation[2];
	pData[3] = 0.0f;			 pData[7] = 0.0f;			  pData[11] = 0.0f;				pData[15] = 1.0f;
}

