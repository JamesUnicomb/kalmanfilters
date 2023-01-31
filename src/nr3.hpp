#ifndef _NR3_HPP_
#define _NR3_HPP_

// all the system #include's we'll ever need
#include <cmath>

// macro-like inline functions

template <class T>
inline T SQR(const T a)
{
	return a * a;
}

template <class T>
inline const T& MAX(const T& a, const T& b)
{
	return b > a ? (b) : (a);
}

inline float MAX(const double& a, const float& b)
{
	return b > a ? (b) : float(a);
}

inline float MAX(const float& a, const double& b)
{
	return b > a ? float(b) : (a);
}

template <class T>
inline const T& MIN(const T& a, const T& b)
{
	return b < a ? (b) : (a);
}

inline float MIN(const double& a, const float& b)
{
	return b < a ? (b) : float(a);
}

inline float MIN(const float& a, const double& b)
{
	return b < a ? float(b) : (a);
}

template <class T>
inline T SIGN(const T& a, const T& b)
{
	return b >= 0 ? (a >= 0 ? a : -a) : (a >= 0 ? -a : a);
}

inline float SIGN(const float& a, const double& b)
{
	return b >= 0 ? (a >= 0 ? a : -a) : (a >= 0 ? -a : a);
}

inline float SIGN(const double& a, const float& b)
{
	return (float)(b >= 0 ? (a >= 0 ? a : -a) : (a >= 0 ? -a : a));
}

template <class T>
inline void SWAP(T& a, T& b)
{
	T dum = a;
	a = b;
	b = dum;
}

#endif /* _NR3_HPP_ */