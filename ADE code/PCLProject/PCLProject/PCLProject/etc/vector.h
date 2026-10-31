/**
*
* @file Vector.h
*
* @brief msdk Vector Template Class
*
* @author Jeong KiJong(chungki@armax.co.kr)
*
* @date
*  - 2008-12-25, Jeong KiJong, Create
*
* @see Copyright (C) 2008 ARmax CO., LTD.
*
*/

//etc/vector.h

#pragma once


/// Vector Template Class
template <typename _T>
class Vector
{

public:


public:

	Vector();
	Vector(IN UINT _Size);
	Vector(IN Vector<_T>& _Vec);
	~Vector();

public:

	bool	Define(IN UINT _Size);
	bool	Define(IN Vector<_T>& _Vec);

	bool	IsNull() {return (m_pData=NULL);}
	UINT		GetSize() {return m_Size;}
	_T*		GetData() {return m_pData;}

	_T			Dot(IN Vector<_T>& _opB);
	Vector<_T>	Cross(IN Vector<_T>& _opB);


public:

	Vector<_T>	operator + (IN Vector<_T>& _opB);
	Vector<_T>	operator - (IN Vector<_T>& _opB);
	Vector<_T>	operator +=(IN Vector<_T>& _opB);
	Vector<_T>	operator -=(IN Vector<_T>& _opB);

	Vector<_T> operator * (IN _T _Scalar);
	Vector<_T> operator / (IN _T _Scalar);
	Vector<_T> operator *=(IN _T _Scalar);
	Vector<_T> operator /=(IN _T _Scalar);

	Vector<_T>& operator =(IN Vector<_T>& _opB);

	bool operator == (IN Vector<_T>& _opB);
	bool operator != (IN Vector<_T>& _opB);

	const _T& operator [](IN UINT _Index) const {return m_pData[_Index];}
	_T& operator [](IN UINT _Index) {return m_pData[_Index];}

private:

	bool __Create(IN UINT _Size);
	void __Delete();

private:

	_T*	m_pData;
	UINT	m_Size;
};

#include "Vector.inl"
