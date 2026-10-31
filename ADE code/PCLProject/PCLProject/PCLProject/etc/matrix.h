/**
*
* @file Matrix.h
*
* @brief Matrix Template Class
*
* @author Jeong KiJong(chungki@armax.co.kr)
*
* @date
*  - 2008-12-25, Jeong KiJong, Create
*
* @see Copyright (C) 2008 ARmax CO., LTD.
*
*/


// etc/Matrix.h
#pragma once

#include "vector.h"

/// Matrix Template Class
template <typename _T>
class Matrix
{

public:

	Matrix();
	Matrix(IN Matrix<_T>& _mx);
	Matrix(IN UINT _Rows, IN UINT _Cols);
	Matrix(IN UINT _Rows, IN UINT _Cols, IN const _T& _Val);
	virtual ~Matrix();

public:

	bool Define(IN Matrix<_T>& _mx);
	void Define(IN UINT _Rows, IN UINT _Cols);
	void Resize(IN UINT _Rows, IN UINT _Cols) {Define(_Rows, _Cols);}
	void Resize(IN UINT _Rows, IN UINT _Cols, IN const _T& _Val);

	bool Identity();

	Matrix<_T> I() {return Inverse();}
	Matrix<_T> T() {return Transpose();}

	Matrix<_T> Inverse();
	Matrix<_T> Transpose();

	_T	Det();

	_T	Sum();
	_T	SumofCol(IN UINT _Col);
	_T	SumofRow(IN UINT _Row);
	
	_T  Norm();
	_T  NormofCol(IN UINT _Col);
	_T  NormofRow(IN UINT _Row);

	_T	Max();
	_T	Min();

	bool GetRow(IN UINT _Index,OUT Vector<_T>& _RowV);
	bool GetColumn(IN UINT _Index,OUT Vector<_T>& _ColV);

	_T	GetAt(IN UINT _Row, IN UINT _Col) {return m_pData[_Row*m_Cols+_Col];}
	void SetAt(IN UINT _Row, IN UINT _Col, IN _T _Val) {m_pData[_Row*m_Cols+_Col]=_Val;}

	bool LUDecompose(OUT Matrix<_T>& _LUd,OUT Vector<_T>& _Idx, OUT _T& _D);
	void LUbacksb(OUT Vector<_T>& _idx, OUT Vector<_T>& b);

	static Matrix<_T>* ZeroMatrix(IN UINT _Row, IN UINT _Col);
	static Matrix<_T>* UnitMatrix(IN UINT _Row, IN UINT _Col);

	bool SwapColumn(IN UINT _Col1, IN UINT _Col2);
	bool SwapRow(IN UINT _Row1, IN UINT _Row2);


	
public:

	_T*		GetData() {return m_pData;}
	UINT		GetRows() {return m_Rows;}
	UINT		GetCols() {return m_Cols;}

	bool	IsSquare() {return (m_Cols==m_Rows);}
	bool	IsNull() {return (m_pData==NULL);}
	bool	IsValid() {return (m_isValid);}

public:

	Matrix<_T>	operator + (IN Matrix<_T>& _mx);
	Matrix<_T>	operator - (IN Matrix<_T>& _mx);
	Matrix<_T>	operator * (IN const Matrix<_T>& _mx);
	Matrix<_T>	operator += (IN Matrix<_T>& _mx);
	Matrix<_T>	operator -= (IN Matrix<_T>& _mx);
	Matrix<_T>	operator *= (IN Matrix<_T>& _mx);

	Matrix<_T>	operator * (IN _T _Scalar);
	Matrix<_T>	operator / (IN _T _Scalar);
	Matrix<_T>	operator *= (IN _T _Scalar);
	Matrix<_T>	operator /= (IN _T _Scalar);

	bool operator == (IN Matrix<_T>& _mx);
	bool operator != (IN Matrix<_T>& _mx);

	Matrix<_T>& operator = (IN Matrix<_T>& _mx);

	_T*		operator [](IN UINT _Row);
	const _T*		operator [](IN UINT _Row) const;

	_T&		operator ()(IN UINT _Row,IN UINT _Col);
	const _T&	operator ()(IN UINT _Row) const;

private:

	bool __Create(IN UINT _Rows, IN UINT _Cols);
	void __Delete();

private:

	UINT		m_Rows;
	UINT		m_Cols;
	_T*		m_pData;

	bool	m_isValid;
	
};


#include "Matrix.inl"
