
template <typename _T>
Matrix<_T>::Matrix()
: m_pData(NULL),m_Cols(0), m_Rows(0),m_isValid(false)
{
}

template <typename _T>
Matrix<_T>::Matrix(IN Matrix<_T>& _mx)
: m_pData(NULL),m_Cols(0), m_Rows(0),m_isValid(true)
{
	Define(_mx);
}

//************************************
// Method:    Matrix
// FullName:  Matrix<_T>::Matrix
// Access:    public 
// Returns:   
// Qualifier: : m_pData(NULL),m_Cols(0), m_Rows(0) ,m_isValid(true)
// Parameter: IN UINT _Rows
// Parameter: IN UINT _Cols
//************************************
template <typename _T>
Matrix<_T>::Matrix(IN UINT _Rows, IN UINT _Cols) 
: m_pData(NULL),m_Cols(0), m_Rows(0) ,m_isValid(true)
{
	Define(_Rows,_Cols);
}

template <typename _T>
Matrix<_T>::Matrix(IN UINT _Rows, IN UINT _Cols, IN const _T& _Val) 
: m_pData(NULL),m_Cols(0), m_Rows(0) ,m_isValid(true)
{
	Define(_Rows,_Cols);
	for(UINT i=0; i<_Rows; i++) {
		for(UINT j=0; j<_Cols; j++)
			m_pData[j*m_Rows+i] = _Val;
	}
}

template <typename _T>
Matrix<_T>::~Matrix()
{ 
	__Delete();
}

template <typename _T>
void Matrix<_T>::Resize(UINT _Rows, UINT _Cols,  IN const _T& _Val)
{
	Define(_Rows,_Cols);
	for(UINT i=0; i<_Rows; i++) {
		for(UINT j=0; j<_Cols; j++)
			m_pData[j*m_Rows+i] = _Val;
	}
}


template <typename _T>
bool Matrix<_T>::Identity()
{
	bool ret=false;
	if (m_Rows==m_Cols && m_pData)
	{
		for (UINT j=0;j<m_Rows;j++)
			for (UINT i=0;i<m_Cols;i++)
				m_pData[j*m_Rows+i]=(i==j)?1:0;

		ret=true;
	}

	return ret;
}

template <typename _T>
bool Matrix<_T>::Define(IN Matrix<_T>& _mx)
{
	bool ret=true;

	Define(_mx.GetRows(),_mx.GetCols());
	{
		for (UINT i=0;i<_mx.GetRows()*_mx.GetCols();i++)
			m_pData[i]=_mx.m_pData[i];

		ret=true;
	}

	return ret;
}

template <typename _T>
void Matrix<_T>::Define(IN UINT _Rows, IN UINT _Cols)
{
	__Delete();	
	__Create(_Rows, _Cols);
}

template <typename _T>
bool Matrix<_T>::__Create(IN UINT _Rows, IN UINT _Cols)
{
	bool ret=false;

	if (_Rows>0 && _Cols>0)
	{
		m_Rows = _Rows;
		m_Cols = _Cols;

		m_pData=new _T[_Rows*_Cols];
		for (UINT i=0;i<_Rows*_Cols;i++)
			m_pData[i]=0;

		ret=true;
		m_isValid=true;
	}

	return ret;
}

template <typename _T>
void Matrix<_T>::__Delete()
{
	if (m_pData)
		delete []m_pData;

	m_pData=NULL;
	m_Rows=0;
	m_Cols=0;
}

template <typename _T>
Matrix<_T>	Matrix<_T>::operator + (IN Matrix<_T>& _mx)
{
	Matrix<_T> ret;
	ret.m_isValid=false;

	if (GetRows()==_mx.GetRows() && GetCols()==_mx.GetCols())
	{
		UINT Rows=GetRows();
		UINT Cols=GetCols();

		ret.Define(Rows,Cols);

		for (UINT i=0;i<Rows*Cols;i++)
			ret.m_pData[i]=m_pData[i]+_mx.m_pData[i];

		ret.m_isValid=true;
	}

	return ret;
}

template <typename _T>
Matrix<_T>	Matrix<_T>::operator - (IN Matrix<_T>& _mx)
{
	Matrix<_T> ret;
	ret.m_isValid=false;

	if (GetRows()==_mx.GetRows() && GetCols()==_mx.GetCols())
	{
		UINT Rows=GetRows();
		UINT Cols=GetCols();

		ret.Define(Rows,Cols);

		for (UINT i=0;i<Rows*Cols;i++)
			ret.m_pData[i]=m_pData[i]-_mx.m_pData[i];
		ret.m_isValid=true;
	}

	return ret;
}

template <typename _T>
Matrix<_T> Matrix<_T>::operator * (IN const Matrix<_T>& _mx)
{
	UINT Row1 = m_Rows;
	UINT	Col1 = m_Cols;
	UINT Row2 = _mx.m_Rows;
	UINT Col2 = _mx.m_Cols;

	//	Error("Inconsistent matrix sizes in multiplication!\n");
	Matrix<_T> ret(Row1, Col2);
	ret.m_isValid=false;
	if (Col1!=Row2) 
		return ret;

	ret.m_isValid=true;
	for(UINT j = 0 ; j < Row1 ; j++ )
		for(UINT  i = 0 ; i < Col2 ; i++ )
			for(UINT k = 0 ; k < Row2 ; k++ )
				ret.m_pData[j*Col2+i] += 
					m_pData[j*Col1+k] * _mx.m_pData[k*Col2+i];

	return ret;
}

template <typename _T>
Matrix<_T>	Matrix<_T>::operator +=(IN Matrix<_T>& _mx)
{
	if (GetRows()==_mx.GetRows() && GetCols()==_mx.GetCols())
	{
		UINT Rows=GetRows();
		UINT Cols=GetCols();

		for (UINT i=0;i<Rows*Cols;i++)
			m_pData[i]+=_mx.m_pData[i];
	}

	return (*this);
}

template <typename _T>
Matrix<_T> Matrix<_T>::operator *=(IN Matrix<_T>& _mx)
{
	UINT Row1 = m_Rows;
	UINT	Col1 = m_Cols;
	UINT Row2 = _mx.Rows;
	UINT Col2 = _mx.Cols;

	//	Error("Inconsistent matrix sizes in multiplication!\n");
	Matrix<_T> ret(Row1, Col2);
	if (Col1!=Row2) return ret;

	for(UINT j = 0 ; j < Row1 ; j++ )
		for(UINT  i = 0 ; i < Col2 ; i++ )
			for(UINT k = 0 ; k < Row2 ; k++ )
			{
				ret.m_pData[j*Col2+i] += 
				m_pData[j*Col1+k] * _mx.m_pData[k*Col2+i];
			}

	Define(ret);

	return (*this);
}

template <typename _T>
Matrix<_T>	Matrix<_T>::operator -= (IN Matrix<_T>& _mx)
{
	if (GetRows()==_mx.GetRows() && GetCols()==_mx.GetCols())
	{
		UINT Rows=GetRows();
		UINT Cols=GetCols();

		for (UINT i=0;i<Rows*Cols;i++)
			m_pData[i]-=_mx.m_pData[i];
	}

	return ret;
}

template <typename _T>
Matrix<_T>	Matrix<_T>::operator * (IN _T _Scalar)
{
	Matrix<_T> ret(GetRows(),GetCols());

	if (!ret.IsNull())
	{
		UINT Size=ret.GetRows()*ret.GetCols();
		for (UINT i=0;i<Size;i++)
			ret.m_pData[i]=m_pData[i]*_Scalar;
	}
	return ret;
}

template <typename _T>
Matrix<_T>	Matrix<_T>::operator / (IN _T _Scalar)
{
	Matrix<_T> ret(GetRows(),GetCols());

	if (!ret.IsNull() && _Scalar!=0)
	{
		UINT Size=ret.GetRows()*ret.GetCols();
		for (UINT i=0;i<Size;i++)
			ret.m_pData[i]=m_pData[i]/_Scalar;
	}
	return ret;
}

template <typename _T>
Matrix<_T>	Matrix<_T>::operator *= (IN _T _Scalar)
{
	if (!ret.IsNull())
	{
		UINT Size=GetRows()*GetCols();
		for (UINT i=0;i<Size;i++)
			m_pData[i]*=_Scalar;
	}

	return (*this);
}

template <typename _T>
Matrix<_T>	Matrix<_T>::operator /= (IN _T _Scalar)
{
	if (!ret.IsNull() && _Scalar!=0)
	{
		UINT Size=GetRows()*GetCols();
		for (UINT i=0;i<Size;i++)
			m_pData[i]/=_Scalar;
	}

	return (*this);
}

template <typename _T>
bool Matrix<_T>::operator == (IN Matrix<_T>& _mx)
{
	bool ret=false;

	if (GetRows()==_mx.GetRows() && GetCols()==_mx.GetCols())
	{
		UINT Size=GetRows()*GetCols();
		for (UINT i=0;i<Size;i++)
			if (m_pData[i]!=_mx.m_pData[i])
				return false;

		ret=true;
	}

	return ret;
}

template <typename _T>
bool Matrix<_T>::operator != (IN Matrix<_T>& _mx)
{
	bool ret=true;

	if (GetRows()==_mx.GetRows() && GetCols()==_mx.GetCols())
	{
		UINT Size=GetRows()*GetCols();
		for (UINT i=0;i<Size;i++)
			if (m_pData[i]!=_mx.m_pData[i])
				return true;

		ret=false;
	}

	return ret;
}

template <typename _T>
Matrix<_T>& Matrix<_T>::operator = (IN Matrix<_T>& _mx)
{
	if (_mx.GetRows()>0 && _mx.GetCols()>0)
		Define(_mx);

	return (*this);
}

template <typename _T>
_T*	Matrix<_T>::operator [](IN UINT _Row)
{
	return &m_pData[_Row*m_Cols];
}

template <typename _T>
const _T* Matrix<_T>::operator [](IN UINT _Row) const
{
	return &m_pData[_Row*m_Cols];
}

template <typename _T>
_T	Matrix<_T>::Sum()
{
	_T ret=0;

	if (!ret.IsNull())
	{
		UINT Size=m_Rows*m_Cols;
		for (UINT i=0;i<Size;i++)
			ret+=m_pData[i];
	}

	return ret;
}

template <typename _T>
_T	Matrix<_T>::SumofCol(IN UINT _Col)
{
	_T ret=0;

	if (!ret.IsNull())
	{
		for (UINT Row=0;Row<m_Rows;Row++)
			ret+=m_pData[Row*m_Cols+_Col];
	}

	return ret;	
}

template <typename _T>
_T	Matrix<_T>::SumofRow(IN UINT _Row)
{
	_T ret=0;

	if (!ret.IsNull())
	{
		for (UINT Col=0;Col<m_Cols;Col++)
			ret+=m_pData[_Row*m_Cols+Col];
	}

	return ret;	
}

template <typename _T>
_T	Matrix<_T>::Max()
{
	_T ret=Constant<_T>::MIN();

	if (!ret.IsNull())
	{
		UINT Size=m_Rows*m_Cols;
		for (UINT i=0;i<Size;i++)
			ret=MAX(ret,m_pData[i]);
	}

	return ret;
}

template <typename _T>
_T	Matrix<_T>::Min()
{
	_T ret=Constant<_T>::MAX();

	if (!ret.IsNull())
	{
		UINT Size=m_Rows*m_Cols;
		for (UINT i=0;i<Size;i++)
			ret=MIN(ret,m_pData[i]);
	}

	return ret;
}

/// Euclidean norm of a matrix
template <typename _T>
_T Matrix<_T>::Norm()
{
	_T ret=0;
	_T Val=0;

	for(UINT j = 0 ; j < m_Rows ; j++ )
		for(UINT i = 0 ; i < m_Cols ; i++ )
		{
			Val = m_pData[j*m_Cols+i];
			ret += (Val * Val);
		}

	return sqrt(ret);
}


template <typename _T>
_T  Matrix<_T>::NormofCol(IN UINT _Col)
{
	_T ret=0;
	_T Val=0;

	for(UINT j = 0 ; j < m_Rows ; j++ )
	{
		Val = m_pData[j*m_Cols+_Col];
		ret += (Val * Val);
	}

	return sqrt(ret);
}

template <typename _T>
_T  Matrix<_T>::NormofRow(IN UINT _Row)
{
	_T ret=0;
	_T Val=0;

	for(UINT i = 0 ; i < m_Cols ; i++ )
	{
		Val = m_pData[_Row*m_Cols+i];
		ret += (Val * Val);
	}

	return sqrt(ret);
}

template <typename _T>
_T Matrix<_T>::Det()
{

	Matrix<_T> LU;
	Vector<_T> idx(m_Rows);

	_T   d;
	LUdecomp(cp, LU, idx, d);

	_T ans = 1.0;
	for(UINT i = 0 ; i < m_Rows ; i++ )
		ans *= LU[i][i];

	return ans;

}

template <typename _T>
Matrix<_T> Matrix<_T>::Inverse()
{
	Matrix<_T> ret(m_Rows, m_Cols);
	ret.m_isValid=false;
	
	if (IsSquare())
	{
		Matrix<_T> LU(*this);
		Vector<_T> Idx(GetRows());

		_T   d=0;
		if (!LUDecompose(LU, Idx, d))
			return ret;

		Vector<_T> Col(GetRows());
		UINT n=GetRows();
		for(UINT i = 0 ; i < n ; i++ )
		{
			for(UINT j = 0 ; j < n ; j++ ) 
				Col[j] = 0.0;

			Col[i] = 1.0;
			LU.LUbacksb(Idx, Col);
			for( UINT j = 0 ; j < n ; j++ ) 
				ret[j][i] = Col[j];
		}

		ret.m_isValid = true;
	}

	return ret;
}

template <typename _T>
Matrix<_T> Matrix<_T>::Transpose()
{
	Matrix<_T> ret(GetCols(),GetRows());

	for (UINT Row=0;Row<m_Rows;Row++)
		for (UINT Col=0;Col<m_Cols;Col++)
			ret[Col][Row]=GetAt(Row,Col);

	return ret;
}

template <typename _T>
void Matrix<_T>::LUbacksb(OUT Vector<_T>& _Idx, OUT Vector<_T>& _Back)
{
	long n = (long)m_Rows;

	long ii=0;
	for(long i = 0 ; i < n ; i++ ) 
	{
		long ip = (long)(_Idx[i]+0.5);
		_T Sum = _Back[ip];
		_Back[ip] = _Back[i];
		if( ii != 0)
		{
			for(long j = ii-1 ; j < i ; j++ ) 
				Sum -= (*this)[i][j]*_Back[j];
		}
		else if( Sum != 0.0 ) 
			ii = i+1;
		_Back[i] = Sum;
	}

	for(long i = n-1 ; i >= 0 ; i-- ) 
	{
		_T Sum = _Back[i];
		for(long j = i+1 ; j < n ; j++ ) 
		{
			Sum -= (*this)[i][j]*_Back[j];
		}
		_Back[i] = Sum/(*this)[i][i];
	}
}

template <typename _T>
bool Matrix<_T>::LUDecompose(OUT Matrix<_T>& _LUd,OUT Vector<_T>& _Idx, OUT _T& _D)
{
	if( !IsSquare() )
		return false;

	UINT n = GetRows();
	Vector<_T> vv(n);		// vv stores the implicit scaling of each row
	_D = 1.0;			// no row interchanges yet

	// Look over rows to get the implicit scaling information
	for(UINT Row = 0 ; Row < n ; Row++ ) 
	{
		_T	Big = 0.0;
		for(UINT Col = 0 ; Col < n ; Col++ )
		{
			Big=max( abs(_LUd[Row][Col]), Big);
		}

		if( Big == 0 ) 
		{
			//cerr << "Singular matrix in routine LUdcmp" << endl;
			return false;
		}
		vv[Row] = 1.0 / Big;
	}

	const _T TINY = 1.0e-20;
	UINT jmax=0;
	_T  dum=0;
	// This is the loop over columns of Crout's method
	for(UINT i = 0 ; i < n ; i++ ) 
	{
		UINT j=0;
		for(j = 0 ; j < i ; j++ )
		{
			_T Sum = _LUd[j][i];
			for(UINT k = 0 ; k < j ; k++ ) 
				Sum -= _LUd[j][k]*_LUd[k][i];
			_LUd[j][i] = Sum;
		}

		_T	Big = 0.0;	// Initialize for the search for largest pivot element
		for(j = i ; j < n ; j++ ) 
		{
			_T Sum = _LUd[j][i];
			for(UINT k = 0 ; k < i ; k++ ) 
				Sum -= _LUd[j][k]*_LUd[k][i];
			_LUd[j][i] = Sum;

			// Is the figure of merit for the pivot better than the best so far?
			if( (dum = vv[i]*abs(Sum)) >= Big ) 
			{
				Big = dum;
				jmax = j;
			}
		}

		if( i != jmax ) // Do we need to interchange rows?
		{
			for(UINT k = 0 ; k < n ; k++ ) 
			{
				dum = _LUd[jmax][k];
				_LUd[jmax][k] = _LUd[i][k];
				_LUd[i][k] = dum;
			}
			_D = -_D;
			vv[jmax] = vv[i];
		}
		_Idx[i] = jmax;

		// If the pivot element is zero, the matrix is singular.
		if( _LUd[i][i] == 0.0 ) 
			_LUd[i][i] = TINY;

		if( j != n-1 ) 
		{
			dum = 1.0 / _LUd[i][i];
			for(UINT j = i+1 ; j < n ; j++ ) 
				_LUd[j][i] *= dum;
		}
	}

	return true;
}

template <typename _T>
Matrix<_T>* Matrix<_T>::ZeroMatrix(IN UINT _Row, IN UINT _Col)
{
	Matrix<_T>*	ret=new Matrix<_T>(_Row, _Col);
	
	return ret;
}

template <typename _T>
Matrix<_T>* Matrix<_T>::UnitMatrix(IN UINT _Row, IN UINT _Col)
{
	Matrix<_T>*	ret=new Matrix<_T>(_Row, _Col);
	pRet->Identity();
	return ret;
}

template <typename _T>
bool Matrix<_T>::GetRow(IN UINT _Index,OUT Vector<_T>& _RowV)
{
	bool ret=true;

	_RowV.Define(GetCols());

	for (UINT Col=0;Col<GetCols();Col++);
		_RowV[Col]=GetAt(_Index, Col);

	return ret;
}

template <typename _T>
bool Matrix<_T>::GetColumn(IN UINT _Index,OUT Vector<_T>& _ColV)
{
	bool ret=true;

	_ColV.Define(GetRows());

	for (UINT Row=0;Row<GetRows();Row++);
		_ColV[Row]=GetAt(Row, _Index);

	return ret;
}

template <typename _T>
_T& Matrix<_T>::operator ()(IN UINT _Row,IN UINT _Col)
{
	
	return m_pData[_Row*m_Cols+_Col];
}

template <typename _T>
const _T&	Matrix<_T>::operator ()(IN UINT _Row) const
{
	return m_pData[_Row*m_Cols+_Col];
}

template <typename _T>
bool Matrix<_T>::SwapColumn(IN UINT _Col1, IN UINT _Col2)
{
	return false;
}

template <typename _T>
bool Matrix<_T>::SwapRow(IN UINT _Row1, IN UINT _Row2)
{
	return false;
}

