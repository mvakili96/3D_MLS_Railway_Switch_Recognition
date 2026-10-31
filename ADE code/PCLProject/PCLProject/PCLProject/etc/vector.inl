
template <typename _T>
Vector<_T>::Vector()
: m_pData(NULL),m_Size(0)
{

}

template <typename _T>
Vector<_T>::~Vector()
{
	__Delete();
}


template <typename _T>
Vector<_T>::Vector(IN UINT _Size)
: m_pData(NULL),m_Size(0)
{
	Define(_Size);
}

template <typename _T>
bool Vector<_T>::Define(IN UINT _Size)
{
	__Delete();
	return __Create(_Size);
}

template <typename _T>
bool Vector<_T>::Define(IN Vector<_T>& _Vec)
{
	__Delete();
	if (__Create(_Vec.GetSize()))
	{
		for (UINT i=0;i<m_Size;i++)
			m_pData[i]=_Vec.m_pData[i];
	}

	return true;
}

template <typename _T>
_T Vector<_T>::Dot(IN Vector<_T>& _opB)
{

	_T ret=0;
	for (UINT i=0;i<m_Size;i++)
	{
		ret+= (m_pData[i]*_opB.m_pData[i]);
	}	
	
	return ret;
}


template <typename _T>
bool Vector<_T>::__Create(IN UINT _Size)
{
	if (_Size==0)
		return false;

	m_pData=new _T[_Size];
	for (UINT i=0;i<_Size;i++)
		m_pData[i]=0;

	m_Size = _Size;

	return true;
}

template <typename _T>
void Vector<_T>::__Delete()
{
	if (m_pData)
		delete []m_pData;

	m_pData=NULL;
	m_Size=0;
}

template <typename _T>
Vector<_T>	Vector<_T>::operator + (IN Vector<_T>& _opB)
{
	Vector<_T>	ret;

	if (GetSize()==_opB.GetSize())
	{
		ret.Define(GetSize());
		for (UINT i=0;i<GetSize();i++)
			ret.m_pData[i]=m_pData[i]+_opB.m_pData[i];
	}

	return ret;
}

template <typename _T>
Vector<_T>	Vector<_T>::operator - (IN Vector<_T>& _opB)
{
	Vector<_T>	ret;

	if (GetSize()==_opB.GetSize())
	{
		ret.Define(GetSize());
		for (UINT i=0;i<GetSize();i++)
			ret.m_pData[i]=m_pData[i]-_opB.m_pData[i];
	}

	return ret;
}

template <typename _T>
Vector<_T>	Vector<_T>::operator += (IN Vector<_T>& _opB)
{
	if (GetSize()==_opB.GetSize())
	{
		for (UINT i=0;i<GetSize();i++)
			m_pData[i]+=_opB.m_pData[i];
	}

	return (*this);
}

template <typename _T>
Vector<_T>	Vector<_T>::operator -= (IN Vector<_T>& _opB)
{
	if (GetSize()==_opB.GetSize())
	{
		for (UINT i=0;i<GetSize();i++)
			m_pData[i]-=_opB.m_pData[i];
	}

	return (*this);
}

template <typename _T>
Vector<_T> Vector<_T>::operator * (IN _T _Scalar)
{
	Vector<_T>	ret;

	if (GetSize()==_opB.GetSize())
	{
		ret.Define(GetSize());
		for (UINT i=0;i<GetSize();i++)
			ret.m_pData[i]=m_pData[i]*_Scalar;
	}

	return ret;	
}

template <typename _T>
Vector<_T> Vector<_T>::operator / (IN _T _Scalar)
{
	Vector<_T>	ret;

	if (GetSize()==_opB.GetSize())
	{
		ret.Define(GetSize());
		for (UINT i=0;i<GetSize();i++)
			ret.m_pData[i]=m_pData[i]/_Scalar;
	}

	return ret;	
}

template <typename _T>
Vector<_T> Vector<_T>::operator *= (IN _T _Scalar)
{

	if (GetSize()==_opB.GetSize())
	{
		for (UINT i=0;i<GetSize();i++)
			m_pData[i]*=_Scalar;
	}

	return (*this);
}

template <typename _T>
Vector<_T> Vector<_T>::operator /= (IN _T _Scalar)
{

	if (GetSize()==_opB.GetSize())
	{
		for (UINT i=0;i<GetSize();i++)
			m_pData[i]/=_Scalar;
	}

	return (*this);
}

template <typename _T>
Vector<_T>& Vector<_T>::operator = (IN Vector<_T>& _opB)
{

	Define(_opB);
	
	if (GetSize()==_opB.GetSize())
	{
		for (UINT i=0;i<m_Size;i++)
			if (m_pData[i]!=_opB.m_pData[i])
				return false;
	}

	return (*this);
}

template <typename _T>
bool Vector<_T>::operator == (IN Vector<_T>& _opB)
{
	bool ret=false;

	if (GetSize()==_opB.GetSize())
	{
		for (UINT i=0;i<m_Size;i++)
			if (m_pData[i]!=_opB.m_pData[i])
				return false;

		ret=true;
	}

	return ret;
}

template <typename _T>
bool Vector<_T>::operator != (IN Vector<_T>& _opB)
{
	bool ret=true;

	if (GetSize()==_opB.GetSize())
	{
		for (UINT i=0;i<m_Size;i++)
			if (m_pData[i]!=_opB.m_pData[i])
				return true;

		ret=false;
	}

	return ret;
}
