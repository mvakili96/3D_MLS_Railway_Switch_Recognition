#include <vector>
using namespace std;
template <class T>
class C2DVector {
public:
	C2DVector():m_dimRow(0), m_dimCol(0) {}
	C2DVector(unsigned int nRow, unsigned int nCol)
	{
		m_dimRow = nRow;
		m_dimCol = nCol;
	
		for(int i=0; i<nRow; i++)
		{
			vector<T> x(nCol);
			size_t y = x.size();
			m_2DVector.push_back(x);
		}
	}

	void SetAt(unsigned int nRow, unsigned int nCol, const T& value)
	{
		if(nRow >= m_dimRow || nCol >= m_dimCol)
			throw out_of_range("Array out of bound");
		else
			m_2DVector[nRow][nCol] = value;
	}

	T GetAt(int nRow, int nCol)
	{
		if(nRow >= m_dimRow || nCol >= m_dimCol)
			throw out_of_range("Array out of bound");
		else
			return m_2DVector[nRow][nCol];
	}

	void GrowRow(unsigned int newSize)
	{
		if(newSize <= m_dimRow)
			return;
		m_dimRow = newSize;
		for(unsigned int i=0; i<newSize-m_dimCol; i++)
		{
			vector<T> x(m_dimRow);
			m_2DVector.push_back(x);
		}
	}

	void GrowCol(unsigned int newSize)
	{
		if(newSize <= m_dimCol)
			return;
		m_dimCol = newSize;
		for(int i=0; i<m_dimRow; i++)
		{
			m_2DVector[i].resize(newSize);
		}
	}

	vector<T>& operator[](unsigned int x)
	{
		return m_2DVector[x];
	}

	const unsigned int GetRowSize() const
	{
		return m_dimRow;
	}

	const unsigned int GetColSize() const
	{
		return m_dimCol;
	}

	void ResetData()
	{
		for(int i=0; i<m_dimRow; i++)
		{
			m_2DVector[i].clear();
		}
		m_2DVector.clear();
	}

private:
	vector<vector<T>> m_2DVector;
	unsigned int m_dimRow;
	unsigned int m_dimCol;
};