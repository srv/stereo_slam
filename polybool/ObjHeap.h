//	ObjHeap.h - general purpose object storage class with iteration
//
//	This file is a part of PolyBoolean software library
//	(C) 1998-1999 Michael Leonov
//	Consult your license regarding permissions and restrictions
//

#ifndef _OBJHEAP_H_
#define _OBJHEAP_H_

//	Example of use:
//		typedef ObjStorageClass<ELEM> HEAP_TYPE;
//
//		HEAP_TYPE heap; 	// construct an empty heap
//		ELEM * node;
//		node = heap.Get();	// get space from heap
//		node = heap.Get(true); // get space from heap and set it to zero
//		node = heap.Get(ELEM(153));	// get space from heap and call copy constructor
//		heap.Put(node); 	// put back to heap (node != NULL) & call destructor
//		heap.Clr(); 		// heap cleanup (done automatically in destructor)
//
//		Example of iteration:
//	for (HEAP_TYPE::iterator iter = heap.begin(); iter != heap.end(); ++iter)
//	{ ELEM & node = iter; [ node.m... <=> iter->m_... ] }
//

template <class T, unsigned int nBlockElements = 16, int ErrorCode = err_no_memory>
class ObjStorageClass
{
	struct ELEM
	{
		char	data[sizeof(T)];
		ELEM *	next;
	};

	// value for constructed elements
	static ELEM * Busy() {
		return reinterpret_cast<ELEM *>(-1);
	}

	struct BLK
	{
		ELEM	contents[nBlockElements];
		BLK *	next;
	};

	BLK *	m_Blocks;
	ELEM *	m_FreeList;

	void * GetFree(bool bClear)
	{
		if (m_FreeList == NULL)
		{	// allocate new block of nBlockElements
			BLK * blk = new BLK;
			if (blk == NULL)
				throw ErrorCode;
			
			ELEM * h = (blk->next = m_Blocks, m_Blocks = blk)->contents;
			for (ELEM * o = h; o - h < nBlockElements; o++)
				o->next = m_FreeList, m_FreeList = o;
		}
		assert(m_FreeList != NULL);
		ELEM * o = m_FreeList; m_FreeList = o->next;
		o->next = Busy();
		if (bClear)
			memset(o->data, 0, sizeof o->data);
		return o->data;
	}

public:
	ObjStorageClass()
		: m_FreeList(NULL), m_Blocks(NULL) {}

	~ObjStorageClass() {
		Clr();
	}

	void Put(T * _pObj)
	{
		assert(_pObj != NULL);
		_pObj->~T();
		ELEM * pObj = reinterpret_cast<ELEM *>(_pObj);
		pObj->next = m_FreeList, m_FreeList = pObj;
	} // Put

	T * Get(bool bClear = false)
	{
		void * n = GetFree(bClear);
		return new (n) T;
	} // Get

	T * Get(const T & ini, bool bClear = false)
	{
		void * n = GetFree(bClear);
		return new (n) T(ini);
	} // Get

			class iterator;
	friend	class iterator;
	class iterator
	{
		ELEM *	m_Node;
		BLK *	m_Blk;

		void NewBlock() {
			m_Node = (m_Blk == NULL) ? NULL : m_Blk->contents;
		}
		void Advance() {
			m_Node++;
			if (m_Node - m_Blk->contents >= nBlockElements)
				m_Blk = m_Blk->next, NewBlock();
		}
		void FindAllocated() {
			while (m_Node != NULL and m_Node->next != Busy())
				Advance();
		}
	public:
		iterator(const iterator & i)
			:	m_Node(i.m_Node), m_Blk(i.m_Blk) {}

		iterator(const BLK * blocks)
			: m_Blk(const_cast<BLK *>(blocks)) {
			NewBlock();
			FindAllocated();
		}

		T & operator*() const {
			return *reinterpret_cast<T *>(m_Node);
		}

		T * operator->() const {
			return &**this;
		}

		iterator & operator++() {
			Advance(), FindAllocated();
			return *this;
		}
		bool operator==(const iterator & _X) const {
			return (m_Node == _X.m_Node);
		}
		bool operator!=(const iterator & _X) const {
			return !(*this == _X);
		}
	}; // class ObjStorageClass::iterator

	iterator begin() const {
		return iterator(m_Blocks);
	}
	iterator end() const {
		return iterator(NULL);
	}

	void Clr()
	{
		// call destructors for allocated elements
		for (iterator i = begin(); i != end(); ++i)
			i->~T();

		// free memory blocks
		while (m_Blocks != NULL)
		{
			BLK * blk = m_Blocks;
			m_Blocks = m_Blocks->next;
			delete blk;
		}
		m_FreeList = NULL;
	} // Clr
}; // class ObjStorageClass

#endif /* _OBJHEAP_H_ */

