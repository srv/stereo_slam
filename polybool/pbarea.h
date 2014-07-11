//	pbarea.h - definition of plane sweep class, for internal use only
//
//	This file is a part of PolyBoolean software library
//	(C) 1998-1999 Michael Leonov
//	Consult your license regarding permissions and restrictions
//

#ifndef _PBAREA_H_
#define _PBAREA_H_

#include "polybool.h"
#include "pbimpl.h"

namespace POLYBOOLEAN
{

///////////////////////// Plane Sweep ///////////////////////////

struct SEGM2
{
	VNODE2* l;	// left point
	VNODE2* r;	// right point

	SEGM2 *	n, * p;		// next, prev in main active list

	bool	m_bRight; // (l->next == r)
}; // struct SEGM2

class BOCTX
{
public:
	typedef void (*INS_PROC)(void ** list, VNODE2 * vn, void * parm);

	BOCTX(INS_PROC InsertProc, void * InsertParm);

	void	Sweep(SEGM2 * aSegms, UINT32 nSegms);
	
protected:
	struct TSEL
	{
		INT32	y;
		void  * list;

		bool operator<(const TSEL & b) const {
			return y <= b.y;
		}
		bool operator==(const TSEL & b) const {
			return y == b.y;
		}
	};

	typedef STD::list<TSEL>	LIST_TSEL;

	struct EVENT
	{
		// data members
		UINT32	id;		// event id, must be unique for each event
		INT32	x, y;	// (rounded) coordinate
		unsigned int	m_type : 2;
		int				m_signX : 2;
		int				m_signY : 2;

		union {
			struct // starting event
			{
				SEGM2 * s;	// responsible segment
			} s;
			struct // ending event
			{
				SEGM2 * s;	// responsible segment
			} e;
			struct // crossing event
			{
				SEGM2 * s0, * s1;	// s0->n == s1 or event is ignored
			} c;
		};

		// operations

		//	due to the infinitesimal shortening rule, event values
		//	should be compared in the order below
		enum TYPE {
			E,	// end
			X,	// intersection
			S	// start
		};


		void SetType(TYPE type) {
			m_type = static_cast<unsigned int>(type);
		}
		TYPE GetType() const {
			assert(m_type != 3); // unknown event type
			return static_cast<TYPE>(m_type);
		}

		void SetSignX(int sign)
		{
			assert(GetType() == X);
			m_signX = sign;
		}
		int GetSignX() const
		{
			assert(GetType() == X);
			return m_signX;
		}
		void SetSignY(int sign)
		{
			assert(GetType() == X);
			m_signY = sign;
		}
		int GetSignY() const
		{
			assert(GetType() == X);
			return m_signY;
		}

		static INT32 Compare(const EVENT &a, const EVENT &b);

		struct EVLS : public STD::binary_function<const EVENT &, const EVENT &, bool>
		{
			bool operator()(const EVENT & x, const EVENT & y) const
			{
				return EVENT::Compare(x, y) > 0;
			}
		};
	}; // struct EVENT

	typedef STD::vector<EVENT>	EVENTLIST;
	typedef STD::vector<SEGM2*>	SAVE_LIST;

	INS_PROC		m_InsertProc;
	void		  *	m_InsertParm;
	STD::priority_queue<EVENT, STD::vector<EVENT>, EVENT::EVLS>	m_E;	// event queue

	class SEGM_LIST
	{
		SEGM2	m_Head;
	public:
		SEGM_LIST()
		{
			m_Head.n = m_Head.p = &m_Head;
#ifndef NDEBUG
			m_Head.l = m_Head.r = NULL;
#endif
		}
		class iterator;
		friend class iterator;
		class iterator : public STD::iterator<std::bidirectional_iterator_tag, SEGM2>
		{
			SEGM2 *	m_Ptr;
		public:
			iterator() {}
			iterator(const iterator & i)
				: m_Ptr(i.m_Ptr) {}
			iterator(SEGM2 * segm)
				: m_Ptr(segm) {}
			bool operator==(const iterator & i) const {
				return m_Ptr == i.m_Ptr;
			}
			bool operator!=(const iterator & i) const {
				return not(*this == i);
			}
			iterator & operator++() {
				m_Ptr = m_Ptr->n;
				return *this;
			}
			iterator & operator--() {
				m_Ptr = m_Ptr->p;
				return *this;
			}
			SEGM2 & operator*() {
				return *m_Ptr;
			}
			SEGM2 * operator->() {
				return &**this;
			}
		}; // class iterator

		iterator end() {
			return iterator(&m_Head);
		}
		iterator begin() {
			return iterator(m_Head.n);
		}

		// insert s before segm
		void insert(iterator segm, SEGM2 & s) {
			(((s.p = segm->p)->n = &s)->n = &*segm)->p = &s;
		}
		void erase(iterator it) {
			(it->n->p = it->p)->n = it->n;
		}
	}; // class SEGM_LIST
	SEGM_LIST		m_S;	// sweep line status
	UINT32			m_nId;	// maximimum event id

	void	CollectEvents(SEGM2 * aSegms, UINT32 nSegms);
	void	Pass2(EVENTLIST * elist, SAVE_LIST * save_list);

	// a bunch of helper functions

	inline void HandleEvent(EVENT & event);
	inline void CheckForCross(SEGM2 * sp, SEGM2 * sn);
	inline void Intercept(LIST_TSEL * list, SEGM2 * segm, INT32 cx);
	inline void InsNewNode(SEGM2 * s, TSEL * tsel, INT32 X);
	inline void InsMainList(SEGM2 * s);
	inline void AddEvent(EVENTLIST * list, const EVENT & e);
}; // class BOCTX

} // namespace POLYBOOLEAN

#endif // _PBAREA_H_

