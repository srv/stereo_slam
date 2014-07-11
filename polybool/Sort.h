//	Sort.h - sorting routine
//
//	This file is a part of PolyBoolean software library
//	(C) 1998-1999 Michael Leonov
//	Consult your license regarding permissions and restrictions
//

#ifndef _SORT_H_
#define _SORT_H_

// merge sort: Knuth TAOCP Vol.3
template<class VNODE>
VNODE * MergeSort(VNODE ** Head)
{
	VNODE *p = *Head;

	if (!p)
		return 0;
	if (!p->nxt())
		return p;

	VNODE elm[4];

	VNODE * q = (elm[0].nxt() = p)->nxt();
	VNODE * r = (elm[1].nxt() = q)->nxt();

	for (;;)
	{
		if (!r) break;
		r = (p = p->nxt() = r)->nxt();
		if (!r) break;
		r = (q = q->nxt() = r)->nxt();
	}
	p->nxt() = q->nxt() = 0;

	unsigned int n = 1;

	for(;;)
	{
		{
			p = (elm + 0)->nxt();
			q = (elm + 1)->nxt();
			if (!q)
				break;
			r = (elm + 2);
			VNODE * s = (elm + 3);

			int	nP = n;
			int nQ = n;
			for (;;)
			{
				{
					VNODE * t = r;
					for (;;)
					{
						if (nP > 0 and (nQ < 1 or VNODE::Compare(
							const_cast<const VNODE &>(*p),
							const_cast<const VNODE &>(*q)) <= 0))
						{
							p = (t = t->nxt() = p)->nxt();
							if (!p) nP = -1; else --nP;
						}
						else 
						{
							if (nQ < 1)
								break;
							q = (t = t->nxt() = q)->nxt();
							if (!q) nQ = -1; else --nQ;
						}
					}
					r = t;
				}
				if (nP < 0)
				{
					if (nQ >= 0)
						nQ = n;
					else
						break;
				}
				else
				{
						nP = n;
					if (nQ >= 0)
						nQ = n;
				}
				{
					VNODE * t = s;
					for (;;)
					{
						if (nP > 0 and (nQ < 1 or VNODE::Compare(
							const_cast<const VNODE &>(*p),
							const_cast<const VNODE &>(*q)) <= 0))
						{
							p = (t = t->nxt() = p)->nxt();
							if (!p) nP = -1; else --nP;
						}
						else 
						{
							if (nQ < 1)
								break;
							q = (t = t->nxt() = q)->nxt();
							if (!q) nQ = -1; else --nQ;
						}
					}
					s = t;
				}
				if (nP < 0)
				{
					if (nQ >= 0)
						nQ = n;
					else
						break;
				}
				else
				{
						nP = n;
					if (nQ >= 0)
						nQ = n;
				}
			}
			r->nxt() = s->nxt() = 0;
			n *= 2;
		}
		{
			p = (elm + 2)->nxt();
			q = (elm + 3)->nxt();
			if (!q)
				break;
			r = (elm + 0);
			VNODE * s = (elm + 1);
			int	nP = n;
			int nQ = n;
			for (;;)
			{
				{
					VNODE * t = r;
					for (;;)
					{
						if (nP > 0 and (nQ < 1 or VNODE::Compare(
							const_cast<const VNODE &>(*p),
							const_cast<const VNODE &>(*q)) <= 0))
						{
							p = (t = t->nxt() = p)->nxt();
							if (!p) nP = -1; else --nP;
						}
						else 
						{
							if (nQ < 1)
								break;
							q = (t = t->nxt() = q)->nxt();
							if (!q) nQ = -1; else --nQ;
						}
					}
					r = t;
				}
				if (nP < 0)
				{
					if (nQ >= 0)
						nQ = n;
					else
						break;
				}
				else
				{
						nP = n;
					if (nQ >= 0)
						nQ = n;
				}
				{
					VNODE * t = s;
					for (;;)
					{
						if (nP > 0 and (nQ < 1 or VNODE::Compare(
							const_cast<const VNODE &>(*p),
							const_cast<const VNODE &>(*q)) <= 0))
						{
							p = (t = t->nxt() = p)->nxt();
							if (!p) nP = -1; else --nP;
						}
						else 
						{
							if (nQ < 1)
								break;
							q = (t = t->nxt() = q)->nxt();
							if (!q) nQ = -1; else --nQ;
						}
					}
					s = t;
				}
				if (nP < 0)
				{
					if (nQ >= 0)
						nQ = n;
					else
						break;
				}
				else
				{
						nP = n;
					if (nQ >= 0)
						nQ = n;
				}
			}
			r->nxt() = s->nxt() = 0;
			n *= 2;
		}
	}
	*Head = p;
	return r;
} // MergeSort

#endif // _SORT_H_

