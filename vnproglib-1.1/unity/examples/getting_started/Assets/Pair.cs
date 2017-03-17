using System;
using System.Collections.Generic;
using System.Text;

namespace VectorNav
{

public class Pair<X, Y>
{
	public X First { get { return _x; } }

	public Y Second { get { return _y; } }

	public Pair(X first, Y second)
	{
		_x = first;
		_y = second;
	}

	private X _x;
	private Y _y;
}

}
