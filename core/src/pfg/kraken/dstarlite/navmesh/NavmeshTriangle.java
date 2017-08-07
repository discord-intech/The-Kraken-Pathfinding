/*
 * Copyright (C) 2013-2017 Pierre-François Gimenez
 * Distributed under the MIT License.
 */

package pfg.kraken.dstarlite.navmesh;

import java.awt.Graphics;
import java.io.Serializable;

import pfg.graphic.Fenetre;
import pfg.graphic.printable.Layer;
import pfg.graphic.printable.Printable;
import pfg.kraken.Couleur;
import pfg.kraken.utils.XY;
import pfg.kraken.utils.XY_RW;

/**
 * A triangle of the navmesh
 * @author pf
 *
 */

public class NavmeshTriangle implements Serializable, Printable
{
	private static final long serialVersionUID = 1L;
	NavmeshNode[] points = new NavmeshNode[3];
	NavmeshEdge[] edges = new NavmeshEdge[3];
	int area;
	
	NavmeshTriangle(NavmeshEdge a, NavmeshEdge b, NavmeshEdge c)
	{
		setEdges(a, b, c);
	}
	
	/**
	 * Returns true iff a, b and c are counterclockwise.
	 * Uses a cross product
	 * @param a
	 * @param b
	 * @param c
	 * @return
	 */
	boolean checkCounterclockwise()
	{
		return crossProduct(points[0].position, points[1].position, points[2].position) >= 0;
	}
	
	boolean checkPoints()
	{
		return points[0] != points[1] && points[0] != points[2] && points[1] != points[2];
	}
	
	/**
	 * Check if the point i isn't in the edge i
	 * @return
	 */
	boolean checkDuality()
	{
		assert checkPoints();
		assert checkTriangle(edges[0], edges[1], edges[2]);
		for(int i = 0; i < 3; i++)
			for(int j = 0; j < 2; j++)
				if(edges[i].points[j] == points[i])
					return false;
		return true;
	}
	
	/**
	 * Return the triangle area
	 * @return
	 */
	private void updateArea()
	{
		assert checkCounterclockwise() : this;
		area = (int) crossProduct(points[0].position, points[1].position, points[2].position) / 2;
	}
	
	/**
	 * Well… a cross product.
	 * @param a
	 * @param b
	 * @param c
	 * @return
	 */
	private double crossProduct(XY a, XY b, XY c)
	{
		return (b.getX() - a.getX()) * (c.getY() - a.getY()) - (b.getY() - a.getY()) * (c.getX() - a.getX());
	}
	
	private boolean checkTriangle(NavmeshEdge a, NavmeshEdge b, NavmeshEdge c)
	{
		NavmeshEdge[] e = new NavmeshEdge[3];
		e[0] = a;
		e[1] = b;
		e[2] = c;
		for(int i = 0; i < 3; i++)
			for(int j = 0; j < 2; j++)
			{
				int nb = 0;
				for(int i2 = 0; i2 < 3; i2++)
					for(int j2 = 0; j2 < 2; j2++)
						if(e[i].points[j] == e[i2].points[j2])
							nb++;
				assert nb == 2 : a.shortString()+" "+b.shortString()+" "+c.shortString(); // each point can be found in two edge ends
			}
		for(int i = 0; i < 3; i++)
			assert e[i].points[0] != e[i].points[1] : e[i].shortString(); // each edge connects two different points
		
		return true;
	}
	
	void setEdges(NavmeshEdge a, NavmeshEdge b, NavmeshEdge c)
	{
		assert checkTriangle(a, b, c);
		
		for(int i = 0; i < 3; i++)
			if(edges[i] != null)
				edges[i].removeTriangle(this);

		a.addTriangle(this);
		b.addTriangle(this);
		c.addTriangle(this);

		edges[0] = a;
		edges[1] = b;
		edges[2] = c;
		
		if(a.points[0] == b.points[0] || a.points[1] == b.points[0])
			points[0] = b.points[1];
		else
			points[0] = b.points[0];
		
		if(b.points[0] == c.points[0] || b.points[1] == c.points[0])
			points[1] = c.points[1];
		else
			points[1] = c.points[0];
		
		if(c.points[0] == b.points[0] || c.points[1] == b.points[0])
			points[2] = b.points[1];
		else
			points[2] = b.points[0];

		correctCounterclockwiseness();

		updateArea();
		assert checkDuality() : this;
		assert checkCounterclockwise() : this;
	}
	
	void correctCounterclockwiseness()
	{
		if(!checkCounterclockwise())
		{
			NavmeshNode tmp = points[0];
			points[0] = points[1];
			points[1] = tmp;
			
			NavmeshEdge tmp2 = edges[0];
			edges[0] = edges[1];
			edges[1] = tmp2;
		}
		
		assert checkDuality() : this;
		assert checkCounterclockwise() : this;
	}

	private XY_RW v0 = new XY_RW(), v1 = new XY_RW(), v2 = new XY_RW();
	
	/**
	 * Use the barycentric method. Source : http://www.blackpawn.com/texts/pointinpoly/default.html
	 * @param position
	 * @return
	 */
	public boolean isInside(XY position)
	{
		assert checkCounterclockwise() : this;
		
		// Compute vectors     
		points[2].position.copy(v0);
		v0.minus(points[0].position);
		
		points[1].position.copy(v1);
		v1.minus(points[0].position);
		
		position.copy(v2);
		v2.minus(points[0].position);

		// Compute dot products
		double dot00 = v0.dot(v0);
		double dot01 = v0.dot(v1);
		double dot02 = v0.dot(v2);
		double dot11 = v1.dot(v1);
		double dot12 = v1.dot(v2);

		// Compute barycentric coordinates
		double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
		double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
		double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

		// Check if point is in triangle
		return (u >= 0) && (v >= 0) && (u + v < 1);
	}
	
	public String toString()
	{
		return "Triangle with points "+points[0].position+", "+points[1].position+", "+points[2].position+" and edges "+edges[0].shortString()+", "+edges[1].shortString()+", "+edges[2].shortString();
	}

	@Override
	public void print(Graphics g, Fenetre f)
	{
		g.setColor(Couleur.NAVMESH_TRIANGLE.couleur);
		g.fillPolygon(new int[]{f.XtoWindow(points[0].position.getX()), f.XtoWindow(points[1].position.getX()), f.XtoWindow(points[2].position.getX())},
				new int[]{f.YtoWindow(points[0].position.getY()), f.YtoWindow(points[1].position.getY()), f.YtoWindow(points[2].position.getY())},
				3);
	}
	
	@Override
	public boolean equals(Object o)
	{
		return o == this;
	}

	@Override
	public int getLayer()
	{
		return Layer.BACKGROUND.ordinal();
	}

}