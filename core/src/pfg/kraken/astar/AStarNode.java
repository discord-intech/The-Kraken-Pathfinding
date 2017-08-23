/*
 * Copyright (C) 2013-2017 Pierre-François Gimenez
 * Distributed under the MIT License.
 */

package pfg.kraken.astar;

import java.awt.Color;
import java.awt.Graphics;

import pfg.graphic.Chart;
import pfg.graphic.GraphicPanel;
import pfg.graphic.printable.Printable;
import pfg.kraken.Couleur;
import pfg.kraken.astar.tentacles.DynamicTentacle;
import pfg.kraken.astar.tentacles.StaticTentacle;
import pfg.kraken.astar.tentacles.Tentacle;
import pfg.kraken.memory.Memorizable;
import pfg.kraken.robot.RobotState;

/**
 * Un nœud de l'A* courbe
 * 
 * @author pf
 *
 */

public class AStarNode implements Memorizable, Printable
{
	private static final long serialVersionUID = -2120732124823178009L;
	public RobotState robot;
	public double g_score; // distance du point de départ à ce point
	public double f_score; // g_score + heuristique = meilleure distance qu'on
							// peut espérer avec ce point
	public AStarNode parent;
	public final StaticTentacle cameFromArcStatique;
	public DynamicTentacle cameFromArcDynamique = null;
	private int indiceMemoryManager;
	private boolean dead = false;

	public AStarNode(RobotState robot, int demieLargeurNonDeploye, int demieLongueurArriere, int demieLongueurAvant)
	{
		cameFromArcStatique = new StaticTentacle(demieLargeurNonDeploye, demieLongueurArriere, demieLongueurAvant);
		this.robot = robot;
	}

	public Tentacle getArc()
	{
		if(parent == null)
			return null;
		if(cameFromArcDynamique != null)
			return cameFromArcDynamique;
		return cameFromArcStatique;
	}

	public void init()
	{
		g_score = Double.MAX_VALUE;
		f_score = Double.MAX_VALUE;
	}

	@Override
	public boolean equals(Object o)
	{
		return o.hashCode() == hashCode();
	}

	@Override
	public int hashCode()
	{
		return robot.getCinematique().hashCode();
	}

	@Override
	public void setIndiceMemoryManager(int indice)
	{
		indiceMemoryManager = indice;
	}

	@Override
	public int getIndiceMemoryManager()
	{
		return indiceMemoryManager;
	}

	/**
	 * Cette copy n'est utilisée qu'à une seule occasion, quand on reconstruit
	 * partiellement le chemin
	 * 
	 * @param modified
	 */
	public void copyReconstruct(AStarNode modified)
	{
		modified.cameFromArcDynamique = null;
		modified.g_score = g_score;
		modified.f_score = f_score;
		robot.copy(modified.robot);
		modified.parent = null;
	}

	@Override
	public void print(Graphics g, GraphicPanel f, Chart aff)
	{
		Tentacle a = getArc();
		if(a != null)
		{
			a.print(g, f, aff);
			double h = f_score;
			double seuil = 5000;
			if(h > seuil)
				h = seuil;

			if(dead)
				g.setColor(Couleur.BLANC.couleur);
			else
				g.setColor(new Color((int) (h * 255 / seuil), 0, (int) (255 - h * 255 / seuil)));

			a.getLast().print(g, f, aff);
		}
	}

	@Override
	public int getLayer()
	{
		return Couleur.TRAJECTOIRE.l.ordinal();
	}

	public void setDead()
	{
		dead = true;
	}
}
