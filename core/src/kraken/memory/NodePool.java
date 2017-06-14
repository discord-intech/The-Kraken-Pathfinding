/*
 * Copyright (C) 2013-2017 Pierre-François Gimenez
 * Distributed under the MIT License.
 */


package kraken.memory;

import config.Config;
import kraken.ConfigInfoKraken;
import kraken.pathfinding.astar.AStarCourbeNode;
import kraken.robot.RobotState;
import kraken.utils.Log;

/**
 * Memory Manager des nœuds du pathfinding courbe
 * 
 * @author pf
 *
 */

public class NodePool extends MemoryPool<AStarCourbeNode>
{
	private int largeur, longueur_arriere, longueur_avant;

	public NodePool(Log log, Config config)
	{
		super(AStarCourbeNode.class, log);
		largeur = config.getInt(ConfigInfoKraken.LARGEUR_NON_DEPLOYE) / 2;
		longueur_arriere = config.getInt(ConfigInfoKraken.DEMI_LONGUEUR_NON_DEPLOYE_ARRIERE);
		longueur_avant = config.getInt(ConfigInfoKraken.DEMI_LONGUEUR_NON_DEPLOYE_AVANT);
		init(config.getInt(ConfigInfoKraken.NB_INSTANCES_NODE));
	}

	@Override
	protected final void make(AStarCourbeNode[] nodes)
	{
		for(int i = 0; i < nodes.length; i++)
			nodes[i] = new AStarCourbeNode(new RobotState(), largeur, longueur_arriere, longueur_avant);
	}

}