/*
 * Copyright (C) 2013-2017 Pierre-François Gimenez
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
 */

package kraken.threads;

import config.Config;
import graphic.PrintBufferInterface;
import kraken.config.ConfigInfoKraken;
import kraken.obstacles.memory.ObstaclesMemory;
import kraken.pathfinding.dstarlite.DStarLite;
import kraken.utils.Log;

/**
 * Thread qui gère la péremption des obstacles en dormant
 * le temps exact entre deux péremptions.
 * 
 * @author pf
 *
 */

public class ThreadPeremption extends Thread
{
	private ObstaclesMemory memory;
	protected Log log;
	private PrintBufferInterface buffer;
	private DStarLite dstarlite;

	private int dureePeremption;
	private boolean printProxObs;

	public ThreadPeremption(Log log, ObstaclesMemory memory, PrintBufferInterface buffer, DStarLite dstarlite, Config config)
	{
		this.log = log;
		this.memory = memory;
		this.buffer = buffer;
		this.dstarlite = dstarlite;
		printProxObs = config.getBoolean(ConfigInfoKraken.GRAPHIC_PROXIMITY_OBSTACLES);
		dureePeremption = config.getInt(ConfigInfoKraken.DUREE_PEREMPTION_OBSTACLES);
	}

	@Override
	public void run()
	{
		Thread.currentThread().setName(getClass().getSimpleName());
		log.debug("Démarrage de " + Thread.currentThread().getName());
		try
		{
			while(true)
			{
				if(memory.deleteOldObstacles())
					dstarlite.updateObstaclesEnnemi();

				// mise à jour des obstacles : on réaffiche
				if(printProxObs)
					synchronized(buffer)
					{
						buffer.notify();
					}

				long prochain = memory.getNextDeathDate();

				/**
				 * S'il n'y a pas d'obstacles, on dort de dureePeremption, qui
				 * est la durée minimale avant la prochaine péremption.
				 */
				if(prochain == Long.MAX_VALUE)
					Thread.sleep(dureePeremption);
				else
					// Il faut toujours s'assurer qu'on dorme un temps positif.
					// Il y a aussi une petite marge
					Thread.sleep(Math.min(dureePeremption, Math.max(prochain - System.currentTimeMillis() + 2, 5)));
			}
		}
		catch(InterruptedException e)
		{
			log.debug("Arrêt de " + Thread.currentThread().getName());
			Thread.currentThread().interrupt();
		}
		catch(Exception e)
		{
			log.debug("Arrêt inattendu de " + Thread.currentThread().getName() + " : " + e);
			e.printStackTrace();
			e.printStackTrace(log.getPrintWriter());
			Thread.currentThread().interrupt();
		}
	}

}