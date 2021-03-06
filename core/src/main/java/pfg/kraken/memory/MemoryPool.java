/*
 * Copyright (C) 2013-2017 Pierre-François Gimenez
 * Distributed under the MIT License.
 */


package pfg.kraken.memory;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import pfg.kraken.LogCategoryKraken;
import pfg.graphic.log.Log;


/**
 * Classe qui fournit des objets
 * Quand on a besoin de beaucoup d'objets, car l'instanciation d'un objet est
 * long.
 * Du coup on réutilise les mêmes objets sans devoir en créer tout le temps de
 * nouveaux.
 * 
 * @author pf
 *
 */

public abstract class MemoryPool<T extends Memorizable>
{

	private int initial_nb_instances;

	private List<T[]> nodes = new ArrayList<T[]>();
	private Class<T> classe;
	protected Log log;
	private int firstAvailable;
	private int tailleMax = 1 << 24;

	protected abstract void make(T[] nodes);
	
	public MemoryPool(Class<T> classe, Log log)
	{
		this.classe = classe;
		this.log = log;
	}
	
	@SuppressWarnings("unchecked")
	protected void init(int nb_instances)
	{
		initial_nb_instances = nb_instances;
		nodes.add((T[]) Array.newInstance(classe, nb_instances));
		firstAvailable = 0;
		// on instancie une fois pour toutes les objets
		log.write("Memory pool initialization (" + nb_instances + " instances of " + classe.getSimpleName() + ")", LogCategoryKraken.PF);

		make(nodes.get(0));
		for(int i = 0; i < nb_instances; i++)
			nodes.get(0)[i].setIndiceMemoryManager(i);
	}

	/**
	 * Donne un objet disponible
	 * 
	 * @return
	 * @throws InterruptedException
	 */
	@SuppressWarnings("unchecked")
	public synchronized T getNewNode()
	{
		// lève une exception s'il n'y a plus de place
		if(firstAvailable == initial_nb_instances * nodes.size())
		{
			/**
			 * Probablement une erreur
			 */
			assert initial_nb_instances * nodes.size() < tailleMax : "Mémoire saturée pour " + classe.getSimpleName();

//			if(nodes.size() + 1 >= 20)
//				log.write("Mémoire trop petite pour les " + classe.getSimpleName() + ", extension (nouvelle taille : " + ((nodes.size() + 1) * initial_nb_instances) + ")", SeverityCategoryKraken.WARNING, LogCategoryKraken.PF);

			T[] newNodes = (T[]) Array.newInstance(classe, initial_nb_instances);

			make(newNodes);
			
			for(int i = 0; i < initial_nb_instances; i++)
				newNodes[i].setIndiceMemoryManager(i + firstAvailable);

			nodes.add(newNodes);
		}

		T out = nodes.get(firstAvailable / initial_nb_instances)[firstAvailable % initial_nb_instances];
		firstAvailable++;
		return out;
	}

	/**
	 * Signale que tous les objets sont disponibles. Très rapide.
	 * 
	 * @param id_astar
	 */
	public synchronized void empty()
	{
		firstAvailable = 0;
	}

	/**
	 * Signale qu'un objet est de nouveau disponible
	 * 
	 * @param objet
	 * @throws MemoryPoolException 
	 */
	public synchronized void destroyNode(T objet)
	{

		int indice_state = objet.getIndiceMemoryManager();

		/**
		 * Invariant: l'objet ne doit pas être déjà détruit
		 */
		assert indice_state < firstAvailable : "Objet déjà détruit ! " + indice_state + " >= " + firstAvailable;
		if(indice_state >= firstAvailable)
			return;

		// On inverse dans le Vector les deux objets,
		// de manière à avoir toujours un Vector trié.
		firstAvailable--;

		if(indice_state != firstAvailable)
		{
			T tmp1 = nodes.get(indice_state / initial_nb_instances)[indice_state % initial_nb_instances];
			T tmp2 = nodes.get(firstAvailable / initial_nb_instances)[firstAvailable % initial_nb_instances];

			tmp1.setIndiceMemoryManager(firstAvailable);
			tmp2.setIndiceMemoryManager(indice_state);

			nodes.get(firstAvailable / initial_nb_instances)[firstAvailable % initial_nb_instances] = tmp1;
			nodes.get(indice_state / initial_nb_instances)[indice_state % initial_nb_instances] = tmp2;
		}
	}

	/**
	 * Retourne le nombre d'élément utilisé
	 */
	public synchronized int getSize()
	{
		return firstAvailable;
	}

}
