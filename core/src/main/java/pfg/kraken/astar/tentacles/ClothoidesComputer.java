/*
 * Copyright (C) 2013-2017 Pierre-François Gimenez
 * Distributed under the MIT License.
 */

package pfg.kraken.astar.tentacles;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;
import pfg.config.Config;
import pfg.kraken.ConfigInfoKraken;
import pfg.kraken.LogCategoryKraken;
import pfg.kraken.SeverityCategoryKraken;
import pfg.kraken.astar.AStarNode;
import pfg.kraken.astar.tentacles.types.ClothoTentacle;
import pfg.kraken.astar.tentacles.types.StraightingTentacle;
import pfg.kraken.astar.tentacles.types.TentacleType;
import pfg.kraken.memory.CinemObsPool;
import pfg.kraken.robot.Cinematique;
import pfg.kraken.robot.CinematiqueObs;
import pfg.kraken.robot.RobotState;
import pfg.kraken.utils.XY;
import pfg.kraken.utils.XY_RW;
import pfg.graphic.log.Log;
import static pfg.kraken.astar.tentacles.Tentacle.*;

/**
 * Classe qui s'occupe de tous les calculs concernant les clothoïdes
 * 
 * @author pf
 *
 */

public class ClothoidesComputer implements TentacleComputer
{
	private Log log;
	private CinemObsPool memory;
	private double rootedMaxAcceleration;
	
	private BigDecimal x, y; // utilisés dans le calcul de trajectoire
	private static final int S_MAX = 10; // courbure max qu'on puisse gérer
	private static final int INDICE_MAX = (int) (S_MAX / PRECISION_TRACE);
	private XY[] trajectoire = new XY[2 * INDICE_MAX - 1];

	public ClothoidesComputer(Log log, Config config, CinemObsPool memory)
	{
		this.memory = memory;
		this.log = log;
		rootedMaxAcceleration = Math.sqrt(config.getDouble(ConfigInfoKraken.MAX_LATERAL_ACCELERATION));
		for(ClothoTentacle t : ClothoTentacle.values())
		{
			if(t.vitesse == 0)
				t.maxSpeed = Double.MAX_VALUE;
			else
				t.maxSpeed = config.getDouble(ConfigInfoKraken.MAX_CURVATURE_DERIVATIVE) / Math.abs(t.vitesse);
		}
		if(!chargePoints()) // le calcul est un peu long, donc on le sauvegarde
		{
			init();
			sauvegardePoints();
		}
	}

	/**
	 * Calcul grâce au développement limité d'Euler
	 * Génère le point de la clothoïde unitaire de courbure = s
	 * 
	 * @param s
	 */
	private void calculeXY(BigDecimal sparam)
	{
		BigDecimal s = sparam;
		x = s;
		BigDecimal factorielle = new BigDecimal(1).setScale(15, RoundingMode.HALF_EVEN);
		BigDecimal b2 = new BigDecimal(1).setScale(15, RoundingMode.HALF_EVEN);
		BigDecimal s2 = s.multiply(s);
		BigDecimal b = b2;
		s = s.multiply(s2);
		y = s.divide(b.multiply(new BigDecimal(3).setScale(15, RoundingMode.HALF_EVEN)), RoundingMode.HALF_EVEN);
		BigDecimal seuil = new BigDecimal(0.000000000001).setScale(15, RoundingMode.HALF_EVEN);
		BigDecimal tmp;

		long i = 1;
		do
		{
			factorielle = factorielle.multiply(new BigDecimal(2 * i).setScale(15, RoundingMode.HALF_EVEN));
			b = b.multiply(b2);
			s = s.multiply(s2);

			tmp = s.divide(factorielle.multiply(b).multiply(new BigDecimal(4 * i + 1).setScale(15, RoundingMode.HALF_EVEN)), RoundingMode.HALF_EVEN);

			if((i & 1) == 0)
				x = x.add(tmp);
			else
				x = x.subtract(tmp);

			factorielle = factorielle.multiply(new BigDecimal(2 * i + 1).setScale(15, RoundingMode.HALF_EVEN));

			b = b.multiply(b2);
			s = s.multiply(s2);
			tmp = s.divide(factorielle.multiply(b).multiply(new BigDecimal(4 * i + 3).setScale(15, RoundingMode.HALF_EVEN)), RoundingMode.HALF_EVEN);

			if((i & 1) == 0)
				y = y.add(tmp);
			else
				y = y.subtract(tmp);

			i++;
		} while(tmp.abs().compareTo(seuil) > 0);
		// On fait en sorte que tourner à gauche ait une courbure positive
		y = y.multiply(new BigDecimal(1000)); // On considère que x et y sont en
												// millimètre et que la courbure
												// est en mètre^-1
		x = x.multiply(new BigDecimal(1000));
	}

	/**
	 * Calcule, une fois pour toutes, les points de la clothoïde unitaire
	 */
	private void init()
	{
		log.write("Computation of the unitary clothoid, please wait (it should take at most one minute). This computation is made once and for all.", LogCategoryKraken.PF);
		for(int s = 0; s < 2 * INDICE_MAX - 1; s++)
		{
			calculeXY(new BigDecimal((s - INDICE_MAX + 1) * PRECISION_TRACE).setScale(15, RoundingMode.HALF_EVEN));
			trajectoire[s] = new XY(x.doubleValue(), y.doubleValue());
			trajectoire[2 * INDICE_MAX - 2 - s] = new XY(-x.doubleValue(), -y.doubleValue());
		}
	}

	public XY[] getTrajectoirePoints()
	{
		return trajectoire;
	}

	public void getTrajectoire(Tentacle depart, ClothoTentacle vitesse, StaticTentacle modified)
	{
		CinematiqueObs last = depart.getLast();
		getTrajectoire(last, vitesse, modified);
	}

	/**
	 * Première trajectoire. On considère que la vitesse initiale du robot est
	 * nulle
	 * 
	 * @param robot
	 * @param vitesse
	 * @param modified
	 */
	final void getTrajectoire(RobotState robot, ClothoTentacle vitesse, StaticTentacle modified)
	{
		getTrajectoire(robot.getCinematique(), vitesse, modified);
	}

	/**
	 * ATTENTION ! La courbure est en m^-1 et pas en mm^-1
	 * En effet, comme le rayon de courbure sera souvent plus petit que le
	 * mètre, on aura une courbure souvent plus grande que 1
	 * Le contenu est mis dans l'arccourbe directement
	 * 
	 * @param position
	 * @param orientationGeometrique
	 * @param courbureGeometrique
	 * @param vitesse
	 * @param distance_mm
	 * @return
	 */
	public final void getTrajectoire(Cinematique cinematiqueInitiale, ClothoTentacle vitesse, StaticTentacle modified)
	{
		// modified.v = vitesse;
		// log.debug(vitesse);
		double courbure = cinematiqueInitiale.courbureGeometrique;
		double orientation = cinematiqueInitiale.orientationGeometrique;
		if(vitesse.rebrousse)
			orientation += Math.PI;

		// on s'arrête, on peut tourner les roues
		if(vitesse.rebrousse || vitesse.arret)
			courbure = vitesse.courbureInitiale;

		modified.vitesse = vitesse;

		boolean marcheAvant = vitesse.rebrousse ^ cinematiqueInitiale.enMarcheAvant;

		// si la dérivée de la courbure est nulle, on est dans le cas
		// particulier d'une trajectoire rectiligne ou circulaire
		if(vitesse.vitesse == 0)
		{
			if(courbure < 0.00001 && courbure > -0.00001)
				getTrajectoireLigneDroite(cinematiqueInitiale.getPosition(), orientation, modified, marcheAvant);
			else
				getTrajectoireCirculaire(cinematiqueInitiale.getPosition(), orientation, courbure, modified, marcheAvant);
			return;
		}

		double coeffMultiplicatif = 1. / vitesse.squaredRootVitesse;
		double sDepart = courbure / vitesse.squaredRootVitesse; // sDepart peut
																// parfaitement
																// être négatif
		if(!vitesse.positif)
			sDepart = -sDepart;
		int pointDepart = (int) ((sDepart / PRECISION_TRACE) + INDICE_MAX - 1 + 0.5); // le
																						// 0.5
																						// vient
																						// du
																						// fait
																						// qu'on
																						// fait
																						// ici
																						// un
																						// arrondi

		if(pointDepart < 0 || pointDepart >= trajectoire.length)
			log.write("Sorti de la clothoïde précalculée !", SeverityCategoryKraken.CRITICAL, LogCategoryKraken.PF);

		double orientationClothoDepart = sDepart * sDepart; // orientation au
															// départ
		if(!vitesse.positif)
			orientationClothoDepart = -orientationClothoDepart;

		double baseOrientation = orientation - orientationClothoDepart;
		double cos = Math.cos(baseOrientation);
		double sin = Math.sin(baseOrientation);

		// le premier point n'est pas position, mais le suivant
		// (afin de ne pas avoir de doublon quand on enchaîne les arcs, entre le
		// dernier point de l'arc t et le premier de l'arc t+1)
		for(int i = 0; i < NB_POINTS; i++)
		{
			sDepart += vitesse.squaredRootVitesse * PRECISION_TRACE;
			computePoint(pointDepart, vitesse, sDepart, coeffMultiplicatif, i, baseOrientation, cos, sin, marcheAvant, cinematiqueInitiale.getPosition(), modified.arcselems[i]);
		}

	}

	/**
	 * Construit un arc courbe dynamique qui ramène la courbure à 0
	 * 
	 * @param position
	 * @param orientation
	 * @param curvature
	 * @param vitesseTr
	 * @param modified
	 * @param enMarcheAvant
	 * @param vitesse
	 * @return
	 * @throws InterruptedException
	 */
	public final DynamicTentacle getTrajectoireRamene(Cinematique cinematiqueInitiale, StraightingTentacle vitesseRamene)
	{
		double courbure = cinematiqueInitiale.courbureGeometrique;
		double orientation = cinematiqueInitiale.orientationGeometrique;

		ClothoTentacle vitesse;
		if(courbure > 0)
			vitesse = vitesseRamene.vitesseDroite;
		else
			vitesse = vitesseRamene.vitesseGauche;

		boolean marcheAvant = cinematiqueInitiale.enMarcheAvant;

		double coeffMultiplicatif = 1. / vitesse.squaredRootVitesse;
		double sDepart = courbure / vitesse.squaredRootVitesse; // sDepart peut
																// parfaitement
																// être négatif
		if(!vitesse.positif)
			sDepart = -sDepart;
		int pointDepart = (int) ((sDepart / PRECISION_TRACE) + INDICE_MAX - 1 + 0.5); // le
																						// 0.5
																						// vient
																						// du
																						// fait
																						// qu'on
																						// fait
																						// ici
																						// un
																						// arrondi

		if(pointDepart < 0 || pointDepart >= trajectoire.length)
			log.write("Sorti de la clothoïde précalculée !", SeverityCategoryKraken.CRITICAL, LogCategoryKraken.PF);

		double orientationClothoDepart = sDepart * sDepart; // orientation au
															// départ
		if(!vitesse.positif)
			orientationClothoDepart = -orientationClothoDepart;

		double baseOrientation = orientation - orientationClothoDepart;
		double cos = Math.cos(baseOrientation);
		double sin = Math.sin(baseOrientation);

		// for(int i = 0; i < NB_POINTS; i++)
		// log.debug("Clotho : "+trajectoire[vitesse.squaredRootVitesse * (i +
		// 1)]);

		// le premier point n'est pas position, mais le suivant
		// (afin de ne pas avoir de doublon quand on enchaîne les arcs, entre le
		// dernier point de l'arc t et le premier de l'arc t+1)
		double sDepartPrecedent;
		int i = 0;
		List<CinematiqueObs> out = new ArrayList<CinematiqueObs>();
		while(true)
		{
			sDepartPrecedent = sDepart;
			sDepart += vitesse.squaredRootVitesse * PRECISION_TRACE;
			if(Math.abs(sDepart) > Math.abs(sDepartPrecedent)) // on vérifie la
																// courbure
				break;
			CinematiqueObs obs = memory.getNewNode();
			out.add(obs);
			computePoint(pointDepart, vitesse, sDepart, coeffMultiplicatif, i, baseOrientation, cos, sin, marcheAvant, cinematiqueInitiale.getPosition(), obs);
			i++;
		}

		if(out.isEmpty())
			return null;

		return new DynamicTentacle(out, vitesseRamene);
	}

	private XY_RW tmp = new XY_RW();
	
	/**
	 * Calcul un point à partir de ces quelques paramètres
	 * 
	 * @param pointDepart : l'indice du point de depart dans trajectoire[]
	 * @param vitesse : la vitesse de courbure
	 * @param sDepart : la valeur de "s" au point de départ
	 * @param coeffMultiplicatif : issu de la vitesse de courbure
	 * @param i : quel point dans la trajectoire
	 * @param baseOrientation : l'orientation au début du mouvement
	 * @param cos : le cos de baseOrientation
	 * @param sin : son sin
	 * @param marcheAvant : si le trajet est fait en marche avant
	 * @param vitesseTr : la vitesse translatoire souhaitée
	 * @param positionInitiale : la position au début du mouvement
	 * @param c : la cinématique modifiée
	 */
	private void computePoint(int pointDepart, ClothoTentacle vitesse, double sDepart, double coeffMultiplicatif, int i, double baseOrientation, double cos, double sin, boolean marcheAvant, XY positionInitiale, CinematiqueObs c)
	{
		trajectoire[pointDepart + vitesse.squaredRootVitesse * (i + 1)].copy(tmp);
		tmp.minus(trajectoire[pointDepart]).scalar(coeffMultiplicatif).Ysym(!vitesse.positif).rotate(cos, sin).plus(positionInitiale);
		
		double orientationClotho = sDepart * sDepart;
		if(!vitesse.positif)
			orientationClotho = -orientationClotho;

		double courbure = sDepart * vitesse.squaredRootVitesse;

		if(!vitesse.positif)
			courbure = -courbure;

		c.update(tmp.getX(), tmp.getY(), baseOrientation + orientationClotho, marcheAvant, courbure, rootedMaxAcceleration);
		c.maxSpeed = Math.min(c.maxSpeed, vitesse.maxSpeed);
	}

	private XY_RW delta = new XY_RW();
	private XY_RW centreCercle = new XY_RW();

	/**
	 * Calcule la trajectoire dans le cas particulier d'une trajectoire
	 * circulaire
	 * 
	 * @param position
	 * @param orientation
	 * @param courbure
	 * @param modified
	 */
	private void getTrajectoireCirculaire(XY position, double orientation, double courbure, StaticTentacle modified, boolean enMarcheAvant)
	{
		// log.debug("Trajectoire circulaire !");
		// rappel = la courbure est l'inverse du rayon de courbure
		// le facteur 1000 vient du fait que la courbure est en mètre^-1
		double rayonCourbure = 1000. / courbure;
		delta.setX(Math.cos(orientation + Math.PI / 2) * rayonCourbure);
		delta.setY(Math.sin(orientation + Math.PI / 2) * rayonCourbure);

		centreCercle.setX(position.getX() + delta.getX());
		centreCercle.setY(position.getY() + delta.getY());

		double angle = PRECISION_TRACE * courbure; // périmètre = angle * rayon

		double cos = Math.cos(angle); // l'angle de rotation autour du cercle
										// est le même que l'angle dont le robot
										// tourne
		double sin = Math.sin(angle);

		for(int i = 0; i < NB_POINTS; i++)
		{
			delta.rotate(cos, sin);
			centreCercle.copy(tmp);
			tmp.minus(delta);			
			modified.arcselems[i].update(tmp.getX(), tmp.getY(), orientation + angle * (i + 1), enMarcheAvant, courbure, rootedMaxAcceleration);
		}
	}

	/**
	 * Calcule la trajectoire dans le cas particulier d'une ligne droite
	 * 
	 * @param position
	 * @param orientation
	 * @param modified
	 */
	private void getTrajectoireLigneDroite(XY position, double orientation, StaticTentacle modified, boolean enMarcheAvant)
	{
		double cos = Math.cos(orientation);
		double sin = Math.sin(orientation);

		for(int i = 0; i < NB_POINTS; i++)
		{
			double distance = (i + 1) * PRECISION_TRACE_MM;
			tmp.setX(position.getX() + distance * cos);
			tmp.setY(position.getY() + distance * sin);
			modified.arcselems[i].update(tmp.getX(), tmp.getY(), orientation, enMarcheAvant, 0, rootedMaxAcceleration);
		}
	}

	/**
	 * Sauvegarde les points de la clothoïde unitaire
	 */
	private void sauvegardePoints()
	{
		log.write("Sauvegarde des points de la clothoïde unitaire", LogCategoryKraken.PF);
		try
		{
			FileOutputStream fichier;
			ObjectOutputStream oos;

			new File("clotho-" + S_MAX + ".dat").createNewFile();
			fichier = new FileOutputStream("clotho-" + S_MAX + ".krk");
			oos = new ObjectOutputStream(fichier);
			oos.writeObject(trajectoire);
			oos.flush();
			oos.close();
		}
		catch(IOException e)
		{
			log.write("Erreur lors de la sauvegarde des points de la clothoïde ! " + e, SeverityCategoryKraken.CRITICAL, LogCategoryKraken.PF);
		}
	}

	/**
	 * Chargement des points de la clothoïde unitaire
	 * 
	 * @return
	 */
	private boolean chargePoints()
	{
		log.write("Clothoid points loaded.",LogCategoryKraken.PF);
		try
		{
			InputStream fichier = getClass().getResourceAsStream("/clotho-"+S_MAX+".krk");
			ObjectInputStream ois = new ObjectInputStream(fichier);
			trajectoire = (XY[]) ois.readObject();
			ois.close();
			return true;
		}
		catch(IOException | ClassNotFoundException | NullPointerException e)
		{
			try {
				FileInputStream fichier = new FileInputStream("clotho-" + S_MAX + ".krk");
				ObjectInputStream ois = new ObjectInputStream(fichier);
				trajectoire = (XY[]) ois.readObject();
				ois.close();
				return true;
			}
			catch(IOException | ClassNotFoundException e1)
			{
				log.write("Chargement échoué ! "+e1.getMessage(), SeverityCategoryKraken.CRITICAL, LogCategoryKraken.PF);
			}
		}
		return false;
	}
/*
	private XY_RW vecteurOrientationDepart = new XY_RW();
	private XY_RW vecteurOrientationDepartRotate = new XY_RW();
	private XY_RW vecteurOrientation = new XY_RW();
*/
	/**
	 * Construit un arc courbe qui fait faire un demi-tour au robot
	 * 
	 * @param cinematiqueInitiale
	 * @param vitesse
	 * @param vitesseMax
	 * @return
	 * @throws InterruptedException
	 */
/*	public final DynamicTentacle getTrajectoireDemiTour(Cinematique cinematiqueInitiale, TurnoverTentacle vitesse)
	{
		List<CinematiqueObs> trajet = getTrajectoireQuartDeTour(cinematiqueInitiale, vitesse.v, false);
		trajet.addAll(getTrajectoireQuartDeTour(trajet.get(trajet.size() - 1), vitesse.v, true)); // on
																									// reprend
																									// à
																									// la
																									// fin
																									// du
																									// premier
																									// quart
																									// de
																									// tour
		return new DynamicTentacle(trajet, vitesse); // TODO :
																							// rebrousse
																							// est
																							// faux…
	}*/

	/**
	 * Construit un arc courbe qui fait un quart de tour au robot
	 * 
	 * @param position
	 * @param orientation
	 * @param curvature
	 * @param vitesseTr
	 * @param modified
	 * @param enMarcheAvant
	 * @param vitesse
	 * @return
	 * @throws InterruptedException
	 */
/*	private final List<CinematiqueObs> getTrajectoireQuartDeTour(Cinematique cinematiqueInitiale, ClothoTentacle vitesse, boolean rebrousse)
	{
		double courbure = cinematiqueInitiale.courbureGeometrique;
		double orientation = cinematiqueInitiale.orientationGeometrique;
		if(rebrousse)
		{
			courbure = 0;
			orientation += Math.PI;
		}

		vecteurOrientationDepart.setX(Math.cos(orientation));
		vecteurOrientationDepart.setY(Math.sin(orientation));
		vecteurOrientationDepart.copy(vecteurOrientationDepartRotate);
		if(vitesse.positif)
			vecteurOrientationDepartRotate.rotate(0, 1);
		else
			vecteurOrientationDepartRotate.rotate(0, -1);

		boolean marcheAvant = rebrousse ^ cinematiqueInitiale.enMarcheAvant;

		double coeffMultiplicatif = 1. / vitesse.squaredRootVitesse;
		double sDepart = courbure / vitesse.squaredRootVitesse; // sDepart peut
																// parfaitement
																// être négatif
		if(!vitesse.positif)
			sDepart = -sDepart;
		int pointDepart = (int) ((sDepart / PRECISION_TRACE) + INDICE_MAX - 1 + 0.5); // le
																						// 0.5
																						// vient
																						// du
																						// fait
																						// qu'on
																						// fait
																						// ici
																						// un
																						// arrondi

		// TODO
		assert pointDepart >= 0 && pointDepart < trajectoire.length;
//		if(pointDepart < 0 || pointDepart >= trajectoire.length)
//			log.critical("Sorti de la clothoïde précalculée !", SeverityCategoryKraken.CRITICAL, LogCategoryKraken.PF);

		double orientationClothoDepart = sDepart * sDepart; // orientation au
															// départ
		if(!vitesse.positif)
			orientationClothoDepart = -orientationClothoDepart;

		double baseOrientation = orientation - orientationClothoDepart;
		double cos = Math.cos(baseOrientation);
		double sin = Math.sin(baseOrientation);

		// le premier point n'est pas position, mais le suivant
		// (afin de ne pas avoir de doublon quand on enchaîne les arcs, entre le
		// dernier point de l'arc t et le premier de l'arc t+1)
		int i = 0;

		List<CinematiqueObs> out = new ArrayList<CinematiqueObs>();
		XY positionInit = cinematiqueInitiale.getPosition();
		do
		{
			sDepart += vitesse.squaredRootVitesse * PRECISION_TRACE;
			CinematiqueObs obs = memory.getNewNode();
			out.add(obs);
			computePoint(pointDepart, vitesse, sDepart, coeffMultiplicatif, i, baseOrientation, cos, sin, marcheAvant, positionInit, obs);
			vecteurOrientation.setX(Math.cos(obs.orientationGeometrique));
			vecteurOrientation.setY(Math.sin(obs.orientationGeometrique));
			i++;
		} while(vecteurOrientation.dot(vecteurOrientationDepart) >= 0 || vecteurOrientation.dot(vecteurOrientationDepartRotate) <= 0);
		return out;
	}*/

	@Override
	public boolean compute(AStarNode current, TentacleType tentacleType, Cinematique arrival, AStarNode modified)
	{
		assert tentacleType instanceof ClothoTentacle : tentacleType;
		
		// si le robot est arrêté (début de trajectoire), et que la vitesse
		// n'est pas prévue pour un arrêt ou un rebroussement, on annule
		// cette idée, plus appliquée, permettait d'avoir les premiers centimètres de la trajectoire à courbure constante
//		if(current.getArc() == null && (!((ClothoTentacle) tentacleType).arret && !((ClothoTentacle) tentacleType).rebrousse))
//			return false;

		getTrajectoire(current.robot.getCinematique(), (ClothoTentacle) tentacleType, modified.cameFromArcStatique);
		return true;
	}

}
