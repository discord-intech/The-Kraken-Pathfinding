/*
 * Copyright (C) 2013-2017 Pierre-François Gimenez
 * Distributed under the MIT License.
 */

package pfg.kraken.astar.tentacles;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import jcuda.CudaException;
import jcuda.Pointer;
import jcuda.Sizeof;
import jcuda.driver.*;
import pfg.config.Config;
import pfg.injector.Injector;
import pfg.injector.InjectorException;
import pfg.kraken.ConfigInfoKraken;
import pfg.kraken.LogCategoryKraken;
import pfg.kraken.SeverityCategoryKraken;
import pfg.kraken.astar.AStarNode;
import pfg.kraken.astar.DirectionStrategy;
import pfg.kraken.astar.tentacles.types.ClothoTentacle;
import pfg.kraken.astar.tentacles.types.TentacleType;
import pfg.kraken.dstarlite.DStarLite;
import pfg.kraken.memory.NodePool;
import pfg.kraken.obstacles.Obstacle;
import pfg.kraken.obstacles.container.DynamicObstacles;
import pfg.kraken.obstacles.container.StaticObstacles;
import pfg.kraken.robot.Cinematique;
import pfg.kraken.utils.XY;
import pfg.graphic.log.Log;

import static jcuda.driver.JCudaDriver.*;
import static pfg.kraken.astar.tentacles.Tentacle.*;

/**
 * Réalise des calculs pour l'A* courbe.
 *
 * @author pf
 *
 */

public class TentacleManager implements Iterable<AStarNode>
{
    protected Log log;
    private DStarLite dstarlite;
    private DynamicObstacles dynamicObs;
    private double courbureMax, maxLinearAcceleration, vitesseMax;
    private int tempsArret;
    private Injector injector;
    private StaticObstacles fixes;
    private NodePool memorymanager;
    private double deltaSpeedFromStop;

    private DirectionStrategy directionstrategyactuelle;
    private Cinematique arrivee = new Cinematique();
    //	private ResearchProfileManager profiles;
    private List<TentacleType> currentProfile = new ArrayList<TentacleType>();
    private Iterator<AStarNode> successeursIter;
    private List<AStarNode> successeurs = new ArrayList<AStarNode>();
//	private List<StaticObstacles> disabledObstaclesFixes = new ArrayList<StaticObstacles>();

    private boolean forceCPUFallback = false; // Used if CUDA or OCL init failed
    private ArrayList[] clothoidTentacleData = new ArrayList[7];

    public TentacleManager(Log log, StaticObstacles fixes, DStarLite dstarlite, Config config, DynamicObstacles dynamicObs, Injector injector, ResearchProfileManager profiles, NodePool memorymanager) throws InjectorException
    {
        this.injector = injector;
        this.fixes = fixes;
        this.dynamicObs = dynamicObs;
        this.log = log;
        this.dstarlite = dstarlite;
        this.memorymanager = memorymanager;

        this.currentProfile = profiles.getProfile(0);
        for(TentacleType t : currentProfile)
            injector.getService(t.getComputer());

        courbureMax = config.getDouble(ConfigInfoKraken.MAX_CURVATURE);
        maxLinearAcceleration = config.getDouble(ConfigInfoKraken.MAX_LINEAR_ACCELERATION);
        deltaSpeedFromStop = Math.sqrt(2 * PRECISION_TRACE * maxLinearAcceleration);
        tempsArret = config.getInt(ConfigInfoKraken.STOP_DURATION);
        coins[0] = fixes.getBottomLeftCorner();
        coins[2] = fixes.getTopRightCorner();
        coins[1] = new XY(coins[0].getX(), coins[2].getY());
        coins[3] = new XY(coins[2].getX(), coins[0].getY());

        clothoidTentacleData[0] = new ArrayList<Integer>();
        clothoidTentacleData[1] = new ArrayList<Float>();
        clothoidTentacleData[2] = new ArrayList<Float>();
        clothoidTentacleData[3] = new ArrayList<Short>();
        clothoidTentacleData[4] = new ArrayList<Short>();
        clothoidTentacleData[5] = new ArrayList<Short>();
        clothoidTentacleData[6] = new ArrayList<TentacleType>();
    }

    private XY[] coins = new XY[4];

    /**
     * Retourne faux si un obstacle est sur la route
     *
     * @param node
     * @return
     * @throws FinMatchException
     */
    public boolean isReachable(AStarNode node)
    {
        // le tout premier nœud n'a pas de parent
        if(node.parent == null)
            return true;

        int nbOmbres = node.getArc().getNbPoints();

        // On vérifie la collision avec les murs
        for(int j = 0; j < nbOmbres; j++)
            for(int i = 0; i < 4; i++)
                if(node.getArc().getPoint(j).obstacle.isColliding(coins[i], coins[(i+1)&3]))
                    return false;

        // Collision avec un obstacle fixe?
        for(Obstacle o : fixes.getObstacles())
            for(int i = 0; i < nbOmbres; i++)
                if(/*!disabledObstaclesFixes.contains(o) && */o.isColliding(node.getArc().getPoint(i).obstacle))
                {
                    // log.debug("Collision avec "+o);
                    return false;
                }

        // Collision avec un obstacle de proximité ?

        try {
            Iterator<Obstacle> iter = dynamicObs.getFutureDynamicObstacles(0); // TODO date !
            while(iter.hasNext())
            {
                Obstacle n = iter.next();
                for(int i = 0; i < nbOmbres; i++)
                    if(n.isColliding(node.getArc().getPoint(i).obstacle))
                    {
                        // log.debug("Collision avec un obstacle de proximité.");
                        return false;
                    }
            }
        } catch(NullPointerException e)
        {
            log.write(e.toString(), SeverityCategoryKraken.CRITICAL, LogCategoryKraken.PF);
        }
		/*
		 * node.state.iterator.reinit();
		 * while(node.state.iterator.hasNext())
		 * if(node.state.iterator.next().isColliding(obs))
		 * {
		 * // log.debug("Collision avec un obstacle de proximité.");
		 * return false;
		 * }
		 */

        return true;
    }

    /**
     * Initialise l'arc manager avec les infos donnée
     *
     * @param directionstrategyactuelle
     * @param sens
     * @param arrivee
     */
    public void configure(DirectionStrategy directionstrategyactuelle, double vitesseMax, Cinematique arrivee)
    {
        this.vitesseMax = vitesseMax;
        this.directionstrategyactuelle = directionstrategyactuelle;
        arrivee.copy(this.arrivee);
    }

    public void configure(DirectionStrategy directionstrategyactuelle, double vitesseMax, XY arrivee)
    {
        this.vitesseMax = vitesseMax;
        this.directionstrategyactuelle = directionstrategyactuelle;
        this.arrivee.updateReel(arrivee.getX(), arrivee.getY(), 0, true, 0);
    }

    /**
     * Réinitialise l'itérateur à partir d'un nouvel état
     *
     * @param current
     * @param directionstrategyactuelle
     */
    public void computeTentacles(AStarNode current)
    {
        successeurs.clear();
        Config config = injector.getExistingService(Config.class);

        try
        {
            //TODO CPU
            for(TentacleType v : currentProfile)
            {
                long time = System.currentTimeMillis();


                if(!forceCPUFallback && v instanceof ClothoTentacle && (config.getString(ConfigInfoKraken.COMPUTE_MODE).equals("CUDA")
                        || config.getString(ConfigInfoKraken.COMPUTE_MODE).equals("OCL")))
                {
                    if(v.isAcceptable(current.robot.getCinematique(), directionstrategyactuelle, courbureMax))
                    {
                        clothoidTentacleData[0].add(((ClothoTentacle) v).vitesse);
                        clothoidTentacleData[1].add((float)((ClothoTentacle) v).squaredRootVitesse);
                        clothoidTentacleData[2].add((float)((ClothoTentacle) v).courbureInitiale);
                        clothoidTentacleData[3].add((short)(((ClothoTentacle) v).positif ? 1 : 0));
                        clothoidTentacleData[4].add((short)(((ClothoTentacle) v).rebrousse ? 1 : 0));
                        clothoidTentacleData[5].add((short)(((ClothoTentacle) v).arret ? 1 : 0));
                        clothoidTentacleData[6].add(v);
                    }
                }
                else
                {
                    //TODO CPU
                    if(v.isAcceptable(current.robot.getCinematique(), directionstrategyactuelle, courbureMax))
                    {
                        AStarNode successeur = memorymanager.getNewNode();
//				assert successeur.cameFromArcDynamique == null;
                        successeur.cameFromArcDynamique = null;
                        successeur.parent = current;

                        current.robot.copy(successeur.robot);
                        if(injector.getExistingService(v.getComputer()).compute(current, v, arrivee, successeur))
                        {
                            // Compute the travel time
                            double duration = successeur.getArc().getDuree(successeur.parent.getArc(), vitesseMax, tempsArret, maxLinearAcceleration, deltaSpeedFromStop);
                            successeur.robot.suitArcCourbe(successeur.getArc(), duration);
                            successeur.g_score = duration;
                            successeurs.add(successeur);
                        }
                    }
                }

                if(v instanceof ClothoTentacle && System.currentTimeMillis()-time > 2)
                    System.out.println("L :"+(System.currentTimeMillis()-time));
            }

            if(!forceCPUFallback && !clothoidTentacleData[0].isEmpty() && config.getString(ConfigInfoKraken.COMPUTE_MODE).equals("CUDA"))
            {

                AStarNode[] toCompute = new AStarNode[clothoidTentacleData[0].size()];

                toCompute = computeClothoTentaclesCUDA(current, config, toCompute);

                for(AStarNode successeur : toCompute)
                {
                    // Compute the travel time
                    double duration = successeur.getArc().getDuree(successeur.parent.getArc(), vitesseMax, tempsArret, maxLinearAcceleration, deltaSpeedFromStop);
                    successeur.robot.suitArcCourbe(successeur.getArc(), duration);
                    successeur.g_score = duration;
                    successeurs.add(successeur);
                }
            }

        }
        catch (CudaException e)
        {
            e.printStackTrace();
            forceCPUFallback = true;
            computeTentacles(current);
        }

        successeursIter = successeurs.iterator();
    }

    public synchronized Double heuristicCostCourbe(Cinematique c)
    {
        if(dstarlite.heuristicCostCourbe(c) == null)
            return null;
        return dstarlite.heuristicCostCourbe(c) / vitesseMax;
    }

    public boolean isArrived(AStarNode successeur)
    {
        return successeur.getArc() != null && successeur.getArc().getLast().getPosition().squaredDistance(arrivee.getPosition()) < 5;
    }

    private AStarNode[] computeClothoTentaclesCUDA(AStarNode current, Config config, AStarNode[] toCompute) throws CudaException
    {
        //TODO CUDA
        cuInit(0);
        CUdevice gc = new CUdevice();
        cuDeviceGet(gc, 0);
        CUcontext context = new CUcontext();
        cuCtxCreate(context, 0, gc);

        CUmodule module = new CUmodule();
        cuModuleLoad(module, config.getString(ConfigInfoKraken.CUDA_CLOTHO_PTX));
        CUfunction function = new CUfunction();
        cuModuleGetFunction(function, module, "kernelFunc");

        ClothoidesComputer comp = injector.getExistingService(ClothoidesComputer.class);

        float[] xUnitary = new float[comp.getTrajectoirePoints().length];
        float[] yUnitary = new float[comp.getTrajectoirePoints().length];
        int[] tentacleSpeed = new int[clothoidTentacleData[0].size()];
        float[] tentacleSquaredRootSpeed = new float[clothoidTentacleData[0].size()];
        float[] tentacleInitialCurvature = new float[clothoidTentacleData[0].size()];
        short[] tentaclePositive = new short[clothoidTentacleData[0].size()];
        short[] tentacleBack = new short[clothoidTentacleData[0].size()];
        short[] tentacleStop = new short[clothoidTentacleData[0].size()];
        float[] xRobot = new float[]{(float) current.robot.getCinematique().getPosition().getX()};
        float[] yRobot = new float[]{(float) current.robot.getCinematique().getPosition().getY()};
        float[] orientationRobot = new float[]{(float) current.robot.getCinematique().orientationGeometrique};
        float[] curvatureRobot = new float[]{(float) current.robot.getCinematique().courbureGeometrique};
        short[] goingForwardRobot = new short[]{(short)(current.robot.getCinematique().enMarcheAvant ? 1 : 0)};

        float[] xOutput = new float[NB_POINTS * tentacleSpeed.length];
        float[] yOutput = new float[NB_POINTS * tentacleSpeed.length];
        float[] orientationOutput = new float[NB_POINTS * tentacleSpeed.length];
        float[] curvatureOutput = new float[NB_POINTS * tentacleSpeed.length];
        short[] goingForwardOutput = new short[NB_POINTS * tentacleSpeed.length];

        for(int i = 0; i < comp.getTrajectoirePoints().length ; i++)
        {
            xUnitary[i] = (float) comp.getTrajectoirePoints()[i].getX();
            yUnitary[i] = (float) comp.getTrajectoirePoints()[i].getY();
        }

        for(int i=0 ; i < clothoidTentacleData[0].size() ; i++)
        {
            tentacleSpeed[i] = (Integer) clothoidTentacleData[0].get(i);
            tentacleSquaredRootSpeed[i] = (Float) clothoidTentacleData[1].get(i);
            tentacleInitialCurvature[i] = (Float) clothoidTentacleData[2].get(i);
            tentaclePositive[i] = (Short) clothoidTentacleData[3].get(i);
            tentacleBack[i] = (Short) clothoidTentacleData[4].get(i);
            tentacleStop[i] = (Short) clothoidTentacleData[5].get(i);
        }

        CUdeviceptr xUnitaryD = new CUdeviceptr();
        cuMemAlloc(xUnitaryD, xUnitary.length * Sizeof.FLOAT);
        cuMemcpyHtoD(xUnitaryD, Pointer.to(xUnitary), xUnitary.length * Sizeof.FLOAT);

        CUdeviceptr yUnitaryD = new CUdeviceptr();
        cuMemAlloc(yUnitaryD, yUnitary.length * Sizeof.FLOAT);
        cuMemcpyHtoD(yUnitaryD, Pointer.to(yUnitary), yUnitary.length * Sizeof.FLOAT);

        CUdeviceptr tentacleSpeedD = new CUdeviceptr();
        cuMemAlloc(tentacleSpeedD, Sizeof.INT);
        cuMemcpyHtoD(tentacleSpeedD, Pointer.to(tentacleSpeed), tentacleSpeed.length * Sizeof.INT);

        CUdeviceptr tentacleSquaredRootSpeedD = new CUdeviceptr();
        cuMemAlloc(tentacleSquaredRootSpeedD, Sizeof.FLOAT);
        cuMemcpyHtoD(tentacleSquaredRootSpeedD, Pointer.to(tentacleSquaredRootSpeed), tentacleSquaredRootSpeed.length * Sizeof.FLOAT);

        CUdeviceptr tentacleInitialCurvatureD = new CUdeviceptr();
        cuMemAlloc(tentacleInitialCurvatureD, Sizeof.FLOAT);
        cuMemcpyHtoD(tentacleInitialCurvatureD, Pointer.to(tentacleInitialCurvature), tentacleInitialCurvature.length * Sizeof.FLOAT);

        CUdeviceptr tentaclePositiveD = new CUdeviceptr();
        cuMemAlloc(tentaclePositiveD, Sizeof.SHORT);
        cuMemcpyHtoD(tentaclePositiveD, Pointer.to(tentaclePositive), tentaclePositive.length * Sizeof.SHORT);

        CUdeviceptr tentacleBackD = new CUdeviceptr();
        cuMemAlloc(tentacleBackD, Sizeof.SHORT);
        cuMemcpyHtoD(tentacleBackD, Pointer.to(tentacleBack), tentacleBack.length * Sizeof.SHORT);

        CUdeviceptr tentacleStopD = new CUdeviceptr();
        cuMemAlloc(tentacleStopD, Sizeof.SHORT);
        cuMemcpyHtoD(tentacleStopD, Pointer.to(tentacleStop), tentacleStop.length * Sizeof.SHORT);

        CUdeviceptr xRobotD = new CUdeviceptr();
        cuMemAlloc(xRobotD, Sizeof.FLOAT);
        cuMemcpyHtoD(xRobotD, Pointer.to(xRobot), Sizeof.FLOAT);

        CUdeviceptr yRobotD = new CUdeviceptr();
        cuMemAlloc(yRobotD, Sizeof.FLOAT);
        cuMemcpyHtoD(yRobotD, Pointer.to(yRobot), Sizeof.FLOAT);

        CUdeviceptr orientationRobotD = new CUdeviceptr();
        cuMemAlloc(orientationRobotD, Sizeof.FLOAT);
        cuMemcpyHtoD(orientationRobotD, Pointer.to(orientationRobot), Sizeof.FLOAT);

        CUdeviceptr curvatureRobotD = new CUdeviceptr();
        cuMemAlloc(curvatureRobotD, Sizeof.FLOAT);
        cuMemcpyHtoD(curvatureRobotD, Pointer.to(curvatureRobot), Sizeof.FLOAT);

        CUdeviceptr goingForwardRobotD = new CUdeviceptr();
        cuMemAlloc(goingForwardRobotD, Sizeof.SHORT);
        cuMemcpyHtoD(goingForwardRobotD, Pointer.to(goingForwardRobot), Sizeof.SHORT);


        CUdeviceptr xOutputD = new CUdeviceptr();
        cuMemAlloc(xOutputD, xOutput.length * Sizeof.FLOAT);

        CUdeviceptr yOutputD = new CUdeviceptr();
        cuMemAlloc(yOutputD, yOutput.length * Sizeof.FLOAT);

        CUdeviceptr orientationOutputD = new CUdeviceptr();
        cuMemAlloc(orientationOutputD, orientationOutput.length * Sizeof.FLOAT);

        CUdeviceptr curvatureOutputD = new CUdeviceptr();
        cuMemAlloc(curvatureOutputD, curvatureOutput.length * Sizeof.FLOAT);

        CUdeviceptr goingForwardOutputD = new CUdeviceptr();
        cuMemAlloc(goingForwardOutputD, goingForwardOutput.length * Sizeof.SHORT);

        Pointer kernelParams = Pointer.to(
                Pointer.to(xUnitaryD),
                Pointer.to(yUnitaryD),
                Pointer.to(tentacleSpeedD),
                Pointer.to(tentacleSquaredRootSpeedD),
                Pointer.to(tentacleInitialCurvatureD),
                Pointer.to(tentaclePositiveD),
                Pointer.to(tentacleBackD),
                Pointer.to(tentacleStopD),
                Pointer.to(xRobotD),
                Pointer.to(yRobotD),
                Pointer.to(orientationRobotD),
                Pointer.to(curvatureRobotD),
                Pointer.to(goingForwardRobotD),
                Pointer.to(xOutputD),
                Pointer.to(yOutputD),
                Pointer.to(orientationOutputD),
                Pointer.to(curvatureOutputD),
                Pointer.to(goingForwardOutputD)
        );

        cuLaunchKernel(function,
                tentacleSpeed.length, NB_POINTS, 1,
                1,1,1,
                0, null,
                kernelParams, null);

        cuCtxSynchronize();

        cuMemcpyDtoH(Pointer.to(xOutput), xOutputD, xOutput.length * Sizeof.FLOAT);
        cuMemcpyDtoH(Pointer.to(yOutput), yOutputD, yOutput.length * Sizeof.FLOAT);
        cuMemcpyDtoH(Pointer.to(orientationOutput), orientationOutputD, orientationOutput.length * Sizeof.FLOAT);
        cuMemcpyDtoH(Pointer.to(curvatureOutput), curvatureOutputD, curvatureOutput.length * Sizeof.FLOAT);
        cuMemcpyDtoH(Pointer.to(goingForwardOutput), goingForwardOutputD, goingForwardOutput.length * Sizeof.SHORT);

        cuMemFree(xUnitaryD);
        cuMemFree(yUnitaryD);
        cuMemFree(tentacleSpeedD);
        cuMemFree(tentacleSquaredRootSpeedD);
        cuMemFree(tentacleInitialCurvatureD);
        cuMemFree(tentaclePositiveD);
        cuMemFree(tentacleBackD);
        cuMemFree(tentacleStopD);
        cuMemFree(xRobotD);
        cuMemFree(yRobotD);
        cuMemFree(orientationRobotD);
        cuMemFree(curvatureRobotD);
        cuMemFree(goingForwardRobotD);
        cuMemFree(xOutputD);
        cuMemFree(yOutputD);
        cuMemFree(orientationOutputD);
        cuMemFree(curvatureOutputD);
        cuMemFree(goingForwardOutputD);

        float RmaxAccel = (float)Math.sqrt(config.getDouble(ConfigInfoKraken.MAX_LATERAL_ACCELERATION));

        for(int j=0 ; j<toCompute.length ; j++)
        {
            AStarNode successeur = memorymanager.getNewNode();
            successeur.cameFromArcStatique.vitesse = (ClothoTentacle) clothoidTentacleData[6].get(j);
            successeur.cameFromArcDynamique = null;
            successeur.parent = current;

            current.robot.copy(successeur.robot);

            for(int i=0 ; i<NB_POINTS ; i++)
            {
                successeur.cameFromArcStatique.arcselems[i].update(xOutput[j * NB_POINTS + i], yOutput[j * NB_POINTS + i], orientationOutput[j * NB_POINTS + i], goingForwardOutput[j * NB_POINTS + i]!=0, curvatureOutput[j * NB_POINTS + i], RmaxAccel);
                successeur.cameFromArcStatique.arcselems[i].maxSpeed = Math.min(successeur.cameFromArcStatique.arcselems[i].maxSpeed, ((ClothoTentacle)clothoidTentacleData[6].get(j)).maxSpeed);
            }

            toCompute[j] = successeur;
        }

        return toCompute;

    }

    @Override
    public Iterator<AStarNode> iterator()
    {
        return successeursIter;
    }
}
