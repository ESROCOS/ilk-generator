package eu.esrocos.kul.robot.generator

import java.util.ArrayList
import java.util.List
import java.util.HashSet

import eu.esrocos.kul.query.queryLang.KinematicsQuery

import eu.esrocos.kul.robot.kinDsl.Robot
import eu.esrocos.kul.robot.generator.common.TreeUtils
import eu.esrocos.kul.robot.generator.common.gr.RobotPoseUtils
import eu.esrocos.kul.robot.generator.common.JacUtils

class RobotUtils
{
    public new(Robot rob)
    {
        this.robot = rob
        this.treeUtils = new TreeUtils(robot)
        this.poseUtils = new RobotPoseUtils(robot, treeUtils)
        this.jacsUtils = new JacUtils(treeUtils, poseUtils)
    }

    public final Robot robot
    public final TreeUtils treeUtils
    public final RobotPoseUtils poseUtils
    public final JacUtils jacsUtils
}

class QueryProcessor
{
    public new(Robot rob, KinematicsQuery query)
    {
        robot = rob
        val rUtils = new RobotUtils(robot)

        if( !query.name.equals(robot.name) ) {
            throw new RuntimeException("This QueryProcessor was created for the robot " + robot.name)
        }

        val fkset = new HashSet<FKSolverSpecs>()

        fks = new ArrayList<FKSolverModel>()
        for( fk : query.fks)
        {
            val spec = new FKSolverSpecs(robot, fk)
            fkset.add( spec )
            fks.add( new FKSolverModel( fk.name, spec, rUtils ) )
        }

        iks = new ArrayList<IKSolverModel>()
        for( ik : query.iks )
        {
            val ikmodel = new IKSolverModel(robot, ik)
            iks.add( ikmodel )

            // Every IK solver requires in turn a specific FK solver, whose model
            // is therefore added to the pool as if it had been requested by the
            // user.

            if( fkset.add( ikmodel.requiredFK ) ) {
                // Add the FKSolverModel required by the IKSolver
                fks.add( new FKSolverModel( ikmodel.getFKSolverID(), ikmodel.requiredFK, rUtils ) )
            } else {
                // The FK solver required by IK is already in the pool (the set
                // did not add the element); let's find it so we can set its ID
                // into the IK model
                for( fk : fks ) {
                    if( fk.specs.equals( ikmodel.requiredFK ) ) {
                        ikmodel.setFKSolverID( fk.name )
                    }
                }
            }
        }
    }


    def public fksolvers() { return fks }
    def public iksolvers() { return iks }
/*
    def boom() {
//        val g = RobotFrameUtils::makeFramesGraph(robot)
//        for( v : g.vertexSet() ) {
//            System.out.println(v.frame.name);
//        }
        val l1 = robot.getLinkByName("link5")
        val l2 = robot.getLinkByName("link5")
        val lca = treeUtils.lowestCommonAncestor(l1 , l2)
        System.out.println( lca.name )
        val lca2 = TreeUtils::commonAncestor(l1, l2)
        System.out.println( lca2.name )
    }


    def public printMapByReference()
    {
        for( k : outputByRef.keySet ) {
            System.out.println("reference: " + k)
            for( poseinfo : outputByRef.get(k) ) {
                System.out.println( poseinfo.pose.target + " (" + poseinfo.framesPath.length + ")" )
            }
            System.out.println();
        }
    }
*/
    private Robot robot
    private List<FKSolverModel> fks
    private List<IKSolverModel> iks


}

