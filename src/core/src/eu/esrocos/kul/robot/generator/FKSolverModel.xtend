package eu.esrocos.kul.robot.generator

import eu.esrocos.kul.robot.generator.common.JacobianInfo
import eu.esrocos.kul.robot.generator.common.JacobianPlaceholder
import eu.esrocos.kul.robot.generator.common.gr.AttachedFrame
import eu.esrocos.kul.robot.generator.common.gr.FrameRelationKind
import eu.esrocos.kul.robot.generator.common.gr.PoseCompose
import eu.esrocos.kul.robot.generator.common.gr.RelativePose
import eu.esrocos.kul.robot.generator.common.gr.RobotFrameUtils
import eu.esrocos.kul.robot.generator.common.gr.RobotPoseUtils
import eu.esrocos.kul.robot.generator.common.gr.RobotPoseUtils.ConstantPose
import eu.esrocos.kul.robot.generator.common.gr.RobotPoseUtils.JointSuccessorPose
import eu.esrocos.kul.robot.generator.common.gr.RobotPoseUtils.PoseInfo
import eu.esrocos.kul.robot.kinDsl.Robot
import eu.esrocos.kul.query.queryLang.FK
import java.util.ArrayList
import java.util.Comparator
import java.util.HashMap
import java.util.HashSet
import java.util.List
import java.util.Map
import java.util.Objects
import java.util.Set
import java.util.TreeSet

class FKSolverSpecs
{
    public new(Robot rob, FK request)
    {
        this.robot = rob
        this.poses = new ArrayList<RelativePose>()
        this.jacs  = new ArrayList<JacobianPlaceholder>()
        for(jac : request.jacobians)
        {
            val target    = RobotFrameUtils::getFrameByName(robot, jac.frames.target.name)
            val reference = RobotFrameUtils::getFrameByName(robot, jac.frames.reference.name)
            if( target===null ) {
                throw new RuntimeException("Fatal: reference frame "+ jac.frames.target.name +" not found on the given robot")
            }
            if( reference===null) {
                throw new RuntimeException("Fatal: reference frame " + jac.frames.reference.name +" not found on the given robot")
            }
            jacs.add( new JacobianPlaceholder(target, reference) )
        }
        for(pose : request.poses)
        {
            val tName = pose.frames.target.name
            val rName = pose.frames.reference.name
            // Build the placeholders for the reference and target frame
            //
            val target    = RobotFrameUtils::getFrameByName(robot, tName)
            val reference = RobotFrameUtils::getFrameByName(robot, rName)
            if( target===null ) {
                throw new RuntimeException("Fatal: reference frame "+ tName +" not found on the given robot")
            }
            if( reference===null) {
                throw new RuntimeException("Fatal: reference frame " + rName +" not found on the given robot")
            }
            poses.add( new RelativePose(target, reference) )
        }
    }

    public new(Robot rob, List<RelativePose> p, List<JacobianPlaceholder> j)
    {
        robot = rob
        poses = p
        jacs  = j
    }

    public override hashCode() {
        return Objects.hash(robot, poses, jacs)
    }

    public override equals(Object _rhs) {
        val rhs = _rhs as FKSolverSpecs
        var boolean equal = robot.equals(rhs.robot)
        equal = equal && (poses.equals(rhs.poses))
        equal = equal && (jacs.equals(rhs.jacs))
        return equal
    }

    public final Robot robot
    public final List<RelativePose>       poses
    public final List<JacobianPlaceholder> jacs
    //TODO add the velocities
}

class FKSolverModel
{
    public new(String name, FKSolverSpecs specs)
    {
        this(name, specs, new RobotUtils(specs.robot))
    }

    public new(String name, FKSolverSpecs specs, RobotUtils utils)
    {
        if( ! specs.robot.equals(utils.robot) ) {
            throw new RuntimeException("Inconsistent arguments")
        }
        this.userName = name
        this.robotu = utils
        this.mySpecs= specs

        reset()
        load( specs.poses, specs.jacs )
    }
    /**
     *
     */
    def private load(List<RelativePose> poses, List<JacobianPlaceholder> jacs)
    {
        // The metadata for each relative pose
        val info = new ArrayList<RobotPoseUtils.PoseInfo>

        // The requested relative poses
        for( pose : poses ) {
            if( outPoses.add(pose) ) {
                info.add( robotu.poseUtils.buildPoseInfo(pose) )
            }
        }

        // The requested Jacobians:
        val jutils   = robotu.jacsUtils
        val localpool = new HashSet<RelativePose>()

        for( J : jacs ) {
            val jinfo = jutils.buildInfo(J)
            if( outJacs.add( jinfo ) ) {
                localpool.addAll( jutils.getRequiredPoses( jinfo ) )
            }
        }
        for( pose : localpool ) {
            info.add( robotu.poseUtils.buildPoseInfo(pose) )
        }

        // We now have the complete list of PoseInfo, load all the internals
        populateMapByReference( info )
        addModelConstantPoses( info )
        //addParentChildComposes( info )
        addAllComposes()
    }

    /**
     *
     */
    def private addModelConstantPoses(List<RobotPoseUtils.PoseInfo> desires)
    {
        for( info : desires)
        {
            // Find all the distance-1 frame pairs which will be referenced
            // afterwards in the compositions.
            val path = info.framesPath
            if( path.length > 0)
            {
                val vertices = path.vertexList     // these are AttachedFrame(s)
                val vi1 = vertices.listIterator(0)
                val vi2 = vertices.listIterator(1)
                val ei  = path.edgeList.iterator // can be used to tag the pose with its kind (geometrical constant of joint dependent)
                do {
                    val v1a = vi1.next  // a AttachedFrame
                    val v2a = vi2.next
                    ////System.out.println( v1a.frame().name() + " " + v2a.frame().name() )
                    val poseKind = ei.next.getKind()
                    if( notThereYet(v1a, v2a) )
                    {
                        if( poseKind == FrameRelationKind.jointPredecessor ||
                            poseKind == FrameRelationKind.user )
                        {
                            // in this case the relative pose is a constant
                            constPoses.add( robotu.poseUtils.getConstantPose(v1a, v2a) )
                        } else {
                            // poseKind must be FrameRelationKind.jointSuccessor
                            jointPoses.add( robotu.poseUtils.getJointSuccessorPose(v1a, v2a) )
                        }
                    }
                } while(vi2.hasNext)
            }
        }
    }


    /**
     *
     */
    def private addAllComposes()
    {
        for( reference : outputByRef.keySet ) {
            var RelativePose last = null
            val pinfos = outputByRef.get( reference )
            for( pinfo : pinfos ) {
                if( last === null) {
                    iterativeCompose( pinfo.pose, pinfo )
                    last = pinfo.pose
                } else {
                    val newpose = new RelativePose( pinfo.pose.target, last.target)
                    val newinfo = robotu.poseUtils.buildPoseInfo(newpose)
                    iterativeCompose( newpose, newinfo)
                    val comp = addCompose(newpose, last)
                    last = comp.result
                }
            }
        }
    }

    def private iterativeCompose(RelativePose rpose, RobotPoseUtils.PoseInfo info)
    {
        val path = info.framesPath
        if( path.length <= 1) return
        val frames = path.vertexList
        val target = rpose.target
        var RelativePose currentPose = new RelativePose(frames.get(0), frames.get(1))
        val iter2 = frames.listIterator(1) // start from the second ...
        val iter3 = frames.listIterator(2) // ... and the third. The first one is 'startF'
        do {
           val fr2 = iter2.next()
           val fr3 = iter3.next()
           if( notThereYet(target, fr3) ) {
               val longer = addCompose( currentPose, new RelativePose(fr2  ,fr3) )
               currentPose = longer.result
           }
        } while(iter3.hasNext())
    }

    def private addCompose(RelativePose p1, RelativePose p2)
    {
        val comp = new PoseCompose(p1, p2)
        this.composes.add( comp )
        return comp
    }


    private static class PoseLengthComparator implements Comparator<RobotPoseUtils.PoseInfo>
    {
        override compare(PoseInfo o1, PoseInfo o2) {
            return Integer.compare(o1.framesPath.length, o2.framesPath.length)
        }
    }
    def private populateMapByReference(List<RobotPoseUtils.PoseInfo> request)
    {
        for( i : request ) {
            val targets = outputByRef.get( i.pose.reference )
            if( targets === null ) {
                outputByRef.put( i.pose.reference, new TreeSet<PoseInfo>(new PoseLengthComparator) )
            }
        }
        for( i : request ) {
            outputByRef.get( i.pose.reference ) .add( i )
        }
    }

    def public getConstantPoses() { return this.constPoses }
    def public getJointPoses()    { return this.jointPoses }
    def public getComposes()      { return this.composes   }
    def public getOutputPoses()   { return this.outPoses   }
    def public getOutputJacobians(){return this.outJacs    }

    def public utils() { return this.robotu }

    def public name() { return this.userName }

    def public specs() { return this.mySpecs }

    def private reset() {
        framePairs.clear()
        constPoses.clear()
        jointPoses.clear()
        composes.clear()
        outPoses.clear()
        outputByRef.clear()
    }

    def private notThereYet(AttachedFrame target, AttachedFrame reference) {
        return notThereYet( target.frame.name, reference.frame.name)
    }
    def private notThereYet(String target, String reference) {
        return framePairs.add( new FrameNamePair(target, reference) )
    }

    private String    userName
    private RobotUtils robotu
    private FKSolverSpecs mySpecs
    private Set<FrameNamePair> framePairs = new HashSet<FrameNamePair>()
    private List<ConstantPose> constPoses = new ArrayList<ConstantPose>()
    private List<JointSuccessorPose> jointPoses = new ArrayList<JointSuccessorPose>()
    private List<PoseCompose> composes = new ArrayList<PoseCompose>()
    private Set<RelativePose> outPoses = new HashSet<RelativePose>()
    private Set<JacobianInfo> outJacs = new HashSet<JacobianInfo>()

    private Map<AttachedFrame, Set<RobotPoseUtils.PoseInfo> > outputByRef =
                              new HashMap<AttachedFrame, Set<RobotPoseUtils.PoseInfo> >()
}

package class FrameNamePair
{
    public new (String targ, String ref) {
        target    = targ
        reference = ref
    }
    def public getTarget()   { return target }
    def public getReference(){ return reference }

    override public int hashCode() {
        return Objects.hash(target, reference)
    }
    override public boolean equals(Object rhs) {
        if( ! (rhs instanceof FrameNamePair) ) {
            return false
        } else {
            val rh = rhs as FrameNamePair;
            return target.equals(rh.target) &&
                   reference.equals(rh.reference)
        }
    }

    String target;
    String reference;
}