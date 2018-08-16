package eu.esrocos.kul.robot.generator.common;

import eu.esrocos.kul.robot.generator.common.gr.AttachedFrame
import eu.esrocos.kul.robot.generator.common.gr.RelativePose
import eu.esrocos.kul.robot.generator.common.gr.RobotFrameUtils
import eu.esrocos.kul.robot.generator.common.gr.RobotPoseUtils
import eu.esrocos.kul.robot.generator.FKSolverModel

import eu.esrocos.kul.robot.kinDsl.Joint
import eu.esrocos.kul.robot.kinDsl.Robot

import java.util.ArrayList
import java.util.HashSet
import java.util.List
import java.util.Objects

class JacUtils
{
    public new(TreeUtils tree, RobotPoseUtils poses) {
        this.treeu = tree
        this.poseu = poses
    }

    def public buildInfo(JacobianPlaceholder J)
    {
        val pinfo = poseu.buildPoseInfo(new RelativePose(J.target, J.reference))
        val jlist = new ArrayList<Joint>()
        val kchain= pinfo.longestLinkToLink.sortBy(link | getID(link))
        if( kchain.length > 1 ) {
            val it1 = kchain.listIterator(0) // next() is the first element
            val it2 = kchain.listIterator(1) // next() is the second element
            while(it1.hasNext() && it2.hasNext()) {
                jlist.add( treeu.connectingJoint(it1.next(), it2.next()) )
            }
        }
        return new JacobianInfo(treeu.robot(), J.target, J.reference, jlist)
    }

    /**
     * The joint-frame-poses required to compute the given Jacobian.
     *
     * Specifically, all the relative poses in the form
     *
     *     {@code <joint frame>_wrt_<Jacobian reference frame>}
     *
     * for each joint of the kinematic chain associated with the Jacobian.
     */
    def public getJointPoses(JacobianInfo J)
    {
        val retlist = new ArrayList<RelativePose>()
        for( j : J.jchain ) {
            val lnk = treeu.predecessor( j )
            val jFr = RobotFrameUtils.getJointFrame( j, lnk )
            retlist.add( new RelativePose(jFr, J.reference) )
        }
        return retlist
    }

    /**
     * All the relative poses (i.e. forward kinematics) required to compute the
     * given Jacobian.
     *
     * These include the joint-frame-poses and the pose
     *
     *     {@code <Jacobian target>_wrt_<Jacobian reference>}
     *
     * @see getJointPoses(JacobianInfo)
     */
    def public getRequiredPoses(JacobianInfo J)
    {
        val retlist = getJointPoses(J)
        retlist.add( new RelativePose(J.target, J.reference) )
        return retlist
    }

    def public static defaultName(JacobianInfo J){
        return "J_" + J.target.frame.name + "_" + J.reference.frame.name
    }
    def public static defaultName(JacobianPlaceholder J) {
        return "J_" + J.target.frame.name + "_" + J.reference.frame.name
    }

    def public static jacobiansSet(List<FKSolverModel> fks) {
        val set = new HashSet<JacobianInfo>
        for( fk : fks ) {
            set.addAll( fk.outputJacobians )
        }
        return set
    }

    private TreeUtils treeu
    private RobotPoseUtils poseu
    private static extension eu.esrocos.kul.robot.generator.Common common = eu.esrocos.kul.robot.generator.Common.getInstance()
}

/**
 * A simple, immutable placeholder for the Jacobian returning the velocity of
 * {@code target} relative to {@code reference}.
 */
class JacobianPlaceholder
{
    public new(AttachedFrame tgt, AttachedFrame ref)
    {
        this.target= tgt
        this.reference= ref
    }

    public final AttachedFrame target
    public final AttachedFrame reference

    def override toString() {
        return "J " + target.toString() + " wrt " + reference.toString()
    }
    def override hashCode() {
        return target.hashCode + 31*reference.hashCode
    }
    def override equals(Object _rhs) {
        val rhs = _rhs as JacobianPlaceholder
        return target.equals(rhs.target) && reference.equals(rhs.reference)
    }
}

class JacobianInfo
{
    public new(Robot robot, AttachedFrame tgt, AttachedFrame ref, List<Joint> chain)
    {
        this.robot = robot
        this.target= tgt
        this.reference= ref
        this.jchain= chain
    }

    def public int cols() {
        return jchain.length
    }

    override public int hashCode() {
        return Objects.hash(robot.name, target, reference)
    }
    def override equals(Object _rhs) {
        val rhs = _rhs as JacobianInfo
        return robot    .equals(rhs.robot)   &&
               target   .equals(rhs.target)  &&
               reference.equals(rhs.reference)
    }

    public final Robot robot
    public final AttachedFrame target
    public final AttachedFrame reference
    public final List<Joint> jchain
}
