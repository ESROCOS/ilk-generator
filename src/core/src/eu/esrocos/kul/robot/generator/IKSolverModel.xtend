package eu.esrocos.kul.robot.generator

import java.util.ArrayList

import eu.esrocos.kul.robot.generator.common.JacobianPlaceholder
import eu.esrocos.kul.robot.generator.common.gr.AttachedFrame
import eu.esrocos.kul.robot.generator.common.gr.RelativePose
import eu.esrocos.kul.robot.generator.common.gr.RobotFrameUtils
import eu.esrocos.kul.robot.kinDsl.Robot
import eu.esrocos.kul.query.queryLang.IK
import eu.esrocos.kul.query.queryLang.VectorKind


class IKSolverModel
{
    public enum IKKind {
        POSITION, VELOCITY
    }

    public new(Robot robot, IK query)
    {
        this.name  = query.name
        this.robot = robot
        if( query.eClass.name.equals("IKvel") ) {
            this.kind = IKKind.VELOCITY
        } else {
            this.kind = IKKind.POSITION
        }

        tsKind = query.specs.vkind

        this.target   = RobotFrameUtils.getFrameByName(robot, query.specs.frames.target.name)
        this.reference= RobotFrameUtils.getFrameByName(robot, query.specs.frames.reference.name)

        val jac = new ArrayList<JacobianPlaceholder>()
        val pose= new ArrayList<RelativePose>()
        jac.add( new JacobianPlaceholder(target, reference) )
        pose.add( new RelativePose(target, reference) )
        requiredFK = new FKSolverSpecs(robot, pose, jac)
        fkSolverID = "fk__" + name // default
    }

    public def String getFKSolverID() { return fkSolverID }
    public def void   setFKSolverID(String id) { fkSolverID = id }

    public final String name
    public final Robot  robot
    public final IKKind kind
    public final VectorKind tsKind
    public final AttachedFrame target
    public final AttachedFrame reference
    public final FKSolverSpecs requiredFK
    private String fkSolverID

}