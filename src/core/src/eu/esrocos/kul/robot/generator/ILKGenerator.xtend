package eu.esrocos.kul.robot.generator

import eu.esrocos.kul.robot.generator.Common
import eu.esrocos.kul.robot.generator.common.JacobianInfo
import eu.esrocos.kul.robot.generator.common.gr.AttachedFrame
import eu.esrocos.kul.robot.generator.common.gr.RobotPoseUtils.JointSuccessorPose
import eu.esrocos.kul.robot.kinDsl.Joint
import eu.esrocos.kul.robot.kinDsl.PrismaticJoint
import eu.esrocos.kul.robot.kinDsl.RevoluteJoint
import java.util.Collection
import org.eclipse.xtend2.lib.StringConcatenation
import java.util.Map
import eu.esrocos.kul.robot.kinDsl.KinDslPackage
import eu.esrocos.kul.robot.generator.common.gr.RelativePose
import eu.esrocos.kul.robot.generator.common.JacUtils

class ILKGenerator
{
    public new( ) {

    }

    def public lua(FKSolverModel solverModel) {
        this.fkmodel = solverModel
        return
    '''

    return {
        solverid = '«fkmodel.name»',
        solver_type = 'forward',
        robot_name = '«fkmodel.utils.robot.name»',

        -- Constant poses (from the model)
        { op='model_const', args={ «FOR kk: fkmodel.constantPoses SEPARATOR","»'«kk.name»'«ENDFOR»} },

        -- Joint-frame poses (from the model)
        «FOR c : fkmodel.jointPoses SEPARATOR"," AFTER","»
            { op='model_T_joint_local', name='«c.name()»', jtype='«c.joint.typestr»', dir='«c.directionTag»', input=«c.joint.coordinateIdx» }
        «ENDFOR»

        «FOR c : fkmodel.composes SEPARATOR ","AFTER","»
            { op='compose', args={ '«c.arg1.name»', '«c.arg2.name»', '«c.result.name»' } }
        «ENDFOR»

        -- Output poses (user's request)
        «FOR c : fkmodel.outputPoses SEPARATOR ","»
            { op='output', otype='pose', target='«c.name»' }
        «ENDFOR»
        «commaIfNotEmpty(fkmodel.outputJacobians)»

        -- Jacobians "declarations" for subsequent reference (user's request)
        «FOR j : fkmodel.outputJacobians SEPARATOR "," AFTER ","»
            { op='geom-jacobian', name='«jacName(j)»', pose='«(new RelativePose(j.target, j.reference)).name»' }
        «ENDFOR»

        -- Jacobians algorithm
        «FOR jac : fkmodel.outputJacobians SEPARATOR "," AFTER ","»
            «jacobianBlock(jac)»
        «ENDFOR»

        -- Output Jacobians (user's request)
        «FOR j : fkmodel.outputJacobians SEPARATOR "," »
            { op='output', otype='jacobian', target='«jacName(j)»' }
        «ENDFOR»
    }
    '''
    }

    def public lua(IKSolverModel solverModel) {
        this.ikmodel = solverModel
        return
        '''
        return {
                solverid = '«ikmodel.name»',
                solver_type = 'inverse',
                robot_name = '«fkmodel.utils.robot.name»',
                { op='ik', kind='«ikkindNames.get(ikmodel.kind)»',
                 vectors='«ikmodel.tsKind.literal»',
                 target='«ikmodel.target.frame.name»',
                 reference='«ikmodel.reference.frame.name»',
                 fk='«ikmodel.getFKSolverID»' }
        }
        '''
    }

    def private <T> commaIfNotEmpty(Collection<T> foo) {
        if(foo.length>0) return ","
        return ""
    }

    def private jacobianBlock(JacobianInfo J)
    {
        val code = new StringConcatenation()
        val jointposes = fkmodel.utils.jacsUtils.getJointPoses(J)
        val posesiter  = jointposes.iterator
        val Jname = jacName(J)
        for( joint : J.jchain ) {
            val col = joint.coordinateIdx
            val jointpose = posesiter.next
            var String typestr = "revolute"
            if(joint.eClass.classifierID == KinDslPackage.PRISMATIC_JOINT) {
                typestr = "prismatic"
            }
            code.append('''{ op='GJac-col', jtype='«typestr»', jac='«Jname»', col=«col», joint_pose='«jointpose.name»' }''')
            if(posesiter.hasNext) {
                code.append(",\n")
            }
        }
        return code
    }

    def private dispatch typestr(RevoluteJoint j)  { return "revolute" }
    def private dispatch typestr(PrismaticJoint j) { return "prismatic" }
    def private int coordinateIdx(Joint j) {
        fkmodel.utils.treeUtils.successor(j).ID - 1
    }

    def private directionTag(JointSuccessorPose pose) {
        val targetKind = pose.target().role()
        if( targetKind == AttachedFrame.FrameRole.link ) {
            return "a_x_b"
        } else {
            return "b_x_a"
        }
    }


    private FKSolverModel fkmodel
    private IKSolverModel ikmodel

    private static extension Common helper = Common.getInstance()
    private static Map<IKSolverModel.IKKind, String> ikkindNames =
                                         #{IKSolverModel.IKKind.VELOCITY->"vel",
                                           IKSolverModel.IKKind.POSITION->"pos"}

    def public static jacName(JacobianInfo J) {
        return JacUtils.defaultName(J)
    }
}