package eu.esrocos.kul.robot.generator

import eu.esrocos.kul.robot.generator.common.gr.RobotPoseUtils.ConstantPose
import java.util.Collection
import org.eclipse.xtend2.lib.StringConcatenation
import java.util.HashSet

class ModelConstants
{
    public new(Collection<FKSolverModel> allFK)
    {
        constPoses = new HashSet<ConstantPose>
        for( fk : allFK ) {
            constPoses.addAll( fk.constantPoses )
        }
    }


    def asLuaTable() '''
    return  {
        «FOR c : constPoses SEPARATOR ","»
            «c.name()» = {
                p = {«c.refToTarget.toString()»},
                r = {«listElementsAsText(c.ref_R_target)»}
            }
        «ENDFOR»
    }
    '''

    def private listElementsAsText(double[][] mx) {
        val text = new StringConcatenation();
        for (row : mx) {
            for (el : row) {
                text.append( String.format("%.6f", el) + ",");
            }
        }
        return text.subSequence(0, text.length-2)
    }


    private Collection<ConstantPose> constPoses = null
}