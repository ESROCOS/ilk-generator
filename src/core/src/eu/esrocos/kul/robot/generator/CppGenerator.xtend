package eu.esrocos.kul.robot.generator

import eu.esrocos.kul.robot.generator.common.JacUtils
import eu.esrocos.kul.robot.kinDsl.Robot
import eu.esrocos.kul.robot.generator.common.JacobianInfo

import java.util.Set

class CppGenerator
{
    public static String backend_ns = "kul"

    public new(Robot rob) {
        this.robot = rob
    }


    def header_robot_defs(Set<JacobianInfo> jacs) '''
        #ifndef EU_ESROCOS_KUL_CODEGENERATOR_ROBOT_«robot.name.toUpperCase»_DEFS_H
        #define EU_ESROCOS_KUL_CODEGENERATOR_ROBOT_«robot.name.toUpperCase»_DEFS_H

        #include <ilk/eigen/core-types.h>

        namespace «robot.name.toLowerCase» {

        constexpr unsigned int dofs_count = «robot.joints.length»;
        typedef «backend_ns»::Matrix< dofs_count, 1> joint_state;

        typedef «backend_ns»::Matrix<6, dofs_count> Jacobian_t;
        «FOR J : jacs»
            typedef Jacobian_t t_«JacUtils.defaultName(J)»;
        «ENDFOR»

        } // robot namespace

        #endif
    '''

    private Robot robot
}