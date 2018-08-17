package eu.esrocos.kul.robot.generator

import eu.esrocos.kul.robot.kinDsl.AbstractLink
import eu.esrocos.kul.robot.kinDsl.Robot
import eu.esrocos.kul.robot.kinDsl.Var

import eu.esrocos.kul.robot.generator.common.NumericUtils
import eu.esrocos.kul.robot.generator.common.TreeUtils

import java.text.DecimalFormat

class URDFGenerator
{
    public new(Robot r, TreeUtils r2) {
        this.robot = r
        this.rtree = r2
    }

    /**
     * Generates and xml URDF description of the robot, as specified in the ROS
     * documentation.
     *
     * The URDF file format requires the inertia tensor to be expressed in a reference frame
     * with origin in the center of mass. Therefore this generator takes the inertia tensor
     * expressed in the link-default-frame and translates it to the COM before printing the values.
     * In addition, the URDF format requires the elements of the inertia tensor,
     * and not the inertia moments as in the Kinematics-DSL format; therefore
     * the signs of the centrifugal moments are swapped.
     */
    def public urdf_text() '''
    <robot name="«robot.name»">
        «IF robot.base.floating»
            <link name="world" />
        «ENDIF»
        <link name="«robot.base.name»">
            «URDF_inertialSection(robot.base)»
        </link>
        «FOR link : robot.links»
            <link name="«link.name»">
                «URDF_inertialSection(link)»
            </link>
        «ENDFOR»
        «IF robot.base.floating»
            <joint name="fbj" type="floating">
                <parent link="world"/>
                <child  link="«robot.base.name»"/>
            </joint>
        «ENDIF»
        «FOR joint : robot.joints»
            «val frame = joint.refFrame»
            «val rpy = NumericUtils::intrinsicToExtrinsic_xyz(frame.rotation.x.asFloat,frame.rotation.y.asFloat,frame.rotation.z.asFloat)»
            «val x = toStr(frame.translation.x)»
            «val y = toStr(frame.translation.y)»
            «val z = toStr(frame.translation.z)»
            <joint name="«joint.name»" type="«joint.typeString»">
                <origin xyz="«x» «y» «z»" rpy="«toStr(rpy.get(0))» «toStr(rpy.get(1))» «toStr(rpy.get(2))»"/>
                <parent link="«rtree.predecessor(joint).name»"/>
                <child  link="«rtree.successor  (joint).name»"/>
                <limit effort="30" velocity="1.0"/>
                <axis xyz="0 0 1"/>
            </joint>
        «ENDFOR»
    </robot>
    '''

    // Translate the inertia properties in the COM frame, as required by the URDF.
    // Invert the centrifugal moments, because URDF wants the elements of
    //  the inertia tensor, not the moments
    def private URDF_inertialSection(AbstractLink link)
        '''
        <inertial>
            «val inertia_lf = link.linkFrameInertiaParams /*inertia params expressed in the default link frame*/»
            «val com = inertia_lf.com»
            <origin xyz="«toStr(com.x)» «toStr(com.y)» «toStr(com.z)»"/>
            <mass value="«toStr(inertia_lf.mass)»"/>
            «val ip_com = NumericUtils.rototranslate(inertia_lf, com.x, com.y, com.z, 0,0,0,false)»
            <inertia ixx="«toStr(ip_com.ixx)»" iyy="«toStr(ip_com.iyy)»" izz="«toStr(ip_com.izz)»" ixy="«toStr(-ip_com.ixy)»" ixz="«toStr(-ip_com.ixz)»" iyz="«toStr(-ip_com.iyz)»"/>
        </inertial>
        '''


    private static DecimalFormat df = new DecimalFormat("0.0#####")
    def private toStr(double n) {
        return df.format(n)
    }
    def private toStr(Var v) {
        return df.format(v.asFloat)
    }

    private Robot robot
    private TreeUtils rtree
    private extension Common common = Common.getInstance()

}
