package eu.esrocos.kul.robot.generator

import java.util.ArrayList
import java.util.List
import java.util.Locale
import org.eclipse.emf.ecore.util.EcoreUtil

import eu.esrocos.kul.robot.kinDsl.AbstractLink
import eu.esrocos.kul.robot.kinDsl.ChildSpec
import eu.esrocos.kul.robot.kinDsl.ChildrenList
import eu.esrocos.kul.robot.kinDsl.DivExpr
import eu.esrocos.kul.robot.kinDsl.Expr
import eu.esrocos.kul.robot.kinDsl.FixedRobotBase
import eu.esrocos.kul.robot.kinDsl.FloatLiteral
import eu.esrocos.kul.robot.kinDsl.FloatingRobotBase
import eu.esrocos.kul.robot.kinDsl.Identifier
import eu.esrocos.kul.robot.kinDsl.Joint
import eu.esrocos.kul.robot.kinDsl.Link
import eu.esrocos.kul.robot.kinDsl.MultExpr
import eu.esrocos.kul.robot.kinDsl.PlainExpr
import eu.esrocos.kul.robot.kinDsl.PrismaticJoint
import eu.esrocos.kul.robot.kinDsl.RevoluteJoint
import eu.esrocos.kul.robot.kinDsl.Robot
import eu.esrocos.kul.robot.kinDsl.RobotBase
import eu.esrocos.kul.robot.kinDsl.Vector3
import eu.esrocos.kul.robot.kinDsl.RefFrame
import eu.esrocos.kul.robot.kinDsl.impl.KinDslFactoryImpl
import eu.esrocos.kul.robot.kinDsl.KinDslFactory
import eu.esrocos.kul.robot.kinDsl.PILiteral
import eu.esrocos.kul.robot.generator.common.NumericUtils
import eu.esrocos.kul.robot.generator.common.NumericUtils.IProperties
import eu.esrocos.kul.robot.kinDsl.InertiaParams

class Common {
    private static Common instance = new Common()

    def public static Common getInstance() {
        return instance
    }
    static KinDslFactory kinDSLFactory = KinDslFactoryImpl::init()
    static FloatLiteral zeroFloat = kinDSLFactory.createFloatLiteral()//relies on default constructor setting it to zero


def dispatch boolean isFloating(RobotBase par) {
    false
}
def dispatch boolean isFloating(FloatingRobotBase par) {
    true
}

def List<AbstractLink> abstractLinks(Robot robot) {
    val list = new ArrayList<AbstractLink>()
    list.add(robot.base)
    list.addAll(robot.links)
    return list
}
// an alias of the above, as I don't like the name that much
def List<AbstractLink> linksAndBase(Robot robot) {
    val list = new ArrayList<AbstractLink>()
    list.add(robot.base)
    list.addAll(robot.links)
    return list
}

/** Returns whether the ChildrenList contains the specified link */
def boolean contains(ChildrenList list, AbstractLink link) {
    for(ChildSpec c : list.children) {
        if(c.link.equals(link)) return true
    }
    return false
}
/** Returns whether the ChildrenList contains the specified joint */
def boolean contains(ChildrenList list, Joint joint) {
    for(ChildSpec c : list.children) {
        if(c.joint.equals(joint)) return true
    }
    return false
}
/** Returns the parent of the specified link, null if it does not exist */
def AbstractLink getParent(AbstractLink link) {
    val links = abstractLinks(link.eContainer() as Robot)
    for(AbstractLink l : links) {
        if(contains(l.childrenList, link)) return l
    }
   return null
}

def int movingBodiesCount(Robot robot) {
    if(robot.base.isFloating()) {
        return robot.links.size + 1;
    } else {
        return robot.links.size
    }
}

/**
 * The total number of degrees of freedom of this robot.
 * This functions considers the degrees of freedom due to the joints of the robot,
 * plus 6 additional dofs if the robot is a floating-base one.
 */
def int getDOFs(Robot robot) {
    if(robot.base.floating) {
        return getJointDOFs(robot) + 6
    } else {
        return getJointDOFs(robot)
    }
}

/**
 * The number of degrees of freedom of this robot due to its joints
 */
def int getJointDOFs(Robot robot) {
    return robot.joints.size
}

def dispatch int getID(Link l) {
    return l.num;
}
def dispatch int getID(FixedRobotBase base) {
    return 0;
}
def dispatch int getID(FloatingRobotBase base) {
    return 1;
}

def listCoordinates(Vector3 vector) {
    '''«vector.x.str» «vector.y.str» «vector.z.str»'''
}

def dispatch getTypeString(RevoluteJoint joint) '''revolute'''
def dispatch getTypeString(PrismaticJoint joint)'''prismatic'''

def dispatch boolean isPrismatic(RevoluteJoint joint) { return false }
def dispatch boolean isPrismatic(PrismaticJoint joint){ return true }

def boolean anyPrismaticJoint(Robot robot) {
    for(Joint j : robot.joints) {
        if(j.isPrismatic) return true;
    }
    return false;
}
def boolean anyRevoluteJoint(Robot robot) {
    for(Joint j : robot.joints) {
        if(j instanceof RevoluteJoint) return true;
    }
    return false;
}

def getVariableName(Joint joint) '''q_«joint.name»'''
def Joint getJointFromVariableName(Robot robot, String varname) {
    return getJointByName(robot, varname.replaceAll("q_", ""))
}
def getFrameName(Joint joint) '''fr_«joint.name»'''
def getFrameName(AbstractLink link) '''fr_«link.name»'''

def getItemNameByFrameName(String frameName) {
    if(! frameName.startsWith("fr_")) return null
    return frameName.replaceFirst("fr_", "")
}

def dispatch CharSequence str(Float num) {
    String::format(Locale::US,"%06.5f", num)
}
def dispatch CharSequence str(FloatLiteral id)'''«str(id.value)»'''
def dispatch CharSequence str(PlainExpr expr) '''«str(expr.identifier)»'''
def dispatch CharSequence str(MultExpr expr)  '''«str(expr.mult)» «str(expr.identifier)»'''
def dispatch CharSequence str(DivExpr expr)   '''«str(expr.identifier)»/«expr.div»'''
def dispatch CharSequence str(Identifier id)  '''«IF id.minus»-«ENDIF»«id.varname»'''


/**
 * Return the float value of an expression.
 * Will throw an exception if the expression is a variable literal, which of
 * course cannot be converted arbitrarily to a floating point value
 */
def dispatch float asFloat(FloatLiteral id) {
    id.value
}
def dispatch float asFloat(PlainExpr id) {
    id.identifier.asFloat
}
def dispatch float asFloat(MultExpr expr) {
    return NumericUtils::mult(expr.mult, expr.identifier.asFloat)
}
def dispatch float asFloat(DivExpr expr) {
    return NumericUtils::div(expr.identifier.asFloat, expr.div)
}
def dispatch float asFloat(PILiteral pi) {
    if(pi.minus) {
        return NumericUtils::invert(java::lang::Math::PI as float)
     }
     else {
         return java::lang::Math::PI as float
     }
}
//def dispatch asFloat(ConstantLiteral c) {
//    throw new RuntimeException("cannot convert to a float a variable literal")
//}



def dispatch boolean isZero(FloatLiteral f) {
    return NumericUtils::isZero(f.value)
}
def dispatch boolean isZero(Expr expr) {
    return false
}


def AbstractLink getLinkByName(Robot robot, String linkName) {
    for(AbstractLink l : robot.abstractLinks) {
        if(l.name.equals(linkName)) {
            return l
        }
    }
    return null
}

def RefFrame getFrameByName(AbstractLink link, String frameName) {
    for(RefFrame f : link.frames) {
        if(f.name.equals(frameName)) {
            return f
        }
    }
    return null
}


def Joint getJointByName(Robot robot, String jointName) {
    for(Joint j : robot.joints) {
        if(j.name.equals(jointName)) {
            return j
        }
    }
    return null
}

def Joint getJointByName(AbstractLink link, String jointName) {
    for(ChildSpec ch : link.childrenList.children) {
        if( ch.joint.name.equals(jointName) ) {
            return ch.joint
        }
    }
    return null
}

def Vector3 zeroVector() {
    val ret = kinDSLFactory.createVector3()
    ret.setX(EcoreUtil::copy(zeroFloat))
    ret.setY(EcoreUtil::copy(zeroFloat))
    ret.setZ(EcoreUtil::copy(zeroFloat))
    return ret
}



def void convert(InertiaParams emf, IProperties out)
{
    out.mass  = asFloat(emf.getMass())
    out.com.x = asFloat(emf.getCom().getX())
    out.com.y = asFloat(emf.getCom().getY())
    out.com.z = asFloat(emf.getCom().getZ())
    out.ixx   = asFloat(emf.getIx())
    out.iyy   = asFloat(emf.getIy())
    out.izz   = asFloat(emf.getIz())
    out.ixy   = asFloat(emf.getIxy())
    out.ixz   = asFloat(emf.getIxz())
    out.iyz   = asFloat(emf.getIyz())
}

/**
 * Returns the inertia parameters of the link in the argument, expressed
 * in the default frame of the link.
 */
def IProperties getLinkFrameInertiaParams(AbstractLink link)
{
    val in = new IProperties
    convert(link.inertiaParams, in)
    // If no frame is specified, then the parameters are already expressed in the link frame
    if(link.inertiaParams.frame === null) {
        return in
    }

    val Vector3 trasl = link.inertiaParams.frame.transform.translation;
    val Vector3 rotat = link.inertiaParams.frame.transform.rotation;
    val  params =
        NumericUtils.rototranslate(in,
            trasl.x.asFloat, trasl.y.asFloat, trasl.z.asFloat,
            rotat.x.asFloat, rotat.y.asFloat, rotat.z.asFloat,
            //these numbers specify the pose of the frame in which the inertia-params
            // are currently expressed, with respect to the link frame. The link frame
            // is the one we want inertia to be expressed. According to docs, the next
            // argument must then be 'true'
            true)
    return params
}


}//end of class

