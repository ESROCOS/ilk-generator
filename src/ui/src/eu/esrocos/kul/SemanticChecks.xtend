package eu.esrocos.kul

import eu.esrocos.kul.robot.generator.common.TreeUtils
import java.util.ArrayList
import java.util.Collections
import eu.esrocos.kul.robot.kinDsl.AbstractLink
import java.util.TreeSet

class SemanticChecks
{
    /**
     * Verifies that the kinematic tree is a connected graph.
     *
     * @return an error message with some information, empty if all is ok
     */
    def public static StringBuffer connectivity(TreeUtils tree)
    {
        val msg = new StringBuffer()
        val sets = tree.getConnectedComponents();
        if( sets !== null) {
            msg.append("The kinematic tree of robot " + tree.robot.name + " is not connected.\n")
            for( set : sets ) {
                msg.append("Connected group: ")
                for( AbstractLink v : set ) {
                    msg.append(v.getName() + " ")
                }
                msg.append("\n")
            }
        }
        return msg
    }

    /**
     * Verifies the consistency of the IDs of the links of the robot.
     *
     * @return an error message with some information, empty if all is ok
     */
    def public static StringBuffer numberingScheme(TreeUtils tree)
    {
        val msg = new StringBuffer()
        // Check for duplicate or missing IDs
        val ids = new ArrayList()
        val max = tree.robot.links.size
        for(link : tree.robot.links) {
            ids.add( link.ID )
        }
        Collections.sort( ids )

        var prev= -22
        for( id : ids) {
            if( id == prev ) {
                msg.append("Duplicate link ID " + id + "\n")
            }
            if( id > max ) {
                msg.append("Link ID " + id + " out of range\n")
            }
            prev= id
        }
        val uniques = new TreeSet()
        uniques.addAll( ids )
        for( var i=1; i<=max; i++) {
            if( ! uniques.contains(i) ) {
                msg.append("Missing link ID " + i + "\n")
            }
        }

        // Check that the ordering: link ID must be greater than the parent's
        val leafs = new ArrayList()
        for(link : tree.robot.links) {
            if(link.childrenList.children.size == 0) {
                leafs.add(link)
            }
        }
        for(leaf : leafs) {
            var AbstractLink link = leaf
            var AbstractLink parent
            do {
                parent = tree.parent(link)
                if( link.ID < parent.ID ) {
                    msg.append("The ID of link " + link.name + " (" +
                           link.ID + ") must be greater than the ID of the parent " +
                           parent.name + " (" + parent.ID + ")\n")
                }
                link = parent
            }
            while( parent != tree.robot.base )
        }
        return msg
    }


    private static extension eu.esrocos.kul.robot.generator.Common common = eu.esrocos.kul.robot.generator.Common.getInstance()
}