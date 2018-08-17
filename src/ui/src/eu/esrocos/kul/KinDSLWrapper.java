package eu.esrocos.kul;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Set;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.generator.IFileSystemAccess;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.resource.XtextResourceSet;

import com.google.inject.Injector;

import eu.esrocos.kul.query.queryLang.KinematicsQuery;
import eu.esrocos.kul.robot.KinDslStandaloneSetup;
import eu.esrocos.kul.robot.QueryDSLAccessor;
import eu.esrocos.kul.robot.generator.CppGenerator;
import eu.esrocos.kul.robot.generator.FKSolverModel;
import eu.esrocos.kul.robot.generator.IKSolverModel;
import eu.esrocos.kul.robot.generator.ILKGenerator;
import eu.esrocos.kul.robot.generator.ModelConstants;
import eu.esrocos.kul.robot.generator.QueryProcessor;
import eu.esrocos.kul.robot.generator.URDFGenerator;
import eu.esrocos.kul.robot.generator.common.JacUtils;
import eu.esrocos.kul.robot.generator.common.JacobianInfo;
import eu.esrocos.kul.robot.generator.common.TreeUtils;
import eu.esrocos.kul.robot.kinDsl.Robot;


/**
 * Wrapper around the code generators of the Kinematics DSL.
 * @author Marco Frigerio
 *
 */
public class KinDSLWrapper
{
	private final Injector injector;
    private XtextResourceSet resourceSet = null;
    private Resource resource = null;

    private Robot robot = null;


    private KinematicsQuery userQuery = null;
    //private eu.esrocos.kul.query.transSpecs.DesiredTransforms desTransformsModel = null;



    public KinDSLWrapper()
    {
        injector = new KinDslStandaloneSetup().createInjectorAndDoEMFRegistration();
        resourceSet = injector.getInstance(XtextResourceSet.class);
        resourceSet.addLoadOption(XtextResource.OPTION_RESOLVE_ALL, Boolean.TRUE);
    }

    public void clearModelPool() {
        resourceSet.getResources().clear();
    }

    /**
     *
     * @param modelFile the File with the robot-model description compliant with the
     *        Kinematics DSL
     * @return the Robot instance corresponding to the given document, created by
     *         the DSL infrastructure
     */
    public Robot getRobotModel(File modelFile)
    {
        if( !modelFile.exists() ) {
            throw new RuntimeException("Could not find file " + modelFile.getName());
        }

        resource = resourceSet.getResource(URI.createURI(modelFile.toURI().toString()), true);
        List<Resource.Diagnostic> errors = resource.getErrors();
        if(errors.size() > 0) {
            StringBuffer msg = new StringBuffer();
            msg.append("\nErrors while loading the document ");
            msg.append(modelFile.getName() + "\n");
            for(Resource.Diagnostic err : errors) {
                msg.append("\n\t " + err.getMessage() +
                        " (line " + err.getLine() + ")\n");
            }
            throw new RuntimeException(msg.toString());
        }
        return (Robot)resource.getContents().get(0);
    }

    /**
     * Performs required initialization for the given robot.
     * @param rob
     * @param transforms
     */
    public void load(Robot rob, File query)
    {
        robot = rob;
        QueryDSLAccessor queryAccessor = new QueryDSLAccessor();

        if(query != null) {
            userQuery = queryAccessor.getModel(query);
            //desTransformsModel = userDesTransformsModel;
        } else {
            //desTransformsModel = null;
            userQuery = null;
        }

    }

    public void generateFiles(IFileSystemAccess fsa)
    {
        QueryProcessor qprocessor = new QueryProcessor(robot, userQuery);
        ILKGenerator ilkgen = new ILKGenerator( );

        for( FKSolverModel fksolver : qprocessor.fksolvers() ) {
            fsa.generateFile(fksolver.name()+".ilk" , ilkgen.lua(fksolver) );
        }
        for( IKSolverModel iksolver : qprocessor.iksolvers() ) {
            fsa.generateFile(iksolver.name+".ilk" , ilkgen.lua(iksolver) );
        }

        ModelConstants genk = new ModelConstants( qprocessor.fksolvers() );
        CppGenerator cppgen = new CppGenerator( robot );
//        RCGCompare cmpGen = new RCGCompare( );
//
        Set<JacobianInfo> jacs = JacUtils.jacobiansSet( qprocessor.fksolvers() );

        fsa.generateFile(Main.deffilename_cppdefs, cppgen.header_robot_defs(jacs) );
        fsa.generateFile(Main.deffilename_consts , genk.asLuaTable() );
//        fsa.generateFile("cmp.cpp" , cmpGen.mainSrc(robot, gen.getConstantPoses(), gen.getOutputs() ) );
    }

    public void generateURDF(String outfilepath) {
        FileWriter out = null;
        try {
            out = new FileWriter( outfilepath );
            TreeUtils tree = new TreeUtils(robot);
            URDFGenerator gen = new URDFGenerator(robot, tree);

            out.write( gen.urdf_text().toString() );
            out.close();
        } catch (IOException e) {
            throw new RuntimeException("while trying to open the file: " + e.getMessage());
        }
    }
}
