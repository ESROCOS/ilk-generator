package eu.esrocos.kul.robot;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.resource.XtextResourceSet;
import org.eclipse.xtext.util.StringInputStream;

import com.google.inject.Injector;

import eu.esrocos.kul.robot.kinDsl.Robot;


/**
 * Class that gives access to the models of Query-DSL.
 *
 * Documents (models) of the Query-DSL list the kinematics solvers for which
 * ILK code should be generated.
 * @author Marco Frigerio
 *
 */
public class QueryDSLAccessor
{
    private Resource resource;
    private final XtextResourceSet set;

    public QueryDSLAccessor() {
        Injector injector = new eu.esrocos.kul.query.QueryLangStandaloneSetup().
                createInjectorAndDoEMFRegistration();

        set = injector.getInstance(XtextResourceSet.class);
        set.addLoadOption(XtextResource.OPTION_RESOLVE_ALL, Boolean.TRUE);
    }

    public eu.esrocos.kul.query.queryLang.KinematicsQuery getModel(Robot robot) {
        String modelFilePath = "models/"+robot.getName() + ".dtdsl";
        return getModel(new java.io.File(modelFilePath));
    }

    public eu.esrocos.kul.query.queryLang.KinematicsQuery getModel(File modelFile) {
        if(modelFile == null) return null;
        if(modelFile.isFile()) {
            return getModel(URI.createURI(modelFile.getAbsolutePath()));
        } else {
            System.out.println("Did not find the file " + modelFile.getPath());
        }
        return null;
    }

    public eu.esrocos.kul.query.queryLang.KinematicsQuery getModel(String model) throws IOException
    {
        URI uri = URI.createURI("dummy:/"+Long.toString(System.nanoTime())+".dtdsl");
        resource = set.createResource(uri);
        InputStream in = new StringInputStream(model);
        resource.load(in, set.getLoadOptions());
        return getModel(uri);
    }

    private eu.esrocos.kul.query.queryLang.KinematicsQuery getModel(final URI uri) {
        resource = set.getResource(uri, true);
        List<Resource.Diagnostic> errors = resource.getErrors();
        if(errors.size() > 0) {
            StringBuffer msg = new StringBuffer();
            msg.append("Errors while loading a document of the Query-DSL ("
                        + uri.toString() + "):\n");
            for(Resource.Diagnostic err : errors) {
                msg.append("\n\t " + err.getMessage() + " (at line " + err.getLine() + ")\n");
            }
            throw new RuntimeException(msg.toString());
        }
        return (eu.esrocos.kul.query.queryLang.KinematicsQuery)resource.getContents().get(0);
    }

}
