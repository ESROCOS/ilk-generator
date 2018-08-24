package eu.esrocos.kul;


import java.io.File;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;
import org.eclipse.xtext.generator.JavaIoFileSystemAccess;
import org.eclipse.xtext.parser.IEncodingProvider;
import org.eclipse.xtext.resource.IResourceServiceProvider;

import eu.esrocos.kul.robot.generator.common.TreeUtils;
import eu.esrocos.kul.robot.kinDsl.Robot;



public class Main
{
    public static void main(String[] args)
    {
        CommandLineParser parser = new DefaultParser();
        CommandLine cmd = null;
        Options     opts= options();
        HelpFormatter hformatter = new HelpFormatter();
        try {
            cmd  = parser.parse( opts, args);
        } catch (ParseException e)
        {
            // automatically generate the help statement
            hformatter.printHelp( "generator", "", opts, "", true );
            System.exit(-1);
        }
        ////System.out.println("\n\n" + cmd.getOptionValue(opt_code__robot) + "\n\n");

        String robotFileName = cmd.getOptionValue(opt_code__robot);
        File  robotModelFile = new File( robotFileName );
        if( !robotModelFile.isFile() ) {
            System.err.println("Could not find file " + robotFileName);
            System.exit(-1);
        }

        KinDSLWrapper kin = new KinDSLWrapper();
        Robot robot = kin.getRobotModel(robotModelFile);
        semanticChecks(robot);

        if( cmd.hasOption(opt_code__urdf) ) {
            String urdf_out = cmd.getOptionValue(opt_code__urdf);
            kin.load(robot , null);
            kin.generateURDF(urdf_out);
            return;
        }

        if( ! cmd.hasOption(opt_code__query) ) {
            hformatter.printHelp( "generator", "", opts, "", true );
            System.exit(-2);
        }

        String outputPath = default_out_dir;
        if( cmd.hasOption(opt_code__outdir) ) {
            outputPath = cmd.getOptionValue(opt_code__outdir);
        }

        String queryFileName = cmd.getOptionValue(opt_code__query);
        File  queryFile      = new File( queryFileName );
        if( !queryFile.isFile() ) {
            System.err.println("Could not find file " + queryFileName);
            System.exit(-1);
        }

        kin.load(robot , queryFile);

        JavaIoFileSystemAccess fileWriter = new JavaIoFileSystemAccess(
                        IResourceServiceProvider.Registry.INSTANCE,
                        new IEncodingProvider.Runtime() );

        fileWriter.setOutputPath(outputPath);

        kin.generateFiles( fileWriter );
        return;

    }

    private static Options options()
    {
        Options opts = new Options();
        Option robotfile = Option.builder( opt_code__robot )
                .required()
                .longOpt("robot-model")
                .hasArg().argName("file")
                .desc( "load the robot model from the given file" )
                .build();
        Option queryfile = Option.builder( opt_code__query )
                .longOpt("query")
                .hasArg().argName("file")
                .desc( "read the query from the given file" )
                .build();
        Option outdir = Option.builder( opt_code__outdir )
                .longOpt("output-dir")
                .hasArg().argName("dir")
                .desc( "the directory where to put the generated files (defaults to " + default_out_dir + ")" )
                .build();
        Option urdf = Option.builder( opt_code__urdf )
                .longOpt("gen-urdf")
                .hasArg().argName("file")
                .desc( "generate an equivalent robot model in the URDF format (ignore the query option)" )
                .build();
        opts.addOption(robotfile);
        opts.addOption(queryfile);
        opts.addOption(outdir);
        opts.addOption(urdf);
        return opts;
    }

    private static void semanticChecks(Robot robot)
    {
        // check connectivity first, to avoid errors in the other checks!

        TreeUtils tree = new TreeUtils(robot);
        StringBuffer err = SemanticChecks.connectivity(tree);
        if( err.length() > 0 ) {
            System.err.println("Connectivity errors detected for robot " + robot.getName() + ":");
            System.err.println( err.toString() );
            System.exit(-37);
        }
        err = SemanticChecks.numberingScheme(tree);
        if( err.length() > 0 ) {
            System.err.println("Numbering scheme errors detected for robot " + robot.getName() + ":");
            System.err.println( err.toString() );
            System.exit(-31);
        }
    }

    private static final String opt_code__robot = "r";
    private static final String opt_code__query = "q";
    private static final String opt_code__outdir= "o";
    private static final String opt_code__urdf  = "u";
    private static final String default_out_dir = "/tmp/kul-esrocos-generator/";

    public static final String deffilename_cppdefs = "robot-defs.h";
    public static final String deffilename_ilksrc  = "fk.ilk";
    public static final String deffilename_consts  = "model-constants.lua";
}
