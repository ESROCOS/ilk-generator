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



public class Main
{
    public static void main(String[] args)
    {
        CommandLineParser parser = new DefaultParser();
        CommandLine cmd = null;
        Options     opts= options();
        try {
            cmd  = parser.parse( opts, args);
        } catch (ParseException e)
        {
            // automatically generate the help statement
            HelpFormatter formatter = new HelpFormatter();
            formatter.printHelp( "generator", opts );
            System.exit(-2);
        }
        ////System.out.println("\n\n" + cmd.getOptionValue(opt_code__robot) + "\n\n");
        
        String robotFileName = cmd.getOptionValue(opt_code__robot);
        String queryFileName = cmd.getOptionValue(opt_code__query);
        File  robotModelFile = new File( robotFileName );
        File  queryFile      = new File( queryFileName );

        if( !robotModelFile.isFile() ) {
            System.err.println("Could not find file " + robotFileName);
            System.exit(-1);
        }
        if( !queryFile.isFile() ) {
            System.err.println("Could not find file " + queryFileName);
            System.exit(-1);
        }

        String outputPath = default_out_dir;
        if( cmd.hasOption(opt_code__outdir) ) {
            outputPath = cmd.getOptionValue(opt_code__outdir);
        }

        KinDSLWrapper kin = new KinDSLWrapper();
        kin.load( kin.getRobotModel(robotModelFile) , queryFile);

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
                .required(true)
                .longOpt("robot-model")
                .hasArg().argName("file")
                .desc( "load the robot model from the given file" )
                .build();
        Option queryfile = Option.builder( opt_code__query )
                .required(true)
                .longOpt("query")
                .hasArg().argName("file")
                .desc( "read the query from the given file" )
                .build();
        Option outdir = Option.builder( opt_code__outdir )
                .required(false)
                .longOpt("output-dir")
                .hasArg().argName("dir")
                .desc( "the directory where to put the generated files (defaults to " + default_out_dir + ")" )
                .build();

        opts.addOption(robotfile);
        opts.addOption(queryfile);
        opts.addOption(outdir);
        return opts;
    }

    private static final String opt_code__robot = "r";
    private static final String opt_code__query = "q";
    private static final String opt_code__outdir= "o";
    private static final String default_out_dir = "/tmp/kul-esrocos-generator/";

    public static final String deffilename_cppdefs = "robot-defs.h";
    public static final String deffilename_ilksrc  = "fk.ilk";
    public static final String deffilename_consts  = "model-constants.lua";
}
