This is the readme file of the ILK-Generator tool.

# Building from source

Just invoke `make` in the root of the project, after making sure that the
prerequisites are installed in your system.

`make release` will create a `release/` subfolder, which contains a
standalone binary distribution of the program.

## Pre-requisites

A Java compiler (`javac`), the Apache Ant+Ivy tools (for the building
workflow and for dependency management).

The following should install the required packages on Ubuntu Linux:

```
sudo apt-get install openjdk-8-jdk ant ivy
```

Afterwards, a symbolic link must be created (in order for Ant to find Ivy...):

```
cd /usr/share/ant/lib
sudo ln -s ../../java/ivy.jar
```

## Known/possible issues

### Proxy server

The build process downloads several dependencies from the Internet. When
installing from behind a proxy, it is necessary to setup the proxy options by
means of environment variables.
See the [Ivy faq](http://ant.apache.org/ivy/faq.html).

For instance:

```
set ANT_OPTS=-Dhttp.proxyHost=myproxy -Dhttp.proxyPort=3128
```

Or for authenticated proxy:

```
set ANT_OPTS=-Dhttp.proxyHost=myproxyhost -Dhttp.proxyPort=8080 -Dhttp.proxyUser=myproxyusername -Dhttp.proxyPassword=myproxypassword -Dhttps.proxyHost=myproxyhost -Dhttps.proxyPort=8080
```

Additionally,
the ILK Generator is currently based on the Xtext workbench. Xtext uses the
ANTLR parser generator. At building time, if the generator is not available, the
Xtext workflow tries to obtain the corresponding file from the Internet.
Possibly due to [a bug](https://bugs.eclipse.org/bugs/show_bug.cgi?id=329683),
the download may ignore the proxy settings.

The following error message is shown:
```
     [java] Downloading ANTLR parser generator failed: Connection timed out (Connection timed out)
     [java] Please install the feature 'Xtext Antlr SDK' manually using the external updatesite:
     [java]
     [java]             'http://download.itemis.com/updates/'.
     [java]
     [java] (see http://www.eclipse.org/Xtext/download.html for details)

BUILD FAILED
/home/esrocos/esrocos_workspace/control/ilk-generator/building/ant/build.xml:175: Java returned: 1
```

As a **workaround**, the file required by Xtext can be downloaded manually:

- Download the file [here](http://download.itemis.com/antlr-generator-3.2.0-patch.jar)
- Copy it to `ilk-generator/building/ant/.antlr-generator-3.2.0-patch.jar`
- Re-run `make`


# Running the tool

Move into the `building/0build` subfolder, generated after a successful build,
and launch the program by running `run.sh`. For example:

```
cd building/0build
./run.sh --robot ../../sample/model/ur5.kindsl --query ../../sample/model/ur5.dtdsl --output-dir ../../sample/gen
```

The command above uses the sample input files available in `sample/model`.

Alternatively, after `make release`, issue the same command (adjusting the
path of the inputs) from the `release/` folder.


