This is the readme file of the ILK-Generator tool.

# Building from source

Just invoke `make` in the root of the project, after making sure that the
prerequisites are installed in your system.

`make release` will create a `release/` subfolder, which contains a
standalone binary distribution of the program.

## Pre-requisites

A Java compiler (`javac`), the Apache Ant+Ivy tools.

The following should install the required packages on Ubuntu Linux:

```
sudo apt-get install openjdk-8-jdk ant ivy
```

Afterwards, a symbolic link must be created (in order for Ant to find Ivy...):

```
cd /usr/share/ant/lib
sudo ln -s ../../java/ivy.jar
```

# Running the tool

Move into the `release/` subfolder, generated after a successful build, and
launch the program by running `run.sh`. For example:

```
cd release/
./run.sh --robot sample/model/ur5.kindsl --query sample/model/ur5.dtdsl --output-dir sample/gen
```

The command above uses the sample input files available in `release/sample/model`.

