#!/bin/bash


if [ "$1" = "" ]
then
    echo "Provide either a Packages.gz or Sources.gz URL"
    exit 255
fi

OUTPUTDIR=$2
if [ "$OUTPUTDIR" = "" ]
then
    OUTPUTDIR=__repo__
fi
test -d $OUTPUTDIR || mkdir $OUTPUTDIR

if echo $1 | grep Packages.gz 
then
    PACKAGESGZ=$1
    SOURCESGZ=
elif echo $1 | grep Sources.gz
then
    PACKAGESGZ=
    SOURCESGZ=$1
fi

echo PACKAGESGZ: $PACKAGESGZ
echo SOURCESGZ: $SOURCESGZ

if [ $PACKAGESGZ != "" ]
then
    echo Downloading packages into $OUTPUTDIR
    wget --no-cache $PACKAGESGZ -O Packages.gz
    ROOTURL=`echo $PACKAGESGZ | sed "s#Packages\.gz##"`
    for f in `zcat Packages.gz | grep Filename: | sed "s#Filename:##"`
    do
        wget -c $ROOTURL/$f -P $OUTPUTDIR
    done

else
    echo Downloading sources into $OUTPUTDIR
    wget --no-cache $SOURCESGZ -O Sources.gz
    ROOTURL=`echo $SOURCESGZ | sed "s#Sources\.gz##"`
    for f in `zgrep "^ " Sources.gz | awk '{print $3}'`
    do
        wget -c $ROOTURL/$f -P $OUTPUTDIR 
    done
fi

echo Done ! Local repository is stored in `pwd`/$OUTPUTDIR
echo Feel free to update its content and run upload_repos.sh
echo to generate new Packages.gz and Sources.gz files, as well
echo as uploading all these files to Google Code

