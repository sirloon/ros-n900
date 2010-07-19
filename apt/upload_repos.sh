#!/bin/bash


if [ "$1" = "" ]
then
    if test -d __repo__
    then
        REPODIR=__repo__
    else
        echo "Provide a directory containing a local repository"
        exit 255
    fi
else
    REPODIR=$1
fi

if [ "$UPLOAD_PROJECT" != "" ]
then
    PROJECT="$UPLOAD_PROJECT"
else
    PROJECT="ros-n900"
fi


echo "Uploading local repository contained in $REPODIR"

which googlecode_upload.py
if [ "$?" != "0" ]
then
    if [ "$GC_UPLOAD" = "" ]
    then
        echo "Can't find googlecode_upload.py script. Either:"
        echo "  - put it in PATH"
        echo "  - or specify GC_UPLOAD environment"
        echo
    fi
else
    GC_UPLOAD=`which googlecode_upload.py`
fi

if [ "$UPLOAD_USER" = "" ]
then
    echo "UPLOAD_USER not set, please enter upload username ?"
    read UPLOAD_USER
fi
if [ "$UPLOAD_PASS" = "" ]
then
    echo "UPLOAD_PASS not set, please enter upload password ?"
    read UPLOAD_PASS
fi


echo "Generating Packages.gz"
dpkg-scanpackages $REPODIR | sed "s#$REPODIR/##" | gzip -9 - > $REPODIR/Packages.gz
echo "Generating Sources.gz"
dpkg-scansources $REPODIR | sed "s#$REPODIR/##" | gzip -9 - > $REPODIR/Sources.gz

echo "Now upload $REPODIR content to Google Code..."
for f in `ls __repo__`
do
    if [ "$f" = "Packages.gz" ]
    then
        $GC_UPLOAD -s "Packages.gz Debian repository file" -p "$PROJECT" -u $UPLOAD_USER -w $UPLOAD_PASS $REPODIR/$f
    elif [ "$f" = "Packages.gz" ]
    then
        $GC_UPLOAD -s "Sources.gz Debian repository file" -p "$PROJECT" -u $UPLOAD_USER -w $UPLOAD_PASS $REPODIR/$f
    elif echo $f | grep \.diff\.gz
    then
        packname=`echo $f | sed "s#\.diff\.gz##"`
        desc=`dpkg -I $REPODIR/$packname*.deb | grep "Description:" | sed "s# Description: ##"`
        $GC_UPLOAD -s "$desc - Source diff" -p "$PROJECT" -u $UPLOAD_USER -w $UPLOAD_PASS $REPODIR/$f
    elif echo $f | grep \.dsc
    then
        packtmp=`echo $f | sed "s#\.dsc##"`
        desc=`dpkg -I $REPODIR/$packname*.deb | grep "Description:" | sed "s# Description: ##"`
        $GC_UPLOAD -s "$desc - Source dsc" -p "$PROJECT" -u $UPLOAD_USER -w $UPLOAD_PASS $REPODIR/$f
    elif echo $f | grep \.tar.gz
    then
        $GC_UPLOAD -s "Source tarball" -p "$PROJECT" -u $UPLOAD_USER -w $UPLOAD_PASS $REPODIR/$f
    elif echo $f | grep \.deb
    then
        desc=`dpkg -I $REPODIR/$f | grep "Description:" | sed "s# Description: ##"`
        $GC_UPLOAD -s "$desc" -p "$PROJECT" -u $UPLOAD_USER -w $UPLOAD_PASS $REPODIR/$f
    else
        echo "Unknown file type for $f"
        exit 1
    fi
done
