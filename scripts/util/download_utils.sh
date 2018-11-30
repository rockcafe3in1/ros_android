download() {
    if [ ! -z $1 ]; then
        cmd_exists curl && curl -L $1 -O || wget $1
    else 
        echo "skipping download of $1 as it's already local"
    fi
}

download_bz2() {
    echo "downloading $1"
    ( cmd_exists curl && curl -L $1 || wget -O - $1 ) | tar jx -C $2
}

download_gz() {
    echo "downloading $1"
    ( cmd_exists curl && curl -L $1 || wget -O - $1 ) | tar zx -C $2
}

download_zip() {
    cmd_exists unzip || die 'could not find unzip'

    echo "downloading $1"

    tmpdir=$(mktemp -d /tmp/rba.XXXX)
    tmpfile=$tmpdir/gtest.zip
    ( cmd_exists curl && curl -L $1 -o $tmpfile || wget $1 -O $tmpfile ) && unzip $tmpfile -d $2
    rm -rf $tmpdir
}
